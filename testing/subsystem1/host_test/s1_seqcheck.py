#!/usr/bin/env python3
"""s1_seqcheck.py - score a Mosquitto capture of the robot's MQTT traffic.

Capture (binary-safe: topic and hex payload, Unix time with nanoseconds):
  mosquitto_sub -h <broker> -v -t 'robot/g10/#' -t '<heartbeat topic>' \
      -F '%U %t %x' > run.cap

Usage:
  python3 s1_seqcheck.py run.cap [--from T] [--to T] [--drain S] [--expect N]
  python3 s1_seqcheck.py --selftest

T is a Unix time or a local HH:MM:SS on the capture's first day.  Without
--from/--to the scored interval runs from the first to the last heartbeat
of each session.  --drain S: the interval ends at the first heartbeat at
least S seconds after the last command (default 6).  --expect N: number of
commands the run must contain (default: no check).

Decoding follows app_program/subsystem1/s1_codec.c:
  telemetry robot/g10/t   14 bytes, MSB first: version 3 bits (must be 1),
                          seq 8 bits, ...
  command   robot/g10/cmd bits 7-6 command (0 start, 1 stop, 2 reset,
                          3 = extension, opcode in byte 2: 0 pause,
                          1 resume), bits 5-0 sequence
  ack       robot/g10/ack bits 7-6 result (0 queued, 1 busy, 2 rejected),
                          bits 5-0 the command's sequence
  heartbeat "hb N s1 tq=.. tl=.. eq=.. ed=a/f/b/o pa=t/e rf=m/c/o
                          cb=ok/err/stale uc=.. if=t/e hr=.. lf=.."

A new session starts when the heartbeat count or the robot's tq counter
goes down (power cycle).  Capture each power cycle to its own file anyway.

Exit status: 0 PASS, 1 FAIL, 2 bad input.
"""

import argparse
import datetime
import re
import sys

PERIOD = 0.2          # telemetry period, s
STALL = 0.6           # arrival gap reported as a stall, s
BURST = 0.05          # inter-arrival below this after a stall = burst
ACK_WINDOW = 5.0      # an ack must arrive within this many s of its command
CMD_NAMES = {0: "start", 1: "stop", 2: "reset"}
EXT_NAMES = {0: "pause", 1: "resume"}
ACK_NAMES = {0: "queued", 1: "busy", 2: "rejected", 3: "reserved"}
HB_RE = re.compile(r"^hb (\d+) s1 (.*)$")


class Msg:
    def __init__(self, t, topic, payload, line):
        self.t, self.topic, self.payload, self.line = t, topic, payload, line


def parse_capture(lines):
    msgs, bad = [], []
    for no, raw in enumerate(lines, 1):
        raw = raw.rstrip("\n")
        if not raw.strip():
            continue
        parts = raw.split(" ")
        try:
            t = float(parts[0])
            topic = parts[1]
            hexs = parts[2] if len(parts) > 2 else ""
            payload = bytes.fromhex(hexs)
        except (ValueError, IndexError):
            bad.append((no, raw[:60]))
            continue
        msgs.append(Msg(t, topic, payload, no))
    return msgs, bad


def parse_counters(text):
    m = HB_RE.match(text)
    if not m:
        return None
    out = {"hb": int(m.group(1))}
    for field in m.group(2).split():
        if "=" not in field:
            continue
        key, vals = field.split("=", 1)
        nums = [int(v) for v in vals.split("/") if v.isdigit()]
        if len(nums) == 1:
            out[key] = nums[0]
        else:
            out[key] = nums
    return out


def split_sessions(msgs):
    sessions, cur, last_hb, last_tq = [], [], None, None
    for m in msgs:
        if m.topic.endswith("heartbeat") or m.payload.startswith(b"hb "):
            c = parse_counters(m.payload.decode("ascii", "replace"))
            if c is not None:
                if ((last_hb is not None and c["hb"] < last_hb) or
                        (last_tq is not None and c.get("tq", 0) < last_tq)):
                    sessions.append(cur)
                    cur = []
                last_hb, last_tq = c["hb"], c.get("tq", 0)
        cur.append(m)
    if cur:
        sessions.append(cur)
    return sessions


def decode_tlm(payload):
    if len(payload) != 14:
        return None
    word = (payload[0] << 8) | payload[1]
    if (word >> 13) != 1:
        return None
    return (word >> 5) & 0xFF


def analyse_telemetry(frames):
    """frames: [(t, seq)] in arrival order -> dict"""
    res = {"received": len(frames), "duplicates": [], "late": [],
           "stalls": [], "ambiguous": [], "idx": []}
    if not frames:
        res.update(missing=[], first=None, last=None)
        return res
    seen = {}
    prev_t, prev_s, prev_i = frames[0][0], frames[0][1], frames[0][1]
    seen[prev_i] = frames[0][0]
    res["idx"].append((frames[0][0], prev_i, prev_s))
    hi = prev_i
    for t, s in frames[1:]:
        d = (s - prev_s) & 0xFF
        dt = t - prev_t
        if d > 128 and dt < 128 * PERIOD:
            idx = prev_i + d - 256            # older frame arriving late
        else:
            idx = prev_i + d
            wraps = round((dt / PERIOD - d) / 256.0)
            if wraps > 0:
                idx += 256 * wraps
                res["ambiguous"].append((t, s, wraps))
        if idx in seen:
            res["duplicates"].append((t, s))
        else:
            if idx < hi:
                res["late"].append((t, s))
            seen[idx] = t
            res["idx"].append((t, idx, s))
        if dt > STALL:
            res["stalls"].append([prev_t, t, dt, d - 1 if d > 0 else 0, 0])
        prev_t, prev_s, prev_i = t, s, idx
        hi = max(hi, idx)
    lo = min(seen)
    res["first"], res["last"] = lo, hi
    res["missing"] = [i for i in range(lo, hi + 1) if i not in seen]
    # frames delivered in a burst right after each stall
    times = [t for t, _ in frames]
    for st in res["stalls"]:
        n, k = 0, times.index(st[1])
        while k + 1 < len(times) and times[k + 1] - times[k] < BURST:
            n, k = n + 1, k + 1
        st[4] = n
    return res


def pair_commands(cmds, acks):
    """cmds: [(t, seq, name)], acks: [(t, seq, result)]"""
    used, rows, dup_cmd = set(), [], []
    seen_cmd = {}
    for t, seq, name in cmds:
        if seq in seen_cmd and t - seen_cmd[seq] < ACK_WINDOW:
            dup_cmd.append((t, seq))
        seen_cmd[seq] = t
        match = None
        for k, (ta, sa, ra) in enumerate(acks):
            if k in used or sa != seq or ta < t - 0.5 or ta > t + ACK_WINDOW:
                continue
            match = k
            break
        if match is None:
            rows.append((t, seq, name, None, None, None))
        else:
            used.add(match)
            ta, _, ra = acks[match]
            rows.append((t, seq, name, ta, ra, ta - t))
    extra = [acks[k] for k in range(len(acks)) if k not in used]
    dup_acks, mismatched = [], []
    paired_seqs = {r[1] for r in rows if r[3] is not None}
    for a in extra:
        (dup_acks if a[1] in paired_seqs else mismatched).append(a)
    return rows, dup_cmd, dup_acks, mismatched


def fmt_t(t):
    return datetime.datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")[:-3]


def to_time(text, day0):
    if text is None:
        return None
    if re.match(r"^\d+(\.\d+)?$", text):
        return float(text)
    h, m, s = text.split(":")
    d = datetime.datetime.fromtimestamp(day0).replace(
        hour=int(h), minute=int(m), second=int(float(s)), microsecond=0)
    return d.timestamp()


def score_session(no, msgs, args, out):
    tlm, cmds, acks, hbs, invalid = [], [], [], [], []
    for m in msgs:
        if m.topic.endswith("/t"):
            s = decode_tlm(m.payload)
            if s is None:
                invalid.append((m.t, m.topic, m.payload.hex()))
            else:
                tlm.append((m.t, s))
        elif m.topic.endswith("/cmd"):
            if len(m.payload) in (1, 2):
                code, seq = m.payload[0] >> 6, m.payload[0] & 0x3F
                if code == 3:
                    name = (EXT_NAMES.get(m.payload[1], "ext?")
                            if len(m.payload) == 2 else "ext?")
                else:
                    name = CMD_NAMES[code]
                cmds.append((m.t, seq, name))
            else:
                invalid.append((m.t, m.topic, m.payload.hex()))
        elif m.topic.endswith("/ack"):
            if len(m.payload) == 1:
                acks.append((m.t, m.payload[0] & 0x3F, m.payload[0] >> 6))
            else:
                invalid.append((m.t, m.topic, m.payload.hex()))
        else:
            c = parse_counters(m.payload.decode("ascii", "replace"))
            if c is not None:
                hbs.append((m.t, c))

    day0 = msgs[0].t
    t_from = to_time(args.start, day0)
    t_to = to_time(args.end, day0)
    if t_from is None and hbs:
        t_from = hbs[0][0]
    if t_to is None and hbs:
        t_to = hbs[-1][0]
        if cmds:
            after = [h for h in hbs if h[0] >= cmds[-1][0] + args.drain]
            if after:
                t_to = after[0][0]
    if t_from is None:
        t_from = msgs[0].t
    if t_to is None:
        t_to = msgs[-1].t

    p = out.append
    p("== session %d: %s .. %s, %d messages" %
      (no, fmt_t(msgs[0].t), fmt_t(msgs[-1].t), len(msgs)))
    p("scored interval %s .. %s (drain end = last heartbeat used)" %
      (fmt_t(t_from), fmt_t(t_to)))
    fails = []

    # ---- commands and acks
    in_cmds = [c for c in cmds if t_from <= c[0] <= t_to]
    rows, dup_cmd, dup_acks, mismatched = pair_commands(
        in_cmds, [a for a in acks if t_from <= a[0] <= t_to + ACK_WINDOW])
    p("commands %d (send = broker arrival time):" % len(rows))
    for t, seq, name, ta, ra, lat in rows:
        if ta is None:
            p("  %s cmd %-6s id %2d  expected queued  NO ACK" %
              (fmt_t(t), name, seq))
            fails.append("no ack for id %d" % seq)
        else:
            ok = (ra == 0)
            p("  %s cmd %-6s id %2d  expected queued  ack id %2d %-8s %4.0f ms%s"
              % (fmt_t(t), name, seq, seq, ACK_NAMES[ra], lat * 1000,
                 "" if ok else "  WRONG RESULT"))
            if not ok:
                fails.append("id %d acked %s" % (seq, ACK_NAMES[ra]))
    for t, seq in dup_cmd:
        p("  duplicate command id %d at %s" % (seq, fmt_t(t)))
    for t, seq, r in dup_acks:
        p("  duplicate ack id %d (%s) at %s" % (seq, ACK_NAMES[r], fmt_t(t)))
        fails.append("duplicate ack id %d" % seq)
    for t, seq, r in mismatched:
        p("  ack id %d (%s) at %s matches no command" %
          (seq, ACK_NAMES[r], fmt_t(t)))
        fails.append("mismatched ack id %d" % seq)
    if args.expect is not None and len(rows) != args.expect:
        fails.append("%d commands, expected %d" % (len(rows), args.expect))

    # ---- telemetry
    ta = analyse_telemetry(tlm)
    p("telemetry: %d frames, %d invalid, %d duplicate, %d late" %
      (ta["received"], len([i for i in invalid if i[1].endswith("/t")]),
       len(ta["duplicates"]), len(ta["late"])))
    for t, s in ta["duplicates"]:
        p("  duplicate seq %d at %s" % (s, fmt_t(t)))
    for t, s in ta["late"]:
        p("  late (out of order) seq %d at %s" % (s, fmt_t(t)))
    for t, s, w in ta["ambiguous"]:
        p("  %d seq wrap(s) assumed before seq %d at %s (long silence)" %
          (w, s, fmt_t(t)))
    for t0, t1, dt, skipped, burst in ta["stalls"]:
        kind = ("missing %d seq" % skipped) if skipped else "no seq missing"
        p("  stall %s .. %s %4.0f ms, %s, %d frames burst after" %
          (fmt_t(t0), fmt_t(t1), dt * 1000, kind, burst))
    for t, topic, hx in invalid:
        p("  invalid %s payload %s at %s" % (topic, hx or "(empty)", fmt_t(t)))

    # ---- firmware counters across the interval
    h_in = [h for h in hbs if t_from <= h[0] <= t_to]
    if len(h_in) < 2:
        p("counters: fewer than 2 heartbeats in the interval: NOT FOUND")
        fails.append("no counter snapshots")
        return fails
    (ta0, c0), (ta1, c1) = h_in[0], h_in[-1]

    def d(key, i=None):
        a, b = c0.get(key, 0), c1.get(key, 0)
        if i is not None:
            a, b = a[i], b[i]
        return b - a

    p("counters %s -> %s:" % (fmt_t(ta0), fmt_t(ta1)))
    p("  " + " ".join("%s+%s" % (k, d(k)) for k in ("tq", "tl", "eq", "uc",
                                                    "hr", "lf")) +
      "  ed+%s  rf+%s  cb+%s" % (
          "/".join(str(d("ed", i)) for i in range(4)),
          "/".join(str(d("rf", i)) for i in range(3)),
          "/".join(str(d("cb", i)) for i in range(3))))
    ev_drop = sum(d("ed", i) for i in range(4))
    if ev_drop:
        fails.append("%d events dropped (ed)" % ev_drop)

    # map frame numbers: frame n (= tq after its post) carries seq (n-1)&255
    idx = [x for x in ta["idx"] if x[0] <= ta0]
    if not idx:
        idx = ta["idx"][:1]
    t_near, i_near, _ = idx[-1]
    off = c0["tq"] - (i_near + 1)
    off = int(round(off / 256.0)) * 256
    n_lo, n_hi = c0["tq"], c1["tq"]
    missing = [i + 1 + off for i in ta["missing"]]
    m_in = [n for n in missing if n_lo < n <= n_hi]
    pend0 = c0["tq"] - c0["pa"][0] - c0["tl"]
    pend1 = c1["tq"] - c1["pa"][0] - c1["tl"]
    dtl = d("tl")
    lo, hi = dtl - pend0, dtl + pend1
    p("  frames %d..%d: %d missing at the subscriber; firmware skipped %d "
      "(S1 held %d at start, %d at end)" % (n_lo + 1, n_hi, len(m_in), dtl,
                                            pend0, pend1))
    if m_in:
        p("  missing seq: " + " ".join(str((n - 1) & 0xFF) for n in m_in[:80]) +
          (" ..." if len(m_in) > 80 else ""))
    if lo <= len(m_in) <= hi:
        p("  gaps reconcile with the firmware skip count")
    else:
        extra = len(m_in) - hi if len(m_in) > hi else len(m_in) - lo
        p("  UNEXPLAINED: %+d frames versus the skip count%s" %
          (extra, (" (uc+%d accepted-but-unconfirmed may cover some)" %
                   d("uc")) if d("uc") else ""))
        fails.append("telemetry gaps do not reconcile (%+d)" % extra)
    if d("uc"):
        p("  reconnection(s) during the interval: uc+%d" % d("uc"))
    return fails


def run(lines, args, out):
    msgs, bad = parse_capture(lines)
    if not msgs:
        out.append("no messages")
        return 2
    for no, raw in bad:
        out.append("unreadable line %d: %s" % (no, raw))
    fails_all = []
    for k, sess in enumerate(split_sessions(msgs), 1):
        fails = score_session(k, sess, args, out)
        out.append("session %d: %s" % (k, "PASS" if not fails else
                                       "FAIL: " + "; ".join(fails)))
        fails_all += fails
    return 0 if not fails_all else 1


# ---------------------------------------------------------------- self-test
def _frame(seq):
    word = (1 << 13) | (seq << 5)
    return bytes([word >> 8, word & 0xFF]) + bytes(12)


def _hb(n, tq, tl, pa, ed=(0, 0, 0, 0), uc=0):
    return ("hb %d s1 tq=%d tl=%d eq=0 ed=%d/%d/%d/%d pa=%d/0 rf=0/0/0 "
            "cb=0/0/0 uc=%d if=0/0 hr=0 lf=0" %
            ((n, tq, tl) + tuple(ed) + (pa, uc))).encode()


def _fixture():
    """Two sessions.  Session 1: 300 frames (wraps at 256), a 1 s stall
    with 5 frames burst late (nothing missing), seqs 100..102 missing and
    counted by the firmware (tl), one duplicate frame, one invalid frame,
    commands 30 and 36 acked, 37 acked busy, 38 never acked, a stray ack.
    Session 2 (after a power cycle): 20 clean frames, seq 3 missing and
    NOT counted by the firmware."""
    t0, L = 1764000000.0, []

    def add(t, topic, payload):
        L.append("%.9f %s %s" % (t, topic, payload.hex()))
    tq = 0
    for n in range(1, 301):                     # frame n has seq (n-1)&255
        t = t0 + n * PERIOD
        tq = n
        if n in (101, 102, 103):                # seq 100..102 skipped in S1
            continue
        if 51 <= n <= 55:                       # stalled, then a burst
            t = t0 + 56 * PERIOD - 0.01 * (56 - n)
        add(t, "robot/g10/t", _frame((n - 1) & 0xFF))
        if n == 150:
            add(t + 0.001, "robot/g10/t", _frame((n - 1) & 0xFF))
        if n == 160:
            add(t + 0.002, "robot/g10/t", b"\x01\x02")
        if n % 25 == 0:
            tl = 3 if n > 103 else 0
            add(t + 0.003, "mtk3smp-pico/heartbeat",
                _hb(n // 25 - 1, tq, tl, tq - tl))
    for t_off, seq, code, res in ((10.0, 30, 0, 0), (20.0, 36, 1, 0),
                                  (30.0, 37, 0, 1), (40.0, 38, 1, None)):
        add(t0 + t_off, "robot/g10/cmd", bytes([(code << 6) | seq]))
        if res is not None:
            add(t0 + t_off + 0.12, "robot/g10/ack", bytes([(res << 6) | seq]))
    add(t0 + 45.0, "robot/g10/ack", bytes([(0 << 6) | 12]))
    t1 = t0 + 100.0
    for n in range(1, 21):
        if n != 4:
            add(t1 + n * PERIOD, "robot/g10/t", _frame(n - 1))
        if n in (1, 20):
            add(t1 + n * PERIOD + 0.003, "mtk3smp-pico/heartbeat",
                _hb(0 if n == 1 else 1, n, 0, n))
    L.sort(key=lambda s: float(s.split()[0]))
    return L


def selftest():
    args = argparse.Namespace(start=None, end=None, drain=6.0, expect=None)
    out = []
    rc = run(_fixture(), args, out)
    text = "\n".join(out)
    checks = [
        ("two sessions split at the power cycle", "== session 2:" in text),
        ("wrap at 256 is not a gap", "missing at the subscriber" in text and
         "UNEXPLAINED: +1" in text),
        ("firmware-counted gap reconciles",
         "3 missing at the subscriber; firmware skipped 3" in text and
         "missing seq: 100 101 102" in text),
        ("late burst is not loss", "no seq missing, 5 frames burst" in text),
        ("duplicate frame", "duplicate seq 149" in text),
        ("invalid frame", "invalid robot/g10/t payload 0102" in text),
        ("ack matched with latency",
         "id 30  expected queued  ack id 30 queued    120 ms" in text),
        ("wrong ack result", "id 37 acked busy" in text),
        ("missing ack", "no ack for id 38" in text),
        ("mismatched ack", "ack id 12 (queued)" in text),
        ("uncounted gap in session 2 is unexplained",
         "session 2: FAIL" in text and "missing seq: 3" in text),
        ("exit status FAIL", rc == 1),
    ]
    failed = 0
    for name, ok in checks:
        print("[TEST] S1-C%02d %s  checker: %s" %
              (checks.index((name, ok)) + 1, "PASS" if ok else "FAIL", name))
        failed += 0 if ok else 1
    if failed:
        print(text)
    print("[TEST] subsystem1 seqcheck SUMMARY %d passed, %d failed" %
          (len(checks) - failed, failed))
    return 1 if failed else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("capture", nargs="?")
    ap.add_argument("--from", dest="start")
    ap.add_argument("--to", dest="end")
    ap.add_argument("--drain", type=float, default=6.0)
    ap.add_argument("--expect", type=int)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if not args.capture:
        ap.error("capture file required")
    with open(args.capture, encoding="ascii", errors="replace") as f:
        lines = f.readlines()
    out = []
    rc = run(lines, args, out)
    print("\n".join(out))
    return rc


if __name__ == "__main__":
    sys.exit(main())
