#!/usr/bin/env node
/*
 * Dashboard checks (S1-D01 ..).  Plain Node, no packages.
 * Loads the CORE block of tools/dashboard.html (the code the page runs),
 * compares it with the firmware's C encoder (tlm_encode, built by the
 * Makefile) and with the firmware sources, and checks the page's theme.
 *   node test_dashboard.js <path to tlm_encode binary>
 */
"use strict";
const fs = require("fs");
const path = require("path");
const { execFileSync } = require("child_process");

const ROOT = path.resolve(__dirname, "../../..");
const HTML_PATH = path.join(ROOT, "tools/dashboard.html");
const HTML = fs.readFileSync(HTML_PATH, "utf8");
const ENCODER = process.argv[2] || path.join(__dirname, "tlm_encode");
const CFG_ENCODER = process.argv[3] || path.join(__dirname, "cfg_encode");
const OBST_ENCODER = path.join(__dirname, "obst_encode"); /* built by the Makefile */

let passed = 0, failed = 0;
function check(id, ok, desc, detail) {
  if (ok) { passed++; console.log("[TEST] " + id + " PASS  " + desc); }
  else { failed++; console.log("[TEST] " + id + " FAIL  " + desc + (detail ? "  -- " + detail : "")); }
}
function loadCore(html) {
  const a = html.indexOf("/* CORE-BEGIN"), b = html.indexOf("/* CORE-END */");
  if (a < 0 || b < 0) throw new Error("CORE-BEGIN / CORE-END markers not found in dashboard.html");
  return new Function(html.slice(a, b) + "\nreturn DashCore;")();
}
const C = loadCore(HTML);
const src = rel => fs.readFileSync(path.join(ROOT, rel), "utf8");
const enc = input => execFileSync(ENCODER, { input }).toString().trim().split("\n");
const hexBytes = h => Uint8Array.from(Buffer.from(h, "hex"));
const cfgEnc = input => execFileSync(CFG_ENCODER, { input }).toString().trim().split("\n");
const toHex = b => Buffer.from(b).toString("hex");

/* ---------- D01-D04: wire parity with the C codec ---------- */
{
  const rows = [];
  const states = C.STATES.length;
  for (let i = 0; i < 40; i++) {
    const seq = (240 + i) & 255; /* crosses the 255 -> 0 wrap */
    rows.push([seq, i % states, (i * 37) % 600 - 300, -((i * 53) % 600 - 300), i * 2011, (i * 97) % 2400 - 1200,
      (i * 7) % 140, i & 1, i % 10, i % 5, i % 20, (i * 311) % 9000 - 4500, i * 900, i * 1700,
      [0, 1, 2, 7, 0x2011, 0x4005, 0x5004, 0xABCD][i % 8], (i >> 1) & 1, (i >> 2) & 1]);
  }
  rows.push([200, 9, 200, -100, 12345, -504, 45, 1, 7, 4, 7, -300, 2000, 5500, 0x3001, 1, 0]); /* test_codec.c S1-H12 */
  rows.push([255, 10, 100000, -100000, 100000, 5000, 900, 1, 15, 7, 99, -99999, 999999, -5, 0xABCDEF, 1, 1]); /* saturation */
  const out = enc(rows.map(r => "T " + r.join(" ")).join("\n") + "\n");
  let bad = [];
  out.forEach((line, i) => {
    const [hex, ...f] = line.split(" ");
    const d = C.decodeTelemetry(hexBytes(hex)), n = f.map(Number);
    const want = { seq: n[0], state: C.STATES[n[1]] || "UNKNOWN", speedL: n[2], speedR: n[3], distance: n[4], line: n[5],
      obstacleMm: n[6] === 127 ? null : n[6] * 10, obstacleDetected: !!n[7], obstacleState: n[8], nav: n[9], barcodes: n[10],
      pitchDeg: n[11] / 100, humpMm: n[12] / 1000, humpMaxMm: n[13] / 1000, fault: n[14], wifi: !!n[15], paused: !!n[16] };
    for (const k in want) if (!d || d[k] !== want[k]) { bad.push("row " + i + " " + k + ": js " + (d && d[k]) + " c " + want[k]); break; }
  });
  check("S1-D01", out.length === rows.length && bad.length === 0, "every C-encoded golden frame (incl. seq 254/255/0/1 wrap and saturation) decodes in JS to what s1_tlm_decode reads", bad.slice(0, 3).join("; "));
  const canon = hexBytes("391265ce607382b6f1fe842cc006"); /* test_codec.c expected_frame */
  const d = C.decodeTelemetry(canon);
  check("S1-D02", d && d.seq === 200 && d.state === "FAULT" && d.speedL === 200 && d.speedR === -100 && d.distance === 12345 &&
    d.line === -504 && d.obstacleMm === 450 && d.fault === 0x3001 && d.paused === false && out[out.length - 2].split(" ")[0] === "391265ce607382b6f1fe842cc006",
  "old fixture parity: the canonical S1-H12 vector is unchanged and decodes field for field");
  check("S1-D03", C.decodeTelemetry(new Uint8Array(13)) === null && C.decodeTelemetry(new Uint8Array(15)) === null &&
    C.decodeTelemetry(Uint8Array.from([0xA0, ...new Array(13).fill(0)])) === null, "wrong length and unknown version are rejected");
  const texts = ["A", "Z", "S", "T", "AZ", "HELLO-1.$", "0123456789"];
  const bc = enc(texts.map(t => "B " + t).join("\n") + "\n");
  check("S1-D04", texts.every((t, i) => C.decodeBarcode(hexBytes(bc[i])) === t) &&
    C.decodeBarcode(Uint8Array.from([0xFF])) === null && C.decodeBarcode(new Uint8Array(9)) === null,
  "barcode payloads from s1_barcode_encode decode to the exact text; bad payloads are rejected");
  const cmdIn = [], cmdWant = [];
  for (const [name, code] of [["start", 0], ["stop", 1], ["reset", 2]]) for (let s = 0; s < 64; s++) { cmdIn.push("C " + code + " " + s); cmdWant.push([name, s]); }
  const cmdOut = enc(cmdIn.join("\n") + "\n");
  const ackIn = []; for (let r = 0; r < 4; r++) for (let s = 0; s < 64; s++) ackIn.push("A " + r + " " + s);
  const ackOut = enc(ackIn.join("\n") + "\n");
  check("S1-D05", cmdWant.every(([n, s], i) => C.encodeCommand(n, s)[0] === parseInt(cmdOut[i], 16)) &&
    ackOut.every((h, i) => { const a = C.decodeAck(Uint8Array.from([parseInt(h, 16)])); return a.result === C.ACK[i >> 6] && a.seq === (i & 63); }) &&
    C.encodeCommand("pause", 5).join() === "197,0" && C.encodeCommand("resume", 5).join() === "197,1",
  "command bytes match s1_cmd_encode for all 64 sequences; acks decode for every result; pause/resume extension bytes");
}

/* ---------- D06-D08: fault catalogue covers the firmware ---------- */
function firmwareFaultCodes(files) {
  const found = new Map();
  const defs = (text, re) => { for (const m of text.matchAll(re)) found.set(parseInt(m[2], 16), m[1]); };
  defs(files.vc_types, /#define\s+(VC_FC_(?!NONE\b)\w+)\s+0x([0-9A-Fa-f]+)/g);
  defs(files.s2, /#define\s+(S2_FC_(?!FAULT_BASE|FAULT_MASK)\w+)\s+0x([0-9A-Fa-f]+)/g);
  defs(files.s3, /#define\s+(S3_FC_\w+)\s+0x([0-9A-Fa-f]+)/g);
  defs(files.s4, /#define\s+(S4_FC_\w+)\s+0x([0-9A-Fa-f]+)/g);
  const enumNames = (text, type) => {
    const m = new RegExp("typedef enum\\s*\\{([^}]*)\\}\\s*" + type + ";").exec(text);
    if (!m) throw new Error(type + " not found");
    let v = -1; const out = [];
    for (let e of m[1].replace(/\/\*[\s\S]*?\*\//g, "").split(",")) {
      e = e.trim(); if (!e) continue;
      const mm = /^(\w+)(?:\s*=\s*(\d+))?$/.exec(e); v = mm[2] != null ? +mm[2] : v + 1; out.push([mm[1], v]);
    }
    return out;
  };
  const s2base = parseInt(/#define\s+S2_FC_FAULT_BASE\s+0x([0-9A-Fa-f]+)/.exec(files.s2)[1], 16);
  for (const [n, v] of enumNames(files.s2h, "motion_fault_t")) if (v) found.set(s2base | v, n);
  const s5base = parseInt(/#define\s+S5_FC_FAULT_BASE\s+0x([0-9A-Fa-f]+)/.exec(files.s5)[1], 16);
  for (const [n, v] of enumNames(files.s5h, "s5_fault_code_t")) if (v) found.set(s5base + v, n);
  return found;
}
function coverage(found) {
  const missing = [], wrong = [], extra = [];
  for (const [code, name] of found) {
    const e = C.FAULTS[code];
    if (!e) missing.push(C.hex4(code) + " " + name); else if (e.def !== name) wrong.push(C.hex4(code) + " " + name + " vs " + e.def);
  }
  for (const code of Object.keys(C.FAULTS).map(Number)) if (!found.has(code)) extra.push(C.hex4(code));
  return { missing, wrong, extra };
}
{
  const files = { vc_types: src("app_program/vehicle_controller/vc_types.h"), s2: src("app_program/vehicle_controller/vc_bridge_s2.c"),
    s2h: src("app_program/subsystem2/subsystem2.h"), s3: src("app_program/vehicle_controller/vc_bridge_s3.c"),
    s4: src("app_program/vehicle_controller/vc_bridge_s4.c"), s5: src("app_program/vehicle_controller/vc_bridge_s5.c"),
    s5h: src("app_program/subsystem5/subsystem5.h") };
  const found = firmwareFaultCodes(files);
  const cov = coverage(found);
  check("S1-D06", found.size >= 20 && !cov.missing.length && !cov.wrong.length && !cov.extra.length,
    "every on-wire fault code (VC, S2 0x20nn + motion_fault_t, S3, S4, S5 0x5000 + s5_fault_code_t) has a named entry with an action, and nothing else",
    JSON.stringify(cov));
  /* a new firmware code must fail the check */
  const planted = firmwareFaultCodes({ ...files, vc_types: files.vc_types + "\n#define VC_FC_NEW_THING 0x000A\n",
    s5h: files.s5h.replace("S5_FAULT_LINE_NOT_FOUND", "S5_FAULT_LINE_NOT_FOUND,\n    S5_FAULT_SOMETHING_NEW") });
  const pc = coverage(planted);
  check("S1-D07", pc.missing.includes("0x000A VC_FC_NEW_THING") && pc.missing.includes("0x5006 S5_FAULT_SOMETHING_NEW"),
    "the coverage check fails when a new fault code is added to the firmware", JSON.stringify(pc.missing));
  /* no other fault-code family hiding elsewhere */
  const stray = [];
  const walk = d => { for (const f of fs.readdirSync(d, { withFileTypes: true })) {
    const p = path.join(d, f.name);
    if (f.isDirectory()) { if (f.name !== "host_test") walk(p); }
    else if (/\.[ch]$/.test(f.name)) for (const m of fs.readFileSync(p, "utf8").matchAll(/#define\s+(\w*_FC_\w+)\s+0x([0-9A-Fa-f]+)/g))
      if (!/(_BASE|_MASK|_NONE)$/.test(m[1]) && !found.has(parseInt(m[2], 16))) stray.push(path.relative(ROOT, p) + ":" + m[1]); } };
  walk(path.join(ROOT, "app_program"));
  check("S1-D08", stray.length === 0, "no *_FC_* fault define in app_program is outside the checked families", stray.join(", "));
  const self = C.describeFault({ src: 6, sev: 2, code: 2 });
  const deg = C.describeFault({ src: 6, sev: 0, code: 2 });
  const unk = C.describeFault({ src: 3, sev: 1, code: 0x3abc });
  check("S1-D09", self.name === "Controller self-check" && self.subsystem === "Vehicle controller" && self.severity === "Critical" &&
    /Check which subsystem is not ready/.test(self.action) && deg.name !== self.name && unk.name === "Unknown 0x3ABC" && !unk.known &&
    C.parseFaultConsole("vc: fault from S6  CRITICAL  code 0x0002").code === 2,
  "{src:6,sev:2,code:2} reads as the controller self-check with its action; sev 0 differs; unknown codes show Unknown 0xNNNN; console form parses");
  const rec = fs.readFileSync(path.join(__dirname, "fixtures/recorded_dashboard_log_2026-11-24.txt"), "utf8").split("\n");
  const fl = rec.filter(l => l.startsWith("[mqtt robot/g10/fault] ")).map(l => C.parseFaultJson(l.slice(23)));
  check("S1-D10", fl.length > 100 && fl.every(f => f && C.describeFault(f).known),
    "all " + fl.length + " fault reports in the owner's recorded dashboard log (24 Nov) parse and are known codes");
  /* the factory map comes from the firmware (READ_FACTORY), not from a table in the page */
  const want = [...C.MAP_LETTERS].map(() => 0);
  for (const m of src("app_program/vehicle_controller/vc_barcode_config.h").replace(/\/\*[\s\S]*?\*\//g, "").matchAll(/X\(([A-Z]),\s*NAV_(\w+)\)/g))
    want[C.MAP_LETTERS.indexOf(m[1])] = { NONE: 0, STRAIGHT: 0, LEFT: 1, RIGHT: 2, U_TURN: 3 }[m[2]];
  const fw = C.unpackMap(hexBytes(cfgEnc("F\n")[0]));
  check("S1-D11", C.sameMap(fw, want) && fw[0] === 0 && fw[7] === 1 && fw[25] === 2 && !/BARCODE_RULES|factoryMap/.test(HTML) &&
    C.configuredAction("S").startsWith("Not known") && C.configuredAction("HELLO").startsWith("None") && C.configuredAction("S", fw.map((v, i) => i === 18 ? 3 : v)) === "U-turn (180°)",
    "the robot's factory map equals vc_barcode_config.h with NAV_NONE as straight (A straight, H left, Z right); the page has no letter table of its own; an action is shown only from the robot's map");
}

/* ---------- D12: theme contrast in both themes ---------- */
function themeVars(block) { const o = {}; for (const m of block.matchAll(/--([\w-]+):\s*(#[0-9a-fA-F]{6})/g)) o[m[1]] = m[2]; return o; }
function lum(hex) { const c = [1, 3, 5].map(i => parseInt(hex.slice(i, i + 2), 16) / 255).map(v => v <= 0.03928 ? v / 12.92 : ((v + 0.055) / 1.055) ** 2.4); return 0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2]; }
function ratio(a, b) { const x = lum(a), y = lum(b); return (Math.max(x, y) + 0.05) / (Math.min(x, y) + 0.05); }
{
  const style = /<style>([\s\S]*?)<\/style>/.exec(HTML)[1];
  const light = themeVars(/:root\s*\{([\s\S]*?)\}/.exec(style)[1]);
  const dark = { ...light, ...themeVars(/:root:not\(\[data-theme="light"\]\)\s*\{([\s\S]*?)\}/.exec(style)[1]) };
  const fams = ["idle", "work", "attn", "fault", "good"];
  const pairs = [["text", "bg", 4.5], ["text", "panel", 4.5], ["muted", "panel", 4.5], ["muted", "bg", 4.5], ["text", "chip", 4.5],
    ["edge", "panel", 3], ["focus", "panel", 3], ["focus", "bg", 3]];
  for (const f of fams) pairs.push([f, f + "-bg", 4.5], [f, "panel", 4.5], ["on-" + f, f + "-solid", 4.5], [f + "-solid", "panel", 3]);
  const bad = [];
  for (const [name, v] of [["light", light], ["dark", dark]])
    for (const [fg, bg, min] of pairs) { const r = ratio(v[fg], v[bg]); if (!(r >= min)) bad.push(name + " " + fg + "/" + bg + " " + r.toFixed(2) + " < " + min); }
  check("S1-D12", bad.length === 0, "text 4.5:1 and boundaries, focus ring and solid buttons 3:1 against their backgrounds, light and dark (" + pairs.length * 2 + " pairs)", bad.join("; "));
  const outside = (style.replace(/:root\s*\{[\s\S]*?\}/, "").replace(/@media \(prefers-color-scheme: dark\)\s*\{\s*:root[^{]*\{[\s\S]*?\}\s*\}/, "") +
    HTML.slice(HTML.indexOf("<script>"))).match(/#[0-9a-fA-F]{3,8}\b(?![\w-])|rgba?\(/g) || [];
  check("S1-D13", outside.length === 0, "colour literals appear only in the two theme blocks", outside.join(" "));
}

/* ---------- D14-D18: MQTT loss window ---------- */
{
  let w = new C.LossWindow(), t = 0;
  for (let s = 250; s < 256; s++) w.add(s, t += 200);
  for (let s = 0; s < 4; s++) w.add(s, t += 200);
  let st = w.stats(t);
  check("S1-D14", st.received === 10 && st.missing === 0 && st.loss === 0, "sequence wrap 255 -> 0 is not a gap", JSON.stringify(st));
  const g = w.add(7, t += 800); /* 4,5,6 missing */
  const dup = w.add(7, t += 10);
  st = w.stats(t);
  check("S1-D15", g.missing === 3 && dup.duplicate && st.received === 11 && st.missing === 3 && Math.abs(st.loss - 3 / 14) < 1e-9,
    "gap counts the skipped frames; a duplicate is ignored; loss = missing / (unique received + missing)", JSON.stringify(st));
  const late = w.stats(t + 61000);
  check("S1-D16", late.received === 0 && late.missing === 0 && late.loss === null, "everything older than 60 s expires; empty window reports no data, not 0%");
  w = new C.LossWindow(); t = 0;
  w.add(10, t += 200); const amb = w.add(15, t += 9000); /* 9 s silence */
  const rb = w.add(3, t += 200); /* went backwards: reboot */
  w.reset(); const rc = w.add(200, t += 200); /* after reconnect */
  const jump = w.add(250, t += 400); /* 49 missing in 0.4 s: impossible */
  st = w.stats(t);
  check("S1-D17", amb.ambiguous && !amb.missing && rb.ambiguous && !rb.missing && rc.missing === 0 && !rc.ambiguous && jump.ambiguous && st.missing === 0,
    "long silence, restart, reconnect and impossible jumps count no invented missing frames", JSON.stringify(st));
  w = new C.LossWindow(); t = 0; let seq = 0;
  const step = (skip, dt) => { seq = (seq + 1 + skip) & 255; return w.add(seq, t += dt); };
  step(0, 200); const a = step(2, 200); const b = step(1, 600); const c = step(3, 1900); const d = step(1, 2100);
  check("S1-D18", a.newBurst && !b.newBurst && !c.newBurst && c.burst.frames === 6 && c.burst.gaps === 3 && d.newBurst && w.bursts === 2,
    "gaps within 2 s of the previous gap form one burst (one event); a gap after more than 2 s starts a new burst");
}

/* ---------- D19-D21: storage cap, restore, damage, quota ---------- */
{
  const mk = n => Array.from({ length: n }, (_, i) => ({ t: i, text: "event " + i + " " + "x".repeat(50) }));
  const p = C.packStore({ savedAt: 1, events: mk(1000), barcodes: mk(5), faults: mk(150), last: { tlm: { state: "FAULT" }, at: 5 }, run: { current: { kind: "fault" } } });
  const back = C.unpackStore(p.json).data;
  check("S1-D19", back.events.length === C.STORE_CAPS.events && back.events[back.events.length - 1].t === 999 && back.faults.length === 100 &&
    back.last.tlm.state === "FAULT" && back.run.current.kind === "fault" && p.json.length <= C.STORE_CAPS.bytes && p.dropped === 750,
  "history is capped by count (newest kept) and restores events, faults, last state and run outcome");
  const big = C.packStore({ savedAt: 1, events: Array.from({ length: 300 }, (_, i) => ({ t: i, text: "y".repeat(2000) })), barcodes: [], faults: [] });
  check("S1-D20", big.json.length <= C.STORE_CAPS.bytes && big.dropped > 0 && C.unpackStore(big.json).data.events.slice(-1)[0].t === 299,
    "the 300 kB byte budget drops the oldest records first");
  const dmg = [C.unpackStore("{not json"), C.unpackStore(JSON.stringify({ v: 99, events: [] })), C.unpackStore(JSON.stringify({ v: 1, events: [{ t: 1 }, null, { x: 2 }], barcodes: [], faults: [] }))];
  let stored = null, calls = 0;
  const quota = { setItem(k, v) { calls++; if (v.length > 20000) { const e = new Error("full"); e.name = "QuotaExceededError"; throw e; } stored = v; } };
  const wr = C.writeStore(quota, { savedAt: 1, events: mk(300).map(e => ({ ...e, text: e.text + "z".repeat(60) })), barcodes: [], faults: [] });
  const dead = C.writeStore({ setItem() { throw new Error("SecurityError"); } }, { savedAt: 1, events: [], barcodes: [], faults: [] });
  check("S1-D21", dmg[0].data === null && /unreadable/.test(dmg[0].note) && dmg[1].data === null && dmg[2].data.events.length === 1 &&
    (wr.ok ? wr.quota && stored.length <= 20000 : !wr.ok) && calls === 2 && !dead.ok,
  "damaged or foreign saved data is discarded with a note; a quota error retries once at half size; blocked storage reports failure");
}

/* ---------- D22-D24: run outcome across reset and rejected start ---------- */
{
  const r = new C.RunTracker(null); let t = 1000;
  r.onState("IDLE", { distance: 0 }, t += 200);
  r.onState("MISSION_START", {}, t += 200);
  r.onState("FOLLOW_LINE", { distance: 0 }, t += 200);
  r.onState("FOLLOW_LINE", { distance: 812 }, t += 200);
  r.onFault({ src: 6, sev: 2, code: 2 }, t += 100);
  const ended = r.onState("FAULT", { distance: 812, fault: 2 }, t += 100);
  r.onFault({ src: 2, sev: 2, code: 0x2011 }, t += 150);
  const o1 = C.describeOutcome(r.current);
  r.onState("IDLE", { distance: 812 }, t += 2000);  /* Reset */
  r.onState("IDLE", { distance: 812 }, t += 2000);  /* Start rejected: still idle */
  const kept = r.current && r.current.at;
  const nr = r.onState("MISSION_START", {}, t += 200);
  check("S1-D22", ended === "ended" && /Controller self-check/.test(o1.title) && o1.secondary.length === 1 && r.current === null &&
    r.previous && r.previous.at === kept && r.previous.distanceMm === 812 && nr === "new-run",
  "fault cause, time and distance survive Reset and a rejected Start; the later 0x2011 is secondary; a confirmed new run moves it to Previous");
  const r2 = new C.RunTracker(null); t = 0;
  r2.onState("FOLLOW_LINE", {}, t += 200);
  r2.onState("FAULT", { distance: 50 }, t += 200);
  r2.onFault({ src: 2, sev: 2, code: 0x2011 }, t += 50);
  const only2011 = C.describeOutcome(r2.current);
  r2.onFault({ src: 5, sev: 2, code: 0x5004 }, t += 100);
  r2.onFault({ src: 6, sev: 2, code: 3 }, t += 100);
  const both = C.describeOutcome(r2.current);
  check("S1-D23", /Motors safe-stopped/.test(only2011.title) && /No earlier cause/.test(only2011.detail) &&
    /No side clear/.test(both.title) && /Cause uncertain/.test(both.detail) && /Line not found/.test(both.detail),
  "0x2011 alone says no earlier cause was reported; a later real cause replaces it; two causes are both kept and labelled uncertain");
  const r3 = new C.RunTracker(null); t = 0;
  r3.onState("FOLLOW_LINE", {}, t += 200);
  r3.onState("MISSION_COMPLETE", { distance: 2345 }, t += 200);
  r3.onFault({ src: 2, sev: 2, code: 0x2011 }, t += 60);
  const comp = C.describeOutcome(r3.current);
  const r4 = new C.RunTracker(r3.snapshot());
  r4.onState("MISSION_COMPLETE", { distance: 2345 }, t += 99999); /* first frame after reload */
  check("S1-D24", comp.tone === "good" && /lifted/.test(comp.title) && r3.current.distanceMm === 2345 && comp.secondary.length === 1 &&
    r4.current.at === r3.current.at,
  "mission complete keeps its distance, a following 0x2011 does not turn it into a fault, and a reload does not restamp it");
}

/* ---------- D25-D27: commands, acks, availability ---------- */
{
  const tr = new C.CommandTracker(62); let t = 0;
  const a = tr.send("start", t); const b = tr.send("stop", t); const c = tr.send("reset", t);
  check("S1-D25", a.entry.seq === 62 && b.entry.seq === 63 && c.entry.seq === 1, "sequence wraps 63 -> 1 and never uses 0 (the robot's malformed-command echo)");
  let r1 = tr.onAck({ result: "queued", seq: 62 }, t += 120);
  tr.tick(t += 2500);
  const noack = tr.list.find(e => e.seq === 63).status;
  const r2 = tr.onAck({ result: "queued", seq: 63 }, t += 900);
  const r3 = tr.onAck({ result: "queued", seq: 62 }, t += 10);
  const r4 = tr.onAck({ result: "rejected", seq: 40 }, t += 10);
  tr.onCommandSeen({ cmd: "pause", seq: 41 }, t);
  const r5 = tr.onAck({ result: "rejected", seq: 41 }, t += 10);
  check("S1-D26", r1.kind === "matched" && noack === "noack" && r2.kind === "late" && /late ack/.test(C.describeCommand(r2.entry).text) &&
    r3.kind === "duplicate" && r4.kind === "unrelated" && r5.kind === "foreign" && /no ack after 2 s/.test(C.describeCommand(tr.list.find(e => e.seq === 1)).text),
  "ack matched by sequence; no ack after 2 s; late ack; repeated ack; ack for another client; unrelated ack");
  tr.onState("FOLLOW_LINE", false, t += 200);
  const conf = tr.list.find(e => e.seq === 62);
  const s2 = tr.send("start", t); /* seq 62 is still recent: skipped */
  const tr2 = new C.CommandTracker(5); t = 0; const p = tr2.send("pause", t);
  tr2.onAck({ result: "queued", seq: p.entry.seq }, t += 100); tr2.tick(t += 4100);
  check("S1-D27", conf.confirm === "confirmed" && s2.entry.seq !== 62 && s2.entry.seq !== 63 && tr2.list[0].confirm === "not-seen" &&
    /no matching state change/.test(C.describeCommand(tr2.list[0]).text),
  "an ack is not a state change: Start is confirmed only by a running state; a pending or recent sequence is not reused");
  const av = x => C.commandAvailability(Object.assign({ mqtt: true, fresh: true, state: "IDLE", paused: false, latch: "unavailable" }, x));
  const offline = av({ mqtt: false });
  const stale = av({ fresh: false, state: "FOLLOW_LINE" });
  const liftAction = C.latchAction({ imuLatch: { active: true, cause: "lift" }, at: 0 }, 0);
  const latched = av({ latch: "set", latchAction: liftAction });
  check("S1-D28", Object.values(offline).every(a => !a.ok) && stale.stop.ok && !stale.start.ok && !stale.pause.ok &&
    !av({ state: "FAULT" }).start.ok && av({ state: "FAULT" }).reset.ok && !av({ state: "FOLLOW_LINE" }).reset.ok &&
    av({ state: "FOLLOW_LINE" }).pause.ok && !av({ state: "LOST_LINE" }).pause.ok && av({ state: "FOLLOW_LINE", paused: true }).resume.ok &&
    av({ state: "MISSION_COMPLETE" }).start.ok && !latched.start.ok &&
    latched.start.why === "Robot was lifted: set it down flat, hands off 2 s, press Reset, then Start." && latched.reset.ok &&
    /cause unavailable/.test(av({ latch: "set" }).start.why),
  "buttons follow transport and firmware guards: Stop stays usable with stale data, Start needs fresh Idle/Complete, Reset is refused mid-run, a confirmed latch blocks Start");
  check("S1-D29", C.staleAfterMs("mqtt") === 1000 && C.staleAfterMs("usb") === 2000 &&
    C.latchStatus(null, 0) === "unavailable" && C.latchStatus({ imuLatch: null, at: 0 }, 1) === "unavailable" &&
    C.latchStatus({ imuLatch: { active: true, cause: null }, at: 0 }, 6500) === "set" &&
    C.latchStatus({ imuLatch: { active: true, cause: null }, at: 0 }, 6501) === "unavailable" &&
    C.latchStatus({ imuLatch: { active: false, cause: null }, at: 0 }, 100) === "clear",
  "MQTT data is stale after 1 s, USB after 2 s; latch is unavailable unless a fresh source reports it (never inferred)");
}

/* ---------- D30-D31: heartbeat and USB ---------- */
{
  const hb = C.parseHeartbeat("hb 12 s1 tq=500 tl=2 eq=9 ed=0/1/0/0 pa=498/9 rf=4/0/0 cb=490/0/1 uc=3 if=5/0 hr=0 lf=0");
  const old = C.parseHeartbeat("hb 7");
  const il = C.parseHeartbeat("hb 8 s1 tq=1 tl=0 il=1");
  check("S1-D30", hb.count === 12 && hb.counters.tl === 2 && hb.counters.ed.join() === "0,1,0,0" && hb.counters.rf.join() === "4,0,0" &&
    hb.imuLatch === null && old.count === 7 && old.counters === null && old.imuLatch === null &&
    il.imuLatch.active === true && il.imuLatch.cause === null && il.counters.tq === 1 && C.parseHeartbeat("garbage") === null,
  "heartbeat counters parse; the older \"hb N\" format still parses with counters unavailable; the il= field is optional");
  const u = C.fromUsbJson({ t: 1, st: "FOLLOW_LINE", v: [120, 116], odo: 300, line: -40, bc: 1, nav: 2, pitch: -150, hump: 0, maxhump: 1, us: 0, obs: 1 });
  check("S1-D31", u.seq === null && u.fault === null && u.paused === null && u.obstacleMm === null && u.obstacleNoEcho && u.pitchDeg === -1.5,
    "USB JSON has no sequence (cannot count toward MQTT loss); unreported fields stay empty, not zero");
}

/* ---------- D32: telemetry CSV, reopened and compared, incl. wrap ---------- */
{
  const hexes = enc([254, 255, 0, 1].map((s, i) => "T " + [s, 2, 120, 116, 1000 + i * 24, -40, 127, 0, 1, 2, 1, -150, 0, 1500, 0, 1, 0].join(" ")).join("\n") + "\n");
  const rows = hexes.map((h, i) => ({ ...C.decodeTelemetry(hexBytes(h.split(" ")[0])), t: 1764000000000 + i * 200 }));
  rows.push({ ...C.fromUsbJson({ st: "IDLE", v: [0, 0], odo: 0, line: 0, bc: 0, nav: 0, pitch: 0, hump: 0, maxhump: 0, us: 0, obs: 0 }), t: 1764000001000, source: 'usb,"quoted"' });
  const csv = C.telemetryCsv(rows);
  const back = C.parseCsv(csv);
  const head = back[0], col = n => head.indexOf(n);
  const ok = back.length === 6 && head[0] === "time_iso" && col("speed_l_mm_s") > 0 && col("distance_mm") > 0 &&
    [254, 255, 0, 1].every((s, i) => back[i + 1][col("seq")] === String(s) && back[i + 1][col("distance_mm")] === String(1000 + i * 24) &&
      back[i + 1][col("time_iso")] === new Date(rows[i].t).toISOString() && back[i + 1][col("obstacle_mm")] === "" && back[i + 1][col("pitch_deg")] === "-1.5") &&
    back[5][col("seq")] === "" && back[5][col("fault_code")] === "" && back[5][col("source")] === 'usb,"quoted"' && /\r\n$/.test(csv);
  check("S1-D32", ok, "CSV has stable headers with units; reopened rows equal the decoded frames across the 255 -> 0 wrap; unreported cells are empty; quotes and commas escape");
  check("S1-D33", C.logLine(Date.UTC(2026, 10, 24), "telemetry", "a\tb\nc") === "2026-11-24T00:00:00.000Z\ttelemetry\ta b c" &&
    /seq=255 st=FOLLOW_LINE/.test(C.tlmLogText(rows[1])), "saved-log lines are timestamped, one per line, with the telemetry sequence");
}

/* ---------- D35-D37: IMU stop latch in the counters line (s1_stats.h) ---------- */
{
  const terr = src("app_program/subsystem4/imu_terrain.h");
  const code = n => +new RegExp("#define\\s+IMU_SAFETY_" + n + "\\s+(\\d+)").exec(terr)[1];
  const [LIFT, OVER, NONE] = [code("LIFT"), code("OVERTURN"), code("NONE")];
  /* firmware text from s1_latch_resolve() + s1_latch_format() */
  const fw = enc(["L 0 1 " + NONE + " 0", "L 1 1 " + LIFT + " 1", "L 1 1 " + OVER + " 1", "L 1 1 " + NONE + " 1",
    "L 1 0 " + LIFT + " 1", "L 0 1 " + LIFT + " 1", "L 1 1 " + LIFT + " 0", "L 1 1 9 1"].join("\n") + "\n")
    .map(l => l.replace(/^\[|\]$/g, ""));
  const base = "hb 40 s1 tq=10 tl=0 eq=2 ed=0/0/0/0 pa=10/2 rf=0/0/0 cb=12/0/0 uc=0 if=0/0 hr=0 lf=0";
  const show = (t, at, now) => { const h = C.parseHeartbeat(base + t); h.at = at; return C.latchText(h, now); };
  const D = "\u2014";
  check("S1-D35", fw[0] === " il=0" && show(fw[0], 0, 0) === "IMU stop clear" &&
    fw[1] === " il=1 ic=lift" && show(fw[1], 0, 0) === "IMU stop latched " + D + " lift" &&
    fw[2] === " il=1 ic=overturn" && show(fw[2], 0, 0) === "IMU stop latched " + D + " overturn" &&
    [3, 4, 5, 7].every(i => fw[i] === " il=1 ic=unknown" && show(fw[i], 0, 0) === "IMU stop latched " + D + " cause unavailable") &&
    fw[6] === " il=0",
  "clear, lift, overturn and unknown cause: the firmware's own text (s1_latch_format) shows as the owner's wording; a cause read while the interlock changed is unknown",
    JSON.stringify(fw));
  /* one robot over time: lift, stale, set down + Reset, then an older build */
  const t0 = 1000;
  const liftHb = Object.assign(C.parseHeartbeat(base + fw[1]), { at: t0 });
  const clearHb = Object.assign(C.parseHeartbeat(base + fw[0]), { at: t0 + 10000 });
  const oldHb = Object.assign(C.parseHeartbeat(base), { at: t0 + 20000 });
  const bad = Object.assign(C.parseHeartbeat(base + " il=2"), { at: t0 });
  const tilt = Object.assign(C.parseHeartbeat(base + " il=1 ic=tilt"), { at: t0 });
  check("S1-D36", C.latchText(liftHb, t0 + 6500) === "IMU stop latched " + D + " lift" &&
    C.latchText(liftHb, t0 + 6501) === "Latch status unavailable" && C.latchCause(liftHb, t0 + 6501) === null &&
    C.latchText(clearHb, t0 + 10000) === "IMU stop clear" && C.latchCause(clearHb, t0 + 10000) === null &&
    oldHb.imuLatch === null && C.latchText(oldHb, t0 + 20000) === "Latch status unavailable" &&
    C.latchText(bad, t0) === "Latch status unavailable" &&
    C.latchText(tilt, t0) === "IMU stop latched " + D + " cause unavailable" && !/overturn/i.test(C.FAULTS[0x4003].name),
  "stale heartbeat, older firmware after a reconnect, and a malformed il= show \"Latch status unavailable\"; a fresh il=0 after Reset clears the old cause; a tilt is never shown as an overturn");
  const usb = C.parseCountersLine("s1 tq=7 tl=1 eq=3 ed=0/0/1/0 pa=6/3 rf=1/0/0 cb=9/0/0 uc=0 if=1/0 hr=0 lf=0" + fw[2]);
  const usbOld = C.parseCountersLine("s1 tq=7 tl=1 eq=3 ed=0/0/1/0 pa=6/3 rf=1/0/0 cb=9/0/0 uc=0 if=1/0 hr=0 lf=0");
  const ui = HTML.slice(HTML.indexOf("/* CORE-END */"));
  check("S1-D37", usb.counters.tq === 7 && usb.counters.ed.join() === "0,0,1,0" && usb.counters.lf === 0 && usb.counters.il === undefined &&
    usb.imuLatch.active && usb.imuLatch.cause === "overturn" && usbOld.counters.tl === 1 && usbOld.imuLatch === null &&
    C.parseHeartbeat(base + fw[1]).counters.cb.join() === "12,0,0" && /C\.parseCountersLine\(line\)/.test(ui) && /C\.latchText\(/.test(ui),
  "existing counters keep working with the latch field, on MQTT and on the USB console line; the page renders the latch from both");
}

/* ---------- D38-D56: barcode mapping, junction evidence, Run view (s1_cfg.h, vc_barcode_map.h) ---------- */
{
  const [facHex] = cfgEnc("F\n");
  const FAC = C.unpackMap(hexBytes(facHex));
  /* D38: fixed goldens, the same bytes as the firmware */
  const only = (pairs) => { const c = new Array(26).fill(0); for (const [l, v] of pairs) c[C.MAP_LETTERS.indexOf(l)] = v; return c; };
  const goldens = [
    [only([["A", 1]]), "40000000000000"],            /* A = left: byte 0 bits 7-6 */
    [only([["Z", 3]]), "00000000000030"],            /* Z = U-turn: byte 6 bits 5-4 */
    [only([["B", 1], ["Z", 3]]), "10000000000030"],  /* the end-to-end map */
    [only([["D", 2], ["E", 1], ["H", 3], ["I", 2], ["L", 1], ["M", 3], ["P", 2], ["Q", 1], ["T", 3], ["U", 1], ["X", 1], ["Y", 2]]),
      "024381c2434180"]                              /* letters across byte boundaries */
  ];
  check("S1-D38", facHex === "00010000000020" && toHex(C.packMap(FAC)) === facHex &&
    goldens.every(([c, h]) => toHex(C.packMap(c)) === h && cfgEnc("P " + c.join(" ") + "\n")[0] === h && C.sameMap(C.unpackMap(hexBytes(h)), c)),
    "factory map from the firmware is 00 01 00 00 00 00 20 and round-trips in JS; A, Z, B+Z and byte-boundary goldens match the firmware", facHex);

  /* D39: every letter x every action, and mixed maps, JS <-> C both ways */
  const lines = [], maps = [];
  for (let i = 0; i < 26; i++) for (let v = 0; v < 4; v++) { const c = new Array(26).fill((v + 1) & 3); c[i] = v; maps.push(c); }
  for (let k = 0; k < 40; k++) maps.push(Array.from({ length: 26 }, (_, i) => (i * 7 + k * 13 + (i * k >> 2)) & 3));
  for (const c of maps) lines.push("P " + c.join(" "));
  const packed = cfgEnc(lines.join("\n") + "\n");
  const unpacked = cfgEnc(packed.map(h => "U " + h).join("\n") + "\n");
  const bad39 = maps.findIndex((c, k) => toHex(C.packMap(c)) !== packed[k] || !C.sameMap(C.unpackMap(hexBytes(packed[k])), c) ||
    unpacked[k] !== c.join(" "));
  check("S1-D39", bad39 < 0 && maps.length === 144,
    "all 26 letters x 4 actions (104 maps) and 40 mixed maps pack and unpack the same in JS and in the firmware (through its NAV enum)",
    bad39 < 0 ? "" : "map " + bad39 + " " + packed[bad39]);

  /* D40: padding bits and wrong lengths are malformed on both sides */
  const pad = ["00000000000001", "00000000000008", "0000000000000f"];
  check("S1-D40", pad.every(h => C.unpackMap(hexBytes(h)) === null && cfgEnc("U " + h + "\n")[0] === "ERR") &&
    C.packMap(new Array(25).fill(0)) === null && C.packMap(new Array(26).fill(4)) === null && C.encodeCfgWrite(5, 0, [1, 2]) === null &&
    C.decodeCfgReply(hexBytes("c18201020000030100010000000021")).codes === null &&
    C.decodeCfgReply(hexBytes("c182010200000301000100000000")) === null && C.decodeCfgReply(hexBytes("c10201020000030100010000000020")) === null,
    "set padding bits (byte 6 bits 3-0) are refused in JS and C; short maps, codes above 3, short replies and non-reply opcodes are refused");

  /* D41: requests and replies are byte-identical with s1_cfg.c */
  const m = FAC.map((v, i) => i === 1 ? 1 : i === 25 ? 3 : v);
  const mHex = toHex(C.packMap(m));
  const [rC, wC, dW, yC, dBad, dF] = cfgEnc("R 65535\nW 4660 65534 " + mHex + "\nD " + toHex(C.encodeCfgWrite(4660, 65534, m)) +
    "\nY 2 4660 3 513 5 " + mHex + "\nD c102000000010000000000000000\nD " + toHex(C.encodeCfgFactory(9)) + "\n");
  const rep = C.decodeCfgReply(hexBytes(yC));
  check("S1-D41", toHex(C.encodeCfgRead(65535)) === rC && rC === "c101ffff" && toHex(C.encodeCfgWrite(4660, 65534, m)) === wC &&
    wC.length === 26 && dW === "2 4660 65534 " + mHex && dBad === "ERR" && dF === "3 9 0 00000000000000" &&
    rep.op === "write" && rep.id === 4660 && rep.result === "stale" && rep.gen === 513 && rep.saved && !rep.missionActive &&
    rep.factoryEqual && !rep.noStorage && C.sameMap(rep.codes, m),
    "READ, READ_FACTORY and WRITE bytes equal s1_cfg; the firmware decodes the page's WRITE and READ_FACTORY; a reply from s1_cfg_encode_reply decodes field by field; id 0 is refused",
    [rC, wC, dW, yC, dF].join(" "));

  /* D42: result codes, flags and topics are the firmware's */
  const hdr = src("app_program/vehicle_controller/vc_barcode_map.h");
  const enumNames = [...hdr.matchAll(/VC_BCMAP_(OK|UNCHANGED|REJ_ACTIVE|REJ_STALE|REJ_MALFORMED|REJ_STORAGE|BUSY)\s*=\s*(\d)/g)].map(x => +x[2]);
  const flagOk = Object.keys(C.MAP_FLAGS).every(k => new RegExp("VC_BCMAP_FLAG_" + k + "\\s+0x0*" + C.MAP_FLAGS[k].toString(16) + "u").test(hdr));
  const wire = [...hdr.matchAll(/#define VC_BCW_(\w+)\s+(\d)u/g)].map(x => x[1] + "=" + x[2]).join();
  const br = src("app_program/subsystem1/s1_bridge.h"), cfgh = src("app_program/subsystem1/s1_cfg.h");
  check("S1-D42", enumNames.join() === "0,1,2,3,4,5,6" && C.MAP_RESULTS.length === 7 && flagOk && wire === "STRAIGHT=0,LEFT=1,RIGHT=2,UTURN=3" &&
    C.MAP_ACTIONS.join("|") === "Go straight|Turn left|Turn right|U-turn (180°)" &&
    /S1_TOPIC_CFG\s+S1_TOPIC_ROOT "cfg"/.test(br) && /S1_TOPIC_CFG_REPLY\s+"cfgr"/.test(br) && /S1_TOPIC_JUNCTION\s+"jx"/.test(br) &&
    /S1_CFG_OP_FACTORY\s+0x03u/.test(cfgh) &&
    C.CFG_TOPIC === "robot/g10/cfg" && C.CFG_REPLY_TOPIC === "robot/g10/cfgr" && C.JX_TOPIC === "robot/g10/jx",
    "result codes, flags, wire action codes, READ_FACTORY and the cfg / cfgr / jx topics match the firmware; exactly the four actions, in wire order");

  /* helpers for the editor checks: replies come from the firmware's encoder */
  const reply = (op, id, result, gen, flags, codes) => C.decodeCfgReply(hexBytes(cfgEnc("Y " + op + " " + id + " " + result + " " + gen + " " + flags + " " + toHex(C.packMap(codes)) + "\n")[0]));
  const idle = { mqtt: true, missionActive: false };
  const withL = (base, l, v) => base.map((x, i) => i === C.MAP_LETTERS.indexOf(l) ? v : x);
  const ready = (ed, gen, codes, flags) => { const r = ed.read(0); ed.onReply(reply(1, r.id, 0, gen, flags == null ? 1 : flags, codes), 1);
    const f = ed.readFactory(1); if (f) ed.onReply(reply(3, f.id, 0, gen, 1, FAC), 2); return ed; };

  /* D43: loading never writes; edits need a read first; Restore only drafts, from the robot's factory map */
  {
    const ed = new C.MapEditor(100);
    const beforeRead = ed.edit(0, 1) === false && ed.restoreDefaults() === false && ed.save(0, idle) === null;
    const r = ed.read(0);
    const readOk = toHex(r.payload) === "c1010064" && ed.save(1, idle) === null;
    ed.onReply(reply(1, r.id, 0, 7, 1, withL(FAC, "A", 2)), 10);
    const noFactory = ed.restoreDefaults() === false;
    const f = ed.readFactory(11);
    ed.onReply(reply(3, f.id, 0, 7, 1, FAC), 12);
    const loaded = ed.robot.gen === 7 && !ed.dirty() && ed.factoryState === "ok" && C.sameMap(ed.factory, FAC);
    ed.restoreDefaults();
    const drafted = ed.pending === null && ed.dirty() && ed.dirtyLetters().join() === "A" && C.sameMap(ed.robot.codes, withL(FAC, "A", 2)) &&
      C.describeMap(ed, idle).word === "Unsaved changes" && ed.changes().map(c => c.letter + ": " + C.MAP_ACTIONS[c.from] + " → " + C.MAP_ACTIONS[c.to]).join() === "A: Turn right → Go straight";
    check("S1-D43", beforeRead && readOk && noFactory && loaded && drafted,
      "nothing can be edited or saved before the robot's map is read; reads send only READ / READ_FACTORY; Restore factory defaults uses the robot's factory map, changes only the draft and lists \"A: Turn right → Go straight\"");
  }

  /* D44: a save is verified only by its own reply with all 26 letters */
  {
    const ed = ready(new C.MapEditor(500), 0, FAC, 4);
    ed.edit(1, 1); ed.edit(25, 3);                  /* B = left, Z = U-turn */
    const w = ed.save(2, idle);
    const sent = w && toHex(w.payload).slice(0, 12) === "c102" + w.id.toString(16).padStart(4, "0") + "0000" && toHex(w.payload).slice(12) === "10010000000030" &&
      C.describeMap(ed, idle).word === "Saving…";
    const other = ed.onReply(reply(2, 9999, 0, 1, 1, ed.pending.codes), 3);         /* another client's id */
    const stillSaving = other.kind === "other" && ed.busy();
    const bad = ed.onReply(reply(2, w.id, 0, 1, 1, withL(ed.pending.codes, "Q", 2)), 4); /* OK but a different map */
    const mismatch = bad.kind === "matched" && ed.last.kind === "mismatch" && /Not verified/.test(C.describeMap(ed, idle).text) &&
      ed.draft && ed.dirty() && C.describeMap(ed, idle).word !== "Saved and verified" && !ed.conflict;
    const w2 = ed.save(5, idle);
    const ok = w2 && ed.onReply(reply(2, w2.id, 0, 2, 1, ed.pending.codes), 6).kind === "matched" && ed.last.kind === "verified" &&
      !ed.dirty() && C.describeMap(ed, idle).word === "Saved and verified" && ed.robot.gen === 2 && ed.robot.verifiedAt === 6;
    const w3 = (ed.edit(0, 1), ed.save(7, idle));
    const old = ed.onReply(reply(2, w2.id, 0, 2, 1, ed.pending.codes), 8);        /* the previous save's reply again */
    check("S1-D44", sent && stillSaving && mismatch && ok && old.kind === "other" && ed.busy(),
      "a WRITE carries the read generation and all letters; a foreign id or the previous save's reply never verifies; OK with a different map is Not verified and keeps the draft; only the matching id with all 26 letters equal is Saved and verified");
  }

  /* D45: no reply, late reply, a dropped connection during Save, older firmware */
  {
    const ed = ready(new C.MapEditor(900), 4, FAC);
    ed.edit(0, 1); const w = ed.save(10, idle);
    const early = ed.tick(10 + C.CFG_REPLY_MS - 1, { robotAlive: true }) === false;
    const late = ed.tick(10 + C.CFG_REPLY_MS, { robotAlive: true }) === true && C.describeMap(ed, idle).word === "Unsaved changes" &&
      /no reply to the save/.test(C.describeMap(ed, idle).text) && ed.dirty() && !ed.busy() && !ed.unsupported;
    const lateOk = ed.onReply(reply(2, w.id, 0, 5, 1, withL(FAC, "A", 1)), 9000).kind === "late" && ed.last.kind === "verified";
    ed.edit(2, 3); ed.save(9100, idle); ed.connectionLost(9200);
    const off = C.describeMap(ed, { mqtt: false, usb: true });
    const old = new C.MapEditor(4000); old.read(0); old.readFactory(0);
    old.tick(C.CFG_REPLY_MS, { robotAlive: true });
    const gone = new C.MapEditor(4100); gone.read(0); gone.tick(C.CFG_REPLY_MS, { robotAlive: false });
    check("S1-D45", early && late && lateOk && off.word === "Offline" && /MQTT dropped during the save/.test(off.text) && /USB is monitor-only/.test(off.text) &&
      /unsaved changes are kept/.test(off.text) && ed.saveBlock({ mqtt: false }) === "Saving needs MQTT." &&
      C.describeMap(old, idle).word === "Unsupported firmware" && /does not support/.test(old.saveBlock(idle)) &&
      C.describeMap(gone, idle).word === "Offline" && !gone.unsupported,
      "no reply in " + C.CFG_REPLY_MS / 1000 + " s keeps the draft; a late reply to the same id still verifies; MQTT dropping during Save says Offline and keeps the draft; " +
      "fresh telemetry but no map reply says Unsupported firmware; no telemetry and no reply says Offline");
  }

  /* D46: reconnect with a changed robot map keeps the draft and flags it */
  {
    const ed = ready(new C.MapEditor(1200), 3, FAC);
    ed.edit(0, 1);                                   /* draft: A = left */
    ed.connectionLost(2);
    const r = ed.read(3);                            /* what connecting sends */
    ed.onReply(reply(1, r.id, 0, 4, 1, withL(FAC, "Z", 0)), 4);  /* someone set Z straight meanwhile */
    const kept = ed.draft && ed.draft[0] === 1 && ed.draft[25] === 2 && ed.conflict && ed.conflict.letters.join() === "Z" &&
      /changed meanwhile \(save 3 → 4, letters Z\)/.test(C.describeMap(ed, idle).text);
    const refused = ed.save(5, idle) === null;
    const forced = ed.save(6, idle, true);
    check("S1-D46", kept && refused && forced && forced.payload[4] === 0 && forced.payload[5] === 4 && ed.dirtyLetters().join() === "A,Z",
      "a read after reconnect never replaces the draft; letters the robot changed meanwhile are flagged; Save needs an explicit confirm and then uses the robot's new generation");
  }

  /* D47: a draft survives a reload as a draft, never as a save */
  {
    const ed = ready(new C.MapEditor(1500), 2, FAC);
    ed.edit(1, 1);
    const json = ed.exportDraft(77);
    const ed2 = new C.MapEditor(1600);
    const imported = ed2.importDraft(json) && ed2.pending === null && ed2.restoredDraftAt === 77 && ed2.dirty();
    const r = ed2.read(80); ed2.onReply(reply(1, r.id, 0, 2, 1, FAC), 81);
    const same = !ed2.conflict && ed2.dirtyLetters().join() === "B" && /Draft restored from this browser/.test(C.describeMap(ed2, idle).text);
    const ed3 = new C.MapEditor(1700); ed3.importDraft(json);
    const r3 = ed3.read(90); ed3.onReply(reply(1, r3.id, 0, 3, 1, withL(FAC, "C", 2)), 91);
    const flagged = ed3.conflict && ed3.conflict.letters.join() === "C" && ed3.draft[1] === 1;
    check("S1-D47", imported && same && flagged && !new C.MapEditor(1).importDraft("{bad") && !new C.MapEditor(1).importDraft('{"v":1,"draft":[1,2]}') &&
      ed.exportDraft(1) && (ed.discard(), ed.exportDraft(1) === null),
      "a reloaded draft comes back unsaved (no request sent); if the robot changed meanwhile its letters are flagged; damaged drafts are ignored; Discard removes it");
  }

  /* D48: Save refused during a mission (UI and robot); the factory map from older firmware */
  {
    const ed = ready(new C.MapEditor(1800), 1, FAC, 3);  /* robot says mission active */
    ed.edit(7, 2);
    const blockUnknown = /mission is running/.test(ed.saveBlock({ mqtt: true, missionActive: null }) || "");
    const blockRun = /mission is running/.test(ed.saveBlock({ mqtt: true, missionActive: true }) || "");
    const w = ed.save(2, idle);                      /* telemetry says idle now */
    ed.onReply(reply(2, w.id, 2, 1, 3, FAC), 3);     /* the robot refuses anyway */
    const rej = ed.last.kind === "rejected" && /a mission is running/.test(C.describeMap(ed, idle).text) && ed.dirty();
    const old = new C.MapEditor(1900); const rr = old.read(0); old.onReply(reply(1, rr.id, 0, 0, 4, FAC), 1);
    const f = old.readFactory(1);
    old.onReply(C.decodeCfgReply(hexBytes(cfgEnc("Y 0 " + f.id + " 4 0 4 00000000000000\n")[0])), 2); /* firmware before READ_FACTORY */
    check("S1-D48", blockUnknown && blockRun && rej && old.factoryState === "unsupported" && old.restoreDefaults() === false,
      "Save is refused while a mission is running (telemetry, or the robot's flag when telemetry is stale) and the robot's own refusal keeps the draft; " +
      "firmware without READ_FACTORY (MALFORMED reply) disables Restore factory defaults instead of guessing a table");
  }

  /* D49: robot/g10/jx bytes and the USB line */
  {
    const [x1, x2, x3] = cfgEnc("X 1 66 1 3\nX 2 0 0 3\nX 3 90 3 513\n");
    const n1 = C.decodeJunction(hexBytes(x1)), n2 = C.decodeJunction(hexBytes(x2)), n3 = C.decodeJunction(hexBytes(x3));
    check("S1-D49", n1.kind === "assigned" && n1.letter === "B" && n1.code === 1 && n1.gen === 3 && n2.kind === "completed" && n2.letter === null &&
      n3.kind === "dropped" && n3.letter === "Z" && n3.code === 3 && n3.gen === 513 &&
      C.decodeJunction(hexBytes("d10142010003ff")) === null && C.decodeJunction(hexBytes("d10442010003")) === null && C.decodeJunction(hexBytes("d10161010003")) === null &&
      JSON.stringify(C.parseJunctionLine("vc: jx completed - 0")) === '{"kind":"completed","letter":null,"code":0,"gen":null}' &&
      C.parseJunctionLine("vc: jx assigned B 1").letter === "B" && C.parseJunctionLine("vc: jx assigned b 1") === null &&
      /"vc: jx %s %c %u\\n"/.test(src("app_program/vehicle_controller/task_vehicle_ctrl.c")),
      "junction notes from s1_cfg_encode_junction decode (assigned / completed / dropped, letter, action, map save); bad length, kind or letter is refused; the USB line format matches the firmware's printf");
  }

  /* D50: Read B -> Left turn pending -> Left turn completed, only on evidence */
  {
    const L = [], codes = withL(FAC, "B", 1);
    C.newRead(L, "B", "mqtt", 1000, codes);
    const s0 = C.describeRead(L[0]).map(x => x.text).join(" > ");
    C.applyJunction(L, { kind: "assigned", letter: "B", code: 1, gen: 1 }, 1010);
    const s1 = C.describeRead(L[0]).map(x => x.text).join(" > ");
    codes[1] = 3; /* the map is edited later: history keeps what the robot said */
    C.applyJunction(L, { kind: "completed", letter: "B", code: 1, gen: 1 }, 4000);
    const s2 = C.describeRead(L[0]).map(x => x.text).join(" > ");
    C.applyJunction(L, { kind: "completed", letter: null, code: 0, gen: 1 }, 6000);
    const j = C.describeRead(L[1]).map(x => x.text).join(" > ");
    C.newRead(L, "Z", "mqtt", 7000, null); C.applyJunction(L, { kind: "assigned", letter: "Z", code: 3, gen: 1 }, 7010);
    C.applyJunction(L, { kind: "dropped", letter: "Z", code: 3, gen: 1 }, 7500);
    C.newRead(L, "T", "mqtt", 8000, null); C.applyJunction(L, { kind: "assigned", letter: "T", code: 0, gen: 1 }, 8010);
    C.applyRunEnd(L, "FAULT", 9000);
    const z = C.describeRead(L[2]).map(x => x.text).join(" > "), t = C.describeRead(L[3]).map(x => x.text).join(" > ");
    /* jx before the barcode message, and a duplicate barcode message: one record */
    const L2 = []; C.applyJunction(L2, { kind: "assigned", letter: "A", code: 1, gen: 2 }, 100); C.newRead(L2, "A", "mqtt", 150, null);
    /* firmware without jx: pending from telemetry, completion never claimed */
    const L3 = []; C.newRead(L3, "B", "usb", 100, null); C.applyTelemetry(L3, { nav: 1, state: "FOLLOW_LINE" }, 300); C.applyTelemetry(L3, { nav: 1, state: "EXECUTE_ACTION" }, 2000);
    const t3 = C.describeRead(L3[0]).map(x => x.text).join(" > ");
    check("S1-D50", s0 === "Read B" && s1 === "Read B > Left turn pending" && s2 === "Read B > Left turn pending > Left turn completed" && L[0].mapCode === 1 &&
      j === "Junction without a barcode > Went straight" && z === "Read Z > U-turn pending > U-turn cancelled (obstacle, timeout or lift)" &&
      t === "Read T > Go straight pending > Not executed: run ended (Fault)" && L2.length === 1 && L2[0].assigned.by === "robot" &&
      t3 === "Read B > Left turn pending (from telemetry) > Junction manoeuvre seen (completion not reported by this firmware)",
      "a read shows only \"Read B\" until the robot assigns; pending and completed appear on the robot's notes; a later map edit does not rewrite history; " +
      "no-barcode junction, cancelled and run-ended reads are shown as such; older firmware never shows a completed turn");
  }

  /* D51: repeated reports grouped; the stop cause is not inferred from a secondary report */
  {
    const r = new C.RunTracker(null);
    r.onState("FOLLOW_LINE", { distance: 0 }, 0);
    r.onFault({ src: 6, sev: 2, code: 1 }, 1000);
    r.onState("FAULT", { distance: 2650, fault: 1 }, 1050);
    for (const t of [1400, 1800, 2200]) r.onFault({ src: 2, sev: 2, code: 0x2011 }, t);
    const o = C.describeOutcome(r.current);
    const g = C.groupFaults([{ src: 2, sev: 2, code: 0x2011, t: 1, run: 1 }, { src: 2, sev: 2, code: 0x2011, t: 2, run: 1 }, { src: 2, sev: 2, code: 0x2011, t: 3, run: 2 }]);
    const only = new C.RunTracker(null); only.onState("FOLLOW_LINE", {}, 0); only.onFault({ src: 2, sev: 2, code: 0x2011 }, 100); only.onState("FAULT", { distance: 5 }, 120);
    const o2 = C.describeOutcome(only.current);
    check("S1-D51", /Operator stop/.test(o.title) && o.groups.length === 1 && o.groups[0].times.length >= 1 && o.secondary.every(s => !/Operator/.test(s)) &&
      g.length === 2 && g[0].times.length === 2 && /No earlier cause was reported/.test(o2.detail),
      "Operator stop stays the cause and following 0x2011 reports are grouped with their times (×N); repeats are grouped per run; a lone 0x2011 says no earlier cause was reported");
  }

  /* D52: run history keeps earlier runs, previous self-check failure included */
  {
    const r = new C.RunTracker(null);
    r.onState("MISSION_START", {}, 0); r.onFault({ src: 6, sev: 2, code: 2 }, 500); r.onState("FAULT", { distance: 0 }, 520);
    r.onState("IDLE", {}, 900); r.onState("MISSION_START", {}, 1000); r.onState("FOLLOW_LINE", {}, 1200);
    r.onFault({ src: 6, sev: 2, code: 1 }, 2000); r.onState("FAULT", { distance: 4075 }, 2020);
    const back = new C.RunTracker(JSON.parse(JSON.stringify(r.snapshot())));
    check("S1-D52", /Operator stop/.test(C.describeOutcome(r.current).title) && r.history.length === 1 && /self-check/.test(C.describeOutcome(r.history[0]).title) &&
      back.history.length === 1 && new C.RunTracker({ current: null, previous: r.history[0] }).history.length === 1,
      "the newest run is the summary; the earlier self-check failure moves to History and survives a reload (also from an older saved snapshot)");
  }

  /* D53: three separate signals and grouped reasons */
  {
    const tr = new C.CommandTracker(5); tr.send("stop", 0);
    const waiting = C.commandSignal(tr.list, 100);
    tr.tick(3000);
    const none = C.commandSignal([], 0), noack = C.commandSignal(tr.list, 3000);
    const av = C.commandAvailability({ mqtt: true, fresh: false, state: null, paused: null, restored: false, latch: "unavailable", latchAction: null, pendingStart: false, pendingResume: false });
    const reasons = C.groupReasons(av);
    const ui = HTML.slice(HTML.indexOf("/* CORE-END */"));
    check("S1-D53", none.text === "Commands: none sent" && /waiting for ack/.test(waiting.text) && /no ack/.test(noack.text) && noack.tone === "attn" &&
      reasons.length === 1 && reasons[0].text === "Start, Pause, Resume: No fresh robot state." &&
      C.fmtAge(3800) === "3.8 s ago" && C.fmtAge(187300) === "3 min ago" &&
      /"Robot data stale · last update " \+ C\.fmtAge\(age\)/.test(ui) && /"Last known: " \+/.test(ui) &&
      /"Broker: connected"/.test(ui) && /id="pAck"/.test(HTML) && /id="pAge"/.test(HTML) && /id="pMqtt"/.test(HTML),
      "broker, robot data and command acks are three pills; a reason shared by several commands is said once; stale data says \"Robot data stale · last update …\" with \"Last known\"");
  }

  /* D54: views, Stop everywhere, empty states, S5 slot, page wiring */
  {
    const ui = HTML.slice(HTML.indexOf("/* CORE-END */"));
    const top = HTML.slice(HTML.indexOf('<div class="top" id="top">'), HTML.indexOf("<main>"));
    const pubs = [...ui.matchAll(/\.publish\(([^,]+),/g)].map(x => x[1].trim());
    const onConnect = (ui.match(/mqttClient\.on\("connect", \(\) => \{[\s\S]*?\n    \}\);/) || [""])[0];
    const sels = (ui.match(/sel\.onchange = \(\) => \{[^\n]*\};/) || [""])[0];
    check("S1-D54", ["tab_run", "tab_map", "tab_diag"].every(id => new RegExp('role="tab" id="' + id + '"').test(top)) &&
      /data-cmd="stop"/.test(top) && /id="fab"[^>]*>[\s\S]*?data-cmd="stop"/.test(HTML) &&
      /id="view_run"[\s\S]*id="connPanel"[\s\S]*id="mqttUrl"[\s\S]*id="view_map"/.test(HTML) &&
      ["c_speed_e", "c_line_e", "c_us_e", "c_pitch_e"].every(id => HTML.includes('id="' + id + '">No samples in the last 60 s.')) &&
      ["ob_front", "ob_width", "ob_length", "ob_id", "ob_detail", "ob_history"].every(id => HTML.includes('id="' + id + '"')) && /function renderObstacleProfile\(\)/.test(ui) &&
      /No barcode reads yet\./.test(ui) && !/BARCODE_RULES/.test(HTML) &&
      pubs.join() === '"robot/g10/cmd",C.CFG_TOPIC' && /publish\(C\.CFG_TOPIC, req\.payload, \{ qos: 0, retain: false \}\)/.test(ui) &&
      /mapRefresh\(null\)/.test(onConnect) && !/save\(/.test(onConnect) && !/save\(|publish|mapSend/.test(sels) && /saveDraft\(\)/.test(sels) &&
      /lab\.htmlFor = "map_" \+ l/.test(ui) && /"● Edited · robot has: "/.test(ui) && /\.maprow\.edited \{ border: 2px dashed/.test(HTML) &&
      ["Read from robot", "Save to robot", "Discard edits", "Restore factory defaults"].every(t => HTML.includes(">" + t + "</button>")) && /id="mapEditedOnly"/.test(HTML),
      "Run / Barcode mapping / Diagnostics tabs with Stop in the top bar and the phone bar; connection details inside Run; charts say \"No samples\" when empty; S5 obstacle ids exist; " +
      "no letter table in the page; only commands and cfg requests are published (cfg QoS 0, retain false); connecting only reads; a dropdown only edits the draft; edited rows say Edited with a dashed border");
  }
}

/* ---------- D60-D66: obstacle size report (robot/g10/obstacle/profile) ---------- */
{
  const out = execFileSync(OBST_ENCODER).toString().trim().split("\n");
  const frames = out.filter(l => l[0] === "F"), names = out.filter(l => l[0] === "N");
  const keys = ["profileId", "rev", "stage", "widthStatus", "lengthStatus", "lengthReason", "frontMm", "widthMm", "lengthMm", "t0Ms", "bypassSide"];
  const bad = [];
  for (const l of frames) {
    const [, hex, ...f] = l.split(" "); const d = C.decodeObstacleProfile(hexBytes(hex));
    if (!d) { bad.push(hex + " rejected"); continue; }
    keys.forEach((k, i) => { if (d[k] !== Number(f[i])) bad.push(hex + " " + k + ": js " + d[k] + " c " + f[i]); });
  }
  check("S1-D60", frames.length >= 200 && bad.length === 0,
    "every frame the firmware's s5_obst_report.c makes (begin, front, length, final; id wrap 65535 -> 1; t0 above 2^31; clamped sizes) decodes in JS to the same fields", bad.slice(0, 3).join("; "));
  const tbl = { stage: C.OBST_STAGE, width: C.OBST_WIDTH, length: C.OBST_LENGTH, reason: C.OBST_REASON };
  const nameBad = names.map(l => /^N (\w+) (\d+) (.*)$/.exec(l)).filter(m => !m || tbl[m[1]][+m[2]] !== m[3]).map(m => m && m[0]);
  const hdr = src("app_program/subsystem5/s5_obst_report.h"), def = n => +(new RegExp("#define " + n + "\\s+(\\d+)").exec(hdr) || [])[1];
  check("S1-D60", names.length === 4 + 5 + 5 + 9 && nameBad.length === 0 && tbl.reason.length === 10 &&
    C.OBST_BYTES === def("S5_OR_BYTES") && C.decodeObstacleProfile(hexBytes("01" + "0001" + "00".repeat(17))) !== null &&
    /#define S5_OR_TOPIC\s+"obstacle\/profile"/.test(hdr) && C.OBST_TOPIC === /S1_TOPIC_ROOT\s+"([^"]+)"/.exec(src("app_program/subsystem1/s1_bridge.h"))[1] + "obstacle/profile",
    "the stage, width, length and reason names, the frame size and the topic are the firmware's", nameBad.slice(0, 3).join("; "));

  /* the golden frame of testing/subsystem5 S5-R01 */
  const golden = "01012c020201020000d200b400f00001e2400100";
  const g = C.decodeObstacleProfile(hexBytes(golden));
  const rej = [];
  const mut = (i, v) => { const b = hexBytes(golden); b[i] = v; return C.decodeObstacleProfile(b); };
  [[0, 2], [4, 4], [5, 5], [6, 5], [7, 10], [18, 3], [19, 1], [8, 0x07], [10, 0x0F], [12, 0x07]].forEach(([i, v]) => { if (mut(i, v) !== null) rej.push("byte " + i + "=" + v); });
  { const b = hexBytes(golden); b[1] = 0; b[2] = 0; if (C.decodeObstacleProfile(b) !== null) rej.push("id 0"); }
  if (C.decodeObstacleProfile(hexBytes(golden).slice(0, 19)) !== null) rej.push("19 bytes");
  if (C.decodeObstacleProfile(hexBytes(golden + "00")) !== null) rej.push("21 bytes");
  check("S1-D61", g && g.profileId === 300 && g.rev === 2 && g.stage === 2 && g.frontMm === 210 && g.widthMm === 180 && g.lengthMm === 240 &&
    g.t0Ms === 123456 && g.bypassSide === 1 && rej.length === 0,
    "the S5 golden frame decodes as documented; wrong length, version, out-of-range stage/status/reason/side/sizes, id 0 and a non-zero reserved byte are rejected", rej.join(", "));

  /* records as the robot sends them, built from bytes */
  const mk = o => {
    const r = Object.assign({ id: 1, rev: 0, stage: 0, ws: 0, ls: 0, lr: 0, front: 0, width: 0, len: 0, t0: 1000, side: 1 }, o);
    const b = new Uint8Array(20);
    b[0] = 1; b[1] = r.id >> 8; b[2] = r.id & 255; b[3] = r.rev; b[4] = r.stage; b[5] = r.ws; b[6] = r.ls; b[7] = r.lr;
    b[8] = r.front >> 8; b[9] = r.front & 255; b[10] = r.width >> 8; b[11] = r.width & 255; b[12] = r.len >> 8; b[13] = r.len & 255;
    b[14] = (r.t0 >>> 24) & 255; b[15] = (r.t0 >>> 16) & 255; b[16] = (r.t0 >>> 8) & 255; b[17] = r.t0 & 255; b[18] = r.side;
    return C.decodeObstacleProfile(b);
  };
  const W = 173, L = 238; /* arbitrary: the page must show what the frame says */
  const front = mk({ rev: 1, stage: 1, ws: 1, ls: 1, front: 205, width: W });
  const meas = mk({ rev: 2, stage: 2, ws: 1, ls: 2, front: 205, width: W, len: L });
  const vf = C.obstacleView(front), vm = C.obstacleView(meas);
  const vu = C.obstacleView(mk({ rev: 3, stage: 3, ws: 1, ls: 4, lr: 3, front: 205, width: W }));
  const vp = C.obstacleView(mk({ rev: 2, stage: 2, ws: 1, ls: 3, front: 205, width: W, len: 84 }));
  const vl = C.obstacleView(mk({ rev: 1, stage: 1, ws: 2, ls: 1, front: 205, width: 250 }));
  const vo = C.obstacleView(mk({ rev: 1, stage: 1, ws: 3, ls: 1, front: 205 }));
  const v0 = C.obstacleView(null);
  check("S1-D62", vf.summary === "Width: " + W + " mm · Length: measuring" && vf.front === "205 mm" && vf.width === W + " mm" &&
    vf.length === "Measuring" && !vf.resolved && vf.title === "Obstacle #1",
    "front profile ready: width shown at once, front distance separate, length Measuring");
  check("S1-D63", vm.summary === "Width: " + W + " mm · Length: " + L + " mm" && vm.resolved &&
    vu.length === "Unavailable (no side echo during the pass)" && vu.summary === "Width: " + W + " mm · Length: unavailable (no side echo during the pass)" &&
    vp.length === "≥ 84 mm (partial)" && vl.width === "≥ 250 mm (lower bound)" && vo.width === "Not measured (one beam)" &&
    v0.summary === "Profile not reported" && v0.front === "–",
    "measured length in mm; unavailable with its reason; partial and lower bound marked as bounds; one beam claims no width; nothing received says Profile not reported");

  const tr = new C.ObstacleTracker();
  const seq = [];
  seq.push(tr.accept(mk({ id: 7, t0: 5000 }), 1, "mqtt"));
  seq.push(tr.accept(mk({ id: 7, t0: 5000, rev: 1, stage: 1, ws: 1, ls: 1, front: 200, width: 150 }), 2, "mqtt"));
  seq.push(tr.accept(mk({ id: 7, t0: 5000, rev: 1, stage: 1, ws: 1, ls: 1, front: 200, width: 150 }), 3, "usb"));
  seq.push(tr.accept(mk({ id: 7, t0: 5000, rev: 3, stage: 3, ws: 1, ls: 2, front: 200, width: 150, len: 190 }), 4, "mqtt"));
  seq.push(tr.accept(mk({ id: 7, t0: 5000, rev: 2, stage: 2, ws: 1, ls: 2, front: 200, width: 150, len: 190 }), 5, "mqtt"));
  const first = tr.current().rec;
  seq.push(tr.accept(mk({ id: 8, t0: 9000 }), 6, "mqtt"));
  const second0 = C.obstacleView(tr.current().rec);
  seq.push(tr.accept(mk({ id: 8, t0: 9000, rev: 1, stage: 1, ws: 1, ls: 1, front: 210, width: 120 }), 7, "mqtt"));
  const second1 = C.obstacleView(tr.current().rec);
  seq.push(tr.accept(mk({ id: 7, t0: 5000, rev: 3, stage: 3, ws: 1, ls: 2, front: 200, width: 150, len: 190 }), 8, "mqtt"));
  seq.push(tr.accept(mk({ id: 8, t0: 70, rev: 0 }), 9, "mqtt")); /* restart: id 8 again, new t0 */
  seq.push(tr.accept(mk({ id: 1, t0: 70, rev: 255 }), 10, "mqtt"));
  seq.push(tr.accept(mk({ id: 1, t0: 70, rev: 0 }), 11, "mqtt")); /* rev wraps 255 -> 0 */
  check("S1-D64", seq.join() === "new,update,dup,update,stale,new,update,stale,new,new,update" && first.lengthMm === 190 &&
    second0.length === "Measuring" && second0.width === "Measuring" && second1.summary === "Width: 120 mm · Length: measuring" &&
    tr.list.length === 4 && tr.dups === 1 && tr.stale === 2,
    "two consecutive obstacles: #2 opens a new entry with no length, #1's length is never shown for #2; repeats, older revs and a late copy of #1 are ignored; a robot restart (same id, new t0) and the rev wrap are handled");

  /* USB: the firmware's own format string, filled in as tm_printf would */
  const av = src("app_program/subsystem5/task_obstacle_avoid.c");
  const fmt = /"s5: obstacle #%d rev %d %s: front %d mm, width %d mm \(%s\), "\s*"length %d mm \(%s, %s\) \| frame %s\\n"/.test(av);
  const line = "s5: obstacle #300 rev 2 length: front 210 mm, width 180 mm (measured), length 240 mm (measured, -) | frame " + golden;
  const u = C.parseObstacleUsbLine(line);
  check("S1-D65", fmt && u && JSON.stringify(u) === JSON.stringify(g) && C.parseObstacleUsbLine(line.slice(0, -2)) === null &&
    C.parseObstacleUsbLine("s5: obstacle ahead") === null,
    "the USB line (format as in task_obstacle_avoid.c) carries the same frame and decodes like MQTT; a cut line is rejected");

  const ui = HTML.slice(HTML.indexOf("/* CORE-END */"));
  const rend = (/function renderObstacleProfile\(\) \{[\s\S]*?\n  \}/.exec(ui) || [""])[0];
  const v0n = C.obstacleView(null), vfs = C.obstacleView(front);
  check("S1-D66", ["ob_front", "ob_width", "ob_length", "ob_id", "ob_detail", "ob_history"].every(id => HTML.includes('id="' + id + '"')) &&
    /<dt>Front<\/dt><dd id="ob_front">[\s\S]*<dt>Width<\/dt><dd id="ob_width">[\s\S]*<dt>Length<\/dt><dd id="ob_length">/.test(HTML) &&
    /topic === C\.OBST_PROFILE_TOPIC\) \{\s*onObstacle\(C\.decodeObstacleProfile\(bytes\), "mqtt"\)/.test(ui) &&
    /onObstacle\(C\.parseObstacleUsbLine\(line\), "usb"\)/.test(ui) && /"obstacle", C\.obstacleLogText\(u\.rec\)/.test(ui) &&
    rend && !/innerHTML/.test(rend) && /txt\("ob_width", v\.width\)/.test(rend) && /txt\("ob_length", v\.length\)/.test(rend) &&
    /const obst = C\.createObstacleTracker\(\)/.test(ui) && C.OBST_PROFILE_TOPIC === C.OBST_TOPIC &&
    v0n.status === "Profile not reported" && vfs.status === "measuring length",
    "the Run obstacle tile shows front distance, width and length as labelled values (text only) and says Profile not reported until a frame arrives; MQTT and USB both feed it; every update goes to the saved run log");
}

/* ---------- D34: page rules ---------- */
{
  const ui = HTML.slice(HTML.indexOf("/* CORE-END */"));
  const inner = (ui.match(/\.innerHTML\s*=/g) || []).length;
  const scripts = [...HTML.matchAll(/<script[^>]*src=/g)].length;
  check("S1-D34", inner === 0 && scripts === 0 && /mqtt@5\.10\.1/.test(HTML) && !/localStorage\.clear\(/.test(HTML),
    "received and restored values are rendered as text (no innerHTML); no extra script tags; MQTT.js 5.10.1 kept; Clear history never wipes all storage");
}

console.log("[TEST] dashboard SUMMARY " + passed + " passed, " + failed + " failed");
process.exit(failed ? 1 : 0);
