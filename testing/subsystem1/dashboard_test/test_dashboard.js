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
  /* barcode rule table mirrors vc_barcode_config.h */
  const table = {};
  for (const m of src("app_program/vehicle_controller/vc_barcode_config.h").replace(/\/\*[\s\S]*?\*\//g, "").matchAll(/X\(([A-Z]),\s*NAV_(\w+)\)/g))
    if (m[2] !== "NONE") table[m[1]] = { LEFT: "Left", RIGHT: "Right", STRAIGHT: "Straight", U_TURN: "U-turn" }[m[2]];
  check("S1-D11", JSON.stringify(table) === JSON.stringify(C.BARCODE_RULES) && C.configuredAction("S").startsWith("Go straight (factory") &&
    C.configuredAction("H").startsWith("Turn left (factory") && C.configuredAction("HELLO").startsWith("None") &&
    C.configuredAction("S", C.factoryMap().map((v, i) => i === 18 ? 3 : v)) === "U-turn (180°)",
    "the dashboard's factory letter actions equal vc_barcode_config.h; a letter without a rule (S) is straight as on the robot; " +
    "multi-letter text has no action; the robot's read map wins once known", JSON.stringify(table));
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

/* ---------- D38-D47: barcode junction actions (s1_cfg.h, vc_barcode_map.h) ---------- */
{
  /* D38: factory map and fixed goldens, the same bytes as the firmware */
  const [fac] = cfgEnc("F\n");
  const only = (pairs) => { const c = new Array(26).fill(0); for (const [l, v] of pairs) c[C.MAP_LETTERS.indexOf(l)] = v; return c; };
  const goldens = [
    [only([["A", 1]]), "40000000000000"],            /* A = left: byte 0 bits 7-6 */
    [only([["Z", 3]]), "00000000000030"],            /* Z = U-turn: byte 6 bits 5-4 */
    [only([["B", 1], ["Z", 3]]), "10000000000030"],  /* the end-to-end map */
    [only([["D", 2], ["E", 1], ["H", 3], ["I", 2], ["L", 1], ["M", 3], ["P", 2], ["Q", 1], ["T", 3], ["U", 1], ["X", 1], ["Y", 2]]),
      "024381c2434180"]
  ];
  const boundary = goldens[3][0];
  const bHex = toHex(C.packMap(boundary));
  const [bC] = cfgEnc("P " + boundary.join(" ") + "\n");
  check("S1-D38", fac === "00010000000020" && toHex(C.packMap(C.factoryMap())) === fac &&
    goldens.every(([c, h]) => toHex(C.packMap(c)) === h && cfgEnc("P " + c.join(" ") + "\n")[0] === h) &&
    bHex === bC && C.sameMap(C.unpackMap(hexBytes(bC)), boundary),
    "factory map packs to 00 01 00 00 00 00 20 in JS and C (H left, Z right, rest straight); A, Z, B+Z goldens and letters across byte boundaries match the firmware",
    fac + " " + bHex + " " + bC);

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
  const m = C.factoryMap().map((v, i) => i === 1 ? 1 : i === 25 ? 3 : v);
  const mHex = toHex(C.packMap(m));
  const [rC, wC, dW, yC, dBad] = cfgEnc("R 65535\nW 4660 65534 " + mHex + "\nD " + toHex(C.encodeCfgWrite(4660, 65534, m)) +
    "\nY 2 4660 3 513 5 " + mHex + "\nD c102000000010000000000000000\n");
  const rep = C.decodeCfgReply(hexBytes(yC));
  check("S1-D41", toHex(C.encodeCfgRead(65535)) === rC && rC === "c101ffff" && toHex(C.encodeCfgWrite(4660, 65534, m)) === wC &&
    wC.length === 26 && dW === "2 4660 65534 " + mHex && dBad === "ERR" &&
    rep.op === "write" && rep.id === 4660 && rep.result === "stale" && rep.gen === 513 && rep.saved && !rep.missionActive &&
    rep.factoryEqual && !rep.noStorage && C.sameMap(rep.codes, m),
    "READ and WRITE bytes equal s1_cfg_encode_*; the firmware decodes the page's WRITE to the same id, gen and map; a reply from s1_cfg_encode_reply decodes field by field; id 0 is refused",
    [rC, wC, dW, yC].join(" "));

  /* D42: result codes, flags and topics are the firmware's */
  const hdr = src("app_program/vehicle_controller/vc_barcode_map.h");
  const enumNames = [...hdr.matchAll(/VC_BCMAP_(OK|UNCHANGED|REJ_ACTIVE|REJ_STALE|REJ_MALFORMED|REJ_STORAGE|BUSY)\s*=\s*(\d)/g)].map(x => +x[2]);
  const flagOk = Object.entries({ SAVED: "SAVED", MISSION_ACTIVE: "MISSION_ACTIVE", FACTORY_EQUAL: "FACTORY_EQUAL", NO_STORAGE: "NO_STORAGE" })
    .every(([k, n]) => new RegExp("VC_BCMAP_FLAG_" + n + "\\s+0x0*" + C.MAP_FLAGS[k].toString(16) + "u").test(hdr));
  const wire = [...hdr.matchAll(/#define VC_BCW_(\w+)\s+(\d)u/g)].map(x => x[1] + "=" + x[2]).join();
  const br = src("app_program/subsystem1/s1_bridge.h");
  check("S1-D42", enumNames.join() === "0,1,2,3,4,5,6" && C.MAP_RESULTS.length === 7 && flagOk && wire === "STRAIGHT=0,LEFT=1,RIGHT=2,UTURN=3" &&
    C.MAP_ACTIONS.join("|") === "Go straight|Turn left|Turn right|U-turn (180°)" &&
    /S1_TOPIC_CFG\s+S1_TOPIC_ROOT "cfg"/.test(br) && /S1_TOPIC_CFG_REPLY\s+"cfgr"/.test(br) &&
    C.CFG_TOPIC === "robot/g10/cfg" && C.CFG_REPLY_TOPIC === "robot/g10/cfgr",
    "result codes, flags, wire action codes and the cfg / cfgr topics match the firmware; exactly the four actions, in wire order");

  /* helpers for the editor checks */
  const reply = (op, id, result, gen, flags, codes) => C.decodeCfgReply(hexBytes(cfgEnc("Y " + op + " " + id + " " + result + " " + gen + " " + flags + " " + toHex(C.packMap(codes)) + "\n")[0]));
  const idle = { mqtt: true, missionActive: false };
  const F = C.factoryMap();
  const withL = (base, l, v) => base.map((x, i) => i === C.MAP_LETTERS.indexOf(l) ? v : x);

  /* D43: loading never writes; edits need a read first; Restore only drafts */
  {
    const ed = new C.MapEditor(100);
    const beforeRead = ed.edit(0, 1) === false && ed.restoreDefaults() === false && ed.save(0, idle) === null;
    const r = ed.read(0);
    const readOk = toHex(r.payload) === "c1010064" && ed.save(1, idle) === null;
    ed.onReply(reply(1, r.id, 0, 7, 1, withL(F, "A", 2)), 10);
    const loaded = ed.robot.gen === 7 && !ed.dirty() && C.describeMap(ed, idle).text.startsWith("Robot map: saved map, save 7");
    ed.restoreDefaults();
    const drafted = ed.pending === null && ed.dirty() && ed.dirtyLetters().join() === "A" && C.sameMap(ed.robot.codes, withL(F, "A", 2)) &&
      C.describeMap(ed, idle).text.startsWith("Unsaved changes: 1 letter (A)");
    check("S1-D43", beforeRead && readOk && loaded && drafted,
      "nothing can be edited or saved before the robot's map is read; a read sends only a 4-byte READ; Restore defaults changes the draft only (no request) and shows Unsaved changes");
  }

  /* D44: a save is verified only by its own reply with all 26 letters */
  {
    const ed = new C.MapEditor(500);
    const r = ed.read(0); ed.onReply(reply(1, r.id, 0, 0, 4, F), 1);
    ed.edit(1, 1); ed.edit(25, 3);                  /* B = left, Z = U-turn */
    const w = ed.save(2, idle);
    const sent = w && toHex(w.payload) === "c10201f50000" + "10010000000030" && C.describeMap(ed, idle).text === "Saving…";
    const old = ed.onReply(reply(1, r.id, 0, 1, 1, ed.pending.codes), 3);           /* the earlier READ's id */
    const other = ed.onReply(reply(2, 9999, 0, 1, 1, ed.pending.codes), 3);         /* another client's id */
    const stillSaving = old.kind === "other" && other.kind === "other" && ed.busy() && ed.last.kind === "read";
    const bad = ed.onReply(reply(2, w.id, 0, 1, 1, withL(ed.pending.codes, "Q", 2)), 4); /* OK but a different map */
    const mismatch = bad.kind === "matched" && ed.last.kind === "mismatch" && /Not verified/.test(C.describeMap(ed, idle).text) &&
      ed.draft && ed.dirty() && !/Saved and verified/.test(C.describeMap(ed, idle).text);
    /* the draft (Q straight) is still unsaved against the robot's Q = right */
    const w2 = ed.save(5, idle);
    const ok = w2 && ed.onReply(reply(2, w2.id, 0, 2, 1, ed.pending.codes), 6).kind === "matched" && ed.last.kind === "verified" &&
      !ed.dirty() && C.describeMap(ed, idle).text.startsWith("Saved and verified") && ed.robot.gen === 2;
    check("S1-D44", sent && stillSaving && mismatch && ok,
      "a WRITE carries the read generation and all letters; a reply with an older or foreign id never verifies; OK with a different map is Not verified and keeps the draft; only the matching id with all 26 letters equal is Saved and verified");
  }

  /* D45: no reply, late reply and a dropped connection */
  {
    const ed = new C.MapEditor(900);
    const r = ed.read(0); ed.onReply(reply(1, r.id, 0, 4, 1, F), 1);
    ed.edit(0, 1); const w = ed.save(10, idle);
    const early = ed.tick(10 + C.CFG_REPLY_MS - 1) === false;
    const late = ed.tick(10 + C.CFG_REPLY_MS) === true && /^Robot unavailable: no reply to the save/.test(C.describeMap(ed, idle).text.split(". ").slice(1).join(". ") || ed.last.text) &&
      ed.dirty() && !ed.busy();
    const lateOk = ed.onReply(reply(2, w.id, 0, 5, 1, withL(F, "A", 1)), 9000).kind === "late" && ed.last.kind === "verified";
    ed.edit(2, 3); ed.save(9100, idle); ed.connectionLost(9200);
    const off = C.describeMap(ed, { mqtt: false, usb: true });
    check("S1-D45", early && late && lateOk && off.text.startsWith("Robot unavailable") && /USB is monitor-only/.test(off.text) &&
      /unsaved changes are kept/.test(off.text) && ed.saveBlock({ mqtt: false }) === "Saving needs MQTT.",
      "no reply within " + C.CFG_REPLY_MS / 1000 + " s says Robot unavailable and keeps the draft; a late reply to the same id still verifies; MQTT down says Robot unavailable, USB is monitor-only, draft kept");
  }

  /* D46: reconnect with a changed robot map keeps the draft and flags it */
  {
    const ed = new C.MapEditor(1200);
    let r = ed.read(0); ed.onReply(reply(1, r.id, 0, 3, 1, F), 1);
    ed.edit(0, 1);                                   /* draft: A = left */
    ed.connectionLost(2);
    r = ed.read(3);                                  /* what connect() sends */
    ed.onReply(reply(1, r.id, 0, 4, 1, withL(F, "Z", 0)), 4);  /* someone set Z straight meanwhile */
    const kept = ed.draft && ed.draft[0] === 1 && ed.draft[25] === 2 && ed.conflict && ed.conflict.letters.join() === "Z" &&
      /robot's map changed meanwhile \(save 3 → 4, letters Z\)/.test(C.describeMap(ed, idle).text);
    const refused = ed.save(5, idle) === null;
    const forced = ed.save(6, idle, true);
    check("S1-D46", kept && refused && forced && forced.payload[4] === 0 && forced.payload[5] === 4 && ed.dirtyLetters().join() === "A,Z",
      "a read after reconnect never replaces the draft; letters the robot changed meanwhile are flagged; Save needs an explicit confirm and then uses the robot's new generation");
  }

  /* D47: mission active, rejections, and the page's wiring */
  {
    const ed = new C.MapEditor(1500);
    const r = ed.read(0); ed.onReply(reply(1, r.id, 0, 1, 3, F), 1);  /* robot says mission active */
    ed.edit(7, 2);
    const blockUnknown = /mission is running/.test(ed.saveBlock({ mqtt: true, missionActive: null }) || "");
    const blockRun = /mission is running/.test(ed.saveBlock({ mqtt: true, missionActive: true }) || "");
    const w = ed.save(2, idle);                      /* telemetry says idle now */
    ed.onReply(reply(2, w.id, 2, 1, 3, F), 3);       /* the robot refuses anyway */
    const rej = ed.last.kind === "rejected" && /a mission is running/.test(C.describeMap(ed, idle).text) && ed.dirty();
    const ui = HTML.slice(HTML.indexOf("/* CORE-END */"));
    const pubs = [...ui.matchAll(/\.publish\(([^,]+),/g)].map(x => x[1].trim());
    const onConnect = (ui.match(/mqttClient\.on\("connect", \(\) => \{[\s\S]*?\n    \}\);/) || [""])[0];
    check("S1-D47", blockUnknown && blockRun && rej &&
      pubs.join() === '"robot/g10/cmd",C.CFG_TOPIC' && /publish\(C\.CFG_TOPIC, req\.payload, \{ qos: 0, retain: false \}\)/.test(ui) &&
      /mapEd\.read\(now\(\)\)/.test(onConnect) && !/save\(/.test(onConnect) &&
      /lab\.htmlFor = "map_" \+ l/.test(ui) && /C\.MAP_ACTIONS\.forEach/.test(ui) && /id="h_map">Barcode junction actions</.test(HTML) &&
      /"● Edited · robot has: "/.test(ui) && /\.maprow\.edited \{ border: 2px dashed/.test(HTML) &&
      ["Read from robot", "Save to robot", "Restore defaults"].every(t => HTML.includes(">" + t + "</button>")),
      "Save is refused while a mission is running (telemetry, or the robot's flag when telemetry is stale) and the robot's own refusal keeps the draft; " +
      "the page publishes only commands and cfg requests, cfg with QoS 0 and retain false; connecting only reads; 26 labelled selects with the four actions; edited rows say Edited and have a dashed border (not colour alone)");
  }
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
