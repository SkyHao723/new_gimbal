// 串口控制台页面解析逻辑的自测（无需浏览器、无需硬件）
// 用法：node tools/host_console/test_parser.js
// 做法：给 index.html 里的 <script> 套一个极简 DOM 桩，喂入真实的串口回包文本，
//       检查状态卡片是否被正确解析；顺带检查 $("id") 引用的元素在 HTML 里都存在。
"use strict";

const fs = require("fs");
const vm = require("vm");
const path = require("path");

const here = __dirname;
const htmlPath = path.join(here, "index.html");
const html = fs.readFileSync(htmlPath, "utf8");
const scriptMatch = html.match(/<script>([\s\S]*?)<\/script>/);
if (!scriptMatch) {
  console.error("FAIL   index.html 里找不到 <script> 块");
  process.exit(1);
}
const code = scriptMatch[1];

/* ---------- 采集到的真实回包（上位机板 host_bringup 固件） ---------- */
const CAPTURE_BOARD = [
  "b",
  "SYSCLK=170000000  CAN PSR=0x0000070F ECR=0x00005900 rx=0 busoff=0 txdrop=0",
  "UART1 dropped=0  UART2(ESP) dropped=0 rx=356 bad=0/0  IMU addr=0x68",
  "commands: USB=2  ESP=89  reply route=1 (1=USB 2=ESP 3=both)",
  "ESP input: only lines starting with '@' are executed (e.g. @m, @c1 T0.5); muted=0 muted_drops=0",
  "> ",
].join("\r\n");

const CAPTURE_IMU = [
  "m",
  "IMU acc[g] -0.000 0.028 1.021   gyro[dps] -0.3 0.1 0.4   temp 30.5 C",
  "> ",
].join("\r\n");

/* "n" 巡检的每块板回复（带 CAN ID 前缀） */
const CAPTURE_SCAN = [
  "n",
  "",
  "[CAN] broadcast '?', collecting replies for 2 s (PSR=0x0000076A ECR=0x0007FC00)",
  "  [0x281] EN=OFF angle=-5.2220  vel=0.0000  tgt=-5.2215  Vlim=4.00 Vbus=24.0 dir=-1 pp=FIXED zero=1.2270",
  "CAN txdrop=0 nFAULT=1",
  "I pk=0.14A allowed=8.00V req=4.00V th=0.00 smp=1210816 trips=0 ok=1",
  "> ",
].join("\r\n");

/* 单板查询（无前缀），使能且 nFAULT 拉低 */
const CAPTURE_NODE2 = [
  "c2 ?",
  "TX node2 -> '?'",
  "EN=ON angle=1.0000  vel=0.0000  tgt=1.5000  Vlim=2.50 Vbus=24.0 dir=-1 pp=FIXED zero=0.1220",
  "CAN txdrop=0 nFAULT=0",
  "> ",
].join("\r\n");

const CAPTURE_R = ["c1 R", "sensor angle=3.1416  vel=-0.0000  mech=3.1416", "> "].join("\r\n");

/* ---------- 极简 DOM 桩 ---------- */
const els = new Map();
const roles = new Map();      // "node1|angle" -> 元素
const termKids = [];

function makeEl(id) {
  return {
    id,
    textContent: "",
    className: "",
    value: "",
    checked: false,
    disabled: false,
    scrollTop: 0,
    scrollHeight: 0,
    childElementCount: 0,
    onclick: null,
    classList: { toggle() { }, add() { }, remove() { }, contains() { return false; } },
    appendChild(child) { this.childElementCount++; termKids.push(child); },
    removeChild() { this.childElementCount--; },
    addEventListener() { },
    querySelector(sel) {
      const m = sel.match(/data-role="([^"]+)"/);
      if (!m) return null;
      const key = id + "|" + m[1];
      if (!roles.has(key)) roles.set(key, { textContent: "", className: "" });
      return roles.get(key);
    },
  };
}
const getEl = (id) => {
  if (!els.has(id)) els.set(id, makeEl(id));
  return els.get(id);
};

const SENT = [];
let fakePort = null;

globalThis.document = {
  getElementById: getEl,
  createElement: () => ({ className: "", textContent: "", click() { } }),
  querySelectorAll: () => [],
};
globalThis.window = { addEventListener() { } };
Object.defineProperty(globalThis, "navigator", {
  value: { serial: { requestPort: async () => fakePort } },
  configurable: true,
});
fakePort = {
  open: async () => { },
  close: async () => { },
  readable: null,
  getInfo: () => ({ usbVendorId: 0x1a86, usbProductId: 0x55d3 }),
  writable: {
    getWriter: () => ({
      write: async (buf) => { SENT.push(new TextDecoder().decode(buf)); },
      releaseLock() { },
    }),
  },
};

vm.runInThisContext(code, { filename: "host_console_script.js" });

/* ---------- 断言 ---------- */
const results = [];
function check(name, ok, extra) {
  results.push(ok);
  console.log((ok ? "PASS  " : "FAIL  ") + name + (extra ? "   [" + extra + "]" : ""));
}

(async () => {
  await connect();
  await send("b");
  feed(CAPTURE_BOARD);
  feed(CAPTURE_IMU);
  feed(CAPTURE_SCAN);
  await send("c2 ?");
  feed(CAPTURE_NODE2);

  check("板级 SYSCLK", getEl("vClock").textContent === "170000000", getEl("vClock").textContent);
  check("板级 CAN PSR", getEl("vPsr").textContent === "0x0000070F", getEl("vPsr").textContent);
  check("板级 UART 丢字节", getEl("vDrop").textContent === "0", getEl("vDrop").textContent);
  check("板级 IMU 地址", getEl("vImu").textContent === "0x68", getEl("vImu").textContent);
  check("IMU 加速度", getEl("vAcc").textContent === "-0.000 0.028 1.021", getEl("vAcc").textContent);
  check("IMU 角速度", getEl("vGyro").textContent === "-0.3 0.1 0.4", getEl("vGyro").textContent);
  check("IMU 温度", getEl("vTemp").textContent === "30.5 °C", getEl("vTemp").textContent);

  check("1 号板 失能", roles.get("node1|en").textContent === "失能", roles.get("node1|en").textContent);
  check("1 号板 nFAULT 正常", roles.get("node1|fault").textContent.includes("✓"), roles.get("node1|fault").textContent);
  check("1 号板 角度", roles.get("node1|angle").textContent === "角度 -5.2220", roles.get("node1|angle").textContent);
  check("1 号板 目标", roles.get("node1|target").textContent === "目标 -5.2215", roles.get("node1|target").textContent);
  check("1 号板 误差(°)", roles.get("node1|err").textContent === "误差 0.029°", roles.get("node1|err").textContent);
  check("1 号板 电压上限", roles.get("node1|vlim").textContent === "上限 4.00 V", roles.get("node1|vlim").textContent);
  check("1 号板 电流", roles.get("node1|pk").textContent === "电流 0.14 A", roles.get("node1|pk").textContent);

  check("2 号板 使能", roles.get("node2|en").textContent === "使能", roles.get("node2|en").textContent);
  check("2 号板 nFAULT 故障", roles.get("node2|fault").textContent.includes("✗"), roles.get("node2|fault").textContent);
  check("2 号板 误差(°)", roles.get("node2|err").textContent === "误差 28.648°", roles.get("node2|err").textContent);

  const rx = termKids.filter((c) => c.className === "rx").map((c) => c.textContent);
  check("回显行被吞掉（不当作回复）", !rx.includes("b") && !rx.includes("c2 ?"), rx.slice(0, 3).join(" | "));

  /* 相对移动：读当前角后发目标 */
  getEl("nodeSel").value = "1";
  getEl("relVal").value = "0.5";
  SENT.length = 0;
  const p = relativeMove(1);
  setTimeout(() => feed(CAPTURE_R), 30);
  await p;
  await new Promise((r) => setTimeout(r, 250));
  check("相对移动 先读角度", SENT.some((x) => x.trim() === "c1 R"), SENT.join(" "));
  check("相对移动 发目标", SENT.some((x) => x.trim() === "c1 T3.6416"), SENT.join(" "));

  /* 静态检查：脚本里 $("id") 引用的元素必须在 HTML 中存在 */
  const ids = new Set([...html.matchAll(/\bid="([^"]+)"/g)].map((m) => m[1]));
  const used = new Set([...code.matchAll(/\$\("([^"]+)"\)/g)].map((m) => m[1]));
  const missing = [...used].filter((x) => !ids.has(x));
  const dupes = [...html.matchAll(/\bid="([^"]+)"/g)].map((m) => m[1]).filter((v, i, a) => a.indexOf(v) !== i);
  check("HTML id 引用完整", missing.length === 0, missing.join(", ") || "ok");
  check("HTML id 无重复", dupes.length === 0, [...new Set(dupes)].join(", ") || "ok");

  const failed = results.filter((r) => !r).length;
  console.log("\n" + (failed === 0 ? "ALL PAGE CHECKS PASSED" : failed + " CHECK(S) FAILED"));
  process.exit(failed === 0 ? 0 : 1);
})();
