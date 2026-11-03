// ブラウザ画面の確認(M3)。mobile_sim と mapper_gmapping を起動しておき、Playwright で開いて操作する
// 使い方: node scripts/ui_test.js [http://localhost:8001/] [スクリーンショットの保存先]
const { chromium } = require("playwright");

const url = process.argv[2] || "http://localhost:8001/";
const shot = process.argv[3] || "ui_test.png";

function check(cond, msg) {
  if (!cond) { console.error("FAILED: " + msg); process.exit(1); }
  console.log("ok -", msg);
}

(async () => {
  const browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1100, height: 820 } });
  const errors = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto(url);
  await page.waitForSelector("#connection.ok", { timeout: 10000 });
  check(true, "connected to juiz");
  check(await page.isVisible("#sim_connection.ok"), "finds mobile_sim through the subsystem");
  check(await page.isVisible("#drive"), "shows the drive pad");
  await page.waitForFunction(() => !document.getElementById("s_pose").textContent.startsWith("-"));

  const pose = async (id) => {
    const t = await page.textContent(id);
    const m = t.match(/x (-?[\d.]+), y (-?[\d.]+), θ (-?[\d.]+)/);
    return m ? { x: Number(m[1]), y: Number(m[2]), th: Number(m[3]) } : null;
  };
  const p0 = await pose("#s_pose");
  check(p0 !== null, `mapper has an estimated pose (${JSON.stringify(p0)})`);

  // W で 0.5 m 以上進めると地図が更新される
  await page.keyboard.down("w");
  await page.waitForTimeout(2500);
  await page.keyboard.up("w");
  await page.keyboard.down("a");
  await page.waitForTimeout(1200);
  await page.keyboard.up("a");
  await page.keyboard.down("w");
  await page.waitForTimeout(1500);
  await page.keyboard.up("w");
  await page.waitForTimeout(3500); // map_update_interval(2 秒)より長く待つ
  const p1 = await pose("#s_pose");
  const truth = await pose("#s_truth");
  check(Math.hypot(p1.x - p0.x, p1.y - p0.y) > 0.5, `driving moves the estimated pose (${JSON.stringify(p1)})`);
  check(Math.hypot(p1.x - truth.x, p1.y - truth.y) < 0.2, `estimate is near the truth (${JSON.stringify(truth)})`);
  const mapText = await page.textContent("#s_map");
  check(!mapText.includes("まだ"), `map is shown (${mapText})`);

  // 保存ボタン
  await page.fill("#save_path", "/tmp/mapper_gmapping_ui_test_map");
  await page.click("#save");
  await page.waitForFunction(() => document.getElementById("message").textContent !== "");
  const msg = await page.textContent("#message");
  check(msg.includes("/tmp/mapper_gmapping_ui_test_map.yaml"), `save button writes the map (${msg})`);

  // 地図を描いたキャンバスに、不明(灰)以外の画素がある
  const painted = await page.evaluate(() => {
    const c = document.getElementById("map");
    const d = c.getContext("2d").getImageData(0, 0, c.width, c.height).data;
    let white = 0, black = 0;
    for (let i = 0; i < d.length; i += 4) {
      if (d[i] > 240 && d[i + 1] > 240 && d[i + 2] > 240) white++;
      if (d[i] < 30 && d[i + 1] < 30 && d[i + 2] < 30) black++;
    }
    return { white, black };
  });
  check(painted.white > 5000 && painted.black > 100, `canvas shows free space and walls (${JSON.stringify(painted)})`);

  await page.screenshot({ path: shot });

  // 捨てて最初から
  await page.click("#reset");
  await page.waitForFunction(() => document.getElementById("message").textContent.includes("捨てました"));
  await page.waitForTimeout(600);
  const updates = await page.textContent("#s_updates");
  check(/^[01] 回/.test(updates), `reset clears the updates (${updates})`);
  check((await page.textContent("#s_error")) === "なし", "reset clears the last error");

  check(errors.length === 0, `no page errors ${JSON.stringify(errors)}`);
  await browser.close();
})().catch((e) => { console.error(e); process.exit(1); });
