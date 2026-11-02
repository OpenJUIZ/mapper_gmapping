"""mobile_sim と mapper_gmapping を IPC でつないで走らせ、SLAM の結果を確かめる(M2 の結合テスト)

mobile_sim をトピック cmd_vel で部屋の中を一周させ、mapper_gmapping が受けたトピック odom・scan から作った
地図と姿勢を、mobile_sim の真の姿勢・真の地図と比べる。

使い方:
  scripts/dev_plugin.sh build
  python3 scripts/integration_test.py --mobile-sim <mobile_sim のリポジトリ> [--juiz <juiz のパス>]
    mobile_sim は app/plugins/<target>/ にプラグインを置いておく(mobile_sim の scripts/dev_plugin.sh)
  python3 scripts/integration_test.py --mobile-sim-juizapp mobile_sim-0.2.0.juizapp
    mobile_sim を juiz run で動かす(ポートは 8000)

IPC の名前 mobile_sim と mapper_gmapping を使うので、同じ機械でそれらを動かしていないこと。
"""
import argparse
import io
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# 部屋を一周して出発点に戻る道のり(mobile_sim の既定の地図。x = 1.5 の仕切り壁の出入口は y = -1〜1)
ROUTE = [
    (0.5, 0.0), (0.5, 3.8), (-4.0, 3.8), (-4.0, -3.8), (0.5, -3.8), (0.5, 0.0),
    (3.0, 0.0), (3.0, -2.5), (4.2, -2.5), (4.2, 1.2), (3.0, 1.2), (3.0, 0.0),
    (0.5, 0.0), (-2.0, 0.0),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--juiz", default="juiz")
    ap.add_argument("--mobile-sim", type=Path, help="mobile_sim のリポジトリ(app/app.conf を使う)")
    ap.add_argument("--mobile-sim-juizapp", type=Path, help="mobile_sim の .juizapp(juiz run で動かす)")
    ap.add_argument("--sim-port", type=int, default=18000)
    ap.add_argument("--port", type=int, default=18001)
    ap.add_argument("--speed", type=float, default=0.5)
    ap.add_argument("--out", type=Path, help="地図(YAML + PNG)を書くディレクトリ")
    args = ap.parse_args()
    if not args.mobile_sim and not args.mobile_sim_juizapp:
        sys.exit("give --mobile-sim or --mobile-sim-juizapp")

    procs = []
    tmps = []
    try:
        if args.mobile_sim_juizapp:
            sim = "http://localhost:8000"
            work = tempfile.mkdtemp()
            env = dict(os.environ, JUIZ_CACHE_DIR=os.path.join(work, "cache"))
            procs.append(start([args.juiz, "run", str(args.mobile_sim_juizapp.resolve())], work, env))
        else:
            sim = f"http://localhost:{args.sim_port}"
            app = args.mobile_sim.resolve() / "app"
            conf = (app / "app.conf").read_text().replace('"port": 8000', f'"port": {args.sim_port}')
            tmp = app / ".mapper_integration_test.conf"
            tmp.write_text(conf)
            tmps.append(tmp)
            procs.append(start([args.juiz, "-f", tmp.name, "-d", "--ipc", "mobile_sim"], app))
        wait_ready(sim, procs[-1])

        base = f"http://localhost:{args.port}"
        conf = (ROOT / "app" / "app.conf").read_text().replace('"port": 8001', f'"port": {args.port}')
        tmp = ROOT / "app" / ".integration_test.conf"
        tmp.write_text(conf)
        tmps.append(tmp)
        procs.append(start([args.juiz, "-f", tmp.name, "-d"], ROOT / "app"))
        wait_ready(base, procs[-1])

        run(sim, base, args)
        print("OK")
    finally:
        for p in reversed(procs):
            p.terminate()
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
        for t in tmps:
            t.unlink(missing_ok=True)


def start(cmd, cwd, env=None):
    log = tempfile.TemporaryFile()
    p = subprocess.Popen(cmd, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT)
    p.log = log
    return p


def wait_ready(base, proc):
    for _ in range(150):
        if proc.poll() is not None:
            proc.log.seek(0)
            sys.exit(f"juiz exited ({base}):\n" + proc.log.read().decode(errors="replace")[-4000:])
        try:
            with urllib.request.urlopen(f"{base}/api/container/list", timeout=1) as r:
                if json.load(r):
                    return
        except OSError:
            pass
        time.sleep(0.2)
    sys.exit(f"juiz did not become ready ({base})")


def call(base, name, type_name, container, args=None, raw=False):
    ident = f"{base}/container_process/{name}::{type_name}:{container}"
    url = f"{base}/api/container_process/call?identifier=" + urllib.parse.quote(ident, safe="")
    req = urllib.request.Request(
        url, data=json.dumps(args or {}).encode(), method="PATCH", headers={"Content-Type": "application/json"}
    )
    with urllib.request.urlopen(req, timeout=30) as r:
        body = r.read()
        return body if raw else json.loads(body)


def sim_call(sim, name, kind, args=None, raw=False):
    return call(sim, name, f"mobile_sim_{kind}", "sim0", args, raw)


def mapper_call(base, name, kind, args=None, raw=False):
    return call(base, name, f"gmapping_{kind}", "mapper0", args, raw)


def push_cmd_vel(sim, vx, omega):
    url = f"{sim}/api/topic/push?topic_name=cmd_vel"
    body = {"input": {"linear": {"x": vx, "y": 0.0}, "angular": omega}}
    req = urllib.request.Request(
        url, data=json.dumps(body).encode(), method="PATCH", headers={"Content-Type": "application/json"}
    )
    with urllib.request.urlopen(req, timeout=5) as r:
        r.read()


def check(cond, msg):
    if not cond:
        sys.exit("FAILED: " + msg)
    print("ok -", msg)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def truth_pose(sim):
    p = sim_call(sim, "get_pose0", "get_pose")["pose"]
    return p["position"]["x"], p["position"]["y"], p["theta"]


def drive(sim, route, speed):
    """真の姿勢を見ながら、cmd_vel で道のりの点を順にたどる"""
    t0 = time.time()
    for gx, gy in route:
        deadline = time.time() + 60
        while True:
            if time.time() > deadline:
                sys.exit(f"FAILED: could not reach {(gx, gy)} (now {truth_pose(sim)})")
            x, y, th = truth_pose(sim)
            dist = math.hypot(gx - x, gy - y)
            if dist < 0.1:
                break
            err = wrap(math.atan2(gy - y, gx - x) - th)
            omega = max(-1.2, min(1.2, 2.0 * err))
            vx = 0.0 if abs(err) > 0.5 else min(speed, dist) * math.cos(err)
            push_cmd_vel(sim, vx, omega)
            time.sleep(0.05)
    push_cmd_vel(sim, 0.0, 0.0)
    return time.time() - t0


def read_png(path):
    from PIL import Image

    return Image.open(path).convert("L")


def run(sim, base, args):
    st = mapper_call(base, "get_state0", "get_state")
    check(st["scans"] > 0, f"mapper receives scans over the topic ({st['scans']} scans)")
    check(st["initialized"], "mapper initializes on the first scan")

    elapsed = drive(sim, ROUTE, args.speed)
    time.sleep(1.0)  # 最後のスキャンが届くのを待つ
    truth = truth_pose(sim)
    odom = sim_call(sim, "get_odom_pose0", "get_odom_pose")["pose"]
    odom = (odom["position"]["x"], odom["position"]["y"], odom["theta"])
    st = mapper_call(base, "get_state0", "get_state")
    slam = (st["pose"]["x"], st["pose"]["y"], st["pose"]["theta"])
    odom_err = math.hypot(odom[0] - truth[0], odom[1] - truth[1])
    slam_err = math.hypot(slam[0] - truth[0], slam[1] - truth[1])
    print(f"   drove {elapsed:.0f} s, {st['scans']} scans, {st['updates']} updates, dropped {st['dropped']}")
    print(f"   truth {fmt(truth)}  odom {fmt(odom)} (err {odom_err:.3f} m)  slam {fmt(slam)} (err {slam_err:.3f} m)")
    check(st["last_error"] == "", f"mapper has no error ({st['last_error']!r})")
    check(st["updates"] >= 50, f"SLAM updated {st['updates']} times")
    # 実時間で動かすので結果は毎回少し違う(手元では 0.01〜0.05 m、0.01〜0.06 rad)。余裕を持たせる
    check(slam_err < 0.2, f"SLAM pose is within 0.2 m of the truth ({slam_err:.3f} m)")
    check(abs(wrap(slam[2] - truth[2])) < 0.12, f"SLAM heading is within 0.12 rad ({wrap(slam[2] - truth[2]):.3f})")
    check(abs(slam[2]) <= math.pi + 1e-9, "SLAM heading is normalized")
    check(slam_err < odom_err, "SLAM pose is better than odometry")

    info = mapper_call(base, "get_map_info0", "get_map_info")
    png = mapper_call(base, "get_map0", "get_map", raw=True)
    check(png[:4] == b"\x89PNG", "get_map returns PNG")
    from PIL import Image

    img = Image.open(io.BytesIO(png)).convert("L")
    check(img.size == (info["width"], info["height"]), f"map size {img.size} matches map info")

    # mapper はカレントディレクトリが app/ なので、絶対パスで渡す
    out = (args.out or Path(tempfile.mkdtemp())).resolve()
    out.mkdir(parents=True, exist_ok=True)
    # 書けないパスはエラーになるが、mapper は落ちない
    try:
        mapper_call(base, "save_map0", "save_map", {"path": str(out / "no" / "such" / "dir" / "map")})
        bad = None
    except urllib.error.HTTPError as e:
        bad = e.code
    check(bad is not None, f"save_map to a missing directory is an error (HTTP {bad})")
    st = mapper_call(base, "get_state0", "get_state")
    check("cannot write" in st["last_error"], f"mapper survives and reports it ({st['last_error']!r})")
    yaml = mapper_call(base, "save_map0", "save_map", {"path": str(out / "map")})
    check(Path(yaml).exists() and (out / "map.png").exists(), f"save_map writes {yaml}")
    compare_maps(sim, read_png(out / "map.png"), info)


def fmt(p):
    return f"({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})"


def compare_maps(sim, slam_img, info):
    """SLAM の地図の占有の格子が、真の地図の壁の近くにあるか(真の地図の壁を見つけているか)"""
    sim_info = sim_call(sim, "get_map_info0", "get_map_info")
    from PIL import Image

    truth_img = Image.open(io.BytesIO(sim_call(sim, "get_map0", "get_map", raw=True))).convert("L")
    tres, tox, toy = sim_info["resolution"], sim_info["origin"][0], sim_info["origin"][1]
    tw, th = truth_img.size
    tpx = truth_img.load()

    def truth_occupied_near(x, y, r):
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                i = int((x - tox) / tres) + dx
                j = th - 1 - int((y - toy) / tres) - dy
                if 0 <= i < tw and 0 <= j < th and tpx[i, j] == 0:
                    return True
        return False

    res, ox, oy = info["resolution"], info["origin"][0], info["origin"][1]
    w, h = slam_img.size
    spx = slam_img.load()
    occupied = [(i, j) for j in range(h) for i in range(w) if spx[i, j] == 0]
    near = 0
    for i, j in occupied:
        x = ox + (i + 0.5) * res
        y = oy + (h - 1 - j + 0.5) * res
        if truth_occupied_near(x, y, 2):  # 0.1 m 以内
            near += 1
    precision = near / max(1, len(occupied))
    print(f"   {len(occupied)} occupied cells, {precision:.1%} within 0.1 m of a true wall")
    check(len(occupied) > 1000, f"the map has walls ({len(occupied)} occupied cells)")
    check(precision > 0.95, f"occupied cells lie on true walls ({precision:.1%})")

    # 真の壁の画素のうち、SLAM の地図で占有になっている割合(外周の壁で確かめる)
    hits = total = 0
    for k in range(-45, 46):
        for (x, y) in [(k * 0.1, 4.95), (k * 0.1, -4.95), (4.95, k * 0.1), (-4.95, k * 0.1)]:
            if x > 3.4 and y < -3.4:
                continue  # 右下の不明な領域の近く
            total += 1
            found = False
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    i = int((x - ox) / res) + dx
                    j = h - 1 - int((y - oy) / res) - dy
                    if 0 <= i < w and 0 <= j < h and spx[i, j] == 0:
                        found = True
            hits += found
    recall = hits / total
    print(f"   outer walls found: {recall:.1%}")
    check(recall > 0.9, f"the outer walls are mapped ({recall:.1%})")


if __name__ == "__main__":
    main()
