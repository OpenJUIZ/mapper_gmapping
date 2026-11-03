#!/bin/sh
# mobile_sim が起動するのを待ってから mapper_gmapping を動かす。
# app.conf の IPC の subsystem を MOBILE_SIM_URL の HTTP の subsystem に置き換えた conf を作り、juiz -f で動かす
# (mobile_sim のイメージは juiz run で動いていないので、juiz run の depends の確かめは使えない)
set -eu
url="${MOBILE_SIM_URL:?MOBILE_SIM_URL を http://<host>:<port> の形で渡してください}"
conf=/opt/mapper_gmapping/.docker.conf
python3 - "$url" "$conf" <<'PY'
import sys, time, urllib.parse, urllib.request
url, conf = sys.argv[1].rstrip("/"), sys.argv[2]
host = urllib.parse.urlparse(url).netloc
src = open("/opt/mapper_gmapping/app.conf").read()
ipc_subsystem = '''  - "package": "mobile_sim"
    "type_name": "ipc"
    "name": "mobile_sim"'''
ipc_broker = '''"brokers":
  - "type_name": "ipc"
    "name": "mapper_gmapping"
'''
if not host or ipc_subsystem not in src or ipc_broker not in src:
    sys.exit("cannot make the conf: check MOBILE_SIM_URL and the subsystems/brokers of app.conf")
src = src.replace(ipc_subsystem, f'  - "type_name": "http"\n    "name": "{host}"').replace(ipc_broker, "")
open(conf, "w").write(src)
for _ in range(300):
    try:
        urllib.request.urlopen(url + "/api/container/list", timeout=2).read()
        break
    except Exception:
        time.sleep(0.2)
else:
    sys.exit(f"mobile_sim did not answer at {url}")
PY
exec juiz -f "$conf" -d "$@"
