#!/bin/sh
# mobile_sim が起動するのを待ってから、インストールした mapper_gmapping を juiz run で動かす。
# juiz run は juizapp.yaml の depends で mobile_sim の版を確かめる
set -eu
url="${MOBILE_SIM_URL:?MOBILE_SIM_URL を http://<host>:8000 の形で渡してください}"
python3 - "$url" <<'PY'
import sys, time, urllib.request
url = sys.argv[1].rstrip("/") + "/api/container/list"
for _ in range(300):
    try:
        urllib.request.urlopen(url, timeout=2).read()
        sys.exit(0)
    except Exception:
        time.sleep(0.2)
sys.exit(f"mobile_sim did not answer at {url}")
PY
exec juiz run mapper_gmapping --depend "mobile_sim=${url}" "$@"
