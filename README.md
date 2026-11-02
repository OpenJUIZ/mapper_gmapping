# mapper_gmapping

[GMapping](https://github.com/OpenJUIZ/openslam_gmapping) を使って、LiDAR のスキャンとオドメトリから地図と自己位置を推定する [JUIZ](https://github.com/OpenJUIZ/juiz) のアプリです。`.juizapp` として配布し、[mobile_sim](https://github.com/OpenJUIZ/mobile_sim) と組み合わせて使います。

> 状態: 開発中です(M2 juiz のコンポーネントと mobile_sim との連携まで)。計画は [docs/plan.md](docs/plan.md)、進み具合は [Issues](https://github.com/OpenJUIZ/mapper_gmapping/issues) を見てください。

## 構成

| パス | 中身 |
| --- | --- |
| `third_party/openslam_gmapping/` | GMapping(git submodule)。静的ライブラリにしてプラグインに入れる |
| `src/core/` | GMapping を包む部分(JUIZ に依存しない) |
| `src/component/` | JUIZ のコンポーネント(C++ のプラグイン) |
| `tests/` | `src/core` のテスト(ctest) |
| `app/` | `juiz` で起動する conf |
| `scripts/` | juiz_sdk の取得、開発用のビルド、結合テスト |

## 開発

CMake 3.16 以降と C++17 のコンパイラが要ります。

```sh
git clone --recursive https://github.com/OpenJUIZ/mapper_gmapping
cd mapper_gmapping
sdk="$(scripts/fetch_juiz_sdk.sh 0.1.0)"   # juiz の Releases から juiz_sdk を取って .juiz_sdk/ に展開する
cmake -B build -DCMAKE_PREFIX_PATH="$sdk"
cmake --build build
ctest --test-dir build
```

- juiz_sdk の版は、使う juiz の版と完全に一致させます(既定は 0.1.0)。
- C++ のプラグインは C ABI なので、Rust のプラグインと違って juiz をビルドした rustc を揃える必要はありません。`pip install juiz-cli==0.1.0` の juiz で読めます。
- juiz_sdk が無いときは `-DMAPPER_GMAPPING_BUILD_COMPONENT=OFF` で `src/core` とテストだけをビルドできます。

### SLAM 本体(`src/core`)

`mapper_gmapping::Mapper` に、スキャンとその時刻のオドメトリの姿勢を順に渡すと、推定した姿勢と地図が得られます。パラメータの名前と既定値は ROS の slam_gmapping と同じです。

```cpp
#include "mapper_gmapping/mapper.h"

mapper_gmapping::Mapper mapper(params);
mapper.add_scan(scan, odom);                      // 移動量が linearUpdate・angularUpdate を超えたら粒子を更新する
auto pose = mapper.pose();                        // 最良の粒子の姿勢
auto grid = mapper.map();                         // 占有格子(-1 不明、0 空き、100 占有)
mapper_gmapping::save_map(grid, "maps/room");    // maps/room.yaml と maps/room.png(ROS map_server・mobile_sim の形式)
```

- 当たらなかったビーム(`range_min`〜`range_max` の外、NaN、inf)は、その方向が空いているという情報としてだけ使います。
- `tests/core_test.cpp` は、10 m x 8 m の部屋をずれのあるオドメトリで 1 周半走り、推定姿勢の誤差と、地図の壁・通った場所・余計な占有セルを確かめます(手元では誤差 0.02 m、オドメトリだけだと 0.79 m)。
- GMapping そのままの 2 つの問題をこちらで直しています: 最初のスキャンの直後に壊すと二重解放になることと、スキャンのコピーが解放されないこと。CI でアドレスサニタイザを付けて確かめています。

### juiz で動かす

```sh
pip install juiz-cli==0.1.0
scripts/dev_plugin.sh          # プラグインと libjuiz_sdk を app/plugins/<target>/ に置く
python3 scripts/smoke_test.py  # juiz を起動して、HTTP からプロセスを呼ぶ
```

### mobile_sim と動かす

mobile_sim(0.2.0 以降)はトピック `odom`(`Pose2DStamped`)と `scan`(`LaserScan`)を出します。`app/app.conf` は IPC で `mobile_sim` につなぎ、この 2 つを購読して地図を作ります。

```sh
# mobile_sim のリポジトリで(scripts/dev_plugin.sh の後)
cd app && juiz -f app.conf -d --ipc mobile_sim
# こちらで
cd app && juiz -f app.conf -d
```

mobile_sim の画面(http://localhost:8000/)でロボットを動かすと、地図が育ちます。`curl` などで呼べる主な ContainerProcess(コンテナ `gmapping`、インスタンス `mapper0`):

| ContainerProcess | 引数 | 戻り値 |
| --- | --- | --- |
| `gmapping_set_odometry` | `odom: Pose2DStamped` | なし。トピック `odom` を購読 |
| `gmapping_add_scan` | `scan: LaserScan` | 地図を更新したか。トピック `scan` を購読(オドメトリが来る前のスキャンは捨てる) |
| `gmapping_get_pose` | なし | `Pose2DStamped`(frame `map`) |
| `gmapping_get_map` | なし | 地図の画像(L8。占有 0、空き 254、不明 205) |
| `gmapping_get_map_info` | なし | `{resolution, origin, width, height, updates, ...}` |
| `gmapping_save_map` | `path` | `<path>.yaml` と `<path>.png`(ROS map_server の形式)を書き、YAML のパスを返す |
| `gmapping_reset` | なし | なし(地図を捨てて最初から) |
| `gmapping_get_state` | なし | 粒子、軌跡、オドメトリ、`map_to_odom` など(画面用) |

GMapping の引数(`particles`、`delta`、`xmin` など slam_gmapping と同じ名前)と `laser_pose`、`map_update_interval` はコンテナの引数で変えられます。

結合テストは mobile_sim を部屋で一周させ、SLAM の姿勢と地図を真の値と比べます(手元では 115 秒走って誤差 0.011 m、オドメトリだけだと 0.69 m、占有セルの 96% が真の壁の 0.1 m 以内)。

```sh
python3 scripts/integration_test.py --mobile-sim ../mobile_sim
```

## ライセンス

MIT または Apache-2.0 のどちらか(デュアルライセンス)です。[LICENSE-MIT](LICENSE-MIT)・[LICENSE-APACHE](LICENSE-APACHE) を見てください。GMapping は BSD-3-Clause です(`third_party/openslam_gmapping/LICENSE`)。
