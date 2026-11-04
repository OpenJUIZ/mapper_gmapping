# mapper_gmapping

[GMapping](https://github.com/OpenJUIZ/openslam_gmapping) を使って、LiDAR のスキャンとオドメトリから地図と自己位置を推定する [JUIZ](https://github.com/OpenJUIZ/juiz) のアプリです。`.juizapp` として配布し、[mobile_sim](https://github.com/OpenJUIZ/mobile_sim) と組み合わせて使います。

> 状態: 0.3 系は juiz 0.2 の System をまたぐ Transform で mobile_sim と座標系を 1 本の木にしています。Docker イメージまであります。計画は [docs/plan.md](docs/plan.md)、進み具合は [Issues](https://github.com/OpenJUIZ/mapper_gmapping/issues) を見てください。

## 使い方

[Releases](https://github.com/OpenJUIZ/mapper_gmapping/releases) から `mapper_gmapping-<version>.juizapp` を、[mobile_sim の Releases](https://github.com/OpenJUIZ/mobile_sim/releases) から 0.3 系の `mobile_sim-<version>.juizapp` を取ってきます。

```sh
pip install juiz-cli==0.2.2
juiz run mobile_sim-<version>.juizapp      # 先に動かしておく(ipc://mobile_sim、http://localhost:8000/)
juiz run mapper_gmapping-<version>.juizapp     # 別の端末で。ブラウザで http://localhost:8001/ を開く
```

mapper は `juizapp.yaml` の `depends`(`mobile_sim >=0.3.1, <0.4`)を見て、動いている mobile_sim の版を確かめてからつなぎます。画面のキーかパッドでロボットを動かすと地図が育ち、「地図を保存」で ROS map_server の形式(YAML + PNG)に書き出せます。保存先の相対パスは `juiz run` を起動したディレクトリからです。

対応するターゲット: Linux(x86_64、aarch64)、macOS(Apple シリコン、Intel)、Windows(x86_64)。

### Docker

mobile_sim のイメージと一緒に動かします。

```sh
git clone --recursive https://github.com/OpenJUIZ/mapper_gmapping.git && cd mapper_gmapping
docker compose -f docker/compose.yml up
# ブラウザで http://localhost:8001/ (地図づくり) と http://localhost:8000/ (シミュレータ) を開く
```

- イメージは `ghcr.io/openjuiz/mapper_gmapping`(linux/amd64 と linux/arm64)です。`:latest` はリリースの版、`:edge` は main の最新です。手元でビルドするときは `up --build` にします。
- 「地図を保存」の地図は `docker/maps/` に出ます(コンテナの `/maps`)。
- 別の場所の mobile_sim につなぐときは、環境変数 `MOBILE_SIM_URL`(既定 `http://mobile_sim:8000`)を変えます。mobile_sim は、mapper が接続してきたアドレスの 8001 番にトピックを送るので、mobile_sim からその番号に届くようにしてください。

## 構成

| パス | 中身 |
| --- | --- |
| `third_party/openslam_gmapping/` | GMapping(git submodule)。静的ライブラリにしてプラグインに入れる |
| `src/core/` | GMapping を包む部分(JUIZ に依存しない) |
| `src/component/` | JUIZ のコンポーネント(C++ のプラグイン) |
| `tests/` | `src/core` のテスト(ctest) |
| `app/` | `juiz` で起動する conf |
| `scripts/` | juiz_sdk の取得、開発用のビルド、結合テスト |
| `docker/` | Dockerfile と compose.yml |

## 開発

CMake 3.16 以降と C++17 のコンパイラが要ります。

```sh
git clone --recursive https://github.com/OpenJUIZ/mapper_gmapping
cd mapper_gmapping
sdk="$(scripts/fetch_juiz_sdk.sh 0.2.2)"   # juiz の Releases から juiz_sdk を取って .juiz_sdk/ に展開する
cmake -B build -DCMAKE_PREFIX_PATH="$sdk"
cmake --build build
ctest --test-dir build
```

- juiz_sdk の版は、使う juiz の版と完全に一致させます(既定は 0.2.2)。
- C++ のプラグインは C ABI なので、Rust のプラグインと違って juiz をビルドした rustc を揃える必要はありません。`pip install juiz-cli==0.2.2` の juiz で読めます。
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
pip install juiz-cli==0.2.2
scripts/dev_plugin.sh          # プラグインと libjuiz_sdk を app/plugins/<target>/ に置く
python3 scripts/smoke_test.py  # juiz を起動して、HTTP からプロセスを呼ぶ
```

### mobile_sim と動かす

mobile_sim(0.3.0 以降)はトピック `odom`(`Pose2DStamped`)、`scan`(`LaserScan`)、`lidar_mount`(`PoseStamped`、ロボットから見た LiDAR の取付位置)を出します。`app/app.conf` は IPC で `mobile_sim` につなぎ、この 3 つを購読して地図を作ります。

```sh
# mobile_sim のリポジトリで(scripts/dev_plugin.sh の後)
cd app && juiz -f app.conf -d --ipc mobile_sim
# こちらで
cd app && juiz -f app.conf -d
```

http://localhost:8001/ を開くと、育っていく地図、推定した姿勢と軌跡、粒子を見られます。mobile_sim につながっていれば、真の姿勢とオドメトリだけの姿勢を重ねて誤差を出し、WASD・矢印キー・パッドでロボットを動かせます(mobile_sim の処理は mapper の HTTP から `ipc://mobile_sim/...` で呼ぶので、mobile_sim 側の設定は要りません)。地図の保存(`save_map`)、PNG のダウンロード、作り直しのボタンもあります。画面は `app/web/` にあります。`curl` などで呼べる主な ContainerProcess(コンテナ `gmapping`、インスタンス `mapper0`):

| ContainerProcess | 引数 | 戻り値 |
| --- | --- | --- |
| `gmapping_set_odometry` | `odom: Pose2DStamped` | なし。トピック `odom` を購読 |
| `gmapping_add_scan` | `scan: LaserScan` | 地図を更新したか。トピック `scan` を購読(オドメトリが来る前のスキャンは捨てる) |
| `gmapping_set_laser_pose` | `mount: PoseStamped` | 取付位置が変わったか。トピック `lidar_mount` を購読(変わったら地図を作り直す) |
| `gmapping_get_pose` | なし | `Pose2DStamped`(frame `map`) |
| `gmapping_get_map` | なし | 地図の画像(L8。占有 0、空き 254、不明 205) |
| `gmapping_get_map_info` | なし | `{resolution, origin, width, height, updates, ...}` |
| `gmapping_save_map` | `path` | `<path>.yaml` と `<path>.png`(ROS map_server の形式)を書き、YAML のパスを返す |
| `gmapping_reset` | なし | なし(地図を捨てて最初から) |
| `gmapping_get_state` | なし | 粒子、軌跡、オドメトリ、`map_to_odom` など(画面用) |
| `gmapping_odom_frame_update`(コンテナ `gmapping_odom_frame`) | `odom`(トピックで受ける) | mapper の `map_to_odom` を自分の ContainerPose として書く |

GMapping の引数(`particles`、`delta`、`xmin` など slam_gmapping と同じ名前)と `laser_pose`(トピック `lidar_mount` が来るまでの取付位置)、`map_update_interval` はコンテナの引数で変えられます。

#### 座標系(Transformations)

juiz 0.2 の Transformations を使い、推定した姿勢を `mapper0` の ContainerPose として持たせています(計画は [mobile_sim の docs/transformations-plan.md](https://github.com/OpenJUIZ/mobile_sim/blob/main/docs/transformations-plan.md))。

- `app/app.conf` に Anchor `map`(SLAM の地図の座標系)と Transform `map -> mapper0` があり、`mapper0` はスキャンのたびに推定した姿勢を書きます。`juiz container pose mapper0 --reference map -s http://localhost:8001` や管理画面の木で見られます。基準はコンテナの引数 `map_frame`(既定 `map`、空なら書かない)で変えられます。画面の推定位置もこの ContainerPose から読み、状態の欄に「座標系」(書けないときはその理由)と、`lidar_mount` から受け取った「LiDAR の取付位置」を出します。
- LiDAR の取付位置は mobile_sim の Transform `robot0 -> lidar0` が持ち、トピック `lidar_mount` で届きます。`wait_laser_pose: true` のときは、届くまでスキャンを使いません。
- juiz 0.2.1 の System をまたぐ Transform で、mobile_sim の座標系と 1 本の木にしています。コンテナ `slam_odom0`(型 `gmapping_odom_frame`)が SLAM の推定したオドメトリの原点(ROS の map -> odom)を `map` から見た自分の位置として書き、conf の Transform `slam_odom0 -> ipc://mobile_sim/container/odom0::static_anchor` は mobile_sim に回されて作られます。mobile_sim は `slam_odom0` の位置を 20 Hz で取りに来るので、mobile_sim の中で `map -> slam_odom0 -> odom0 -> robot0 -> lidar0` がつながり、`juiz container pose lidar0 --reference map`(mobile_sim の 8000 番)が読めます。mapper の側からも `/api/container/pose?identifier=ipc://mobile_sim/container/lidar0::mobile_lidar&reference=map` で読めます。
- この Transform の終点は既定のつなぎ先 `ipc://mobile_sim/...` で書いています。juiz 0.2.2 から `juiz run` がこれを決まったつなぎ先に置き換えるので、`juiz run --depend mobile_sim=http://192.168.0.10:8000` で別の機械の mobile_sim にもそのままつながります(juiz#425)。`juiz -f` で動かすときは置き換わらないので、Docker では `entrypoint.sh` が置き換えます。
- 地図・姿勢・状態を読むだけの Process(`gmapping_get_pose`・`get_map`・`get_map_info`・`save_map`・`get_state`)は readonly です(juiz 0.2.2)。juiz は読み出しロックで呼ぶので、画面が状態と地図を同時に取りに来ても互いに待ちません。

結合テストは mobile_sim を部屋で一周させ、SLAM の姿勢と地図を真の値と比べます(手元では 115 秒走って誤差 0.011 m、オドメトリだけだと 0.69 m、占有セルの 96% が真の壁の 0.1 m 以内)。

```sh
python3 scripts/integration_test.py --mobile-sim ../mobile_sim
# 続けてブラウザ画面も Playwright で確かめる(npm install playwright が要る)
python3 scripts/integration_test.py --mobile-sim ../mobile_sim --ui ui_test.png
```

### パッケージを作る

```sh
scripts/dev_plugin.sh                  # この機械のプラグインを app/plugins/<target>/ に置く
scripts/pack.sh app/plugins dist       # dist/mapper_gmapping-<version>.juizapp
python3 scripts/integration_test.py --mobile-sim-juizapp mobile_sim-<version>.juizapp --juizapp dist/mapper_gmapping-<version>.juizapp
```

動いている mobile_sim と mapper(`docker compose` など)にも同じテストを当てられます(`--mapper-out` は mapper から見た `--out` のパス)。

```sh
python3 scripts/integration_test.py --sim-url http://localhost:8000 --mapper-url http://localhost:8001 --out docker/maps --mapper-out /maps
```

CI の Release ワークフローは 5 ターゲットでビルド・テストして `.juizapp` にまとめ、mobile_sim の main から作った `.juizapp` と一緒に `juiz run` で動かして確かめます。GitHub の画面で `v<version>` の Release を作ると、その Release に `.juizapp` が付き、Docker ワークフローが `ghcr.io/openjuiz/mapper_gmapping:<version>` と `:latest` を公開します(`app/juizapp.yaml` と `CMakeLists.txt` の版をタグに合わせておく)。

## ライセンス

MIT または Apache-2.0 のどちらか(デュアルライセンス)です。[LICENSE-MIT](LICENSE-MIT)・[LICENSE-APACHE](LICENSE-APACHE) を見てください。GMapping は BSD-3-Clause です(`third_party/openslam_gmapping/LICENSE`)。
