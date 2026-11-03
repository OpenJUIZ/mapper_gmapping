# mapper_gmapping 計画書(GMapping による SLAM の juizapp)

2026-11-02 作成

## 目的と要件

[OpenJUIZ/openslam_gmapping](https://github.com/OpenJUIZ/openslam_gmapping)(ROS への依存を外した GMapping)を JUIZ のコンポーネントで包み、LiDAR のスキャンとオドメトリから地図と自己位置を推定する SLAM を `.juizapp` として配布する。まずは [mobile_sim](https://github.com/OpenJUIZ/mobile_sim) と組み合わせて、シミュレータの中で地図を作れるようにする。

| # | 要件 | 方針 |
| --- | --- | --- |
| 1 | openslam_gmapping で SLAM をする | C++ のコンポーネント 1 つに、GMapping の `GridSlamProcessor` を持つコンテナを置く |
| 2 | juizapp として配布する | `plugins/<target>/` に C++ のプラグインと libjuiz_sdk を入れる(app-package.md §12.2) |
| 3 | mobile_sim と一緒に動く | mobile_sim(`kind: base`)に `depends` でつなぎ、トピック `scan` と `odom` を購読する |
| 4 | 作った地図を使える | ROS map_server と同じ YAML と PNG で保存する。mobile_sim の mapconf と同じ形式なので、そのまま mobile_sim に読ませられる |
| 5 | 様子を見られる | ブラウザ画面で、育っていく地図と推定した姿勢を表示する |

## 調べたこと

### openslam_gmapping

- 2014 年の fork だが、今の g++ 13 と clang でもそのままビルドできた(警告のみ)。既定で静的ライブラリ 6 つ(utils、sensor_*、scanmatcher、gridfastslam)ができる。外部の依存は無い。
- 使う API は ROS の slam_gmapping と同じ: `setSensorMap`、`init(粒子数, xmin, ymin, xmax, ymax, delta, 初期姿勢)`、`setMatchingParameters`、`setMotionModelParameters`、`setUpdateDistances`、`processScan(RangeReading)`、`getBestParticleIndex`。地図は最良の粒子の軌跡から `ScanMatcher` で作り直す(slam_gmapping の `updateMap` と同じやり方)。
- 1 スキャンのビーム数の上限は 2048(`LASER_MAXBEAMS`)。mobile_sim の既定 271 本は問題ない。
- `cmake_minimum_required(VERSION 2.8)` のため、CMake 4 では `CMAKE_POLICY_VERSION_MINIMUM=3.5` を付けないと構成できない。fork 側で 3.10 に上げる小さな変更を推奨する。
- リポジトリに LICENSE ファイルが無く、一部のファイルに古い CC BY-NC-SA 2.0 の表記が残っている(上流は 2019 年に BSD-3-Clause に変更済み)。配布する前に上流を取り込み LICENSE を足すことを推奨する([決めてほしいこと](#決めてほしいこととリスク))。

### juiz の C++ 対応(0.1.0)

- `bindings/cppjuiz` のヘッダで、Process・Container・ContainerProcess・Component を C++ で書ける(`COMPONENT_CONTAINER_FACTORY`、`COMPONENT_CONTAINER_PROCESS_FACTORY`、`ProcessManifest().add_typed_arg<T>()` など)。
- 型は `juiz_generate_types()`(juiz-idl)で IDL から C++ の構造体を作れる。`juiz_std/sensor.jidl` の `LaserScan`、`geometry.jidl` の `Pose2D` などを使えば、mobile_sim(Rust)と型の指紋が一致する。地図は `juiz::Image`(L8)で返せる。占有格子の標準型は無いので、画像 + 情報の dict で返す(mobile_sim の `get_map` / `get_map_info` と同じ形)。
- 配布用には juiz の Releases に `juiz_sdk-0.1.0-<target>.tar.gz` があり、CMake の `find_package(juiz_sdk 0.1.0 EXACT CONFIG)` と `juiz_sdk_bundle(<target>)` で、プラグインの隣に libjuiz_sdk を置いた `plugins/<target>/` を作れる。
- C++ のプラグインは C ABI なので、**Rust のプラグインのような rustc の一致の制約が無い**。`juiz: ">=0.1.0"` で配れ、`cargo install` で入れた juiz でも動く。mobile_sim でいちばん苦労した点が無くなる。
- Linux 向けは glibc が古めの環境(Ubuntu 22.04)でビルドする(§12.2)。libstdc++ は同梱しない。

### mobile_sim の側

- `mobile_sim_get_scan`(`LaserScan`、当たらないビームは `range_max + 1`)と `mobile_sim_get_odometry`(`{time, pose, twist, distance}`)はあるが、今の `app.conf` は**どちらもトピックに配信していない**(`cmd_vel` の購読だけ)。
- オドメトリは**真値そのまま**で、誤差が無い。これでは SLAM の補正が効いているか確かめられないので、オドメトリにノイズ(ドリフト)を足す設定が要る。
- このため mobile_sim にも小さな変更を入れて 0.2.0 として出す(M2)。

## 設計

### 全体の形

```
juiz run mobile_sim                      (kind: base, ipc://mobile_sim, http :8000)
  main_ec 20 Hz: step → get_odom_pose → topic "odom" → get_scan → topic "scan"   ← M2 で追加
  (main_loop_ec は 1 システムに 1 つだけなので、別の EC にせず同じ周回で出す)
        │ ipc
        ▼
juiz run mapper_gmapping-x.y.z.juizapp   (kind: app, depends: mobile_sim, http :8001)
  gmapping_set_odometry  ← subscribe "odom"(最新を持つだけ)
  gmapping_add_scan      ← subscribe "scan"(最新のオドメトリと組にして processScan)
  gmapping_get_pose / get_map / get_map_info / save_map / get_state
```

- スキャンとオドメトリは別のトピックで受け、スキャンが来たときに手元の最新のオドメトリと組にする。mobile_sim は 50 Hz で積分しているので、ずれは 20 ms 以下。足りなければ mobile_sim 側に「スキャンとその時刻のオドメトリを一緒に返す」プロセスを足す。
- トピック名は `scan`・`odom` と一般的な名前にしておき、実機の基礎パッケージが同じ名前で配信すれば、そのまま使えるようにする。

### リポジトリ構成

- `CMakeLists.txt`: 最上位。C++17
- `third_party/openslam_gmapping/`: git submodule(コミットを固定)。静的ライブラリ(`-fPIC`)にしてプラグインに静的リンクする。配布物には `libmapper_gmapping.so` と `libjuiz_sdk.so` だけが入る
- `src/core/`: GMapping を包む juiz 非依存の C++ ライブラリ(パラメータ、スキャン・オドメトリの変換、地図の取り出し、map_server 形式の保存)
- `src/component/`: juiz のコンポーネント(コンテナとプロセス)
- `idl/`: 必要なら追加の型(地図の情報など)
- `tests/`: core の単体テスト(ctest)。合成した部屋の中でスキャンを作って地図と姿勢を確かめる
- `app/`: `juiz pack` の入力(`juizapp.yaml`、`app.conf`、`web/`)
- `scripts/`: 結合テスト、pack、開発用のビルド
- `docker/`、`.github/workflows/`

### コンテナとプロセス

コンテナ `gmapping`(インスタンス名 `mapper0`)。引数は ROS の slam_gmapping と同じ名前にして、既存の調整の知見をそのまま使えるようにする(`particles`、`delta`、`maxUrange`、`sigma`、`kernelSize`、`lstep`、`astep`、`iterations`、`lsigma`、`ogain`、`lskip`、`srr`、`srt`、`str`、`stt`、`linearUpdate`、`angularUpdate`、`resampleThreshold`、`xmin`…`ymax`、`map_update_interval`)。

| プロセス | 入力 | 出力 | 使い方 |
| --- | --- | --- | --- |
| `gmapping_set_odometry` | odom(mobile_sim の odometry の dict) | なし | トピック `odom` を購読 |
| `gmapping_add_scan` | `LaserScan` | 更新したか(bool) | トピック `scan` を購読。最初のスキャンでレーザーの設定から GMapping を初期化 |
| `gmapping_get_pose` | なし | `Pose2DStamped`(最良の粒子) | トピック `slam_pose` に配信してもよい |
| `gmapping_get_map` | なし | 画像(L8。白 = 空き、黒 = 占有、灰 = 不明) | 画面 |
| `gmapping_get_map_info` | なし | resolution、origin、幅・高さ、更新回数 | 画面 |
| `gmapping_save_map` | path | なし | `<path>.yaml` と `<path>.png` を map_server 形式で書く |
| `gmapping_reset` | なし | なし | 地図と粒子を捨ててやり直す |
| `gmapping_get_state` | なし | 推定姿勢、粒子の位置、軌跡、更新回数 | 画面が数 Hz で取りに来る |

- `processScan` は粒子数 30 で数十 ms かかることがある。トピックの購読で呼ばれるので、EC を止めないよう、地図の作り直しは `map_update_interval`(既定 2 s)ごとにする。
- GMapping は標準出力に多くを書くので、既定で黙らせ、`verbose` で出せるようにする。
- 新しくコードを書く。ROS の slam_gmapping(ラッパー)は CC BY-NC-SA なのでコードは写さず、パラメータの名前と既定値だけ合わせる。

### ブラウザ画面

mapper の HTTP ブローカー(8001)で `web/` を配信する。地図(数秒ごと)、推定姿勢と軌跡、粒子を Canvas に描く。mobile_sim の画面(8000)を同時に開けば真値と見比べられる。余裕があれば同じ画面に mobile_sim の真の姿勢も重ね、誤差を表示する。

### juizapp

- `kind: app`、`depends: { mobile_sim: "^0.2" }`、`juiz: ">=0.1.0"`。HTTP は 8001(mobile_sim の 8000 とぶつけない)。
- ターゲットは mobile_sim と同じ 5 つ(Linux x86_64・arm64、macOS Intel・Apple Silicon、Windows x86_64)。Windows は fork が VC2010 でビルドできた記録があるが、今の MSVC では未確認。

## 実装の進め方

各段階を 1 つの PR にし、GitHub の issue で管理する。

1. **M0 雛形**: CMake、submodule、ライセンス、juiz_sdk 0.1.0 のアーカイブを取ってきて空のプラグインを作る CI(Linux)。
2. **M1 SLAM 本体**(`src/core`): GMapping の初期化・スキャン追加・地図の取り出し・map_server 形式の保存。合成した部屋をロボットが一周するデータで、地図の壁の位置と推定姿勢の誤差を ctest で確かめる。
3. **M2 juiz コンポーネントと mobile_sim の変更**: コンテナとプロセス、`app.conf`。mobile_sim には「スキャンとオドメトリをトピックに配信する EC」と「オドメトリのノイズ」を足して 0.2.0 にする(mobile_sim 側の PR)。結合テストは、両方を起動して `cmd_vel` で決まった経路を走らせ、できた地図を元の地図と比べる(占有セルの一致率)のと、推定姿勢と真値の誤差を見る。
4. **M3 ブラウザ画面**: 地図・姿勢・粒子の表示と、地図の保存ボタン。Playwright で確かめる。
   - できたもの: 真の姿勢とオドメトリの重ね描きと誤差、推定位置から描いたスキャン、mobile_sim の操作も同じ画面に入れた。mobile_sim は mapper の subsystem なので、mapper の HTTP から `ipc://mobile_sim/...` の識別子で呼べ、CORS の設定は要らない。
5. **M4 juizapp 化と配布**: 5 ターゲットのビルド、`juiz pack`、`juiz run` での結合テスト、Releases への添付(mobile_sim と同じく GitHub の画面で Release を作る運用)。
   - conf の subsystem に `package: mobile_sim` を書き、`juiz run` が depends の確認に使うようにした(`juiz -f` でもそのまま動く)。GMapping の LICENSE(BSD-3-Clause)をパッケージに入れる。
6. **M5 Docker**: mobile_sim と mapper を一緒に動かす compose.yml と、ghcr.io への公開。

その後に、保存した地図で自己位置推定(AMCL 相当)、実機の基礎パッケージとのつなぎ方、ループの閉じ方の調整などを考える。

## 決めてほしいこととリスク

次の 5 点は推奨で進め、違う場合だけ指摘してほしい。

| 決めること | 推奨 | 理由 |
| --- | --- | --- |
| openslam_gmapping の取り込み方 | git submodule でコミットを固定し、静的リンク | 配布物が 2 ファイルで済み、依存の .so を集めなくてよい |
| パッケージの種類 | `kind: app`、`depends: mobile_sim` | `juiz run` だけで mobile_sim に自動でつながる。実機の基礎を使うときは、同じトピック名で配信すれば conf の subsystems で差し替えられる |
| mobile_sim の変更 | トピック配信の EC とオドメトリのノイズを足し、mobile_sim 0.2.0 を出す | 今の mobile_sim はスキャンを配信せず、オドメトリに誤差が無い |
| 地図の形式 | ROS map_server の YAML + PNG | mobile_sim の mapconf と同じなので、作った地図をシミュレータに戻せる |
| 実装言語 | C++ だけ(Rust は使わない) | rustc の一致の制約が無く、どの juiz 0.1.0 以降でも動く |

既知のリスクは次の 4 つ。

- **ライセンス**: GMapping は元は CC BY-NC-SA 2.0(非営利)で、OpenJUIZ/openslam_gmapping(2014 年の fork、2026-11-02 に ysuga から移管)の一部のファイル(gui、carmenwrapper)にもその表記が残っている。元の ros-perception/openslam_gmapping は 2019 年の 0.2.0(#29)で著作者の Grisetti・Stachniss・Burgard の名前のまま BSD-3-Clause に変わった。fork に上流の 0.2.x を取り込み、LICENSE(BSD-3-Clause)を足してから配布したい。mapper_gmapping 自体は mobile_sim と同じ MIT OR Apache-2.0 を推奨。
- **Windows・macOS のビルド**: 古いコードなので、MSVC で直しが要るかもしれない。直しは OpenJUIZ/openslam_gmapping への PR にするか、こちらでパッチを当てるかをそのときに相談する。
- **スキャンとオドメトリの時刻合わせ**: 別のトピックで受けるので少しずれる。シミュレータでは問題ない見込みだが、結合テストで地図がぶれるなら mobile_sim に組で返すプロセスを足す。
- **処理時間**: 粒子数を増やすと `processScan` が重くなる。トピックの購読側で詰まらないか M2 で測り、必要なら別スレッドにする。
  - M2 で測った結果: 30 粒子で更新 1 回は約 0.1 秒。20 Hz のスキャンのうち更新は 0.5 m か 0.5 rad 動いたときだけなので、結合テスト(115 秒、2078 スキャン)で詰まりは無かった。別スレッドは M3 で画面を付けて様子を見てから決める。
  - M3: 画面から 5 Hz で get_state を呼びながら走らせても詰まりは無かったので、別スレッドにはしない。
