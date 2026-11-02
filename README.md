# mapper_gmapping

[GMapping](https://github.com/OpenJUIZ/openslam_gmapping) を使って、LiDAR のスキャンとオドメトリから地図と自己位置を推定する [JUIZ](https://github.com/OpenJUIZ/juiz) のアプリです。`.juizapp` として配布し、[mobile_sim](https://github.com/OpenJUIZ/mobile_sim) と組み合わせて使います。

> 状態: 開発中です(M0 雛形)。計画は [docs/plan.md](docs/plan.md)、進み具合は [Issues](https://github.com/OpenJUIZ/mapper_gmapping/issues) を見てください。

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

### juiz で動かす

```sh
pip install juiz-cli==0.1.0
scripts/dev_plugin.sh          # プラグインと libjuiz_sdk を app/plugins/<target>/ に置く
python3 scripts/smoke_test.py  # juiz を起動して、HTTP からプロセスを呼ぶ
```

## ライセンス

MIT または Apache-2.0 のどちらか(デュアルライセンス)です。[LICENSE-MIT](LICENSE-MIT)・[LICENSE-APACHE](LICENSE-APACHE) を見てください。GMapping は BSD-3-Clause です(`third_party/openslam_gmapping/LICENSE`)。
