# syntax=docker/dockerfile:1
# mapper_gmapping のイメージ。リポジトリのルートを context にしてビルドする(サブモジュールも取ってくること):
#   git submodule update --init
#   docker build -f docker/Dockerfile -t mapper_gmapping .
#
# C++ のプラグインは C ABI なので、juiz は pip の juiz-cli をそのまま使う。
# juiz_sdk の版は juiz と揃える(CMakeLists.txt の MAPPER_GMAPPING_JUIZ_SDK_VERSION と app/juizapp.yaml の juiz)。
ARG JUIZ_VERSION=0.1.0

# プラグインをビルドしてテストする
FROM debian:bookworm AS build
ARG JUIZ_VERSION
RUN apt-get update \
    && apt-get install -y --no-install-recommends cmake g++ make curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY . .
RUN sdk=$(scripts/fetch_juiz_sdk.sh "${JUIZ_VERSION}") \
    && target=$(basename "$sdk" | sed -E 's/^juiz_sdk-[^-]+-//') \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$sdk" \
         -DMAPPER_GMAPPING_JUIZ_SDK_VERSION="${JUIZ_VERSION}" \
    && cmake --build build --parallel \
    && ctest --test-dir build --output-on-failure \
    && mkdir -p "/plugins/$target" \
    && cp build/src/component/libmapper_gmapping.so build/src/component/libjuiz_sdk.so "/plugins/$target/"

FROM python:3.12-slim-bookworm AS base
ARG JUIZ_VERSION
RUN pip install --no-cache-dir "juiz-cli==${JUIZ_VERSION}"

# 配布と同じ .juizapp を作る(このアーキのプラグインだけ)
FROM base AS pack
WORKDIR /src
COPY . .
COPY --from=build /plugins /plugins
RUN scripts/pack.sh /plugins /dist

FROM base
RUN --mount=type=bind,from=pack,source=/dist,target=/dist \
    juiz install /dist/mapper_gmapping-*.juizapp
COPY docker/entrypoint.sh /usr/local/bin/mapper_gmapping-entrypoint
# 画面の「保存」の相対パスはここから解決される。地図を残すときは -v ./maps:/maps
WORKDIR /maps
VOLUME ["/maps"]
# つなぐ mobile_sim(juiz の HTTP)。mobile_sim からは、こちらが接続してきたアドレスの 8001 番にトピックが届く
ENV MOBILE_SIM_URL=http://mobile_sim:8000
EXPOSE 8001
ENTRYPOINT ["mapper_gmapping-entrypoint"]
