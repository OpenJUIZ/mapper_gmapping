cmake_minimum_required(VERSION 3.16)
project(mapper_gmapping VERSION 0.1.0 LANGUAGES CXX)

# GMapping (OpenJUIZ/openslam_gmapping) で SLAM をする juiz のコンポーネント。計画は docs/plan.md
#
#   cmake -B build -DCMAKE_PREFIX_PATH=<juiz_sdk のアーカイブを展開したディレクトリ>
#   cmake --build build && ctest --test-dir build
#
# juiz_sdk が無いときは MAPPER_GMAPPING_BUILD_COMPONENT=OFF で core とテストだけをビルドできる。

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MAPPER_GMAPPING_BUILD_COMPONENT "juiz のコンポーネント(プラグイン)をビルドする。juiz_sdk が要る" ON)
set(MAPPER_GMAPPING_JUIZ_SDK_VERSION 0.1.0 CACHE STRING "使う juiz_sdk の版(juiz と完全に一致させる)")

if (MSVC)
  add_compile_options(/utf-8)
endif ()

# GMapping は静的ライブラリにしてプラグインに入れる(配るのはプラグインと libjuiz_sdk だけにする)
# (CMP0077: openslam_gmapping の option() がこの変数を上書きしないように)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(third_party/openslam_gmapping EXCLUDE_FROM_ALL)

add_subdirectory(src/core)

if (MAPPER_GMAPPING_BUILD_COMPONENT)
  find_package(juiz_sdk ${MAPPER_GMAPPING_JUIZ_SDK_VERSION} EXACT CONFIG REQUIRED)
  add_subdirectory(src/component)
endif ()

include(CTest)
if (BUILD_TESTING)
  add_subdirectory(tests)
endif ()
