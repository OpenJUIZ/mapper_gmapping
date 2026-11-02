# juiz のコンポーネント。.juizapp の plugins/<target>/ に、このライブラリと libjuiz_sdk を置く
add_library(mapper_gmapping SHARED mapper_gmapping_component.cpp)
target_link_libraries(mapper_gmapping PRIVATE mapper_gmapping_core juiz_sdk::juiz_sdk)
# rpath を $ORIGIN (macOS は @loader_path) だけにし、ビルドしたライブラリの隣に libjuiz_sdk を写す
juiz_sdk_bundle(mapper_gmapping)
