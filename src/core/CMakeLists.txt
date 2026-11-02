# GMapping を包む、juiz に依存しない部分
add_library(mapper_gmapping_core STATIC
  version.cpp)
target_include_directories(mapper_gmapping_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mapper_gmapping_core PUBLIC openslam_gmapping::gridfastslam)
target_compile_definitions(mapper_gmapping_core PRIVATE
  MAPPER_GMAPPING_VERSION="${PROJECT_VERSION}")
