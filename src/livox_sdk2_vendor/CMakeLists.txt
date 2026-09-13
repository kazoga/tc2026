cmake_minimum_required(VERSION 3.14)
project(livox_sdk2_vendor)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 上流のサンプル実行ファイルは配布せず、SDK と公開ヘッダーだけを構築する。
add_subdirectory(third_party/Livox-SDK2/sdk_core)
# 上流 comm/define.h の std::uint*_t に必要なヘッダーを GCC 13 でも保証する。
target_compile_options(livox_lidar_sdk_shared PRIVATE -include cstdint)
target_compile_options(livox_lidar_sdk_static PRIVATE -include cstdint)
target_link_libraries(livox_lidar_sdk_shared PRIVATE Threads::Threads)
target_link_libraries(livox_lidar_sdk_static PRIVATE Threads::Threads)
ament_export_include_directories(include)
ament_export_libraries(livox_lidar_sdk_shared)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
