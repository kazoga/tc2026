cmake_minimum_required(VERSION 3.8)
project(ypspur_ros2)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# -----------------------------------------------------------------------------
# yp-spur (third_party submodule) + Issue #245 ワークアラウンドパッチ適用
# -----------------------------------------------------------------------------

set(YPSPUR_DIR         ${CMAKE_CURRENT_SOURCE_DIR}/third_party/yp-spur)
set(YPSPUR_BUILD_DIR   ${CMAKE_CURRENT_BINARY_DIR}/yp-spur-patched)
set(YPSPUR_PATCH       ${CMAKE_CURRENT_SOURCE_DIR}/third_party/patches/0001-fix-tcflush-kernel-6.x.patch)
get_filename_component(YPSPUR_GIT_CEILING_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../.. ABSOLUTE)

if(NOT EXISTS ${YPSPUR_DIR}/CMakeLists.txt)
  message(FATAL_ERROR
    "yp-spur submodule is not initialized at ${YPSPUR_DIR}.\n"
    "Run: git submodule update --init --recursive")
endif()

# third_party の submodule を直接変更せず、ビルド用コピーにパッチを適用する。
file(REMOVE_RECURSE ${YPSPUR_BUILD_DIR})
file(COPY ${YPSPUR_DIR}/ DESTINATION ${YPSPUR_BUILD_DIR}
  PATTERN ".git" EXCLUDE
)

# 冪等: --check が成功 (=未適用) ならパッチを当てる。
# 失敗時は reverse --check で適用済みか確認し、どちらでもなければ不整合として停止する。
execute_process(
  COMMAND ${CMAKE_COMMAND} -E env
    GIT_CEILING_DIRECTORIES=${YPSPUR_GIT_CEILING_DIR}
    git apply --check ${YPSPUR_PATCH}
  WORKING_DIRECTORY ${YPSPUR_BUILD_DIR}
  RESULT_VARIABLE _ypspur_patch_can_apply
  OUTPUT_QUIET ERROR_QUIET
)
if(_ypspur_patch_can_apply EQUAL 0)
  message(STATUS "Applying yp-spur patch (0001-fix-tcflush-kernel-6.x)")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env
      GIT_CEILING_DIRECTORIES=${YPSPUR_GIT_CEILING_DIR}
      git apply ${YPSPUR_PATCH}
    WORKING_DIRECTORY ${YPSPUR_BUILD_DIR}
    RESULT_VARIABLE _ypspur_patch_result
  )
  if(NOT _ypspur_patch_result EQUAL 0)
    message(FATAL_ERROR "Failed to apply yp-spur patch ${YPSPUR_PATCH}")
  endif()
else()
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env
      GIT_CEILING_DIRECTORIES=${YPSPUR_GIT_CEILING_DIR}
      git apply --reverse --check ${YPSPUR_PATCH}
    WORKING_DIRECTORY ${YPSPUR_BUILD_DIR}
    RESULT_VARIABLE _ypspur_patch_already_applied
    OUTPUT_QUIET ERROR_QUIET
  )
  if(_ypspur_patch_already_applied EQUAL 0)
    message(STATUS "yp-spur patch already applied")
  else()
    message(FATAL_ERROR "Failed to apply yp-spur patch ${YPSPUR_PATCH}")
  endif()
endif()

# yp-spur 本体をビルド・install ツリーに同居させる
add_subdirectory(${YPSPUR_BUILD_DIR} ${CMAKE_CURRENT_BINARY_DIR}/yp-spur-build)

# -----------------------------------------------------------------------------
# ypspur_ros2 ノード
# -----------------------------------------------------------------------------

add_executable(ypspur_node src/ypspur_node.cpp)
target_include_directories(ypspur_node PRIVATE
  ${YPSPUR_BUILD_DIR}/include
)
target_link_libraries(ypspur_node ypspur)
ament_target_dependencies(ypspur_node
  rclcpp
  geometry_msgs
  nav_msgs
)

install(TARGETS ypspur_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
