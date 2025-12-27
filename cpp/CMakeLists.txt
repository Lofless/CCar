cmake_minimum_required(VERSION 3.16)

project(
  mujoco_mpc
  VERSION 0.1.0
  DESCRIPTION "MuJoCo predictive control"
)

enable_language(C)
enable_language(CXX)
if(APPLE)
  enable_language(OBJC)
  enable_language(OBJCXX)
endif()

cmake_policy(SET CMP0069 NEW)
cmake_policy(SET CMP0072 NEW)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(MpcOptions)
include(MujocoMacOS)
include(FindOrFetch)

find_package(Python REQUIRED)
find_package(Threads REQUIRED)

set(MUJOCO_BUILD_EXAMPLES OFF)
set(MUJOCO_BUILD_SIMULATE ON)
set(MUJOCO_BUILD_TESTS OFF)

set(MUJOCO_REV
    088079eff0450e32b98ee743141780ed68307506
    CACHE STRING "Pinned MuJoCo revision"
)

findorfetch(
  USE_SYSTEM_PACKAGE OFF
  PACKAGE_NAME mujoco
  LIBRARY_NAME mujoco
  GIT_REPO https://github.com/google-deepmind/mujoco.git
  GIT_TAG ${MUJOCO_REV}
  TARGETS mujoco::mujoco mujoco::platform_ui_adapter
  EXCLUDE_FROM_ALL
)

set(ABSL_REF
    fb3621f4f897824c0dbe0615fa94543df6192f30
    CACHE STRING "Pinned abseil revision"
    FORCE
)

findorfetch(
  USE_SYSTEM_PACKAGE OFF
  PACKAGE_NAME absl
  LIBRARY_NAME abseil-cpp
  GIT_REPO https://github.com/abseil/abseil-cpp.git
  GIT_TAG ${ABSL_REF}
  TARGETS absl::core_headers
  EXCLUDE_FROM_ALL
)

add_definitions(-DSOURCE_DIR=\"${CMAKE_SOURCE_DIR}\")

add_subdirectory(mjpc)
