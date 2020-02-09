# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
project(bioport
  VERSION 0.1
  DESCRIPTION "A platform for building interface devices")

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_C_FLAGS_DEBUG "-O0 -g")
include_directories("src/invensense/driver/include"
  "src/invensense/driver/eMPL-hal"
  "src/invensense/mpl"
  "src/invensense"
  "src/invensense/mpl-static/liblibmplmpu_m4_hardfp"
  "src/invensense/mllite")
add_compile_definitions(MPU6050 EMPL USE_DMP ZEPHYR)

find_library(MPL_LIB libmplmpu ./src/invensense)

FILE(GLOB app_sources "src/*.c" "src/invensense/eMPL-hal/*.c" "src/invensense/mllite/*.c" "src/*.cpp" "src/mpu6050/*.cpp" "src/I2Cdev/*.cpp")
target_sources(app PRIVATE
  ${app_sources}
  )

target_link_libraries(app LINK_PUBLIC ${MPL_LIB})
