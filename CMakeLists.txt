# Copyright 2024 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

# Prefer clang when present.
find_program(CMAKE_C_COMPILER NAMES   $ENV{CC}  clang   gcc PATHS ENV PATH NO_DEFAULT_PATH)
find_program(CMAKE_CXX_COMPILER NAMES $ENV{CXX} clang++ g++ PATHS ENV PATH NO_DEFAULT_PATH)

project(Wilson LANGUAGES C CXX)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(MACOSX TRUE)
endif()

# Prefer to link with static libraries when possible.
set(CMAKE_FIND_LIBRARY_SUFFIXES .a ${CMAKE_FIND_LIBRARY_SUFFIXES})

# Allow include() to find things in cmake/
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/third_party/cmake")

if (EMSCRIPTEN)
  # Enable LTO
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

  # Workaround for linking error when LTO is enabled.
  add_link_options("-Wl,-u,htons" "-Wl,-u,htonl")
endif()

set(ABSL_PROPAGATE_CXX_STD on)
set(CMAKE_CXX_EXTENSIONS on)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Include third party dependencies.
include(CPM)
add_subdirectory(third_party)

# Enable test discovery, this should come after third-party so we don't execute
# all of _their_ tests as well.
enable_testing()

# Default compile options.
add_compile_options(-Wall -Wextra -Wno-comment -Wno-unused-parameter -Wno-sign-compare -Wno-deprecated-declarations -fpermissive)

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  # To silence warnings in blend2d about gnu extensions.
  add_compile_options("-Wno-gnu-anonymous-struct")
  add_compile_options("-Wno-nested-anon-types")
  add_compile_options("-Wno-gcc-compat")

  if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    # Enable thread annotation checking in clang.
    add_compile_options("-Wthread-safety")
  endif()
endif()

# Including CMAKE_BINARY_DIR/inc here lets us see generated headers too.
include_directories(inc)
include_directories(third_party)
include_directories("${CMAKE_BINARY_DIR}/inc/")

# Configure any compiler flags needed for fuzzing.
fuzztest_setup_fuzzing_flags()

# Typical installation location for MacPorts.
if (MACOSX)
  include_directories(/opt/local/include)
  link_directories(/opt/local/lib)
endif()

add_subdirectory(dat)
add_subdirectory(src)
add_subdirectory(test)
