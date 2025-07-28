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

# Ensure GoogleTest test discovery macros are in scope.
#   Workaround for https://gitlab.kitware.com/cmake/cmake/-/issues/25477
include(GoogleTest)
include(doctest)

# Compile main function that supports GTest, FuzzTest and Benchmark.
add_library(test_main "test_main.cc")
target_link_libraries(
  test_main benchmark fuzztest::fuzztest fuzztest::init_fuzztest gmock gtest)

# Add a doctest based test file and auto-discover its tests.
function(add_doctest name source)
  add_executable(${name} ${source})
  doctest_discover_tests(${name})
endfunction()

# Add a gtest based test file and auto-discover its tests.
function(add_gtest name source)
  add_executable(${name} ${source})
  target_link_libraries(${name} PRIVATE test_main s2)
  gtest_discover_tests(${name} ${name})
endfunction()

# Enable test discovery.
enable_testing()

add_doctest(wilson_test wilson_test.cc)
target_link_libraries(wilson_test s2)

add_gtest(rasterize_test rasterize_test.cc)
add_gtest(fixed_test fixed_test.cc)
