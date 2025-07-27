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

# Enable test discovery.
enable_testing()

add_executable(wilson_test wilson_test.cc)
target_link_libraries(wilson_test s2)

add_executable(rasterize_test rasterize_test.cc)
target_link_libraries(rasterize_test PRIVATE gmock gtest s2)
link_fuzztest(rasterize_test)
gtest_discover_tests(rasterize_test rasterize_test)

add_executable(fixed_test fixed_test.cc)
target_link_libraries(fixed_test PRIVATE gmock gtest s2)
link_fuzztest(fixed_test)
gtest_discover_tests(fixed_test fixed_test)

include(doctest)
doctest_discover_tests(wilson_test)
