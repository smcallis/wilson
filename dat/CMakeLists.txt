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

# Any embedded resources will be available by linking against this library
add_library(resources STATIC)

function(embed_resource input prefix getter)
  cmake_parse_arguments(ARGS "" "namespace" "" ${ARGN})

  # Rewrite the filename as something we can use for a symbol.
  string(REGEX REPLACE "\\." "_" symbol "${input}")

  # Generate a .cc file containing the embedded string and size.
  set(cc_file "${CMAKE_CURRENT_BINARY_DIR}/${prefix}.cc")
  add_custom_command(
    OUTPUT "${cc_file}"
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/genc.sh ${CMAKE_CURRENT_SOURCE_DIR}/${input} ${cc_file} ${symbol} ${ARGS_namespace}
    DEPENDS ${input} genc.sh
  )

  # Generate a .h file to include the data.
  set(h_file "${CMAKE_CURRENT_BINARY_DIR}/../inc/wilson/generated/${prefix}.h")
  add_custom_command(
    OUTPUT "${h_file}"
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/genh.sh ${h_file} ${symbol} ${getter} ${ARGS_namespace}
    DEPENDS genh.sh
  )

  # Add the sources to the library so they get built.
  target_sources(resources PUBLIC "${cc_file}" "${h_file}")
endfunction()

# Define files to embed here.
embed_resource("land.idx" "land" "LandIndex")
embed_resource("land_simplified.idx" "land_simplified" "LandSimplifiedIndex")
