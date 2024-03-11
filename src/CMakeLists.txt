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

add_executable(
  wilson
  wilson.cc
  projection.cc
)

target_link_libraries(
  wilson
  absl::flags_parse
  resources
  imgui
  s2
  Blend2D::Blend2D
  z
)

# Slightly different names for the SDL dependency in emscripten.
if (NOT EMSCRIPTEN)
  target_link_libraries(wilson SDL2::SDL2)
else()
  target_link_libraries(wilson SDL2)
endif()

if (EMSCRIPTEN)
  set(CMAKE_EXECUTABLE_SUFFIX ".html")

  function(add_link_option target option)
    set_target_properties(${target} PROPERTIES LINK_FLAGS "${${target}_LINK_FLAGS} ${option}")
  endfunction()

  # Unused but maintained for posterity
  # function(emscripten_embed_file target file destination)
  #   get_filename_component(absolute_file ${file} ABSOLUTE)
  #   get_target_property(${target}_LINK_FLAGS ${target} LINK_FLAGS)
  #   if(NOT ${target}_LINK_FLAGS)
  #     set(${target}_LINK_FLAGS )
  #   endif()
  #   add_link_option(${target} "--embed-file ${absolute_file}@${destination}")
  # endfunction()

  # Allow a dynamic heap.
  add_link_option(wilson "-sALLOW_MEMORY_GROWTH")
endif()
