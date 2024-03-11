#pragma once

// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "imgui/imgui.h"

#include "wilson/graphics/pixel.h"

namespace w {

constexpr static ImGuiColorEditFlags kDefaultColorFlags = //
  ImGuiColorEditFlags_NoInputs |
  ImGuiColorEditFlags_NoLabel  |
  ImGuiColorEditFlags_Uint8;

// Convenience methods for selecting colors using built-in pixel types.
void ChooseColor(const char* label, pixel& color, ImGuiColorEditFlags flags) {
  // ColorEdit4 needs four floats to read/write so we need to convert from our
  // pixel format to float, then back after the color is picked.
  float alpha = color.a/255.0f;
  ImVec4 colorf(
    (color.r/255.0f)/alpha,
    (color.g/255.0f)/alpha,
    (color.b/255.0f)/alpha,
    alpha);

  ImGui::ColorEdit4(label, (float*)&colorf, flags);
  color = pixel(colorf.x*255.0f, colorf.y*255.0f, colorf.z*255.0f, colorf.w*255.0f);
}

void ChooseColor(const char* label, pixel& pixel) {
  ChooseColor(label, pixel, kDefaultColorFlags);
}


}  // namespace w
