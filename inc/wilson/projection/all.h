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

#include "absl/flags/flag.h"

#include "wilson/projection/equirectangular.h"
#include "wilson/projection/gnomonic.h"
#include "wilson/projection/orthographic.h"
#include "wilson/projection/projection.h"

ABSL_FLAG(bool, experimental_projections, false, "Enable experimental projections");

namespace w {

// A struct containing metadata about a projection along with a factory function
// to create one.  This lets us store a list of available projections and
// iterate it to e.g. display in a gui.
struct ProjectionInfo {
  using MakeProjectionFn = absl::AnyInvocable <
      std::unique_ptr<IProjection>(Quaternion rotation, double scale) const>;

  template <typename T>
  static ProjectionInfo Create(  //
      absl::string_view name, bool experimental = false) {

    const auto Factory = [](Quaternion rotation, double scale) {
      auto projection = std::make_unique<T>();
      projection->SetRotation(rotation);
      projection->SetScale(scale);
      return projection;
    };

    return {name, Factory, experimental};
  }

  absl::string_view name;
  MakeProjectionFn factory;
  bool experimental = false;
};

// Configure projections here.
static const ProjectionInfo kProjectionList[] = {
  ProjectionInfo::Create<Equirectangular>("Equirectangular", false),
  ProjectionInfo::Create<Gnomonic>("Gnomonic", false),
  ProjectionInfo::Create<Orthographic>("Orthographic", false)
};

// Calls a function with each configured projection's ProjectionInfo entry.
static inline void EachProjection(
  absl::FunctionRef<void(const ProjectionInfo&)> visitor) {
  for (const auto& info : kProjectionList) {
    if (!info.experimental || absl::GetFlag(FLAGS_experimental_projections)) {
      visitor(info);
    }
  }
}

}  // namespace wilson
