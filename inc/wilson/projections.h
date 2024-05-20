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
#include "wilson/projection.h"

#include "wilson/proj/cubic.h"
#include "wilson/proj/equirectangular.h"
#include "wilson/proj/gnomonic.h"
#include "wilson/proj/orthographic.h"

ABSL_FLAG(bool, experimental_projections, false, "Enable experimental projections");

namespace w {

struct ProjectionInfo {
  using MakeProjectionFn = absl::AnyInvocable <
      std::unique_ptr<IProjection>(Quaternion rotation, double scale) const>;

  template <typename T>
  static ProjectionInfo Create(  //
      absl::string_view name, bool experimental = false) {

    const auto Factory = [](Quaternion rotation, double scale) {
      auto projection = std::make_unique<T>();
      projection->set_rotation(rotation);
      projection->set_scale(scale);
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
  ProjectionInfo::Create<Cubic>("Cubic", false),
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
