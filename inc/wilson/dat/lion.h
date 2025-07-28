// Copyright 2025 Google LLC
// Author: smcallis@google.com (Sean McAllister)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once

#include <string>
#include <vector>
#include <utility>
#include <cstdio>

#include "absl/log/check.h"
#include "absl/types/span.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "absl/functional/function_ref.h"
#include "s2/r2.h"
#include "wilson/dat/lion_data.h"

namespace w {

using R2Loop = std::vector<R2Point>;
using LionVisitor = absl::FunctionRef<  //
    void(absl::string_view, absl::Span<const R2Loop>)>;

// Yields the loops of the lion graphic spans per color.
static inline void VisitLionLoops(LionVisitor emit) {
  R2Loop loop;
  std::vector<R2Loop> loops;

  for (absl::string_view line : lion) {
    if (line[0] == '#') {
      continue;
    }

    std::pair<std::string, std::string> pieces = absl::StrSplit(line, " | ");
    std::string color = pieces.first;

    for (const auto token : absl::StrSplit(pieces.second, ' ')) {
      if (token == "M" || token == "L") {
        if (token == "M") {
          if (!loop.empty()) {
            loops.emplace_back(std::move(loop));
          }
          loop.clear();
        }
        continue;
      }

      if (!token.empty()) {
        int x, y;
        int nscanned = sscanf(token.data(), "%d,%d", &x, &y);
        DCHECK_EQ(nscanned, 2);
        loop.emplace_back(2 * x, 2 * y);
      }
    }

    if (!loop.empty()) {
      loops.emplace_back(std::move(loop));
    }

    // Emit this set of loops with the current color.
    if (!loops.empty()) {
      emit(color, loops);
      loops.clear();
    }
  }
}

}  // namespace w
