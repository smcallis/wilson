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

#include <cmath>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"

namespace w {

// Format a number with engineering notation (nearest power of 10 divisible by
// three).  Proper SI units are printed if base units are given.
inline std::string engnot(
  double num, absl::string_view units="", absl::string_view fmt="%+.3f") {

  // Discard the sign bit and find the scaling for the value.
  double val = std::abs(num);
  int    pow = 0;
  std::string prefix;
  if (val >= 1) {
         if (val >= 1e+24) { prefix = "Y"; pow = +24; } // yotta
    else if (val >= 1e+21) { prefix = "Z"; pow = +21; } // zetta
    else if (val >= 1e+18) { prefix = "E"; pow = +18; } // exa
    else if (val >= 1e+15) { prefix = "P"; pow = +15; } // peta
    else if (val >= 1e+12) { prefix = "T"; pow = +12; } // tera
    else if (val >= 1e+09) { prefix = "G"; pow = +9;  } // giga
    else if (val >= 1e+06) { prefix = "M"; pow = +6;  } // mega
    else if (val >= 1e+03) { prefix = "k"; pow = +3;  } // kilo
  } else {
         if (val >= 1e-03) { prefix = "m"; pow = -3;  } // milli
    else if (val >= 1e-06) { prefix = "µ"; pow = -6;  } // micro
    else if (val >= 1e-09) { prefix = "n"; pow = -9;  } // nano
    else if (val >= 1e-12) { prefix = "p"; pow = -12; } // pico
    else if (val >= 1e-15) { prefix = "f"; pow = -15; } // femto
    else if (val >= 1e-18) { prefix = "a"; pow = -18; } // atto
    else if (val >= 1e-21) { prefix = "z"; pow = -21; } // zept
    else if (val >= 1e-24) { prefix = "y"; pow = -24; } // yocto
  }

  // Scale value and restore sign bit.
  val *= std::pow(10, -pow);
  val = std::copysign(val, num);

  auto format = absl::ParsedFormat<'f'>::New(fmt);
  if (format) {
    if (!units.empty()) {
      return absl::StrCat(
        absl::StrFormat(*format, val), " ", prefix, units);
    } else {
      if (pow != 0) {
        return absl::StrCat(
          absl::StrFormat(*format, val), absl::StrFormat("e%i", std::abs(pow)));
      } else {
        return absl::StrFormat(*format, val);
      }
    }
  }

  return "<badfmt>";
}

} // namespace w
