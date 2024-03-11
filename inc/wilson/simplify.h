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

#include "wilson/r2shape.h"

#include <vector>

// Implementation of a very simple polgon simplification algorithm.  This just
// discards vertices that aren't at least tolerance pixels away from the
// previous vertex, meaning they don't contribute meaningfully to the drawn
// geometry.
//
// Expects geometry already in screen space so that we can reason about
// distances in pixels to achieve results that look good when drawn to the
// screen.

namespace w {

static inline R2Shape Simplify(const R2Shape& shape, double tolerance=1, R2Shape out={}) {
    using std::swap;

    out.clear();
    for (int chain=0; chain < shape.nchains(); ++chain) {
        absl::Span<const R2Point> points = shape.chain_vertices(chain);
        if (points.empty()) {
            continue;
        }

        int last = 0;
        out.Append(points[0]);
        for (int i=1; i < points.size(); ++i) {
            if ((points[i]-points[last]).Norm2() >= tolerance*tolerance) {
                out.Append(points[i]);
                last = i;
            }
        }
        out.EndChain();
    }
    return out;
}

} // namespace w
