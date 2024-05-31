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

#include "wilson/geometry/r2shape.h"

#include <vector>

namespace w {

// Implementation of a very simple polgon simplification algorithm.  This just
// discards vertices that aren't at least sqrt(max_sq_error) pixels away from
// the previous vertex, meaning they don't contribute meaningfully to the drawn
// geometry.
//
// Expects geometry already in screen space so that we can reason about
// distances in pixels to achieve results that look good when drawn to the
// screen.
//
// max_sq_error is the maximum -squared- error between the true projection of
// the shape's edges and the edges in screen space, in pixels.
//
// Any existing path information is erased.
static inline void Simplify(absl::Nonnull<R2Shape *> out,
    const R2Shape& shape, double max_sq_error=1) {
    out->Clear();
    for (int chain=0; chain < shape.nchains(); ++chain) {
        absl::Span<const R2Point> points = shape.chain_vertices(chain);
        if (points.empty()) {
            continue;
        }

        int last = 0;
        out->Append(points[0]);
        for (int i=1; i < points.size(); ++i) {
            if ((points[i]-points[last]).Norm2() >= max_sq_error) {
                out->Append(points[i]);
                last = i;
            }
        }
        out->Break();
    }
}

} // namespace w
