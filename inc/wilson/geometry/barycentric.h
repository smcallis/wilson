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

#include "s2/r2.h"

namespace w {

struct BaryCoord {
  double u,v,w;

  // Evaluates the barycentric coordinates in terms of three new points
  // representing a triangle ABC.  Since barycentric coordinates are invariant
  // under projection, this can be used to e.g. convert a point in a screen
  // space triangle to a point in a world-space triangle.
  template <typename Pnt>
  Pnt Evaluate(Pnt a, Pnt b, Pnt c) const {
    return u*a + v*b + w*c;
  }

  // Returns true if the point is inside the generating triangle.
  bool inside() const {
    return u >= 0 && v >= 0 && w >= 0;
  }
};

template <typename Pnt>
class Barycentric {
public:
  Barycentric() = default;

  // Prepares to compute barycentric coordinates relative to ABC
  Barycentric(Pnt a, Pnt b, Pnt c) : a_(a) {
    // Reference all the points relative to A.
    ba_ = b-a;
    ca_ = c-a;

    // Pre-compute dot products that don't change with the point.
    d00_ = ba_.DotProd(ba_);
    d01_ = ba_.DotProd(ca_);
    d11_ = ca_.DotProd(ca_);
  }

  // Convert a point into barycentric coordinates.
  BaryCoord ConvertPoint(Pnt pnt) const {
    Pnt pa = pnt-a_;
    double d20 = pa.DotProd(ba_);
    double d21 = pa.DotProd(ca_);

    double denom = d00_*d11_ - d01_*d01_;
    double v = (d11_*d20 - d01_*d21)/denom;
    double w = (d00_*d21 - d01_*d20)/denom;
    return {1 - v - w, v, w};
  }

private:
  Pnt a_;
  Pnt ba_, ca_;
  double d00_,d01_,d11_;
};

} // namespace w
