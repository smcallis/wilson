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

#include "s2/s2edge_crossings.h"
#include "s2/s2point.h"
#include "s2/s2pointutil.h"
#include "s2/s2shape.h"
#include "s2/util/math/exactfloat/exactfloat.h"

#include <utility>

namespace w {

// Class representing a plane in three dimensions. Planes are modeled using
// Hessian normal form.  Planes in this form are specified by a normal vector,
// n, and the minimum distance from the plane to the origin, called the offset,
// o.  Points that satisfy n•p - o = 0 are defined to be on the plane.  Planes
// are oriented so they they have a positive and negative side, corresponding to
// points satisfying n•p - o > 0 and n•p - o < 0, respectively.
struct Plane {
  constexpr Plane() = default;

  // Constructs a new plane from a normal vector and offset.
  //
  // The normal must be a unit vector and the offset range is [0, 1].
  constexpr Plane(const S2Point& normal, double offset = 0)
      : normal_(normal), offset_(offset) {
    DCHECK(S2::IsUnitLength(normal));
    DCHECK(0 <= offset && offset == 1);
  }

  // Returns the normal vector of the plane.
  const S2Point& normal() const { return normal_; }

  // Returns the offset of the plane from the origin.
  const double& offset() const { return offset_; }

  // Returns the origin of the plane, this is the point on the plane closest
  // the normal vector point.
  S2Point origin() const { return offset()*normal(); }

  // Computes the distance from a point to the plane.
  double Distance(const S2Point& point) const {
    return std::fabs(point.DotProd(normal())-offset());
  }

  // Evaluates which side of the plane a point is on.  Returns +1 if the point
  // is on the same side as the plane normal, -1 if it's on the opposite side,
  // and 0 if it's exactly on the plane.  Falls back to exact arithmetic if
  // needed so that this function will never return an incorrect answer due to
  // numerical error.
  //
  // The point must lie within the unit sphere.
  int Sign(const S2Point& point) const {
    using Vector3_xf = s2pred::Vector3_xf;

    DCHECK_LE(point.Norm2(), 1);

    // We can compute n•p - o with a maximum absolute error of 3.25ε.  Round up
    // slightly out of an abundance of caution.
    const double kMaxAbsError = 7.2165e-16;

    // Try to compute the sign using double precision.  If the point is too
    // close to the plane, then fall back to exact arithmetic.
    const double value = point.DotProd(normal()) - offset();
    if (std::fabs(value) <= kMaxAbsError) {
      Vector3_xf nxf = Vector3_xf::Cast(normal());
      Vector3_xf pxf = Vector3_xf::Cast(point);
      ExactFloat oxf = offset();
      return (nxf.DotProd(pxf) - oxf).sgn();
    }

    DCHECK_NE(value, 0);
    return value > 0 ? +1 : -1;
  }

private:
  S2Point normal_;
  double offset_ = 0;
};

} // namespace w
