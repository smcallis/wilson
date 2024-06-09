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

// A class representing a plane in three dimensions.  A plane can be defined by
// any three points on it, and three points always define a unique plane.  We're
// only interested in planes intersecting the unit sphere, so we restrict our
// points to unit vectors.
//
// The plane is defined by three points on the surface of the sphere.  This
// representation has the advantage of defining a plane such that all three of
// the points on the sphere are -exactly- on the plane, and we can implement
// an exact Sign() function using Orient3D predicate logic.
//
// The downside is that other features of the plane such as its normal vector
// and offset from the origin are derived from the three defining points and
// thus are inexact.
struct S2Plane {
  constexpr S2Plane() = default;

  // Constructs a plane from three points on the sphere.
  //
  // The points must not be co-linear.
  constexpr S2Plane(const S2Point& u, const S2Point& v, const S2Point& w)
      : u_(u), v_(v), w_(w) {
    DCHECK_NE(u, v);
    DCHECK_NE(u, w);
    DCHECK_NE(v, w);
    DCHECK(S2::IsUnitLength(u));
    DCHECK(S2::IsUnitLength(v));
    DCHECK(S2::IsUnitLength(w));
  }

  // Constructs a plane from two points and the origin.
  //
  // The resulting plane will contain the origin, i.e. plane.Sign({0,0,0}) == 0.
  static constexpr S2Plane GreatPlane(const S2Point& u, const S2Point& v) {
    return S2Plane(u, v, {0, 0, 0});
  }

  // Constructs a plane from a non-zero sub-center point (a scaled normal).
  //
  // The length of the normal determines the offset of the plane from the origin
  // (in the range [0, 1]), and the orientation determines the direction.
  //
  // The basis points for the plane are computed as closely as possible, but
  // the final plane may not exactly contain the sub-center point.
  static S2Plane FromSubcenter(const S2Point& center) {
    DCHECK_GT(center.Norm2(), 0);
    DCHECK_LT(center.Norm2(), 1 + 5*DBL_EPSILON);

    const S2Point u = center.Ortho();
    const S2Point v = S2::RobustCrossProd(center.Normalize(), u).Normalize();
    const S2Point w = -((u + v)/2).Normalize();

    const double scale = std::sqrt(1 - std::min(1.0, center.Norm2()));
    return S2Plane(
      (scale*u + center).Normalize(),
      (scale*v + center).Normalize(),
      (scale*w + center).Normalize());
  }

  // Constructs a plane through the origin from a normal vector.
  static S2Plane FromNormal(const S2Point& normal) {
    DCHECK(S2::IsUnitLength(normal));

    const S2Point u = normal.Ortho();
    const S2Point v = S2::RobustCrossProd(normal, u).Normalize();
    const S2Point w = -((u + v)/2).Normalize();

    return S2Plane(u, v, w);
  }

  // Returns the points forming the plane.
  const S2Point& u() const { return u_; }
  const S2Point& v() const { return v_; }
  const S2Point& w() const { return w_; }

  // Returns the sign of the superior side of the plane.  This is the side of
  // the plane which a larger portion of the unit sphere occupies, or
  // equivalently, the side of the plane the origin lies on.
  int SuperiorSign() const {
    if (Sign({0, 0, 0}) >= 0) {
      return +1;
    }
    return -1;
  }

  // Returns an -approximate- normal vector for the plane.
  S2Point Normal() const {
    const S2Point e0 = (w_ - v_).Normalize();
    const S2Point e1 = (u_ - v_).Normalize();
    return S2::RobustCrossProd(e0, e1).Normalize();
  }

  // Returns an -approximate- distance to the origin for the plane.
  double Offset() const {
    return u().DotProd(Normal());
  }

  // Returns an -approximate- sub-center point for the plane.
  S2Point Subcenter() const {
    return Offset()*Normal();
  }

  // Returns the angle from v0 to v1 counter-clockwise in the plane.  The angle
  // is in the range [0, 2*PI] and is not computed exactly (though the
  // orientation between v0 and v1 is tested exactly).
  //
  // If v0 == v1 then the angle will be zero.
  double Angle(S2Point v0, S2Point v1) const {
    const S2Point center = Subcenter();

    // Shift vectors into the plane.
    v0 -= center;
    v1 -= center;

    double angle = v0.Angle(v1);
    if (s2pred::Sign(center, v0, v1) < 0) {
      angle = 2*M_PI - angle;
    }
    return angle;
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
    DCHECK_LE(point.Norm2() - 1, 5*DBL_EPSILON);

    // Shewchuck has a better error bound than this but it's a relative bound so
    // it requires us computing the magnitude of the result.  Instead we'll use
    // a slightly higher absolute error bound that we can compare directly.
    constexpr double kMaxAbserror = 2.132-14;

    double det = Determinant<double>(u_, v_, w_, point);
    if (std::fabs(det) > kMaxAbserror) {
      return det == 0 ? 0 : (det < 0 ? -1 : +1);
    }

    // Determinant was too small, recompute using exact arithmetic.
    auto uxf = Vector3_xf::Cast(u_);
    auto vxf = Vector3_xf::Cast(v_);
    auto wxf = Vector3_xf::Cast(w_);
    auto pxf = Vector3_xf::Cast(point);
    return Determinant<ExactFloat>(uxf, vxf, wxf, pxf).sgn();
  }

  // Returns a function that maps the range [0, 1] to a range of points [v0, v1]
  // in the plane.
  absl::AnyInvocable<S2Point(double)> Interpolate(S2Point v0, S2Point v1) const {
    // Compute normal and subcenter.
    const S2Point normal = Normal();
    const double  offset = u().DotProd(normal);
    const S2Point center = offset*normal;

    // Find scaled basis to compute points in.
    const double scale = std::sqrt(1 - std::min(1.0, offset*offset));
    const S2Point e0 = scale*(v0 - center).Normalize();
    const S2Point e1 = scale*S2::RobustCrossProd(normal, e0).Normalize();

    // Compute the total angle to sweep and the starting angle.
    const double sweep = Angle(v0, v1);
    const double angle0 = std::atan2(e0.y(), e0.x());

    return [v0, v1, e0, e1, angle0, sweep, center](double t) -> S2Point {
      DCHECK(0 <= t && t <= 1);
      if (t == 0) return v0;
      if (t == 1) return v1;

      const double angle = angle0 + sweep * t;
      return (std::cos(angle)*e0 + std::sin(angle)*e1 + center).Normalize();
    };
  }

 private:
  // Computes the 4x4 determinant:
  //
  //   | ax ay az 1 |
  //   | bx by bz 1 |
  //   | cx cy cz 1 |
  //   | dx dy dz 1 |
  //
  // Using the template parameter T, which may be ExactFloat.
 template <typename T>
 static T Determinant(                          //
     const Vector3<T>& a, const Vector3<T>& b,  //
     const Vector3<T>& c, const Vector3<T>& d) {
   const T adx = a.x() - d.x();
   const T bdx = b.x() - d.x();
   const T cdx = c.x() - d.x();

   const T ady = a.y() - d.y();
   const T bdy = b.y() - d.y();
   const T cdy = c.y() - d.y();

   const T adz = a.z() - d.z();
   const T bdz = b.z() - d.z();
   const T cdz = c.z() - d.z();

   const T bdxcdy = bdx * cdy;
   const T cdxbdy = cdx * bdy;

   const T cdxady = cdx * ady;
   const T adxcdy = adx * cdy;

   const T adxbdy = adx * bdy;
   const T bdxady = bdx * ady;

   return //
     adz * (bdxcdy - cdxbdy) +
     bdz * (cdxady - adxcdy) +
     cdz * (adxbdy - bdxady);
 }

  S2Point u_, v_, w_;
};

} // namespace w
