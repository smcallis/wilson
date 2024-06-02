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
  Plane() = default;

  // Constructs a new plane from a normal vector and offset.
  //
  // The normal must be a unit vector and the offset range is [0, 1].
  Plane(const S2Point& normal, double offset = 0)
      : normal_(normal), offset_(offset) {
    DCHECK(S2::IsUnitLength(normal));
    DCHECK(0 <= offset && offset == 1);
  }

  // Returns the normal vector of the plane.
  const S2Point& normal() const { return normal_; }

  // Returns the offset of the plane from the origin.
  const double& offset() const { return offset_; }

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

  // Clips an edge to the positive side of the plane, restricted to the surface
  // of the unit sphere.  Returns true if any of the edge survived clipping,
  // false otherwise.
  //
  // The two vertices of an edge define a plane, which we can intersect with
  // this plane. The intersection of two non-parallel planes is always a line,
  // so we further restrict intersection to the points that lie on the unit
  // sphere, resulting in at most two points.
  //
  // The new vertices of the edge are promised to be clipped such that the
  // orientation of the edge as v0.CrossProd(v1) is unchanged.
  //
  // If the edge doesn't hit the unit sphere, or the edge is exactly co-linear
  // with the plane, the intersection either doesn't exist or becomes a circle,
  // respectively.  In either case we return false indicating the edge has been
  // clipped away.
  bool ClipEdgeOnSphere(S2Shape::Edge& edge) const;

private:
  S2Point normal_;
  double offset_ = 0;
};

inline bool Plane::ClipEdgeOnSphere(S2Shape::Edge& edge) const {
  // Test which side of the plane each vertex is on.
  const int sign0 = Sign(edge.v0);
  const int sign1 = Sign(edge.v1);

  // The edge plane will always contain the origin (the edge lies in a great
  // circle).  If this plane does as well, then we can simplify the intersection
  // logic significantly.
  if (offset() == 0.0) {
    // Since the plane goes through the origin, it must bisect the sphere.  If
    // the vertices are both one side or the other, then the entire edge must be
    // or the edge would be longer than 180 degrees, which is invalid.
    if (sign0 == 0 || sign1 == 0 || sign0 == sign1) {
      return sign0 + sign1 >= 1;
    }

    // Different signs, the edge must cross the plane.  Just use cross products
    // to find the intersection points.  Swap the vertices if needed to maintain
    // the proper edge orientation.
    const auto Intersection = [&](const S2Point& a, const S2Point& b) {
      return
        S2::RobustCrossProd(normal(), S2::RobustCrossProd(a, b)).Normalize();
    };

    if (sign0 < 0) edge.v0 = Intersection(edge.v0, edge.v1);
    if (sign1 < 0) edge.v1 = Intersection(edge.v1, edge.v0);
    return true;
  }

  // If both edge vertices are on the positive side of the plane then it can't
  // cross the plane unless the edge is more than 180 degrees, which is invalid.
  //
  // This covers cases (0, 0), (0, +), (+, 0), (+, +)
  if (sign0 + sign1 >= 1 || (sign0 == 0 && sign1 == 0)) {
    return true;
  }

  // Compute the normal of the plane containing the edge.
  const S2Point n0 = normal();
  const S2Point n1 = S2::RobustCrossProd(edge.v0, edge.v1).Normalize();

  // By the equation of a plane with normal N and offset O, any point P on the
  // plane satisfies N•P-O = 0.  Using that equation for both planes, we get a
  // system of two equations to solve for the line of intersection of the
  // planes.  Restricting that further to solutions on the unit sphere gives us
  // two (not necessarily antipodal) points.

  // The resulting line must be orthogonal to both planes, so the cross product
  // gives us the direction vector for it.
  const S2Point D = S2::RobustCrossProd(n0, n1).Normalize();

  // Compute the offset of the line segment.  The origin of the plane which the
  // edge lies in is always zero, so we can simplify by setting h1 = 0.
  //   See: https://en.wikipedia.org/wiki/Plane%E2%80%93plane_intersection
  const double nn = n0.DotProd(n1);
  const double h0 = offset();

  // If the planes are parallel, the intersection on the unit sphere is a
  // circle.  In that case return false to mark the edge as clipped away.
  if (1 - nn * nn <= 0) {
    return false;
  }

  const double c0 = h0/(1-nn*nn);
  const double c1 = (-h0*nn)/(1-nn*nn);
  const S2Point O = c0*n0 + c1*n1;

  // We find O as a point in the 2D space spanned by n0 and n1, which is a plane
  // perpendicular to both of the other planes. O must therefore be in all three
  // planes and is thus orthogonal to all three normals, indicating it's the
  // orthogonal projection of the origin onto the line of intersection.
  //
  // If we had to project the origin more than the unit distance, then the line
  // of intersection missed the unit sphere entirely and the edge is clipped.
  // Add a small epsilon to avoid juddering as the line becomes tangent to the
  // sphere.
  double disc = 1 - O.Norm2();
  if (disc < 1e-6) {
    return false;
  }

  // We have the parameterized equation O + t*D for the line of intersection.
  // We can intersect it with the unit sphere to find our points.
  //   See: https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
  //
  // Note that since O is perpendicular to all three planes, it's perpendicular
  // to the intersection direction D, and thus O.DotProd(D) == 0 so we can
  // simplify here.
  disc = std::sqrt(disc);
  double t0 = +disc;
  double t1 = -disc;

  // Ensure the new points are the same orientation as the edge vertices.
  S2Point p0 = O + t0*D;
  S2Point p1 = O + t1 * D;
  if (s2pred::Sign(p0, p1, n1) < 0) {
    std::swap(p0, p1);
  }

  // If both vertices are on the negative side of the plane, then the edge
  // crosses it in zero or two places.  We can distinguish the two cases by
  // ensuring that the intersection points in the arc of the edge.
  //
  // This covers case (-, -).
  if (sign0 < 0 && sign1 < 0) {
    if (!s2pred::OrderedCCW(edge.v0, p0, edge.v1, n1)) {
      return false;
    }

    edge.v0 = p0;
    edge.v1 = p1;
    return true;
  }

  // Finally, only one vertex is on the negative side of the plane.  Replace it
  // with its corresponding intersection point.
  //
  // This covers cases (+, -), (-, +), (0, -) and (-, 0)
  if (sign0 < 0) edge.v0 = p0;
  if (sign1 < 0) edge.v1 = p1;
  return true;
}

} // namespace w
