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

#include "s2/s2point.h"
#include "s2/s2shape.h"

#include <utility>

namespace w {

// Code for specifying and intersecting planes in three dimensions.
struct Plane {
  Plane() = default;
  Plane(const S2Point& normal, const S2Point& origin={})
    : normal_(normal), origin_(origin), zero_origin_(origin == S2Point()) {}

  S2Point origin() const { return origin_; }
  S2Point normal() const { return normal_; }

  // Evaluates which side of the plane the given point is on.  Returns +1 if the
  // point is on the same side as the plane normal, -1 if it's on the opposite
  // side, and 0 if it's exactly on the plane.
  int Sign(const S2Point& pnt) const {
    double val = (pnt-origin()).DotProd(normal());
    if (val == 0) {
      return 0;
    }
    return val > 0 ? +1 : -1;
  }

  // Clips an S2Shape edge to the positive side of this plane, restricted to the
  // surface of the unit sphere.  Returns true if any of the edge survived
  // clipping, false otherwise.
  //
  // The two vertices of an edge define a plane, which we can intersect with
  // this plane. The intersection of two non-parallel planes is always a line,
  // so we further restrict intersection to the points that lie on the unit
  // sphere, resulting in at most two points.
  //
  // The new vertices of the edge are promised to be clipped s.t. the
  // orientation of the edge as v0.CrossProd(v1) is unchanged.
  bool ClipEdgeOnSphere(S2Shape::Edge&) const;

private:
  S2Point normal_;
  S2Point origin_;
  bool zero_origin_;
};

inline bool Plane::ClipEdgeOnSphere(S2Shape::Edge& edge) const {
  // Test which side of the plane each vertex is on.
  const int sign0 = Sign(edge.v0);
  const int sign1 = Sign(edge.v1);

  // The plane containing the edge will always pass through the origin, if this
  // plane does as well, then we can simplify the logic required significantly.
  //
  if (zero_origin_) {
    // The plane must bisect the sphere, so any edge that doesn't cross it must
    // be entirely on one side or the other, or it would be longer than 180
    // degrees, which isn't allowed.  So check to see if the signs of the
    // vertices are the same.  If they are, keep or reject the edge depending
    // which side it's on.
    if ((sign0 < 0) == (sign1 < 0)) {
      return (sign0 >= 0);
    }

    // Different signs, the edge must cross the plane.  Just use cross products
    // to find the intersection points
    const S2Point v0 = edge.v0;
    const S2Point v1 = edge.v1;
    if (sign0 < 0) edge.v0 = normal().CrossProd(v0.CrossProd(v1)).Normalize();
    if (sign1 < 0) edge.v1 = normal().CrossProd(v1.CrossProd(v0)).Normalize();
    return true;
  }

  // Both vertices are on the positive side of the plane, there's nothing to do,
  // just keep the vertices as-is and signal success.
  if (sign0 >= 0 && sign1 >= 0) {
    return true;
  }

  // Compute the normal of the plane containing the edge.  Edges are always on
  // great circles, which means the plane goes through the origin and thus the
  // origin is always zero.
  const S2Point n0 = normal();
  const S2Point o0 = origin();
  const S2Point n1 = edge.v0.CrossProd(edge.v1).Normalize();

  // By the equation of a plane with normal N and origin O, any point P on the
  // plane satisfies N•(P-O) = 0.  Using that equation for both planes, we get
  // a system of two equations to solve for the line of intersection of the
  // planes.  Restricting that further to solutions on the unit sphere gives
  // us two (not necessarily antipodal) points.

  // The resulting line must be orthogonal to both planes, so the cross
  // product gives us the direction vector for it.
  const S2Point D = n0.CrossProd(n1).Normalize();

  // Compute the offset of the line segment.  The origin of the plane which the
  // edge lies in is always zero, so we can simplify.
  //   See: https://en.wikipedia.org/wiki/Plane%E2%80%93plane_intersection
  const double nn = n0.DotProd(n1);
  const double h0 = n0.DotProd(o0);
  const double den = 1/(1-nn*nn);
  const double c0 = h0*den;
  const double c1 = (-h0*nn)*den;

  const S2Point O = c0*n0 + c1*n1;

  // We find O as a point in the 2D space spanned by n0 and n1, which is a plane
  // perpendicular to both of the other planes. O must therefore be in all three
  // planes and is thus orthogonal to all three normals, indicating it's the
  // orthogonal projection of the origin onto the line of intersection.
  //
  // If we had to project more than the unit distance, then the line cannot
  // intersect the sphere and the edge is clipped.  Add a small epsilon to
  // avoid juddering as the line becomes tangent to the sphere.
  if (O.Norm2() > 1+1e-6) {
    return false;
  }

  // We have the parameterized equation O + t*D for the line of intersection.
  // We can intersect it with the unit sphere to find our points.
  //   See: https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
  //
  // Note that since O is perpendicular to all three planes, it's perpendicular
  // to the intersection direction D, and thus O.DotProd(D) == 0 so we can
  // ignore it.
  double disc = 1-O.Norm2();
  if (disc <= 0) {
    return false; // No intersection with the sphere, the edge is clipped.
  }

  disc = std::sqrt(disc);
  double t0 = +disc;
  double t1 = -disc;

  // Ensure the new points are the same orientation as the edge vertices.
  S2Point p0 = O + t0*D;
  S2Point p1 = O + t1*D;
  if (p0.CrossProd(p1).DotProd(n1) < 0) {
    std::swap(p0, p1);
  }

  // If both vertices are on the negative side, we need both clip points.
  if (sign0 < 0 && sign1 < 0) {
    // If the intersection points weren't in the small arc of the edge, then the
    // edge is occluded.
    S2Point ap = edge.v0.CrossProd(p0);
    S2Point pb = p0.CrossProd(edge.v1);
    if (n1.DotProd(ap) < 0 || pb.DotProd(ap) < 0) {
      return false;
    }

    edge.v0 = p0;
    edge.v1 = p1;
  } else {
    // Only one vertex is on the negative side, replace it with its
    // corresponding intersection point.
    if (sign0 < 0) edge.v0 = p0;
    if (sign1 < 0) edge.v1 = p1;
  }

  return true;
}

} // namespace w
