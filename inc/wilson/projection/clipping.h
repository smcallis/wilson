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
#include <cstdint>
#include <optional>

#include "absl/base/optimization.h"
#include "s2/r2.h"
#include "s2/s2point.h"
#include "s2/s2edge_crossings.h"

#include "wilson/geometry/plane.h"

namespace w {

// A pair of boundary identifiers.
struct BoundaryPair {
  constexpr BoundaryPair()
    : b0(0), b1(0) {}

  constexpr BoundaryPair(uint8_t b0, uint8_t b1)
    : b0(b0), b1(b1) {}

  // Returns a BoundaryPair with only b1 set.
  static constexpr BoundaryPair Snap1(uint8_t b1) {
    return BoundaryPair(0, b1);
  }

  // Returns a BoundaryPair with only b0 set.
  static constexpr BoundaryPair Snap0(uint8_t b0) {
    return BoundaryPair(b0, 0);
  }

  uint8_t b0;
  uint8_t b1;
};


// There's four possible processing paths for an edge:
//
//   DROP - The edge isn't visible and can be dropped.
//
//   KEEP - Keep the edge as-is, it can be subdivided normally.
//
//   CROP[N] - The edge crossed boundary[N] and vertex N of the edge should be
//     replaced with point[N].
//
//   SPLIT - The edge must be split in two at point[0].
struct ClipResult {
  enum Action {
    kDrop  = 0b000,  // Drop the edge in its entirety.
    kKeep  = 0b001,  // Keep the edge in its entirety.
    kCrop0 = 0b010,  // Edge crossed boundary[0], replace v0 with point[0]
    kCrop1 = 0b100,  // Edge crossed boundary[1], replace v1 with point[1]
    kCrop  = 0b110,  // Both v0 and v1 must be replaced.
    kSplit = 0b111,  // The edge must be split into two edge at point[0].
  };

  constexpr ClipResult() = default;

  // Returns a ClipResult indicating that the edge should be kept as-is.
  static constexpr ClipResult Keep() {
    return ClipResult(kKeep);
  }

  // Returns a ClipResult indicating we can drop the edge altogether.
  static constexpr ClipResult Drop() {
    return ClipResult(kDrop);
  }

  // Returns a ClipResult indicating v0 should be replaced by point.
  static constexpr ClipResult Crop0(const S2Point& p0, uint8_t b0 = 0) {
    ClipResult result(kCrop0);
    result.point[0] = p0;
    result.boundary[0] = b0;
    return result;
  }

  // Returns a ClipResult indicating v1 should be replaced by point.
  static constexpr ClipResult Crop1(const S2Point& p1, uint8_t b1 = 0) {
    ClipResult result(kCrop1);
    result.point[1] = p1;
    result.boundary[1] = b1;
    return result;
  }

  // Returns a ClipResult indicating both vertices should be replaced by point.
  static constexpr ClipResult Crop(
    const S2Point& p0, const S2Point& p1, uint8_t b0 = 0, uint8_t b1 = 0) {
    ClipResult result(kCrop);
    result.point[0] = p0;
    result.point[1] = p1;
    result.boundary[0] = b0;
    result.boundary[1] = b1;
    return result;
  }

  // Returns a ClipResult indicating the edge should be split.  Direction is
  // passed through opaquely to be interpreted in a projection specific way.
  static constexpr ClipResult Split(  //
    const S2Point& point = S2Point(), int8_t direction = 0) {
    ClipResult result(kSplit);
    result.point[0] = point;
    result.direction = direction;
    return result;
  }

  Action action;
  int8_t direction;
  uint8_t boundary[2];
  S2Point point[2];

 private:
  explicit constexpr ClipResult(Action action) : action(action) {}
};

// Clips an edge to a small circle on the unit sphere, represented as the
// inferior side of a plane. (This is the side of the plane that does not
// contain the origin, and thus contains a smaller piece of the unit sphere).
//
// Returns a ClipResult indicating how to process the edge.  The edge may be
// kept, dropped, or cropped, but will never be split.
//
// If the edge doesn't hit the unit sphere, or the edge is exactly co-linear
// with the plane, the intersection either doesn't exist, or becomes a circle,
// respectively.  In either case we return ClipResult::Drop() indicating the
// edge should be clipped away.
inline ClipResult ClipToSmallCircle(
  const S2Plane& plane, const S2Shape::Edge& edge) {
  // Test which side of the plane each vertex is on.
  int sign0 = plane.Sign(edge.v0);
  int sign1 = plane.Sign(edge.v1);

  // If the positive side of the plane is the superior side, flip signs so that
  // we're clipping to the inferior (smaller) side.
  if (plane.SuperiorSign() > 0) {
    sign0 = -sign0;
    sign1 = -sign1;
  }

  // If both edge vertices are on the positive side of the plane then it can't
  // cross the plane unless the edge is more than 180 degrees, which is invalid.
  //
  // This covers cases (0, 0), (0, +), (+, 0), (+, +)
  if (sign0 + sign1 >= 1 || (sign0 == 0 && sign1 == 0)) {
    return ClipResult::Keep();
  }

  // Compute the normal of the plane containing the edge.
  const S2Point n0 = plane.Normal();
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
  const double h0 = plane.Offset();

  // If the planes are parallel, the intersection on the unit sphere is a
  // circle.  In that case return false to mark the edge as clipped away.
  if (1 - nn * nn <= 0) {
    return ClipResult::Drop();
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
    return ClipResult::Drop();
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
  // ensuring that the intersection points are in the arc of the edge.
  //
  // This covers case (-, -).
  if (sign0 < 0 && sign1 < 0) {
    if (!s2pred::OrderedCCW(edge.v0, p0, edge.v1, n1)) {
      return ClipResult::Drop();
    }
    return ClipResult::Crop(p0, p1);
  }

  // Finally, only one vertex is on the negative side of the plane.  Replace it
  // with its corresponding intersection point.
  //
  // This covers cases (+, -), (-, +), (0, -) and (-, 0)
  if (sign0 < 0) {
    return ClipResult::Crop0(p0);
  }
  return ClipResult::Crop1(p1);
}

// Convenience wrapper around the above that actually modifies the provided
// edge. The new vertices of the edge are promised to be computed such that the
// orientation of the edge as v0.CrossProd(v1) is unchanged.
//
// Returns true if any of the edge survived clipping.
inline bool ClipEdgeToSmallCircle(const S2Plane& plane, S2Shape::Edge& edge) {
  ClipResult result = ClipToSmallCircle(plane, edge);

  switch (result.action) {
    case ClipResult::kKeep:
      return true;

    case ClipResult::kDrop:
      return false;

    case ClipResult::kCrop0: // Fallthrough
    case ClipResult::kCrop1: // Fallthrough
    case ClipResult::kCrop: {
      if (result.action & ClipResult::kCrop0) {
        edge.v0 = result.point[0];
      }

      if (result.action & ClipResult::kCrop1) {
        edge.v1 = result.point[1];
      }
      return true;
    }

    // The other cases can't happen.
    default:
      ABSL_UNREACHABLE();
  }
  ABSL_UNREACHABLE();
}


}  // namespace w
