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

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

#include "absl/base/optimization.h"
#include "absl/functional/bind_front.h"
#include "blend2d.h"
#include "s2/s2cap.h"
#include "s2/s2cell.h"
#include "s2/s2edge_crossings.h"
#include "s2/s2point.h"
#include "s2/s2latlng.h"
#include "s2/s2edge_distances.h"
#include "s2/s2predicates.h"
#include "s2/s2wedge_relations.h"

#include "wilson/plane.h"
#include "wilson/projection.h"
#include "wilson/quaternion.h"
#include "wilson/r2shape.h"
#include "wilson/proj/clipping.h"

namespace w {

struct Equirectangular final : Projection<Equirectangular> {
  // Import this explicitly so we can see it with our other Project() overloads.
  using Projection::Project;

  // Equirectangular requires a 2:1 aspect ratio fit into the visible region.
  static constexpr double kAspectRatio = 2.0;

  S2Cap Viewcap() const override {
    return S2Cap::Full();
  }

  void UpdateTransforms() override final {
    // The equirectangular projection is always a rectangle with 2:1 aspect
    // ratio on screen.
    //
    // We set our kAspectRatio constant to 2 so the projection base class always
    // sets unit_scale() so that a 2:1 unit rectangle is inside of the window
    // region in unit space, regardless of the window aspect ratio.
    //
    // This is equivalent to making the height unit-scale and the width twice
    // that.  We further adjust the dimensions by the current scale() value to
    // accommodate zooming.
    double hh = Scale()*UnitScale();
    double ww = 2*hh;

    R2Point center = {Width()/2, Height()/2};
    outline_ = region2(
      R2Point(-ww/2, -hh/2) + center,
      R2Point(+ww/2, +hh/2) + center
    );

    // Normal to the plane containing the antimeridian and a plane perpendicular
    // to it where the antimeridian cut is on the positive side.
    amnorm_ = Unrotate({0, 1, 0});
    amperp_ = Unrotate({-1, 0, 0});
    ampole_ = S2::RobustCrossProd(amnorm_, amperp_).Normalize();

    // And an normal vector for the plane of the equator.
    eqnorm_ = Unrotate({0, 0, 1});
  }

  // Populates a path representing the outline of the sphere on screen.  May
  // encompass the entire screen.
  void MakeOutline(absl::Nonnull<ChainSink*> out) const override {
    out->Break();
    out->Append(R2Point(outline_.lo().x(), outline_.hi().y()));
    out->Append(R2Point(outline_.hi().x(), outline_.hi().y()));
    out->Append(R2Point(outline_.hi().x(), outline_.lo().y()));
    out->Append(R2Point(outline_.lo().x(), outline_.lo().y()));
    out->Close();
  }

  // Populates a path with a graticule with lines of latitude and longitude.
  void MakeGraticule(absl::Nonnull<ChainSink*> out) const override {
    // generate_graticule(path);
  }

  // Projects a point from world space to unit space.
  R2Point WorldToUnit(const S2Point& p) const override {
    return Scale() * R2Point(
      std::atan2(p.y(), p.x()) / M_PI,
      -std::asin(p.z()) / M_PI
    );
  }

  // Attempts to convert a point from unit space back to world space.  If
  // `nearest` is true then the closest point on the projection is returned,
  // otherwise false is returned if the point is out of bounds.
  bool UnitToWorld(  //
    absl::Nonnull<S2Point*> out, const R2Point& proj, bool nearest) const override {
    double lat = proj.y()/Scale();
    double lon = proj.x()/Scale();

    if (std::fabs(lat) > 0.5 || std::fabs(lon) > 1) {
      if (!nearest) {
        return false;
      }

      // Clamp the latitude and longitude
      lat = std::min(0.5, std::max(-0.5, lat));
      lon = std::min(1.0, std::max(-1.0, lon));
    }

    double clat = std::cos(M_PI*lat);
    double slat = std::sin(M_PI*lat);
    double clon = std::cos(M_PI*lon);
    double slon = std::sin(M_PI*lon);

    *out = S2Point(clat * clon, clat * slon, -slat);
    return true;
  }

  // Projects a point from world space to screen space unconditionally.
  R2Point Project(const S2Point& point) const override {
    return UnitToScreen(WorldToUnit(Rotate(PreRotate(point))));
  }

  // Projects a point into screen space.  Returns true if it's visible.
  bool Project(absl::Nonnull<R2Point*> out, const S2Point& point) const override {
    *out = Project(point);
    return true;  // All points are visible.
  }

  // Projection functions.
  void Project(absl::Nonnull<ChainSink*> out, const S2Shape::Edge&) const override;
  void Project(absl::Nonnull<ChainSink*> out, const S2Shape&) const override;
  void Project(absl::Nonnull<ChainSink*> out, absl::Nonnull<ChainStitcher*>, const S2Shape&, ContainsPointFn contains) const override;

private:
  region2 outline_;
  S2Point amnorm_;
  S2Point amperp_;
  S2Point ampole_;
  S2Point eqnorm_;

  // Identifiers for each of the four boundary segments of the projection.
  static constexpr uint8_t kNone  = 0;
  static constexpr uint8_t kNorth = 1;
  static constexpr uint8_t kEast  = 2;
  static constexpr uint8_t kSouth = 3;
  static constexpr uint8_t kWest  = 4;

  // Returns true if a boundary is east or west.
  static constexpr inline bool IsEastWest(uint8_t b) {
    return b && b % 2 == 0;
  }

  // Returns true if a boundary is north or south.
  static constexpr inline bool IsNorthSouth(uint8_t b) {
    return b && b % 2 == 1;
  }

  // A crossing where an edge crossed the anti-meridian.  We store the vertex
  // index, the boundary that was crossed and the direction of the crossing.
  struct Crossing {
    Crossing() = default;

    static Crossing Incoming(int vertex, uint8_t boundary) {
      Crossing crossing;
      crossing.vertex = vertex;
      crossing.boundary = boundary;
      crossing.direction = +1;
      return crossing;
    }

    static Crossing Outgoing(int vertex, uint8_t boundary) {
      Crossing crossing;
      crossing.vertex = vertex;
      crossing.boundary = boundary;
      crossing.direction = -1;
      return crossing;
    }

    // Returns true if this is an incoming crossing.
    bool incoming() const { return direction > 0; }

    // Returns true if this is an outgoing crossing.
    bool outgoing() const { return direction < 0; }

    int vertex;
    uint8_t boundary;
    char direction;
  };

  // Projects a point and snaps it to the given boundary, if any.
  R2Point ProjectAndSnap(const S2Point& point, uint8_t b) const {
    R2Point projected;
    Project(&projected, point);

    switch (b) {
      case kWest:  projected.x(outline_.lo().x()); break;
      case kEast:  projected.x(outline_.hi().x()); break;
      case kNorth: projected.y(outline_.lo().y()); break;
      case kSouth: projected.y(outline_.hi().y()); break;
      default:
        break;
    }
    return projected;
  }

  // Subdivides an edge and appends it to the output.  Takes a BoundaryPair that
  // dictates whether to snap the endpoints to a boundary segment or not.  If
  // snapping is requested, then the chain is broken before appending p0.
  //
  // Expects that the edge has been properly clipped so that projecting will not
  // wrap in screen space, which will lead to unpredictable results.
  void Subdivide(  //
    ChainSink* out, const S2Shape::Edge& edge, BoundaryPair bs = {}) const {

    // The real function, defined here because we may have to call it twice.
    const auto Run = [&](const S2Shape::Edge& edge, BoundaryPair bs) {
      R2Point p0 = ProjectAndSnap(edge.v0, bs.b0);
      R2Point p1 = ProjectAndSnap(edge.v1, bs.b1);

      // If we're clamping a vertex to the north or south pole (it happens),
      // then just straight vertical lines.
      bool subdivide = true;
      if (IsNorthSouth(bs.b0)) {
        DCHECK(!IsNorthSouth(bs.b1));
        p0.x(p1.x());
        subdivide = false;
      }

      if (IsNorthSouth(bs.b1)) {
        DCHECK(!IsNorthSouth(bs.b0));
        p1.x(p0.x());
        subdivide = false;
      }

      if (bs.b0 || out->ChainEmpty()) {
        out->Break();
        out->Append(p0);
      }

      if (subdivide && (p1-p0).Norm2() > MaxSqError()) {
        Subdivide(out, edge, {p0, p1});
      }
      out->Append(p1);
    };

    const int sign0 = s2pred::SignDotProd(edge.v0, eqnorm_);
    const int sign1 = s2pred::SignDotProd(edge.v1, eqnorm_);

    // If the vertices are on opposite sides of the equator, split the edge to
    // avoid the inflection point that will occur at the equator, which can
    // introduce error in the subdivision.
    if (sign0 && sign0 == -sign1) {
      S2Point isect = edge.v0.CrossProd(edge.v1).CrossProd(eqnorm_).Normalize();
      if (sign0 - sign1 < 0) {
        isect = -isect;
      }

      R2Point proj;
      Project(&proj, isect);

      Run({edge.v0, isect}, BoundaryPair::Snap0(bs.b0));
      out->Append(proj);
      Run({isect, edge.v1}, BoundaryPair::Snap1(bs.b1));
    } else {
      Run(edge, bs);
    }
  }

  // Helper method that recursively subdivides an edge.
  void Subdivide(absl::Nonnull<ChainSink*> out,
    const S2Shape::Edge& s2edge, const R2Shape::Edge& r2edge) const {

    // Compute a point halfway along the edge.
    S2Point v2 = S2::Interpolate(s2edge.v0, s2edge.v1, 0.5);

    R2Point p2;
    Project(&p2, v2);

    // Compute the distance from the projected point to the line from p0 to p1.
    R2Point vp = p2 - r2edge.v0;
    R2Point vn = r2edge.v1 - r2edge.v0;
    double dist2 = (vp - (vp.DotProd(vn)*vn)/vn.Norm2()).Norm2();

    // If we're too far from the line, recursively subdivide each half.
    if (dist2 > MaxSqError()) {
      Subdivide(out, {s2edge.v0, v2}, {r2edge.v0, p2});
      out->Append(p2);          //
      Subdivide(out, {v2, s2edge.v1}, {p2, r2edge.v1});
    }
  }

  // Processes an edge and returns a ClipResult to use to process it.
  ClipResult ClipEdge(const S2Shape::Edge& edge) const {
    // Check for antipodal vertices, which should never occur.
    DCHECK(edge.v0 != -edge.v1);

    int sign0 = s2pred::SignDotProd(edge.v0, amnorm_);
    int sign1 = s2pred::SignDotProd(edge.v1, amnorm_);

    const auto InAmHemisphere = [&](const S2Point& point) {
      return s2pred::SignDotProd(point, amperp_) > 0;
    };

    // The vertex signs are the same.
    if (ABSL_PREDICT_TRUE(sign0 == sign1)) {
      // The edge is entirely on one side of the anti-meridian, just keep it.
      // This is by far the most common case, so most testing will end here.
      if (ABSL_PREDICT_TRUE(sign0 != 0)) {
        return ClipResult::Keep();
      }

      // The edge is exactly on the prime meridian plane.
      if (sign0 == 0) {
        // Look at the signs relative to the normal defining the hemisphere of
        // the anti-meridian.
        sign0 = s2pred::SignDotProd(edge.v0, amperp_);
        sign1 = s2pred::SignDotProd(edge.v1, amperp_);

        if (sign0 == sign1) {
          // Keep the edge if it's in the other hemisphere as the anti-meridian,
          // and drop it if it lands exactly on the anti-meridian itself.
          if (sign0 == -1) return ClipResult::Keep();
          if (sign0 == +1) return ClipResult::Drop();

          // Both vertices landed exactly on the perpendicular plane too?! They
          // must both be at the intersection of two perpendicular planes
          // through the origin, which means they're at the poles.
          //
          // Either they're on the same pole and we can't see the edge or
          // they're on opposite poles and thus antipodal points, which can't
          // happen.  Either way, we can drop the edge.
          DCHECK_EQ(sign0, 0);
          return ClipResult::Drop();
        }

        DCHECK(sign0 != sign1);

        // Signs aren't equal.  If one or the other is zero, then snap to the
        // appropriate boundary.
        if (sign0 == 0) {
          sign0 = s2pred::SignDotProd(edge.v0, ampole_);
          if (sign0 > 0) return ClipResult::Snap0(kNorth);
          if (sign0 < 0) return ClipResult::Snap0(kSouth);
          ABSL_UNREACHABLE(); // v0 would have to be at the origin.
        }

        if (sign1 == 0) {
          sign1 = s2pred::SignDotProd(edge.v1, ampole_);
          if (sign1 > 0) return ClipResult::Snap1(kNorth);
          if (sign1 < 0) return ClipResult::Snap1(kSouth);
          ABSL_UNREACHABLE(); // v1 would have to be at the origin.
        }

        // Signs aren't equal, and neither is zero, so they're one of either
        // (-, +) or (+, -).  Meaning this line extends directly over a pole.
        // Let's cheat a little bit and find a mid-point and see which
        // hemisphere it's in to determine which pole to snap to.
        const S2Point mid = S2::Interpolate(edge.v0, edge.v1, 0.5);
        int midsign = s2pred::SignDotProd(mid, ampole_);

        if (sign0 > 0) {
          DCHECK_LT(sign1, 0);
          if (midsign > 0) return ClipResult::Snap0(kNorth);
          if (midsign < 0) return ClipResult::Snap0(kSouth);
          ABSL_UNREACHABLE();
        }

        if (sign1 > 0) {
          DCHECK_LT(sign0, 0);
          if (midsign > 0) return ClipResult::Snap1(kNorth);
          if (midsign < 0) return ClipResult::Snap1(kSouth);
          ABSL_UNREACHABLE();
        }
      }
      ABSL_UNREACHABLE();
    }

    // The vertex signs are different.

    // v0 was exactly on the meridian.  If it's in the same hemisphere as the
    // anti-meridian, then we have to snap it to a boundary, otherwise just keep
    // the edge as-is.
    if (sign0 == 0) {
      DCHECK_NE(sign1, 0);
      if (InAmHemisphere(edge.v0)) {
        if (sign1 < 0) return ClipResult::Snap0(kWest);
        if (sign1 > 0) return ClipResult::Snap0(kEast);
        ABSL_UNREACHABLE();
      }
      return ClipResult::Keep();
    }

    // v1 was exactly on the meridian.  If it's in the same hemisphere as the
    // anti-meridian, then we have to snap it to a boundary, otherwise just keep
    // the edge as-is.
    if (sign1 == 0) {
      DCHECK_NE(sign0, 0);
      if (InAmHemisphere(edge.v1)) {
        if (sign0 < 0) return ClipResult::Snap1(kWest);
        if (sign0 > 0) return ClipResult::Snap1(kEast);
        ABSL_UNREACHABLE();
      }
      return ClipResult::Keep();
    }

    // The edge definitely crosses the anti-meridian plane.  See if it crossed
    // in the same hemisphere as the anti-meridian.  Find the intersection point.
    S2Point isect = edge.v0.CrossProd(edge.v1).CrossProd(amnorm_).Normalize();
    if (sign0 - sign1 < 0) {
      isect = -isect;
    }

    // If it crossed in the opposite hemisphere just keep the edge.
    if (!InAmHemisphere(isect)) {
      return ClipResult::Keep();
    }

    // The edge crossed the anti-meridian.  The signs must be opposite so we can
    // chop the edge into two pieces.
    DCHECK_EQ(sign0, -sign1);

    return ClipResult::Chop(isect, sign0 - sign1);
  }
};

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

inline void Equirectangular::Project(absl::Nonnull<ChainSink*> out,
  const S2Shape::Edge& edge) const {

  const ClipResult ans = ClipEdge(edge);

  switch (ans.status()) {
    // The edge was dropped, we can just ignore it.
    case ClipResult::kDrop:
      return;

    // The edge was kept as-is, we can just subdivide it normally.
    case ClipResult::kKeep:
      Subdivide(out, edge);
      return;

    // The edge needs to be chopped in half since it wrapped.
    case ClipResult::kChop: {
      const uint8_t b0 = ans.direction > 0 ? kEast : kWest;
      const uint8_t b1 = ans.direction > 0 ? kWest : kEast;

      Subdivide(out, {edge.v0, *ans.isect}, BoundaryPair::Snap1(b0));
      out->Break();
      Subdivide(out, {*ans.isect, edge.v1}, BoundaryPair::Snap0(b1));
      return;
    }

    // One or the other vertex needs to be snapped to a boundary.
    case ClipResult::kSnap: {
      const uint8_t b0 = ans.boundary.b0;
      const uint8_t b1 = ans.boundary.b1;

      if (b0) {
        DCHECK(!b1);
        out->Break();
        Subdivide(out, edge, BoundaryPair::Snap0(b0));
      }

      if (b1) {
        DCHECK(!b0);
        Subdivide(out, edge, BoundaryPair::Snap1(b1));
        out->Break();
      }
      return;
    }
  }
  ABSL_UNREACHABLE();
}

inline void Equirectangular::Project(  //
  absl::Nonnull<ChainSink*> out, const S2Shape& shape) const {
  DCHECK_LT(shape.dimension(), 2);

  // Points don't need anything fancy, just project them.
  out->Clear();
  if (shape.dimension() == 0) {
    for (int chain = 0; chain < shape.num_chains(); ++chain) {
      for (int i = 0, n = shape.chain(chain).length; i < n; ++i) {
        R2Point proj;
        if (!Project(&proj, shape.chain_edge(chain, i).v0)) {
          continue;
        }
        out->Append(proj);
        out->Break();
      }
    }
    return;
  }

  // Subdivide edges and split chains as needed.
  for (int chain = 0; chain < shape.num_chains(); ++chain) {
    out->Break();

    for (int i = 0, n = shape.chain(chain).length; i < n; ++i) {
      const S2Shape::Edge& edge = shape.chain_edge(chain, i);

      ClipResult ans = ClipEdge(edge);

      switch (ans.status()) {
        // The edge was dropped, ignore it.
        case ClipResult::kDrop:
          break;

        // The edge was kept as-is, just subdivide normally.
        case ClipResult::kKeep:
          Subdivide(out, edge);
          break;

        // We have to chop the edge into two pieces.
        case ClipResult::kChop: {
          const uint8_t b0 = ans.direction > 0 ? kEast : kWest;
          const uint8_t b1 = ans.direction > 0 ? kWest : kEast;

          Subdivide(out, {edge.v0, *ans.isect}, BoundaryPair::Snap1(b0));
          out->Break();
          Subdivide(out, {*ans.isect, edge.v1}, BoundaryPair::Snap0(b1));
          break;
        }

        // We need to snap one vertex or the other.
        case ClipResult::kSnap: {
          const uint8_t b0 = ans.boundary.b0;
          const uint8_t b1 = ans.boundary.b1;

          if (b0) {
            DCHECK(!b1);
            Subdivide(out, edge, BoundaryPair::Snap0(b0));
          }

          if (b1) {
            DCHECK(!b0);
            Subdivide(out, edge, BoundaryPair::Snap1(b1));
          }
          break;
        }
      }
    }
  }
}

inline void Equirectangular::Project(absl::Nonnull<ChainSink*> out,
  absl::Nonnull<ChainStitcher*> stitcher,
  const S2Shape& shape, ContainsPointFn contains) const {

  // Adds a chain of vertices to the output.
  const auto AddChain = [&](absl::Span<const R2Point> vertices) {
    out->Append(vertices);
    out->Close();
  };

  // Delegate points and polygons to the other Project().
  if (shape.dimension() != 2) {
    return Project(out, shape);
  }

  absl::InlinedVector<Crossing, 16> crossings;
  stitcher->Clear();

  // Subdivide edges and split chains as needed.
  for (int chain = 0; chain < shape.num_chains(); ++chain) {
    stitcher->Break();

    const int start = stitcher->NextVertex();
    for (int i = 0, n = shape.chain(chain).length; i < n; ++i) {
      const S2Shape::Edge& edge = shape.chain_edge(chain, i);

      ClipResult ans = ClipEdge(edge);

      switch (ans.status()) {
        // Edge was dropped, ignore it.
        case ClipResult::kDrop:
          break;

        // Edge was kept as-is, just subdivide it normally.
        case ClipResult::kKeep:
          Subdivide(stitcher, edge);
          break;

        // We have to chop the edge into two pieces.
        case ClipResult::kChop: {
          const uint8_t b0 = ans.direction > 0 ? kEast : kWest;
          const uint8_t b1 = ans.direction > 0 ? kWest : kEast;

          // Tessellate the first half of the edge.  Snap isect to a boundary.
          Subdivide(stitcher, {edge.v0, *ans.isect}, BoundaryPair::Snap1(b0));
          stitcher->Break();

          // Add crossings for the two boundaries we crossed.
          const int idx = stitcher->LastVertex();
          crossings.emplace_back(Crossing::Outgoing(idx + 0, b0));
          crossings.emplace_back(Crossing::Incoming(idx + 1, b1));

          // Tessellate the second half of the edge.  Snap isect to a boundary.
          Subdivide(stitcher, {*ans.isect, edge.v1}, BoundaryPair::Snap0(b1));
          break;
        }

        // We need to snap one vertex or the other.
        case ClipResult::kSnap: {
          const uint8_t b0 = ans.boundary.b0;
          const uint8_t b1 = ans.boundary.b1;

          if (b0) {
            DCHECK(!b1);
            const int next = stitcher->NextVertex();
            Subdivide(stitcher, edge, BoundaryPair::Snap0(b0));

            // Don't add crossings for the poles.
            if (IsEastWest(b0)) {
              crossings.push_back(Crossing::Incoming(next, b0));
            }
          }

          if (b1) {
            DCHECK(!b0);
            Subdivide(stitcher, edge, BoundaryPair::Snap1(b1));

            // Don't add crossings for the poles.
            if (IsEastWest(b1)) {
              crossings.push_back(  //
                Crossing::Outgoing(stitcher->LastVertex(), b1));
            }
          }
          break;
        }
      }
    }

    // If the chain closed, remove the repeat point and connect to the start.
    if (stitcher->Size() > start && stitcher->Back() == (*stitcher)[start]) {
      stitcher->PopBack();
      stitcher->Connect(stitcher->LastVertex(), start);
    }
  }

  // Usually there's no crossings.  The polygon must entirely contain or not
  // contain the boundary of the projection.  Test the north pole to break the
  // tie.  If the boundary is contained, emit the outline as a shell first, then
  // we can emit the polygon chains.
  if (ABSL_PREDICT_TRUE(crossings.empty())) {
    if (contains(Unrotate({0, 0, 1}))) {
      MakeOutline(out);
    }

    if (!stitcher->EmitChains(AddChain)) {
      fprintf(stderr, "[Equirectangular] Saw infinite loop splicing chains!\n");
    }
    return;
  }

  // Sort crossings CCW around the projection boundary, noting any duplicates.
  bool duplicates = false;
  absl::c_sort(crossings, [&](const Crossing& a, const Crossing& b) {
    if (a.boundary < b.boundary) return true;
    if (a.boundary > b.boundary) return false;

    const double ay = (*stitcher)[a.vertex].y();
    const double by = (*stitcher)[b.vertex].y();

    if (ay == by) {
      duplicates = true;
      return false;
    }

    // Crossings should only be on the east and west boundaries.
    DCHECK(IsEastWest(a.boundary));

    // Note that these are in screen space, so Y increases -down-.
    if (a.boundary == kEast) {
      return ay > by;
    } else {
      return ay < by;
    }
  });

  if (duplicates) {
    // Returns one past the end of the duplicate points starting at index.
    const auto DuplicateEnd = [&](int index) {
      DCHECK_LT(index, crossings.size());
      const R2Point& curr = (*stitcher)[crossings[index + 0].vertex];

      int end = index;
      for (; end < crossings.size(); ++end) {
        if (curr != (*stitcher)[crossings[end].vertex]) {
          break;
        }
      }
      return end;
    };

    // If we have two crossings at -exactly- the same point, we can't order the
    // crossings by just sorting.  We'll walk around the projection from the
    // bottom-right corner (the south pole) counter-clockwise.  The crossings
    // should alternate between incoming and outgoing.

    // This should happen -very- rarely, so we don't have to worry about being
    // efficient here.  When we run into duplicates then we'll just swap
    // elements in the duplicate range to make the order consistent.
    bool outgoing = !contains(Unrotate({0, 0, -1}));

    for (int i = 0; i < crossings.size(); ++i) {
      if (crossings[i].outgoing() != outgoing) {
        const int end = DuplicateEnd(i);

        int j = i + 1;
        for (; j < end; ++j) {
          if (crossings[j].outgoing() == outgoing) {
            std::swap(crossings[j], crossings[i]);
            break;
          }
        }

        // This should never happen, but the world is a strange place.
        if (j == end) {
          fprintf(stderr, "[Equirectangular] - Couldn't permute crossings!\n");
          return;
        }
      }
      outgoing = !crossings[i].outgoing();
    }
  }

  // Now stitch crossings together.
  for (int ii = 0, N = crossings.size(); ii < N; ++ii) {
    // Skip to an outgoing crossing.
    if (crossings[ii].incoming()) {
      continue;
    }
    const Crossing& outgoing = crossings[ii];

    // Find the next incoming crossing.  Since vertices can land on the same
    // point, it may not necessarily be the next vertex, but should exist.
    int jj = (ii + 1) % N;
    for (; jj != ii; jj = (jj + 1) % N) {
      if (crossings[jj].incoming()) {
        break;
      }
    }
    DCHECK_NE(ii, jj);
    const Crossing& incoming = crossings[jj];

    DCHECK_GE(outgoing.vertex, 0);
    DCHECK_GE(incoming.vertex, 0);

    // When stitching between the crossings, if the outgoing crossing is on one
    // boundary and the incoming crossing is on the other, we have to stitch
    // over one or both of the poles.
    absl::InlinedVector<int, 4> corners;
    if (outgoing.boundary == incoming.boundary) {
      if (ii > jj) {
        // Crossings on the same boundary but outgoing was after incoming so we
        // have to walk all the way around the corners.
        if (outgoing.boundary == kEast) {
          for (int corner : {1, 0, 2, 3}) {
            corners.emplace_back(corner);
          }
        } else {
          for (int corner : {3, 2, 1, 0}) {
            corners.emplace_back(corner);
          }
        }
      }
    } else {
      // Crossings on different boundaries, stitch two corners.
      if (outgoing.boundary == kEast) {
        corners.emplace_back(1);
        corners.emplace_back(0);
      } else {
        corners.emplace_back(3);
        corners.emplace_back(2);
      }
    }

    int last = outgoing.vertex;
    for (int corner : corners) {
      stitcher->Break();
      stitcher->Append(outline_.GetVertex(corner));
      last = stitcher->Connect(last, stitcher->LastVertex());
    }
    stitcher->Connect(last, incoming.vertex);
  }

  if (!stitcher->EmitChains(AddChain)) {
    fprintf(stderr, "[Equirectangular] Saw infinite loop splicing chains!\n");
  }
}

} // namespace w
