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

#include "absl/base/nullability.h"
#include "s2/s2cap.h"
#include "s2/s2cell.h"
#include "s2/s2point.h"
#include "s2/s2latlng.h"
#include "s2/s2edge_distances.h"
#include "s2/s2wedge_relations.h"

#include "wilson/geometry/chain_stitcher.h"
#include "wilson/geometry/plane.h"
#include "wilson/geometry/quaternion.h"
#include "wilson/geometry/r2shape.h"
#include "wilson/geometry/vertex_sink.h"
#include "wilson/projection/clipping.h"
#include "wilson/projection/projection.h"

namespace w {

struct Gnomonic final : Projection<Gnomonic> {
  // Import this explicitly so we can see it with our other Project() overloads.
  using Projection::Project;

  // Gnomonic projections are infinite in extent since they're a tangent
  // projection.  We have to limit the portion of the sphere that's visible.  By
  // default we'll show a 120 degree segment of the sphere.
  static constexpr double kDefaultViewAngle = 2 * M_PI / 3;

  Gnomonic() {
    SetViewAngle(kDefaultViewAngle);
    UpdateTransforms();
  }

  S2Cap Viewcap() const override {
    // Create a cap and expand it to cover the corners of the projection.  If
    // any of the corners miss the sphere, then just return the entire view
    // angle.
    S2Cap cap(Nadir(), S1ChordAngle::Zero());
    for (int i = 0; i < 4; ++i) {
      S2Point corner;
      if (!Unproject(&corner, Screen().GetVertex(i))) {
        return S2Cap(Nadir(), S1ChordAngle::Radians(ViewAngle() / 2));
      }
      cap.AddPoint(corner);
    }
    return cap;
  }

  // Sets the angle of the cone that's visible in the viewport.  Only the
  // absolute value of the angle is considered, and it must be less than 180
  // degrees.  Angles larger than that are ignored.
  void SetViewAngle(double angle) {
    angle = std::abs(angle);
    if (angle >= M_PI) {
      return;
    }

    view_angle_ = angle;

    // Compute the sin and cosine of the complementary angle PI/2 - angle/2,
    // which is just the cosine and sine of angle/2.
    sin_angle_ = std::cos(angle / 2);
    cos_angle_ = std::sin(angle / 2);
    cot_angle_ = cos_angle_/sin_angle_;

    UpdateTransforms();
  }

  // Return the current view angle.
  double ViewAngle() const {
    return view_angle_;
  }

  // Appends an outline for the projection to the given output.
  void AppendOutline(R2VertexSink* absl_nonnull out) const override {
    // The gnomonic projection is always a unit circle in unit space, we can
    // just multiply the radius by the scale to get the correct size.
    R2Point origin = UnitToScreen({0,0});
    R2Point radius = UnitToScreen({Scale(), 0});

    AppendCircle(out, origin, (radius - origin).x(), 0.125 /* pixels */);
  }

  // Populates a path with a graticule with lines of latitude and longitude.
  void MakeGraticule(R2VertexSink* absl_nonnull out) const override {
    out->Clear();
    // generate_graticule(path);
  }

  R2Point WorldToUnit(const S2Point& p) const override {
    return (Scale()*sin_angle_/cos_angle_)*R2Point(p.y()/p.x(), -p.z()/p.x());
  }

  bool UnitToWorld(S2Point* absl_nonnull out, const R2Point& proj, bool nearest) const override {
    R2Point point = (cot_angle_/Scale())*proj;
    if (point.Norm2() <= cot_angle_*cot_angle_) {
      *out = S2Point(1, point.x(), -point.y()).Normalize();
      return true;
    }

    // The point missed the sphere, if nearest is true find a vector of the
    // appropriate radius in the clip plane as the closest point on the sphere.
    if (nearest) {
      point = point.Normalize()*cos_angle_;
      *out = S2Point(sin_angle_, point.x(), -point.y()).Normalize();
      return true;
    }

    return false;
  }

  // Projects a point from world space to screen space unconditionally.
  R2Point Project(const S2Point& point) const override {
    return UnitToScreen(WorldToUnit(Rotate(PreRotate(point))));
  }

  // Projects a point into screen space.  Returns true if it's visible.
  bool Project(R2Point* absl_nonnull out, const S2Point& point) const override {
    if (clip_plane_.Sign(point) > 0) {
      *out = Project(point);
      return true;
    }
    return false;
  }

  // Projection functions.
  void Project(R2VertexSink* absl_nonnull out, const S2Shape::Edge&) const override;
  void Project(R2VertexSink* absl_nonnull out, const S2Shape&) const override;
  void Project(R2VertexSink* absl_nonnull out, ChainStitcher* absl_nonnull, const S2Shape&, ContainsPointFn contains) const override;

 protected:
  void UpdateTransforms() override {
    // Set a new plane to clip geometry to.  The clip plane is offset by the sine
    // of the complementary angle.
    clip_plane_ = S2Plane::FromSubcenter(sin_angle_ * Nadir());
  }

 private:
  // A crossing where an edge crossed the clip plane.
  struct Crossing {
    Crossing() = default;

    Crossing(const S2Point& point, int vertex, int direction)
      : point(point), vertex(vertex), direction(direction) {}

    static Crossing Incoming(const S2Point& point, int vertex) {
      return Crossing(point, vertex, +1);
    }

    static Crossing Outgoing(const S2Point& point, int vertex) {
      return Crossing(point, vertex, -1);
    }

    bool incoming() const { return direction > 0; }
    bool outgoing() const { return direction < 0; }

    S2Point point;
    int vertex;
    int direction;  // -1 for outgoing and +1 for incoming.
  };

  using CrossingVector = absl::InlinedVector<Crossing, 16>;

  // Subdivides an edge and appends it to the output.  Geodesics always become
  // straight lines in gnomonic projections, so subdividing an edge just
  // requires projecting the endpoints.
  //
  // Expects that the edge has been properly clipped so that projecting will not
  // wrap in screen space, which will lead to unpredictable results.
  void Subdivide(R2VertexSink* absl_nonnull out, const S2Shape::Edge& edge) const {
    if (out->ChainEmpty()) {
      out->Append(Project(edge.v0));
    }
    out->Append(Project(edge.v1));
  }

  // Stitch from v0 to v1 counter clockwise along the projection boundary.  We
  // may have to stitch up to a full 360 degrees, so we test the edge length
  // around the nadir and split the edge before subdividing each piece.
  //
  // If needed, the angle between the vectors is subtracted from 2*PI to stitch
  // from v0 to v1 counter-clockwise around the nadir().  When the vertices are
  // the same, the angle between them will be zero.  This can be overridden by
  // passing full = true to force a full 360 degree arc.
  void Stitch(R2VertexSink* absl_nonnull out, const S2Point& v0, const S2Point& v1, bool full = false) const {
    // Split at slightly less than 180 degrees to avoid numerical issues.
    constexpr double kSplitThreshold = 0.95 * M_PI;

    const double angle = full ? 2 * M_PI : clip_plane_.Angle(v0, v1);

    if (angle >= kSplitThreshold) {
      auto curve = //
        S2Plane::FromSubcenter(clip_plane_.Subcenter()).Interpolate(v0, v1);

      // Sample the curve at 1/3 and 2/3 and to break it into smaller pieces.
      const S2Point a = v0;
      const S2Point b = curve(1 / 3.0);
      const S2Point c = curve(2 / 3.0);
      const S2Point d = v1;

      StitchInternal(out, a, b);
      StitchInternal(out, b, c);
      StitchInternal(out, c, d);
    } else {
      StitchInternal(out, v0, v1);
    }
  }

  // Does the real work of subdividing and projecting an arc from v0 to v1.
  void StitchInternal(
    R2VertexSink* absl_nonnull out, const S2Point& v0, const S2Point& v1) const {
    // Projections are promised to be clipped to their outline, so we can
    // overdraw a bit past the actual projection boundary which lets us
    // subdivide the curve much more coarsely without any visible artifacts.
    constexpr double kOverdraw = 0.10;

    const R2Point center = Project(Nadir());
    const R2Point p0 = Project(v0) - center;
    const R2Point p1 = Project(v1) - center;

    // The signed angle between p0 and p1.
    const double sweep = p0.Angle(p1);

    // The sagitta h of a chord subtending an angle θ is:
    //
    //    h = radius*(1-cos(θ/2))
    //
    // We can solve for θ given a desired sagitta:
    //
    //    θ = 2*std::acos(1-h/radius)
    //
    // We'll require a sagitta of half the total overdraw budget.  This will
    // determine the maximum angular step size needed to achieve a given
    // distance from the circle.
    const double step = 2 * std::acos(1 - kOverdraw / 2);

    // We can just divide this into the sweep to solve for the number of steps.
    const int nsteps = std::ceil(std::fabs(sweep) / step) + 1;

    if (out->ChainEmpty()) {
      out->Append(center + (1 + kOverdraw)*p0);
    }

    // p0 is on the outline, so it's length tells us the radius.  Expand it by
    // the overdraw factor to draw in the clip space past the outline.
    const double radius = (1+kOverdraw)*p0.Norm();

    // Step through subdividing the arc.
    const double angle0 = std::atan2(p0.y(), p0.x());
    for (int i = 1; i < nsteps; ++i) {
      const double angle = angle0 + i * sweep / (nsteps - 1);
      out->Append(center + radius*R2Point(std::cos(angle), std::sin(angle)));
    }
    out->Append(center + (1 + kOverdraw)*p1);
  }

  // We have to limit the field of view with gnomonic projection to avoid
  // distortion becoming too large.  We can achieve this by clipping geometry
  // against a plane, whose normal is the current nadir() point, and which is
  // offset towards the viewer by sin((π-θ)/2) in the direction of the nadir:
  //
  //  Visible
  //  ├─────┤
  //   nadir
  // ----•----      ┬
  //   ╲   ╱        │ sin((π-θ)/2)
  //    ╲θ╱ (π-θ)/2 │
  // ---------      ┴
  double view_angle_;
  double sin_angle_;
  double cos_angle_;
  double cot_angle_;

  S2Plane clip_plane_;
};

inline void Gnomonic::Project(R2VertexSink* absl_nonnull out,
  const S2Shape::Edge& edge) const {

  S2Shape::Edge copy = edge;
  if (ClipEdgeToSmallCircle(clip_plane_, copy)) {
    Subdivide(out, copy);
  }
}

inline void Gnomonic::Project(  //
  R2VertexSink* absl_nonnull out, const S2Shape& shape) const {
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

      ClipResult result = ClipToSmallCircle(clip_plane_, edge);

      switch (result.action) {
        case ClipResult::kDrop:
          break;

        case ClipResult::kKeep:
          Subdivide(out, edge);
          break;

        case ClipResult::kCrop0:  // incoming
          Subdivide(out, {result.point[0], edge.v1});
          break;

        case ClipResult::kCrop1:  // outgoing
          Subdivide(out, {edge.v0, result.point[1]});
          out->Break();
          break;

        case ClipResult::kCrop:
          Subdivide(out, {result.point[0], result.point[1]});
          out->Break();
          break;

        // The other cases can't happen.
        default:
          ABSL_UNREACHABLE();
      }
    }
  }
}

inline void Gnomonic::Project(R2VertexSink* absl_nonnull out,
  ChainStitcher* absl_nonnull stitcher, const S2Shape& shape, ContainsPointFn contains) const {

  // Adds a chain of vertices to the output.
  const auto AddChain = [&](absl::Span<const R2Point> vertices) {
    out->Append(vertices);
    out->Close();
  };

  // Delegate points and polygons to the other Project().
  if (shape.dimension() != 2) {
    return Project(out, shape);
  }

  CrossingVector crossings;
  stitcher->Clear();

  // Subdivide edges and split chains as needed.
  for (int chain = 0; chain < shape.num_chains(); ++chain) {
    stitcher->Break();

    const int start = stitcher->NextVertex();
    for (int i = 0, n = shape.chain(chain).length; i < n; ++i) {
      const S2Shape::Edge& edge = shape.chain_edge(chain, i);

      ClipResult result = ClipToSmallCircle(clip_plane_, edge);

      switch (result.action) {
        case ClipResult::kDrop:
          break;

        case ClipResult::kKeep:
          Subdivide(stitcher, edge);
          break;

        case ClipResult::kCrop0: {  // incoming
          const S2Point v0 = result.point[0];
          crossings.push_back(Crossing::Incoming(v0, stitcher->NextVertex()));
          Subdivide(stitcher, {v0, edge.v1});
          break;
        }

        case ClipResult::kCrop1: {  // outgoing
          const S2Point v1 = result.point[1];
          Subdivide(stitcher, {edge.v0, v1});
          stitcher->Break();
          crossings.push_back(Crossing::Outgoing(v1, stitcher->LastVertex()));
          break;
        }

        case ClipResult::kCrop: {
          const S2Point v0 = result.point[0];
          const S2Point v1 = result.point[1];

          crossings.push_back(Crossing::Incoming(v0, stitcher->NextVertex()));
          Subdivide(stitcher, {v0, v1});
          stitcher->Break();
          crossings.push_back(Crossing::Outgoing(v1, stitcher->LastVertex()));
          break;
        }

        // The other cases can't happen.
        default:
          ABSL_UNREACHABLE();
      }
    }

    // If the chain closed, remove the repeat point and connect to the start.
    if (stitcher->Size() > start && stitcher->Back() == (*stitcher)[start]) {
      stitcher->PopBack();
      stitcher->Connect(stitcher->LastVertex(), start);
    }
  }

  // Since we use exact predicates, the number of crossings should be even.
  DCHECK(crossings.size() % 2 == 0);

  // Usually there's no crossings.  The polygon must entirely contain or not
  // contain the boundary of the projection.  Test a point on the outline to
  // break the tie.  If the boundary is contained, emit the outline as a shell
  // first, then we can emit the polygon chains.
  if (ABSL_PREDICT_TRUE(crossings.empty())) {
    if (contains(clip_plane_.u())) {
      AppendOutline(out);
    }

    if (!stitcher->EmitChains(AddChain)) {
      fprintf(stderr, "[Gnomonic] Saw infinite loop splicing chains!\n");
    }
    return;
  }

  bool duplicates = false;
  absl::c_sort(crossings, [&](const Crossing& a, const Crossing& b) {
      // OrderedCCW returns true when any two input points are equal, and we
      // want it to be false because we want a < comparison here, not <=.
      if (a.point == b.point) {
        duplicates = true;
        return false;
      }
      return s2pred::OrderedCCW(clip_plane_.u(), a.point, b.point, Nadir());
    });

  if (duplicates) {
    // Returns one past the end of the duplicate points starting at index.
    const auto DuplicateEnd = [&](int index) {
      DCHECK_LT(index, crossings.size());
      const S2Point& curr = crossings[index + 0].point;

      int end = index;
      for (; end < crossings.size(); ++end) {
        if (curr != crossings[end].point) {
          break;
        }
      }
      return end;
    };

    // If we have two crossings at -exactly- the same point, we can't order the
    // crossings by just sorting.  We'll walk around the projection from a
    // reference point counter-clockwise.  The crossings should alternate
    // between incoming and outgoing.

    // This should happen -very- rarely, so we don't have to worry about being
    // efficient here.  When we run into duplicates then we'll just swap
    // elements in the duplicate range to make the order consistent.
    bool outgoing = !contains(clip_plane_.u());
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
          fprintf(stderr, "[Gnomonic] - Couldn't permute crossings!\n");
          return;
        }
      }
      outgoing = !crossings[i].outgoing();
    }
  }

  // Now stitch crossings together.
  for (int ii = 0, n = crossings.size(); ii < n; ++ii) {
    // Skip to an outgoing crossing.
    if (crossings[ii].incoming()) {
      continue;
    }
    const Crossing& outgoing = crossings[ii];
    const Crossing& incoming = crossings[(ii + 1) % n];
    DCHECK(incoming.incoming());

    // Stitch around the projection boundary between the points.
    const int start = stitcher->NextVertex();
    if (outgoing.point != incoming.point) {
      stitcher->Break();
      Stitch(stitcher, outgoing.point, incoming.point);
    }

    // And splice the subdivided edge into the loop.
    if (stitcher->Size() - start == 0) {
      stitcher->Connect(outgoing.vertex, incoming.vertex);
    } else {
      stitcher->Connect(outgoing.vertex, start);
      stitcher->Connect(stitcher->LastVertex(), incoming.vertex);
    }
  }

  if (!stitcher->EmitChains(AddChain)) {
    fprintf(stderr, "[Gnomonic] Saw infinite loop splicing chains!\n");
  }
}

} // namespace w
