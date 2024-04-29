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

#include <array>
#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "blend2d.h"
#include "s2/s2cell.h"
#include "s2/s2coords.h"
#include "s2/s2edge_clipping.h"
#include "s2/s2edge_crosser.h"
#include "s2/s2edge_crossings.h"
#include "s2/s2edge_distances.h"
#include "s2/s2point.h"
#include "s2/s2latlng.h"
#include "s2/s2wedge_relations.h"

#include "wilson/barycentric.h"
#include "wilson/plane.h"
#include "wilson/projection.h"
#include "wilson/quaternion.h"
#include "wilson/r2shape.h"

namespace w {

struct Cubic final : Projection<Cubic> {
  // Include Project function explicitly to have visibility with our overloads.
  using Projection::Project;

  // We have to expand the scale by sqrt(3) to inscribe a cube into a sphere.
  static constexpr double kNaturalScale = 1.732;

  Cubic() {
    UpdateTransforms();
  }

  // Populates an R2Shape with the outline of the projected sphere on screen.
  R2Shape& MakeOutline(absl::Nonnull<R2Shape*> out) const override {
    out->clear();
    for (const PointAngle& pa : r2outline_) {
      out->Append(pa.screen);
    }
    out->EndChain();
    return *out;
  }

  // Populates an R2Shape with a graticule with lines of latitude and longitude.
  R2Shape& MakeGraticule(absl::Nonnull<R2Shape*> out) const override {
    out->clear();
    //generate_graticule(path);
    return *out;
  }

  // Clips a single S2Point to the visible portion of the sphere.  Returns true
  // if the point is visible and false otherwise.
  bool Clip(S2Point point) const override {
    const int face = S2::GetFace(point);
    for (const auto& visible : visible_faces_) {
      if (visible.id == face) {
        return true;
      }
    }
    return false;
  }

  // Clips an edge to the visible portion of the sphere, possibly splitting it
  // into multiple discontinuous pieces if needed.  For a cubic projection this
  // is equivalent to clipping edges to the (at most) 3 visible faces.
  EdgeList& Clip(absl::Nonnull<EdgeList*> edges, const S2Shape::Edge& edge) const override {
    edges->clear();

    // When an edge crosses between face cells, we may get slightly different
    // endpoint for the edge segments due to numerical error, if that happens we
    // want to stitch the edge segments back together into a single edge.
    //
    // This is the UV distance tolerance before we snap vertices together.
    constexpr double kUVMergeDistance = 1e-6;

    // Checks if a UV point is with the clipping error of a face boundary.
    const auto WithinEdgeError = [](const R2Point& uv) {
      return \
        std::fabs(std::fabs(uv.x())-1) < S2::kEdgeClipErrorUVCoord ||
        std::fabs(std::fabs(uv.y())-1) < S2::kEdgeClipErrorUVCoord;
    };

    // Clip the edge to each visible face.  If the edge is very close to
    // coincident with the edge of a face, we need to de-duplicate it when we
    // merge edges, because it will appear on both faces.
    bool dedup = false;
    for (const auto& face : visible_faces_) {
      R2Point uv0,uv1;
      if (S2::ClipToFace(edge.v0, edge.v1, face.id, &uv0, &uv1)) {
        // The edge hit the face, so convert the clipped vertices back to XYZ
        // space and normalize them to be unit magnitude points.
        edges->emplace_back(
          S2::FaceUVtoXYZ(face.id, uv0).Normalize(),
          S2::FaceUVtoXYZ(face.id, uv1).Normalize()
        );

        // Check if either vertex was within the error margin of the face.
        bool too_close0 = WithinEdgeError(uv0);
        bool too_close1 = WithinEdgeError(uv1);

        // If either was too close.  If the edges are horizontal or vertical
        // then we need to de-duplicate the results.
        if (too_close0 || too_close1) {
          dedup |= std::fabs(uv0.x()-uv1.x()) < 2*S2::kEdgeClipErrorUVCoord;
          dedup |= std::fabs(uv0.y()-uv1.y()) < 2*S2::kEdgeClipErrorUVCoord;
        }
      }
    }

    // Zero or one edges means the edge was entirely clipped away or entirely
    // contained by the face, so we can just return the edge.  This is a very
    // common case, most of the time edges will not cross faces, and it will let
    // us avoid trying to merge segments back together.
    if (edges->size() <= 1) {
      return *edges;
    }

    // Determines if two points are within the merge radius of each other.
    const auto Mergeable = [](const S2Point& v0, const S2Point& v1) {
      return (v0-v1).Norm2() < kUVMergeDistance*kUVMergeDistance;
    };

    // Otherwise repeatedly look at the generated edges and stitch any segments
    // whose endpoints are closer than some threshold back together before
    // returning.
    const auto MergeEdges = [&]() {
      for (int i=0; i < edges->size(); ++i) {
        S2Shape::Edge& e0 = (*edges)[i];

        for (int j=0; j < edges->size(); ++j) {
          if (i == j) {
            continue;
          }
          S2Shape::Edge& e1 = (*edges)[j];

          // Decide whether to merge the edge, either because one is a
          // continuation of the other, or the edge clipped to two faces (rare).
          bool merge = Mergeable(e0.v1, e1.v0);
          if (dedup) {
            merge |= (Mergeable(e0.v0, e1.v0) && Mergeable(e0.v1, e1.v1));
          }

          // We can merge by just setting the first edges vertices and doing
          // a swap-and-pop to remove the other edge.  Return true to continue
          // iterating.
          if (merge) {
            e0.v1 = e1.v1;
            std::swap((*edges)[j], edges->back());
            edges->pop_back();
            return true;
          }
        }
      }

      // Didn't merge any edges, done.
      return false;
    };
    while (MergeEdges());

    return *edges;
  }

  // Takes two vertices presumed to have been generated by Clip() and connects
  // them together, adding segments along the projection boundary as needed.
  // This is used when a chain exits the projection and renters at a later
  // point, we need to put edges along the boundary to close it.
  R2Shape& Stitch(absl::Nonnull<R2Shape*> out, const S2Shape::Edge& edge, const S2Point& v0) const override {
    // How many times A must be incremented to reach B, assuming that we take
    // the value modulo N.
    const auto ModuloDistance = [](int a, int b, int n) {
      if (a < b) {
        return (b+0)-a;
      } else {
        return (b+n)-a;
      }
    };

    const R2Point p1 = Project(edge.v1);
    const R2Point p0 = Project(v0);

    // The vertices must be on the edge of a face since they were clipped. We
    // have all the vertices for the visible faces already, ordered CW around
    // the center point.  So we can find the vertices to stitch with by finding
    // the nearest points in that list, and adding the points between them.
    const R2Point center = Project(nadir());
    PointAngle pa1(p1, center);
    PointAngle pa0(p0, center);

    int beg = absl::c_lower_bound(r2outline_, pa1)-r2outline_.begin();
    int end = absl::c_upper_bound(r2outline_, pa0)-r2outline_.begin();

    // Sweep the shortest path through the vertices.
    const int N = r2outline_.size();
    if (ModuloDistance(end, beg, N) < ModuloDistance(beg, end, N)) {
      std::swap(beg, end);
    }

    out->Append(p1);
    for (int i=beg % N; i != end % N; i = (i+1) % N) {
      out->Append(r2outline_[i].screen);
    }

    return *out;
  }

  S2Point BeforeRotate(S2Point p) const override {
    // Dividing each point by its largest magnitude component puts it on one of
    // the six cube faces, but leaves it in 3D Cartesian coordinates.
    return p/std::fabs(p[p.LargestAbsComponent()]);
  }

  // Convert from world space to unit space.  We just project orthographically
  // after mapping points to the cube face and drop the Z coordinate.
  R2Point WorldToUnit(S2Point p) const override {
    S2Point proj = world_to_unit_*p;
    return {proj.x(), proj.y()};
  }

  // Convert from unit space back to world space.
  bool UnitToWorld(absl::Nonnull<S2Point*> out, R2Point pnt, bool nearest=false) const override {
    // Treat each square face as being composed of an upper and lower triangular
    // half.  We can get barycentric coordinates for the point in each triangle
    // and easily check whether it's contained in the face or not.
    for (const ProjectedFace& face : visible_faces_) {
      // Check the lower half of the face.
      BaryCoord coord = face.lower.ConvertPoint(pnt);
      if (coord.inside()) {
        S2Cell c = S2Cell::FromFace(face.id);
        *out = coord.Evaluate(c.GetVertex(0), c.GetVertex(1), c.GetVertex(2));
        *out = Rotate(*out);
        return true;
      }

      // Check the upper half of the face.
      coord = face.upper.ConvertPoint(pnt);
      if (coord.inside()) {
        S2Cell c = S2Cell::FromFace(face.id);
        *out = coord.Evaluate(c.GetVertex(2), c.GetVertex(3), c.GetVertex(0));
        *out = Rotate(*out);
        return true;
      }
    }

    // We didn't hit any faces, if a nearest point was requested then project
    // onto the closest outline edge.
    if (nearest) {
      // Find the closest outline segment to project onto.
      double min_dist = +INFINITY;
      int    min_vert = -1;
      double min_frac = -1;

      const int N = r2outline_.size();
      for (int i=0; i < r2outline_.size(); ++i) {
        R2Point A = r2outline_[i].unit;
        R2Point B = r2outline_[(i+1) % N ].unit;

        R2Point C = pnt;
        B -= A;
        C -= A;

        double frac = std::min(1.0, std::max(0.0, C.DotProd(B)/B.Norm2()));
        double dist = (frac*B - C).Norm2();

        if (dist < min_dist) {
          min_dist = dist;
          min_vert = i;
          min_frac = frac;
        }
      }

      assert(min_idx >= 0 && "No minimum? Should never happen.");

      // If we were closest to the interior of an edge, return that.
      S2Point A = r2outline_[min_vert].world;
      S2Point B = r2outline_[(min_vert + 1) % N].world;

      *out = Rotate(S2::Interpolate(A, B, min_frac));
      return true;
    }

    return false;
  }

protected:
  void UpdateTransforms() override;

private:
  // Store a face split into two triangular halves in unit space.
  struct ProjectedFace {
    ProjectedFace(int id, std::array<R2Point, 4> vertices) : id(id) {
      upper = Barycentric<R2Point>(vertices[2], vertices[3], vertices[0]);
      lower = Barycentric<R2Point>(vertices[0], vertices[1], vertices[2]);
    }

    int id;
    Barycentric<R2Point> upper;
    Barycentric<R2Point> lower;
  };

  // List of currently visible faces.
  std::vector<ProjectedFace> visible_faces_;

  // A point in world, screen and unit coordinates along with its angle around
  // some center point in screen space.  This can be sorted by angle to find
  // vertices for outline stitching.
  struct PointAngle {
    PointAngle(R2Point screen, R2Point center, S2Point world={}, R2Point unit={})
      : world(world), screen(screen), unit(unit) {
      // Note that we negate the y coordinate to accommodate screen coordinates
      // so that points are ordered CW _on screen_.
      const R2Point pc = screen-center;
      angle = std::atan2(-pc.y(), pc.x());
    }

    S2Point world;
    R2Point screen;
    R2Point unit;
    double angle;

    bool operator<(const PointAngle& b) const { return angle < b.angle; }
    bool operator==(const PointAngle& b) const { return angle == b.angle; }
  };

  // Vertices for the projection, sorted CW in screen space by angle.
  std::vector<PointAngle> r2outline_;

  // Forward and reverse transformation matrices.
  Affine3 world_to_unit_, unit_to_world_;
};

inline void Cubic::UpdateTransforms() {
  // Normal vectors for each cube face.
  static constexpr S2Point kFaceNormals[] = {
    {+1, 0, 0},
    { 0,+1, 0},
    { 0, 0,+1},
    {-1, 0, 0},
    { 0,-1, 0},
    { 0, 0,-1}
  };

  // Generate the world -> unit space projection.  Looking at the sphere with
  // lat/lon (0,0) facing us.  The world Z axis corresponds to the unit Y
  // coordinate world Y axis corresponds to the unit X axis.
  world_to_unit_ =
    Affine3::Permute(1)                        // X=Y and Y=Z after permuting.
    *Affine3::Orthographic(-1,+1,-1,+1,-1,+1)  // Project coordinates.
    *Affine3::Scale(scale());                  // Zoom in on sphere.

  // Inverse projection is the reverse sequence of inverted transforms, but
  // without the orthographic matrix.  We can't fully unproject because the
  // orthographic matrix isn't invertible, but we can take it the rest of the
  // way manually in UnitToWorld().
  //
  // Note that the orthographic matrix naturally flip the z coordinate (which
  // aligns us with screen space), so we need to negate it when going back.
  unit_to_world_ =
    Affine3::Scale({1/scale(), 1/scale(), -1/scale()})
    *Affine3::Permute(-1);

  // Determine which faces are visible.
  absl::flat_hash_map<S2Point, int> vertex_map;
  std::array<R2Point,4> unit_vertices;

  visible_faces_.clear();
  for (int face=0; face < 6; ++face) {
    S2Cell cell = S2Cell::FromFace(face);

    // If the face normal has a positive dot product with the nadir point, then
    // the face is visible; grab its vertices to build the outline.
    if (kFaceNormals[face].DotProd(nadir()) > 0) {
      for (int k=0; k < 4; ++k) {
        S2Point vertex = cell.GetVertex(k);
        vertex_map[vertex]++;
        unit_vertices[k] = ScreenToUnit(Project(vertex));
      }
      visible_faces_.emplace_back(face, unit_vertices);
    }
  }

  // If any vertex occurred 3 times, it must be the shared corner of three
  // faces.  That vertex is interior to the overall outline, so remove it.
  for (auto iter = vertex_map.begin(); iter != vertex_map.end(); iter++) {
    if (iter->second == 3) {
      vertex_map.erase(iter);
      break;
    }
  }

  // Project the outline points and sort them in CW order around the nadir.
  const R2Point center = Project(nadir());
  r2outline_.clear();
  for (auto pair : vertex_map) {
    const R2Point screen = Project(pair.first);
    r2outline_.emplace_back(screen, center, pair.first, ScreenToUnit(screen));
  }
  absl::c_sort(r2outline_);
}

} // namespace w
