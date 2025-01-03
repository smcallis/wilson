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
#include "s2/s2predicates.h"
#include "s2/s2latlng.h"
#include "s2/s2wedge_relations.h"

#include "wilson/geometry/barycentric.h"
#include "wilson/geometry/plane.h"
#include "wilson/geometry/quaternion.h"
#include "wilson/geometry/r2shape.h"
#include "wilson/projection/clipping.h"
#include "wilson/projection/projection.h"

namespace w {

// Pre-computed S2Cell instances for each cube face.
static const S2Cell kFaceCells[] = {
  S2Cell::FromFace(0),
  S2Cell::FromFace(1),
  S2Cell::FromFace(2),
  S2Cell::FromFace(3),
  S2Cell::FromFace(4),
  S2Cell::FromFace(5),
};

struct FaceBoundary {
  constexpr FaceBoundary() = default;
  constexpr FaceBoundary(int face, int boundary)
    : face(face), boundary(boundary) {}
  
  int8_t face = 0;
  int8_t boundary = 0;
};

// Maps from (face, boundary) to the adjacent (face, boundary).
inline constexpr FaceBoundary kAdjacentFace[6][4] = {
    /* face 0 */ {{5, 2}, {1, 3}, {2, 3}, {4, 2}},
    /* face 1 */ {{5, 1}, {3, 0}, {2, 0}, {0, 1}},
    /* face 2 */ {{1, 2}, {3, 3}, {4, 3}, {0, 2}},
    /* face 3 */ {{1, 1}, {5, 0}, {4, 0}, {2, 1}},
    /* face 4 */ {{3, 2}, {5, 3}, {0, 3}, {2, 2}},
    /* face 5 */ {{3, 1}, {1, 0}, {0, 0}, {4, 1}}
};

// Returns true if two faces are adjacent.
inline constexpr bool FacesAreAdjacent(uint8_t face0, uint8_t face1) {
  // We just have to exclude the face itself and its opposite face.
  return face0 != face1 && face1 != (face0 + 3) % 6;
}

// Given two adjacent faces, face0, and face1, gives the boundary of face0
// that connects the two faces.
inline constexpr uint8_t kAdjacentFaceBoundary[6][6] = {
    /* face 0 */ {0xFF, 1, 2, 0xFF, 3, 0},
    /* face 1 */ {3, 0xFF, 2, 1, 0xFF, 0},
    /* face 2 */ {3, 0, 0xFF, 1, 2, 0xFF},
    /* face 3 */ {0xFF, 0, 3, 0xFF, 2, 1},
    /* face 4 */ {2, 0xFF, 3, 0, 0xFF, 1},
    /* face 5 */ {2, 1, 0xFF, 0, 3, 0xFF}
};

// Face polygons that are constructed once and only once.
static const S2Polygon& FacePolygon(int face) {
  static const S2Polygon* polygons[] = {
      new S2Polygon(kFaceCells[0]), new S2Polygon(kFaceCells[1]),
      new S2Polygon(kFaceCells[2]), new S2Polygon(kFaceCells[3]),
      new S2Polygon(kFaceCells[4]), new S2Polygon(kFaceCells[5])};
  return *polygons[face];
}

struct Cubic final : Projection<Cubic> {
  using Projection<Cubic>::Project;
  
  // We have to expand the scale by sqrt(3) to inscribe a cube into a sphere.
  static constexpr double kNaturalScale = 1.732;

  Cubic() {
    UpdateTransforms();
  }

  // Populates an R2Shape with the outline of the projected sphere on screen.
  void AppendOutline(absl::Nonnull<R2VertexSink*> out) const final {
    for (const TriplePoint& pa : r2outline_) {
      out->Append(pa.screen);
    }
    out->Break();
  }

  // Populates an R2Shape with a graticule with lines of latitude and longitude.
  void MakeGraticule(absl::Nonnull<R2VertexSink*> out) const final {
    out->Clear();
    //generate_graticule(path);
    //return *out;
  }

  // Projects a point from world space to screen space unconditionally.
  R2Point Project(const S2Point& point) const final {
    return UnitToScreen(WorldToUnit(Rotate(PreRotate(point))));
  }

  // Projects a point into screen space.  Returns true if it's visible.
  bool Project(absl::Nonnull<R2Point*> out, const S2Point& point) const final {
    const int face = S2::GetFace(point);
    for (const auto& visible : visible_faces_) {
      if (visible.face == face) {
        *out = Project(point);
        return true;
      }
    }
    return false;
  }

  S2Point PreRotate(const S2Point& point) const final {
    // Dividing each point by its largest magnitude component puts it on one of
    // the six cube faces, but leaves it in 3D Cartesian coordinates.
    return point/std::fabs(point[point.LargestAbsComponent()]);
  }

  // Convert from world space to unit space.  We just project orthographically
  // after mapping points to the cube face and drop the Z coordinate.
  R2Point WorldToUnit(const S2Point& p) const final {
    return static_cast<R2Point>(world_to_unit_*p);
  }

  // Convert from unit space back to world space.
  bool UnitToWorld(absl::Nonnull<S2Point*> out,  //
                   const R2Point& point, bool nearest = false) const final {
    // Treat each square face as being composed of an upper and lower triangular
    // half.  We can get barycentric coordinates for the point in each triangle
    // and easily check whether it's contained in the face or not.
    for (const ProjectedFace& face : visible_faces_) {
      // Check the lower half of the face.
      BaryCoord coord = face.lower.ConvertPoint(point);
      if (coord.inside()) {
        const S2Cell& c = kFaceCells[face.face];
        *out = coord.Evaluate(c.GetVertex(0), c.GetVertex(1), c.GetVertex(2));
        *out = Rotate(*out).Normalize();
        return true;
      }

      // Check the upper half of the face.
      coord = face.upper.ConvertPoint(point);
      if (coord.inside()) {
        const S2Cell& c = kFaceCells[face.face];
        *out = coord.Evaluate(c.GetVertex(2), c.GetVertex(3), c.GetVertex(0));
        *out = Rotate(*out).Normalize();
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

        R2Point C = point;
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

      //assert(min_idx >= 0 && "No minimum? Should never happen.");

      // If we were closest to the interior of an edge, return that.
      S2Point A = r2outline_[min_vert].world;
      S2Point B = r2outline_[(min_vert + 1) % N].world;

      *out = Rotate(S2::Interpolate(A, B, min_frac)).Normalize();
      return true;
    }

    return false;
  }

  // Projection functions.
  void Project(absl::Nonnull<R2VertexSink*> out, const S2Shape::Edge&) const final;
  void Project(absl::Nonnull<R2VertexSink*> out, const S2Shape&) const final;
  void Project(absl::Nonnull<R2VertexSink*> out, absl::Nonnull<ChainStitcher*>, const S2Shape&, ContainsPointFn contains) const final;

protected:
  void UpdateTransforms() override;

private:
  // A struct that holds an S2Point and its projection in unit and screen space.
  struct TriplePoint {
    S2Point world;
    R2Point unit;
    R2Point screen;
  };

  // A crossing where an edge crossed a face boundary.
  struct Crossing {
    constexpr Crossing() = default;
    constexpr Crossing(const S2Point& point, int direction)
      : point(point), direction(direction) {}

    static constexpr Crossing Incoming(const S2Point& point) {
      return Crossing(point, +1);
    }

    static constexpr Crossing Outgoing(const S2Point& point) {
      return Crossing(point, -1);
    }

    static constexpr Crossing Neutral(const S2Point& point) {
      return Crossing(point, 0);
    }

    bool outgoing() const { return direction < 0; }
    bool incoming() const { return direction > 0; }

    S2Point point;
    int direction; // -1 for outgoing, +1 for incoming, or 0 for neutral.
  };

  using CrossingVector = absl::InlinedVector<Crossing, 16>;

  // Subdivides an edge and appends it to the output.  Geodesics always become
  // straight lines in the cubic projection, so subdividing an edge just
  // requires projecting the endpoints.
  //
  // Expects that the edge has been properly clipped so that projecting will not
  // wrap in screen space, which will lead to unpredictable results.
  void Subdivide(absl::Nonnull<R2VertexSink*> out, const S2Shape::Edge& edge) const {
    if (out->ChainEmpty()) {
      out->Append(Project(edge.v0));
    }
    out->Append(Project(edge.v1));
  }

  // Processes an edge and returns a ClipResult to use to process it.  Appends
  // any crossings to the given crossings vector.
  ClipResult ClipEdge(CrossingVector& crossings,
    const S2Shape::Edge& edge, int face0, int face1) const;

  // Store a face split into two triangular halves in unit space.
  struct ProjectedFace {
    ProjectedFace(int face, std::array<R2Point, 4> vertices) : face(face) {
      upper = Barycentric<R2Point>(vertices[2], vertices[3], vertices[0]);
      lower = Barycentric<R2Point>(vertices[0], vertices[1], vertices[2]);
    }

    int face;
    Barycentric<R2Point> upper;
    Barycentric<R2Point> lower;
  };

  // // Clipping logic.  If crossings is given, it is populated as well.
  // EdgeList& ClipInternal(absl::Nonnull<EdgeList*> edges,
  //   const S2Shape::Edge& edge, CrossingVector* crossings = nullptr) const;

  // List of currently visible faces.
  std::vector<ProjectedFace> visible_faces_;
  uint8_t visible_face_mask_ = 0;

  // Returns true if a face is visible.
  bool FaceIsVisible(const int face) const {
    return (visible_face_mask_ & (1 << face)) != 0;
  };

  // Vertices for the projection, sorted CW in screen space by angle.
  std::vector<TriplePoint> r2outline_;
  std::vector<S2Point> s2outline_;

  // Forward and reverse transformation matrices.
  Affine3 world_to_unit_, unit_to_world_;
};

inline void Cubic::UpdateTransforms() {
  // Normal vectors for each cube face.
  static constexpr S2Point kFaceNormals[] = {
    {+1,  0,  0},
    { 0, +1,  0},
    { 0,  0, +1},
    {-1,  0,  0},
    { 0, -1,  0},
    { 0,  0, -1}
  };

  // Generate the world -> unit space projection.  Looking at the sphere with
  // lat/lon (0,0) facing us.  The world Z axis corresponds to the unit Y
  // coordinate world Y axis corresponds to the unit X axis.
  world_to_unit_ =
    Affine3::Permute(1)                        // X=Y and Y=Z after permuting.
    *Affine3::Orthographic(-1,+1,-1,+1,-1,+1)  // Project coordinates.
    *Affine3::Scale(Scale());                  // Zoom in on sphere.

  // Inverse projection is the reverse sequence of inverted transforms, but
  // without the orthographic matrix.  We can't fully unproject because the
  // orthographic matrix isn't invertible, but we can take it the rest of the
  // way manually in UnitToWorld().
  //
  // Note that the orthographic matrix naturally flip the z coordinate (which
  // aligns us with screen space), so we need to negate it when going back.
  unit_to_world_ =
    Affine3::Scale({1/Scale(), 1/Scale(), -1/Scale()})
    *Affine3::Permute(-1);

  // Determine which faces are visible.
  absl::flat_hash_map<S2Point, int> vertex_map;
  std::array<R2Point, 4> unit_vertices;

  visible_face_mask_ = 0;
  visible_faces_.clear();
  for (int face = 0; face < 6; ++face) {
    // If the face normal doesn't have a positive dot product with the nadir
    // then it's facing away from us and thus not visible, skip it.
    if (s2pred::SignDotProd(kFaceNormals[face], Nadir()) <= 0) {
      continue;
    }

    for (int k = 0; k < 4; ++k) {
      const S2Point& vertex = kFaceCells[face].GetVertex(k);
      vertex_map[vertex]++;
      unit_vertices[k] = ScreenToUnit(Project(vertex));
    }
    visible_faces_.emplace_back(face, unit_vertices);
    visible_face_mask_ |= 1 << face;
  }

  // If any vertex occurred 3 times, it must be the shared corner of three
  // faces.  That vertex is interior to the overall outline so skip it.
  s2outline_.clear();
  for (const auto& [point, count] : vertex_map) {
    if (count == 3) {
      continue;
    }
    s2outline_.emplace_back(point);
  }

  // Sort the S2 outline vertices CCW around the nadir.
  const S2Point& first = s2outline_[0];
  std::sort(s2outline_.begin() + 1, s2outline_.end(),
    [&](const S2Point& a, const S2Point& b) {
      return s2pred::OrderedCCW(first, a, b, Nadir());
    });

  // Project the outline points and sort them in CW order around the nadir.
  // //const R2Point center = Project(Nadir());
  // r2outline_.clear();
  // for (auto pair : vertex_map) {
  //   const R2Point screen = Project(pair.first);
  //   r2outline_.emplace_back(screen, center, pair.first,
  //   ScreenToUnit(screen));
  // }
  // absl::c_sort(r2outline_);

  r2outline_.clear();
  for (const S2Point& point : s2outline_) {
    const R2Point screen = Project(point);
    r2outline_.push_back({point, ScreenToUnit(screen), screen});
  }
}

// inline Cubic::EdgeList& Cubic::ClipInternal(absl::Nonnull<EdgeList*> edges,
//   const S2Shape::Edge& edge, CrossingVector* crossings) const {
//   edges->clear();

//   // When an edge crosses between face cells, we may get slightly different
//   // endpoint for the edge segments due to numerical error, if that happens we
//   // want to stitch the edge segments back together into a single edge.
//   //
//   // This is the UV distance tolerance before we snap vertices together.
//   constexpr double kUVMergeDistance = 1e-6;

//   // Checks if a UV point is with the clipping error of a face boundary.
//   const auto WithinEdgeError = [](const R2Point& uv) {
//     return \
//       std::fabs(std::fabs(uv.x())-1) < S2::kEdgeClipErrorUVCoord ||
//       std::fabs(std::fabs(uv.y())-1) < S2::kEdgeClipErrorUVCoord;
//   };

//   // Test if the edge is entirely inside one face.
//   int face0 = S2::GetFace(edge.v0);
//   int face1 = S2::GetFace(edge.v1);
//   if (face0 == face1 && visible_face_set_.contains(face0)) {
//     edges->emplace_back(edge);
//     return *edges;
//   }

//   // Clip the edge to each visible face.  If the edge is very close to
//   // coincident with the edge of a face, we need to de-duplicate it when we
//   // merge edges, because it will appear on both faces.
//   bool dedup = false;
//   for (const auto& face : visible_faces_) {
//     R2Point uv0,uv1;
//     if (S2::ClipToFace(edge.v0, edge.v1, face.id, &uv0, &uv1)) {
//       // The edge hit the face, so convert the clipped vertices back to XYZ
//       // space and normalize them to be unit magnitude points.
//       edges->emplace_back(
//         S2::FaceUVtoXYZ(face.id, uv0).Normalize(),
//         S2::FaceUVtoXYZ(face.id, uv1).Normalize()
//       );

//       // Check if either vertex was within the error margin of the face.
//       bool too_close0 = WithinEdgeError(uv0);
//       bool too_close1 = WithinEdgeError(uv1);

//       // If either was too close.  If the edges are horizontal or vertical
//       // then we need to de-duplicate the results.
//       if (too_close0 || too_close1) {
//         dedup |= std::fabs(uv0.x()-uv1.x()) < 2*S2::kEdgeClipErrorUVCoord;
//         dedup |= std::fabs(uv0.y()-uv1.y()) < 2*S2::kEdgeClipErrorUVCoord;
//       }
//     }
//   }

//   // Zero or one edges means the edge was entirely clipped away or entirely
//   // contained by the face, so we can just return the edge.  This is a very
//   // common case, most of the time edges will not cross faces, and it will let
//   // us avoid trying to merge segments back together.
//   if (edges->size() <= 1) {
//     return *edges;
//   }

//   // Determines if two points are within the merge radius of each other.
//   const auto Mergeable = [](const S2Point& v0, const S2Point& v1) {
//     return (v0-v1).Norm2() < kUVMergeDistance*kUVMergeDistance;
//   };

//   // Otherwise repeatedly look at the generated edges and stitch any segments
//   // whose endpoints are closer than some threshold back together before
//   // returning.
//   const auto MergeEdges = [&]() {
//     for (int i=0; i < edges->size(); ++i) {
//       S2Shape::Edge& e0 = (*edges)[i];

//       for (int j=0; j < edges->size(); ++j) {
//         if (i == j) {
//           continue;
//         }
//         S2Shape::Edge& e1 = (*edges)[j];

//         // Decide whether to merge the edge, either because one is a
//         // continuation of the other, or the edge clipped to two faces (rare).
//         bool merge = Mergeable(e0.v1, e1.v0);
//         if (dedup) {
//           merge |= (Mergeable(e0.v0, e1.v0) && Mergeable(e0.v1, e1.v1));
//         }

//         // We can merge by just setting the first edges vertices and doing
//         // a swap-and-pop to remove the other edge.  Return true to continue
//         // iterating.
//         if (merge) {
//           e0.v1 = e1.v1;
//           std::swap((*edges)[j], edges->back());
//           edges->pop_back();
//           return true;
//         }
//       }
//     }

//     // Didn't merge any edges, done.
//     return false;
//   };
//   while (MergeEdges());

//   return *edges;
// }

// Returns the face that contains the given point when the face is treated as a
// polygon.  This result may not be the same as S2::GetFace.
static inline int ContainingFace(const S2Point& point) {
  const int face = S2::GetFace(point);
  if (ABSL_PREDICT_TRUE(FacePolygon(face).Contains(point))) {
    return face;
  }

  for (int k = 0; k < 4; ++k) {
    int adjacent_face = kAdjacentFace[face][k].face;
    if (FacePolygon(adjacent_face).Contains(point)) {
      return adjacent_face;
    }
  }

  ABSL_UNREACHABLE();
}

inline ClipResult Cubic::ClipEdge(CrossingVector& crossings,
  const S2Shape::Edge& edge, int face0, int face1) const {

  // Returns a direction value for a crossing between two adjacent faces.  If we
  // cross from a visible face to a non-visible face, the direction is < 0,
  // otherwise it's > 0.
  const auto CrossingDirection = [&](int face0, int face1) constexpr {
    DCHECK(FacesAreAdjacent(face0, face1));
           
    const bool visible0 = FaceIsVisible(face0);
    const bool visible1 = FaceIsVisible(face1);

    if (visible0 == visible1) {
      return 0;
    }
    return visible0 ? -1 : +1;
  };

  // If the edge is entirely on one face, keep it iff the face is visible.
  if (ABSL_PREDICT_TRUE(face0 == face1)) {
    return FaceIsVisible(face0) ? ClipResult::Keep() : ClipResult::Drop();
  }

#if 0  
  // The edge spans two or more faces.  If the faces are adjacent then we know
  // which boundary segment to test against, so just do it directly.
  if (FacesAreAdjacent(face0, face1)) {
    const S2Cell& cell = kFaceCells[face0];

    // Lookup the common boundary segment between the faces.
    const int k = kAdjacentFaceBoundary[face0][face1];
    const S2Shape::Edge bound = {cell.GetVertex(k), cell.GetVertex(k + 1)};

    // We know that the vertices are contained by different faces so we should
    // always see a crossing (either interior or a vertex crossing).
    const int sign = S2::CrossingSign(bound.v0, bound.v1, edge.v0, edge.v1);
    DCHECK_GE(sign, 0);

    // A proper interior crossing, just compute the intersection point.
    if (sign == 1) {
      crossings.emplace_back(
          S2::GetIntersection(bound.v0, bound.v1, edge.v0, edge.v1),
          CrossingDirection(face0, face1));
      return ClipResult::Split();
    }

    DCHECK_EQ(sign, 0);
    
    // Sign must be zero.  If the edge equals the boundary edge just keep it.
    if (bound == edge || bound.Reversed() == edge) {
      return ClipResult::Keep();
    }

    // Vertex 1 landed on the boundary so most of the edge is on face0.
    if (bound.v0 == edge.v1 || bound.v1 == edge.v1) {
      return FaceIsVisible(face0) ? ClipResult::Keep() : ClipResult::Drop();
    }

    // Vertex 0 landed on the boundary. Most of the edge is on face1.  
    if (bound.v0 == edge.v0 || bound.v1 == edge.v0) {
      return FaceIsVisible(face1) ? ClipResult::Keep() : ClipResult::Drop();
    }

    ABSL_UNREACHABLE();
  }
#endif

  // The edge spans 2 or more faces, so we'll track where it enters and exits
  // each face, emitting crossings every time we switch faces.  The crossings
  // will tell us how the visibility changes at each point.
  FaceBoundary current {face0, -1};
  while (current.face != face1) {
    const S2Cell& cell = kFaceCells[current.face];

    S2EdgeCrosser crosser(&edge.v0, &edge.v1);
    for (int k = 0; k < 4; ++k) {
      // Skip the boundary we entered the face on to avoid an infinite loop.
      if (k == current.boundary) {
        continue;
      }
      
      const S2Shape::Edge bound = {cell.GetVertex(k), cell.GetVertex(k+1)};

      int sign = crosser.CrossingSign(&bound.v0, &bound.v1);
      if (sign < 0) {
        continue; // no intersection
      }

      S2Point isect;
      int crossed_boundary;
      if (sign == 0) {
        // If the edge is exactly equal to the boundary segment.  We can keep
        // the edge if either of the two faces it straddles are visible.
        if (edge == bound || edge == bound.Reversed()) {
          const int adjacent_face = kAdjacentFace[current.face][k].face;
          
          if (FaceIsVisible(current.face) || FaceIsVisible(adjacent_face)) {
            return ClipResult::Keep();
          }
          return ClipResult::Drop();
        }

        // Now we know that only one edge vertex landed on a boundary vertex. If
        // it was v1 then the edge must end on this face and we can stop here.
        if (bound.v0 == edge.v1 || bound.v1 == edge.v1) {
          return ClipResult::Keep();
        }

        // Otherwise v0 must have landed on a boundary vertex.  The edge could
        // be on this face or the two other faces incident on the common vertex.
        //
        // We know a few things to help us here:
        //   1. We're not on face1 per the while loop, so v1 is on another face.
        //   2. v1 isn't also on a boundary vertex or we would have quit above.
        //
        // So, we can find the three boundary segments that are incident on the
        // common point and determine which of the three the edge falls into
        // with a wedge test, which will tell us how to proceed.
        //
        // This is hard to visualize so hopefully this diagram will help.  We
        // need to determine the common boundary segment, kcommon, and the three
        // points a, b, c:
        //
        //  b
        //    ╲   
        //     ╲   face 
        //      ╲＿＿＿＿＿ a
        //       ⎸ kcommon 
        //       ⎸ 
        //       ⎸  next
        //       ⎸
        //      c
        const int kcommon = (edge.v0 == bound.v0) ? k : k + 1;
        
        const S2Point a = cell.GetVertex(kcommon + 1);
        const S2Point b = cell.GetVertex(kcommon + 3); // k - 1

        const FaceBoundary adjacent = kAdjacentFace[current.face][kcommon];
        const S2Cell next_cell(S2CellId::FromFace(adjacent.face));
        const S2Point c = next_cell.GetVertex(adjacent.boundary + 2);

        if (s2pred::OrderedCCW(a, edge.v1, b, edge.v0)) {
          // The edge falls into the wedge of this face.  Since v1 must be on
          // another face, the edge will cross another boundary segment and we
          // can ignore this one.
          continue;
        }

        // Otherwise the edge lands in one of the two adjacent faces.  Either
        // way we'll add a crossing at v0 to allow the visibility to toggle if
        // needed, and move to the proper adjacent face.
        isect = edge.v0;
        if (s2pred::OrderedCCW(c, edge.v1, b, edge.v0)) {
          crossed_boundary = kcommon;
        } else {
          crossed_boundary = (kcommon + 3) % 4; // kcommon - 1
        }
      } else {
        DCHECK_EQ(sign, 1);
        isect = S2::GetIntersection(edge.v0, edge.v1, bound.v0, bound.v1);
        crossed_boundary = k;
      }

      // Add a crossing at the intersection point and move to the face across
      // the boundary that we determined we crossed.
      const FaceBoundary next = kAdjacentFace[current.face][crossed_boundary];
      crossings.emplace_back(isect, CrossingDirection(current.face, next.face));
      current = next;
      break;
    }

    // If we see face0 again we've gone into an infinite loop, sad.
    DCHECK_NE(current.face, face0);
  }
  
  // Split the edge based on the crossings.
  return ClipResult::Split();
}

inline void Cubic::Project(  //
  absl::Nonnull<R2VertexSink*> out, const S2Shape::Edge& edge) const {

  const int face0 = ContainingFace(edge.v0);
  const int face1 = ContainingFace(edge.v1);

  if (edge.v0 == edge.v1) {
    return;
  }    
  
  CrossingVector crossings;

  const ClipResult result = ClipEdge(crossings, edge, face0, face1);
  switch (result.action) {
    case ClipResult::kDrop:
      return;

    case ClipResult::kKeep:
      Subdivide(out, edge);
      return;

    case ClipResult::kSplit: {
      DCHECK(!crossings.empty());
      
      // Subdivide visible portions of the edge.
      bool visible = FaceIsVisible(face0);

      S2Point last_point = edge.v0;
      for (const Crossing& crossing : crossings) {
        if (visible) {
          Subdivide(out, {last_point, crossing.point});
          if (crossing.outgoing()) {
            out->Break();
          }
        }

        if (crossing.outgoing()) visible = false;
        if (crossing.incoming()) visible = true;

        last_point = crossing.point;
      }

      if (FaceIsVisible(face1)) {
        Subdivide(out, {last_point, edge.v1});
      }
      
      return;
    }

    default:  // The other cases can't occur.
      ABSL_UNREACHABLE();
  };
}

inline void Cubic::Project(  //
    absl::Nonnull<R2VertexSink*> out, const S2Shape& shape) const {
  DCHECK_LT(shape.dimension(), 2);

  if (shape.dimension() == 0) {
    // Points don't have any extent so we don't have to clip edges or stitch
    // loops back together, so we can just project them to screen space.
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

  out->Break();
  for (int chain = 0; chain < shape.num_chains(); ++chain) {
    for (int i = 0, n = shape.chain(chain).length; i < n; ++i) {
      Project(out, shape.chain_edge(chain, i));
    }
    out->Break();
  }
}

inline void Cubic::Project(absl::Nonnull<R2VertexSink*> out,
  absl::Nonnull<ChainStitcher*> stitcher,
  const S2Shape& shape, ContainsPointFn contains) const {
  // // Adds a chain of vertices to the output.
  // const auto AddChain = [&](absl::Span<const R2Point> vertices) {
  //   out->Append(vertices);
  //   out->Close();
  // };

  // // Delegate points and polygons to the other Project().
  // if (shape.dimension() != 2) {
  //   return Project(out, shape);
  // }

  // CrossingVector crossings;
  // stitcher->Clear();

  // // Subdivide edges and split chains as needed.
  // for (int chain = 0; chain < shape.num_chains(); ++chain) {
  //   stitcher->Break();
    
  //   const int start = stitcher->NextVertex();
  //   for (int i = 0, n = shape.chain(chain).length; i < n; ++i) {
  //     const S2Shape::Edge& edge = shape.chain_edge(chain, i);

  //     const int face0 = ContainingFace(edge.v0);
  //     const int face1 = ContainingFace(edge.v1);

  //     const ClipResult result = ClipEdge(crossings, edge, face0, face1);
  //     switch (result.action) {
  //       case ClipResult::kDrop:
  //         break;

  //       case ClipResult::kKeep:
  //         Subdivide(out, edge);
  //         break;

  //       case ClipResult::kSplit: {
  //         DCHECK(!crossings.empty());
      
  //         // Subdivide visible portions of the edge.
  //         bool visible = FaceIsVisible(face0);
  //         S2Point last_point = edge.v0;
  //         for (const Crossing& crossing : crossings) {
  //           if (visible) {
  //             Subdivide(out, {last_point, crossing.point});
  //             if (crossing.outgoing()) {
  //               out->Break();
  //             }
  //           }

  //           if (crossing.outgoing()) visible = false;
  //           if (crossing.incoming()) visible = true;
            
  //           last_point = crossing.point;
  //         }

  //         if (FaceIsVisible(face1)) {
  //           Subdivide(out, {last_point, edge.v1});
  //         }
      
  //         break;
  //       }

  //       default:  // The other cases can't occur.
  //         ABSL_UNREACHABLE();
  //     };
  //   }

  //   // If the chain closed, remove the repeat point and connect to the start.
  //   if (stitcher->Size() > start && stitcher->Back() == (*stitcher)[start]) {
  //     stitcher->PopBack();
  //     stitcher->Connect(stitcher->LastVertex(), start);
  //   }
  // }

  // // Usually there's no crossings.  The polygon must entirely contain or not
  // // contain the boundary of the projection.  Test the north pole to break the
  // // tie.  If the boundary is contained, emit the outline as a shell first, then
  // // we can emit the polygon chains.
  // if (ABSL_PREDICT_TRUE(crossings.empty())) {
  //   if (contains(Unrotate(s2outline_[0]))) {
  //     AppendOutline(out);
  //   }

  //   if (!stitcher->EmitChains(AddChain)) {
  //     fprintf(stderr, "[Cubic] Saw infinite loop splicing chains!\n");
  //   }
  //   return;
  // }

  // if (!stitcher->EmitChains(AddChain)) {
  //   fprintf(stderr, "[Cubic] Saw infinite loop splicing chains!\n");
  // }

  // //   const int start = stitcher->NextVertex();
  // //   for (int i = 0, n = shape.chain(chain).length; i < n; ++i) {
  // //     const S2Shape::Edge& edge = shape.chain_edge(chain, i);

  // //     const int face0 = ContainingFace(edge.v0);
  // //     const int face1 = ContainingFace(edge.v1);
      
  // //     ClipResult result = ClipEdge(crossings, edge, face0, face1);

  // //     switch (result.action) {
  // //       // Edge was dropped, ignore it.
  // //       case ClipResult::kDrop:
  // //         break;

  // //       // Edge was kept as-is, just subdivide it normally.
  // //       case ClipResult::kKeep:
  // //         Subdivide(stitcher, edge);
  // //         break;

  // //       // We have to split the edge into two pieces.
  // //       case ClipResult::kSplit: {
  // //         const uint8_t b0 = result.direction > 0 ? kEast : kWest;
  // //         const uint8_t b1 = result.direction > 0 ? kWest : kEast;
  // //         const S2Point& isect = result.point[0];

  // //         // Tessellate the first half of the edge.  Snap to a boundary.
  // //         Subdivide(stitcher, {edge.v0, isect}, BoundaryPair::Snap1(b0));
  // //         stitcher->Break();

  // //         // Add crossings for the two boundaries we crossed.
  // //         const int idx = stitcher->LastVertex();
  // //         crossings.emplace_back(Crossing::Outgoing(idx + 0, b0));
  // //         crossings.emplace_back(Crossing::Incoming(idx + 1, b1));

  // //         // Tessellate the second half of the edge.  Snap isect to a boundary.
  // //         Subdivide(stitcher, {isect, edge.v1}, BoundaryPair::Snap0(b1));
  // //         break;
  // //       }

  // //       // We need to snap one vertex or the other.
  // //       case ClipResult::kCrop0: {
  // //         const uint8_t b0 = result.boundary[0];
  // //         const int next = stitcher->NextVertex();
  // //         Subdivide(stitcher, edge, BoundaryPair::Snap0(b0));

  // //         // Don't add crossings for the poles.
  // //         if (IsEastWest(b0)) {
  // //           crossings.push_back(Crossing::Incoming(next, b0));
  // //         }
  // //         break;
  // //       }

  // //       case ClipResult::kCrop1: {
  // //         const uint8_t b1 = result.boundary[1];
  // //         Subdivide(stitcher, edge, BoundaryPair::Snap1(b1));

  // //         // Don't add crossings for the poles.
  // //         if (IsEastWest(b1)) {
  // //           crossings.push_back(  //
  // //             Crossing::Outgoing(stitcher->LastVertex(), b1));
  // //         }
  // //         break;
  // //       }

  // //       // The other cases shouldn't happen.
  // //       default:
  // //         ABSL_UNREACHABLE();
  // //     }
  // //   }

  // //   // If the chain closed, remove the repeat point and connect to the start.
  // //   if (stitcher->Size() > start && stitcher->Back() == (*stitcher)[start]) {
  // //     stitcher->PopBack();
  // //     stitcher->Connect(stitcher->LastVertex(), start);
  // //   }
  // // }

  // // // Usually there's no crossings.  The polygon must entirely contain or not
  // // // contain the boundary of the projection.  Test the north pole to break the
  // // // tie.  If the boundary is contained, emit the outline as a shell first, then
  // // // we can emit the polygon chains.
  // // if (ABSL_PREDICT_TRUE(crossings.empty())) {
  // //   if (contains(Unrotate({0, 0, 1}))) {
  // //     AppendOutline(out);
  // //   }

  // //   if (!stitcher->EmitChains(AddChain)) {
  // //     fprintf(stderr, "[Equirectangular] Saw infinite loop splicing chains!\n");
  // //   }
  // //   return;
  // // }

  // // // Sort crossings CCW around the projection boundary, noting any duplicates.
  // // bool duplicates = false;
  // // absl::c_sort(crossings, [&](const Crossing& a, const Crossing& b) {
  // //   if (a.boundary < b.boundary) return true;
  // //   if (a.boundary > b.boundary) return false;

  // //   const double ay = (*stitcher)[a.vertex].y();
  // //   const double by = (*stitcher)[b.vertex].y();

  // //   if (ay == by) {
  // //     duplicates = true;
  // //     return false;
  // //   }

  // //   // Crossings should only be on the east and west boundaries.
  // //   DCHECK(IsEastWest(a.boundary));

  // //   // Note that these are in screen space, so Y increases -down-.
  // //   if (a.boundary == kEast) {
  // //     return ay > by;
  // //   } else {
  // //     return ay < by;
  // //   }
  // // });

  // // if (duplicates) {
  // //   // Returns one past the end of the duplicate points starting at index.
  // //   const auto DuplicateEnd = [&](int index) {
  // //     DCHECK_LT(index, crossings.size());
  // //     const R2Point& curr = (*stitcher)[crossings[index + 0].vertex];

  // //     int end = index;
  // //     for (; end < crossings.size(); ++end) {
  // //       if (curr != (*stitcher)[crossings[end].vertex]) {
  // //         break;
  // //       }
  // //     }
  // //     return end;
  // //   };

  // //   // If we have two crossings at -exactly- the same point, we can't order the
  // //   // crossings by just sorting.  We'll walk around the projection from the
  // //   // bottom-right corner (the south pole) counter-clockwise.  The crossings
  // //   // should alternate between incoming and outgoing.

  // //   // This should happen -very- rarely, so we don't have to worry about being
  // //   // efficient here.  When we run into duplicates then we'll just swap
  // //   // elements in the duplicate range to make the order consistent.
  // //   bool outgoing = !contains(Unrotate({0, 0, -1}));

  // //   for (int i = 0; i < crossings.size(); ++i) {
  // //     if (crossings[i].outgoing() != outgoing) {
  // //       const int end = DuplicateEnd(i);

  // //       int j = i + 1;
  // //       for (; j < end; ++j) {
  // //         if (crossings[j].outgoing() == outgoing) {
  // //           std::swap(crossings[j], crossings[i]);
  // //           break;
  // //         }
  // //       }

  // //       // This should never happen, but the world is a strange place.
  // //       if (j == end) {
  // //         fprintf(stderr, "[Equirectangular] - Couldn't permute crossings!\n");
  // //         return;
  // //       }
  // //     }
  // //     outgoing = !crossings[i].outgoing();
  // //   }
  // // }

  // // // Now stitch crossings together.
  // // for (int ii = 0, N = crossings.size(); ii < N; ++ii) {
  // //   // Skip to an outgoing crossing.
  // //   if (crossings[ii].incoming()) {
  // //     continue;
  // //   }
  // //   const Crossing& outgoing = crossings[ii];

  // //   // Find the next incoming crossing.  Since vertices can land on the same
  // //   // point, it may not necessarily be the next vertex, but should exist.
  // //   int jj = (ii + 1) % N;
  // //   for (; jj != ii; jj = (jj + 1) % N) {
  // //     if (crossings[jj].incoming()) {
  // //       break;
  // //     }
  // //   }
  // //   DCHECK_NE(ii, jj);
  // //   const Crossing& incoming = crossings[jj];

  // //   DCHECK_GE(outgoing.vertex, 0);
  // //   DCHECK_GE(incoming.vertex, 0);

  // //   // When stitching between the crossings, if the outgoing crossing is on one
  // //   // boundary and the incoming crossing is on the other, we have to stitch
  // //   // over one or both of the poles.
  // //   absl::InlinedVector<int, 4> corners;
  // //   if (outgoing.boundary == incoming.boundary) {
  // //     if (ii > jj) {
  // //       // Crossings on the same boundary but outgoing was after incoming so we
  // //       // have to walk all the way around the corners.
  // //       if (outgoing.boundary == kEast) {
  // //         for (int corner : {1, 0, 2, 3}) {
  // //           corners.emplace_back(corner);
  // //         }
  // //       } else {
  // //         for (int corner : {3, 2, 1, 0}) {
  // //           corners.emplace_back(corner);
  // //         }
  // //       }
  // //     }
  // //   } else {
  // //     // Crossings on different boundaries, stitch two corners.
  // //     if (outgoing.boundary == kEast) {
  // //       corners.emplace_back(1);
  // //       corners.emplace_back(0);
  // //     } else {
  // //       corners.emplace_back(3);
  // //       corners.emplace_back(2);
  // //     }
  // //   }

  // //   int last = outgoing.vertex;
  // //   for (int corner : corners) {
  // //     stitcher->Break();
  // //     stitcher->Append(outline_.GetVertex(corner));
  // //     last = stitcher->Connect(last, stitcher->LastVertex());
  // //   }
  // //   stitcher->Connect(last, incoming.vertex);
  // // }

  // // if (!stitcher->EmitChains(AddChain)) {
  // //   fprintf(stderr, "[Equirectangular] Saw infinite loop splicing chains!\n");
  // // }
}

} // namespace w
