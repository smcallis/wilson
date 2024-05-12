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

#include "blend2d.h"
#include "s2/s2cap.h"
#include "s2/s2cell.h"
#include "s2/s2point.h"
#include "s2/s2latlng.h"
#include "s2/s2edge_distances.h"
#include "s2/s2wedge_relations.h"

#include "wilson/plane.h"
#include "wilson/projection.h"
#include "wilson/quaternion.h"
#include "wilson/r2shape.h"

namespace w {

struct Equirectangular final : Projection<Equirectangular> {
  // Import this explicitly so we can see it with our other Project() overloads.
  using Projection::Project;

  // Equirectangular requires a 2:1 aspect ratio fit into the visible region.
  static constexpr double kAspectRatio = 2.0;

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
    double hh = scale()*unit_scale();
    double ww = 2*hh;

    R2Point center = {width()/2, height()/2};
    outline_ = region2(
      R2Point(-ww/2, -hh/2) + center,
      R2Point(+ww/2, +hh/2) + center
    );
  }

  // Populates a path representing the outline of the sphere on screen.  May
  // encompass the entire screen.
  R2Shape& MakeOutline(absl::Nonnull<R2Shape*> out) const override {
    out->Clear();
    out->Append(R2Point(outline_.lo().x(), outline_.lo().y()));
    out->Append(R2Point(outline_.hi().x(), outline_.lo().y()));
    out->Append(R2Point(outline_.hi().x(), outline_.hi().y()));
    out->Append(R2Point(outline_.lo().x(), outline_.hi().y()));
    out->CloseChain();
    return *out;
  }

  // Populates a path with a graticule with lines of latitude and longitude.
  R2Shape& MakeGraticule(absl::Nonnull<R2Shape*> out) const override {
    out->Clear();
    // generate_graticule(path);
    return *out;
  }

  // Clips a single S2Point to the visible portion of the sphere.
  // Returns true if the point is visible and false otherwise.
  bool Clip(S2Point point) const override {
    // All points are visible in an equirectangular projection.
    return true;
  }

  EdgeList& Clip(  //
    absl::Nonnull<EdgeList*> edges, const S2Shape::Edge& edge) const override {
    return ClipInternal(edges, edge);
  }

  void Stitch(absl::Nonnull<R2Shape*> out, const S2Shape::Edge& edge, const S2Point& v0) const override {
    // The edges of the projection are always straight vertical or horizontal
    // lines so we can just use a line to stitch points together.
    out->Append(Project(edge.v1));
    out->Append(Project(v0));
  }

  R2Point WorldToUnit(S2Point p) const override {
    return scale()*R2Point(std::atan2(p.y(), p.x())/M_PI, -std::asin(p.z())/M_PI);
  }

  bool UnitToWorld(absl::Nonnull<S2Point*> out, R2Point proj, bool nearest=false) const override {
    double lat = proj.y()/scale();
    double lon = proj.x()/scale();

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

  R2Shape& Project(absl::Nonnull<R2Shape *> out, absl::Nonnull<ChainStitcher*>, const S2Shape&, double max_sq_error) const override;

private:
  region2 outline_;

  // A crossing where an edge crossed the antimeridian.
  struct Crossing {
    Crossing() = default;

    static Crossing Incoming(int vertex, uint8_t boundary) {
      Crossing crossing;
      crossing.vertex = vertex;
      crossing.boundary = boundary;
      crossing.incoming = true;
      return crossing;
    }

    static Crossing Outgoing(int vertex, uint8_t boundary) {
      Crossing crossing = Incoming(vertex, boundary);
      crossing.incoming = false;
      return crossing;
    }

    int vertex;
    uint8_t boundary;  // In the range [0,1] for left and right.
    bool incoming;
  };

  using CrossingVector = absl::InlinedVector<Crossing, 16>;

  // Clipping logic.  If crossings is given, it is populated as well.
  EdgeList& ClipInternal(absl::Nonnull<EdgeList*> edges,
    const S2Shape::Edge& edge, CrossingVector* crossings = nullptr) const {
    edges->clear();

    // Returns -1, 0, or +1 depending on the value of x.
    const auto Sign = [](double x) {
      return x < 0 ? -1 : ((x > 0) ? +1 : 0);
    };

    // Normal to the plane containing the antimeridian and a plane perpendicular
    // to it where the antimeridian cut is on the positive side.
    const S2Point amnorm = Unrotate({0, 1, 0});
    const S2Point amperp = Unrotate({-1, 0, 0});

    int sign0 = Sign(edge.v0.DotProd(amnorm));
    int sign1 = Sign(edge.v1.DotProd(amnorm));

    // If both vertices were on one side, then there's nothing to do.
    if ((sign0 < 0 && sign1 < 0) || (sign0 > 0 && sign1 > 0)) {
      edges->emplace_back(edge);
      return *edges;
    }

    // Find the intersection point, flip it to the correct half of the sphere
    // based on the orientation of the vertices across the anti-meridian.
    S2Point isect = edge.v0.CrossProd(edge.v1).CrossProd(amnorm).Normalize();
    if (sign1-sign0 > 0) {
      isect = -isect;
    }

    // The edge crossed the anti-meridian plane, but it did so in the opposite
    // hemisphere, so the edge wasn't cut.
    if (isect.DotProd(amperp) <= 0) {
      edges->emplace_back(edge);
      return *edges;
    }

    // Both signs are zero, edge is on the antimeridian, ignore it.
    if (sign0 == 0 && sign1 == 0) {
      return *edges;
    }

    // The edges weren't both on one side and the intersection point was in the
    // correct half of the sphere for the edge to have crossed the meridian.
    //
    // Altogether, there are nine possible combinations of -1, 0, +1 sign for
    // the two dot products, and we've handled two, (-,- and +,+), so we need to
    // handle the remaining seven cases.
    //
    // If the signs are actually opposite (-,+ or +,-), then we can just split
    // the edge and perturb the vertices to the right side to avoid accidental
    // meridian wrapping due to numerical error.
    if (sign0 == -sign1) {
      const S2Shape::Edge edge0(
        edge.v0, S2::Interpolate(edge.v0, isect, 1 - 1e-6));

      const S2Shape::Edge edge1(
        S2::Interpolate(isect, edge.v1, 1e-6), edge.v1);

      edges->emplace_back(edge0);
      edges->emplace_back(edge1);

      if (crossings) {
        crossings->push_back(Crossing::Outgoing(-1, sign0 < 0 ? 1 : 0));
        crossings->push_back(Crossing::Incoming(-1, sign1 < 0 ? 1 : 0));
      }

      return *edges;
    }

    // One or the other sign was zero.  Bump the zero vertex off of the
    // antimeridian towards the other vertex.
    if (sign0 == 0) {
      edges->emplace_back(S2::Interpolate(edge.v0, edge.v1, 1e-6), edge.v1);
      return *edges;
    }

    if (sign1 == 0) {
      edges->emplace_back(edge.v0, S2::Interpolate(edge.v0, edge.v1, 1-1e-6));
      return *edges;
    }

    return *edges;
  }
};


inline R2Shape& Equirectangular::Project(absl::Nonnull<R2Shape *> out,
  absl::Nonnull<ChainStitcher*> stitcher, const S2Shape& shape, double max_sq_error) const {

  EdgeList edges;
  CrossingVector crossings;

  stitcher->Clear();

  // Subdivide edges and split chains as needed.
  for (int chain = 0; chain < shape.num_chains(); ++chain) {
    stitcher->Break();

    int start = stitcher->size();
    int nedge = shape.chain(chain).length;
    for (int i = 0; i < nedge; ++i) {
      ClipInternal(&edges, shape.chain_edge(chain, i), &crossings);

      if (edges.size() == 2) {
        // The edge was subdivided so two crossings were appended.  Fix up their
        // actual vertex indices after we subdivide the edges.
        Subdivide(stitcher, edges[0], max_sq_error);
        int index = stitcher->size() - 1;
        stitcher->Break();
        Subdivide(stitcher, edges[1], max_sq_error);

        crossings[crossings.size()-2].vertex = index;
        crossings[crossings.size()-1].vertex = index + 1;
      } else {
        for (const S2Shape::Edge& edge : edges) {
          Subdivide(stitcher, edge, max_sq_error);
        }
      }
    }

    // Ensure that polygon chains are closed properly if need be.
    if (shape.dimension() == 2) {
      if (stitcher->size() > start && (*stitcher)[start] == stitcher->back()) {
        stitcher->pop_back();
        stitcher->Connect(stitcher->size()-1, start);
      }
    }
  }

  // Sort crossings CCW around the projection boundary.
  absl::c_sort(crossings, [&](const Crossing& a, const Crossing& b) {
    if (a.boundary < b.boundary) return true;
    if (a.boundary > b.boundary) return false;

    if (a.boundary == 0) {
      return (*stitcher)[a.vertex].y() > (*stitcher)[b.vertex].y();
    } else {
      return (*stitcher)[a.vertex].y() < (*stitcher)[b.vertex].y();
    }
  });

  // Now stitch crossings together.
  const int ncrossings = crossings.size();
  for (int ii = 0; ii < ncrossings; ++ii) {
    DCHECK_NE(crossings[ii].vertex, -1);

    // Skip to an outgoing crossing.
    const Crossing& outgoing = crossings[ii];
    if (outgoing.incoming) {
      continue;
    }

    // Find the next incoming crossing.
    int jj = (ii + 1) % ncrossings;
    for (; jj != ii; jj = (jj + 1) % ncrossings) {
      if (crossings[jj].incoming) {
        break;
      }
    }
    DCHECK_NE(ii, jj);
    if (ii == jj) {
      LOG(ERROR) << "Error degenerate crossings";
      exit(-1);
    }

    const Crossing& incoming = crossings[jj];

    // When stitching between the crossings, if the outgoing crossing is on one
    // boundary and the incoming crossing is on the other, we have to stitch
    // over one or both of the poles.
    absl::InlinedVector<int, 4> corners;
    if (outgoing.boundary == incoming.boundary) {
      if (ii > jj) {
        // Crossings on the same boundary but outgoing was after incoming so we
        // have to walk all the way around the corners.
        if (outgoing.boundary == 0) {
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
      if (outgoing.boundary == 0) {
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

      last = stitcher->Connect(last, stitcher->size() - 1);
    }
    stitcher->Connect(last, incoming.vertex);
  }

  const auto AddChain = [&](absl::Span<const R2Point> vertices) {
    out->AddChain(vertices, shape.dimension() == 2);
  };

  if (!stitcher->EmitChains(AddChain)) {
    fprintf(stderr, "Detected infinite loop splicing chains\n");
  }

  return *out;
}

} // namespace w
