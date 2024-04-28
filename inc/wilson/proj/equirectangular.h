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
    out->clear();
    out->Append(R2Point(outline_.lo().x(), outline_.lo().y()));
    out->Append(R2Point(outline_.hi().x(), outline_.lo().y()));
    out->Append(R2Point(outline_.hi().x(), outline_.hi().y()));
    out->Append(R2Point(outline_.lo().x(), outline_.hi().y()));
    out->CloseChain();
    return *out;
  }

  // Populates a path with a graticule with lines of latitude and longitude.
  R2Shape& MakeGraticule(absl::Nonnull<R2Shape*> out) const override {
    out->clear();
    // generate_graticule(path);
    return *out;
  }

  EdgeList& Clip(absl::Nonnull<EdgeList*> edges, const S2Shape::Edge& edge) const override {
    // Equirectangular projection has a cut at the anti-meridian.  Test if the
    // edge crosses it and split the edge if so.
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
    S2Point isect = edge.v0.CrossProd(edge.v1).CrossProd(amnorm);
    if (sign1-sign0 > 0) {
      isect = -isect;
    }

    // The edge crossed the anti-meridian plane, but it did so in the opposite
    // hemisphere, so the edge wasn't cut.
    if (isect.DotProd(amperp) <= 0) {
      edges->emplace_back(edge);
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
      edges->emplace_back(edge.v0, S2::Interpolate(edge.v0, isect, 1-1e-6));
      edges->emplace_back(S2::Interpolate(isect, edge.v1, 1e-6), edge.v1);
      return *edges;
    }

    // Otherwise one or the other sign was zero.  When this happens we need to
    // be careful because we'll see _two_ edges incident on a given vertex
    // exactly on the meridian.  We want to cut one of these edges but not both.
    //
    // For the purposes of computing longitude, a point exactly on the meridian
    // is at -180 degrees, so we want to split the edge when that sign would
    // change.  So if the other vertex is on the positive side of the cut, then
    // we'll split the edge.
    if (sign0 == 0 && sign1 == 0) {
      return *edges;
    }

    if (sign0 == 0) {
      edges->emplace_back(edge.v0, S2::Interpolate(edge.v0, edge.v1, 1e-6));
      return *edges;
    }

    if (sign1 == 0) {
      edges->emplace_back(edge.v0, S2::Interpolate(edge.v0, edge.v1, 1-1e-6));
      return *edges;
    }

    return *edges;
  }

  R2Shape& Stitch(absl::Nonnull<R2Shape*> out, const S2Point& v1, const S2Point& v0) const override {
    // The edges of the projection are always straight vertical or horizontal
    // lines so we can just use a line to stitch points together.
    out->Append(Project(v1));
    out->Append(Project(v0));
    return *out;
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

  R2Shape& Project(absl::Nonnull<R2Shape *> out,  //
    const S2Shape& shape, double max_sq_error) const override;

private:
  region2 outline_;
};


R2Shape& Equirectangular::Project(absl::Nonnull<R2Shape *> out,  //
  const S2Shape& shape, double max_sq_error) const {
  std::vector<S2Shape::Edge> chain_edges;
  std::vector<int> cuts;

  // Subdivide a range of edges from chain_edges into the shape.
  const auto SubdivideRange= [&](int beg, int end) {
    for (int i=beg; i < end; ++i) {
      Subdivide(out, chain_edges[i], false, max_sq_error);
    }
  };

  // Stitch end points of two edges together.  Stitches vertex 1 of edge b to
  // vertex 0 of edge a.
  const auto StitchEdges = [&](int idx_b, int idx_a) {
    const S2Shape::Edge& edge_a = chain_edges[idx_a];
    const S2Shape::Edge& edge_b = chain_edges[idx_b];
    Stitch(out, edge_b.v1, edge_a.v0);
  };

  // Test whether two points are on opposite sides of the anti-meridian cut.
  const S2Point amnorm = Unrotate({0, 1, 0});

  EdgeList edges;
  for (int chain=0; chain < shape.num_chains(); ++chain) {
    // Reserve additional space if needed and clear the lists.
    int nedge = shape.chain(chain).length;
    chain_edges.reserve(nedge);

    cuts.clear();
    chain_edges.clear();

    // Gather all the clipped chain edges together, noting cut locations.
    for (int i=0; i < nedge; ++i) {
      Clip(&edges, shape.chain_edge(chain, i));
      for (int j=0; j < edges.size(); ++j) {
        if (j > 0) {
          cuts.emplace_back(chain_edges.size());
        }
        chain_edges.emplace_back(edges[j]);
      }
    }

    // Now stitch the cuts back together into new chains.  If there were no
    // cuts, then we can just form a single contiguous chain.
    if (cuts.empty()) {
      SubdivideRange(0, chain_edges.size());
      out->EndChain();
      continue;
    }

    // Start by subdividing and closing the first cut range.
    int beg = cuts.back();
    int end = cuts[0];
    for (int i=0; i < cuts.size(); ++i, beg = end) {
      end = cuts[i];
      if (i == 0) {
        SubdivideRange(beg, chain_edges.size());
        SubdivideRange(0, end);
      } else {
        SubdivideRange(beg, end);
      }

      // If we had an odd number of cuts then part of this chain enclosed the
      // pole.  This makes sense since the anti-meridian only cuts a hemisphere,
      // so topologically, if the chain doesn't contain the pole, it has to
      // cross it an even number of times to close.
      if (cuts.size() % 2 == 1) {
        // Check which side of the cut the end points are on.  If they're
        // different sides this cut is the one wrapping the pole.
        const S2Point& epnt = chain_edges[end-1].v1;
        const S2Point& bpnt = chain_edges[beg].v0;

        double dot0 = epnt.DotProd(amnorm);
        double dot1 = bpnt.DotProd(amnorm);
        if (dot0*dot1 < 0) {
          if (dot1 < 0) {
            // Wrapping the north pole, stitch around it.
            out->Append(Project(epnt));
            out->Append(R2Point(outline_.hi().x(), outline_.lo().y()));
            out->Append(R2Point(outline_.lo().x(), outline_.lo().y()));
            out->Append(Project(bpnt));
          } else {
            // Wrapping the south pole, stitch around it.
            out->Append(Project(epnt));
            out->Append(R2Point(outline_.lo().x(), outline_.hi().y()));
            out->Append(R2Point(outline_.hi().x(), outline_.hi().y()));
            out->Append(Project(bpnt));
          }
          out->EndChain();
          continue;
        }
      }

      // Otherwise it's a normal cut, just stitch it together and continue.
      StitchEdges(end-1, beg);
      out->EndChain();
    }
  }

  return *out;
}

} // namespace w
