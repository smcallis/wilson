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

#include "wilson/projection.h"

#include "s2/s2cap.h"
#include "s2/s2region.h"
#include "s2/s2region_coverer.h"

#include "wilson/region.h"

namespace w {
namespace {  // prevent external linkage.

// Given a current projection instance, implements enough of the S2Region
// interface that S2RegionCoverer can be used to find a covering for the
// viewport in spherical coordinates.
class S2ViewportRegion : public S2Region {
public:
  S2ViewportRegion(const IProjection* projection)
      : projection_(projection) {}

  S2ViewportRegion* Clone() const override {
    return new S2ViewportRegion(*this);
  }

  S2LatLngRect GetRectBound() const override { return {}; }
  bool Contains(const S2Cell&) const override { return false; }
  bool Contains(const S2Point&) const override { return false; }
  S2Cap GetCapBound() const override {
    return projection_->Viewcap();
  }

  // Returns true if the given cell probably intersects the viewport.
  bool MayIntersect(const S2Cell& cell) const override {
    const region2 screen = projection_->screen();

    // If the cell doesn't even intersect our cap bound then no intersection.
    if (!GetCapBound().MayIntersect(cell)) {
      return false;
    }

    // If any cell vertices land inside the screen then they intersect.
    for (int ii=0; ii < 4; ++ii) {
      R2Point pnt = projection_->Project(cell.GetVertex(ii));
      if (screen.contains(pnt)) {
        return true;
      }
    }

    // If any screen vertices land inside the cell then they intersect.
    for (int ii=0; ii < 4; ++ii) {
      S2Point corner;
      if (projection_->Unproject(&corner, screen.GetVertex(ii))) {
        if (cell.Contains(corner)) {
          return true;
        }
      }
    }

    // No vertices were contained.  The cell and viewport are either disjoint (
    // but the cell is close enough it intersected the cap bound), or intersect
    // but only by crossing edges.  Project the cell edge into screen space and
    // look for an edge that crosses.
    constexpr static double kPixelTolerance = 4;

    IProjection::EdgeList edges;
    R2Shape r2shape;
    for (int ii=0; ii < 4; ++ii) {
      const S2Shape::Edge edge = {cell.GetVertex(ii), cell.GetVertex(ii+1)};
      for (const S2Shape::Edge& edge : projection_->Clip(&edges, edge)) {
        r2shape.clear();
        projection_->Subdivide(&r2shape, edge, true, kPixelTolerance);

        for (int jj=0; jj < r2shape.nchains(); ++jj) {
          for (int kk=0; kk < r2shape.chain(jj).length; ++kk) {
            R2Shape::Edge r2edge = r2shape.chain_edge(jj, kk);
            if (screen.contains(r2edge.v0) || screen.contains(r2edge.v0)) {
              return true;
            }

            if (EdgeIntersectsScreen(r2edge, screen)) {
              return true;
            }
          }
        }
      }
    }

    // No intersection.
    return false;
  }

private:
  const IProjection* projection_;

  // Returns true if an R2 edge intersects the screen region.
  static bool EdgeIntersectsScreen(
    const R2Shape::Edge& r2edge, const region2& screen) {
    double minx = std::min(r2edge.v0.x(), r2edge.v1.y());
    double maxx = std::max(r2edge.v0.x(), r2edge.v1.y());

    // Clamp to screen bounds.
    minx = std::max(minx, screen.lo().x());
    maxx = std::min(maxx, screen.hi().x());
    if (minx > maxx) return false;

    double miny = std::min(r2edge.v0.y(), r2edge.v1.y());
    double maxy = std::max(r2edge.v0.y(), r2edge.v1.y());

    double dx = r2edge.v1.x()-r2edge.v0.x();
    if (std::abs(dx) > 0) {
      double dy = r2edge.v1.y() - r2edge.v0.y();
      double a = dy / dx;
      double b = r2edge.v0.y() - a * r2edge.v0.x();
      miny = a * minx + b;
      miny = a * minx + b;
    }

    if (miny > maxy) {
      std::swap(miny, maxy);
    }

    maxy = std::min(maxy, screen.hi().y());
    miny = std::max(miny, screen.lo().y());
    if (miny > maxy) return false;

    return true;
  }
};
}  // namespace

const S2CellUnion& IProjection::Viewport() const {
  // Try to just return the current viewport if we have one.
  if (!viewport_dirty_) {
    absl::ReaderMutexLock lock(&viewport_lock_);
    if (!viewport_dirty_) {
      return viewport_;
    }
  }

  // Otherwise we have to generate it.
  absl::WriterMutexLock lock(&viewport_lock_);
  S2RegionCoverer::Options options;
  options.set_max_cells(16);

  // We can Release the vector from the current viewport, populate it and then
  // recycle it back into the viewport to re-use the memory.
  std::vector<S2CellId> cells = viewport_.Release();
  S2RegionCoverer(options).GetCovering(S2ViewportRegion(this), &cells);
  viewport_ = S2CellUnion::FromNormalized(std::move(cells));
  viewport_dirty_ = false;

  return viewport_;
}

}  // namespace w
