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

#include <memory>

// External requirements
#include "absl/container/flat_hash_set.h"
#include "blend2d.h"
#include "blend2d/context.h"
#include "s2/s2cell_iterator_join.h"
#include "s2/s2polygon.h"
#include "s2/s2shapeutil_coding.h"
#include "s2/util/coding/coder.h"

// Project requirements
#include "wilson/chain_stitcher.h"
#include "wilson/graticule.h"
#include "wilson/quaternion.h"
#include "wilson/region.h"
#include "wilson/simplify.h"
#include "wilson/timing.h"

#include "wilson/generated/land_simplified.h"
#include "wilson/graphics/pixbuffer.h"
#include "wilson/projection/all.h"

// An inset globe that can be drawn to show the current viewport.
namespace w {

namespace internal {
static MutableS2ShapeIndex LoadSimplifiedLandIndex() {
  Decoder decoder(LandSimplifiedIndex().data(), LandSimplifiedIndex().size());
  MutableS2ShapeIndex index;
  index.Init(&decoder, s2shapeutil::FullDecodeShapeFactory(&decoder));
  return index;
}

static const MutableS2ShapeIndex& GetSimplifiedLandIndex() {
  static const MutableS2ShapeIndex index = LoadSimplifiedLandIndex();
  return index;
}
}  // namespace internal

class Inset {
public:
  Inset() : projection_(std::make_unique<Orthographic>()) {}

  // Sets the rotation value for the inset.
  void SetRotation(const Quaternion& rotation) {
    projection_->SetRotation(rotation);
  }

  // Sets the position of the top-left corner of the inset.
  void SetPosition(const vec2i& pos) {
    pos_ = pos;
  }

  // Sets the size of the inset.
  void Resize(int ww, int hh) {
    projection_->Resize(ww, hh);
    texture_.resize(ww, hh);
  }

  // Redraws the inset, highlighting information from the active projection.
  void Redraw(BLContext& ctx, const IProjection& projection) {
    const S2ShapeIndex& index = internal::GetSimplifiedLandIndex();

    S2ContainsPointQuery<S2ShapeIndex>::Options options;
    options.set_vertex_model(S2VertexModel::CLOSED);
    S2ContainsPointQuery<S2ShapeIndex> query(&index, options);

    // Returns a function that can test point containment in the given shape.
    const auto ShapeContainsFn = [&](const S2Shape& shape) {
      return [&](const S2Point& point) {
        return query.ShapeContains(shape, point);
      };
    };

    // Draw to the texture pixbuffer using a new context.
    outline_.Clear();
    projection_->MakeOutline(&outline_);
    {
      BLContext ctx;
      ctx.begin(texture_.image());

      // Clear the texture to a mostly transparent black.
      texture_.clear(pixel(0x77000000));

      // Clear the on-screen outline of the sphere.
      timeit("fill", "Time to fill geometry", [&]() {
        timeit("outline", "Time to fill outline", [&]() {
          ctx.setFillStyle(pixel(0xffe6ecee));
          ctx.fillPath(outline_.path());
        });

        timeit("land", "Time to fill land", [&]() {
          ctx.setFillStyle(pixel(0xff777777));

          r2shape_.Clear();
          for (const S2Shape* shape : index) {
            projection_->Project(
              &r2shape_, &chain_stitcher_, *shape, ShapeContainsFn(*shape));
          }

          Simplify(&simplified_, r2shape_);
          ctx.fillPath(simplified_.path());
        });
      });

      // Draw graticules.
      ctx.setStrokeStyle(pixel(0xffd0d0d0));
      timeit("graticules", "Time to draw graticules", [&]() {
        constexpr int kNlevel = 4;
        for (int level=kNlevel-1; level >= 0; --level) {
          S2Graticule(&r2shape_, *projection_.get(), level);
          ctx.setStrokeWidth(3-2.5*(double)level/(kNlevel-1));
          ctx.strokePath(r2shape_.path());
        }
      });

      // Convert the viewport into a polygon.  Note we have to check whether the
      // output polygon has significantly less area than the cell union and flip
      // it because it doesn't maintain orientation for very full cell unions.
      timeit("viewport", "Time to draw the viewport outline", [&]() {
        r2shape_.Clear();
        for (S2CellId cell : projection.Viewport()) {
          S2Polygon polygon{S2Cell(cell)};

          projection_->Project(
            &r2shape_, &chain_stitcher_, S2Polygon::Shape(&polygon),
            [&](const S2Point& point) { return polygon.Contains(point); });
        }

        ctx.setFillStyle(pixel(0x77e34234));
        ctx.fillPath(r2shape_.path());
      });

      // Draw the viewcap outline.  If it gets very close to 90 degrees, then
      // thicken the stroke so we don't get ugly drawing near the boundary.
      timeit("viewcap", "Time to draw the viewcap outline", [&]() {
        const S2Cap cap = projection.Viewcap();
        double width = 1;
        if (M_PI / 2 - cap.radius().radians() < 1e-3) {
          width = 2;
        }

        ctx.setStrokeStyle(pixel(0xff00316e));
        ctx.setStrokeWidth(width);

        r2shape_.Clear();
        projection_->Project(&r2shape_, &chain_stitcher_, cap);
        ctx.strokePath(r2shape_.path());
      });
    }

    // Refill the outline using the texture as a source, this will remove any
    // overdraw or slight numerical error from the edges of the projection.
    timeit("blit", "Time to reblit buffer", [&]() {
      BLPattern texture(texture_.image());
      ctx.save();
      ctx.setCompOp(BL_COMP_OP_SRC_COPY);
      ctx.setFillStyle(texture);
      ctx.fillPath(outline_.path());
      ctx.restore();
    });
  }

 private:
  ChainStitcher chain_stitcher_;
  std::unique_ptr<IProjection> projection_;
  vec2i pos_ = vec2i(0,0);
  Pixbuffer texture_;

  // Storage for shapes.
  R2Shape outline_;
  R2Shape r2shape_;
  R2Shape simplified_;
};

}  // namespace w
