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
#include "s2/s2cell_iterator_join.h"
#include "s2/s2polygon.h"
#include "s2/s2shapeutil_coding.h"
#include "s2/util/coding/coder.h"

// Project requirements
#include "wilson/generated/land_simplified.h"
#include "wilson/graphics/pixbuffer.h"
#include "wilson/graticule.h"
#include "wilson/projections.h"
#include "wilson/quaternion.h"
#include "wilson/region.h"
#include "wilson/simplify.h"
#include "wilson/timing.h"

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
  void Redraw(BLContext& ctx, const Projection& projection) {
    // Draw to the texture pixbuffer using a new context.
    R2Shape outline = projection_->Outline();
    {
      BLContext ctx;
      ctx.begin(texture_.image());

      // Clear the texture to a mostly transparent black.
      texture_.clear(pixel(0x77000000));

      // Clear the on-screen outline of the sphere.
      ctx.setFillStyle(pixel(0xffe6ecee));
      ctx.fillPath(outline.path());

      ctx.setFillStyle(pixel(0xff777777));

      timeit("fill", "Time to fill land", [&]() {
        const S2ShapeIndex& index = internal::GetSimplifiedLandIndex();
        R2Shape r2shape;
        for (const S2Shape* shape : index) {
          r2shape = projection_->Project(*shape, std::move(r2shape));
          r2shape = Simplify(std::move(r2shape));
          ctx.fillPath(r2shape.path());
        }
      });

      // Draw graticules.
      R2Shape path;
      ctx.setStrokeStyle(pixel(0xffd0d0d0));

      timeit("graticules", "Time to draw graticules", [&]() {
        constexpr int kNlevel = 4;
        for (int level=kNlevel-1; level >= 0; --level) {
          path = S2Graticule(*projection_.get(), level, std::move(path));
          ctx.setStrokeWidth(3-2.5*(double)level/(kNlevel-1));
          ctx.strokePath(path.path());
        }
      });

      ctx.setFillStyle(pixel(0x77e34234));

      // Convert the viewport into a polygon.  Note we have to check whether the
      // output polygon has significantly less area than the cell union and flip
      // it because it doesn't maintain orientation for very full cell unions.
      R2Shape shapes;
      for (S2CellId cell : projection.Viewport()) {
        S2Polygon polygon{S2Cell(cell)};

        R2Shape shape = projection_->Project(S2Polygon::Shape(&polygon));
        shapes.Append(shape);
      }

      ctx.setStrokeStyle(pixel(0x77800000));
      ctx.setStrokeWidth(1);
      ctx.fillPath(shapes.path());

      // Draw the viewcap outline.
      ctx.setStrokeStyle(pixel(0xff00316e));
      ctx.setStrokeWidth(1);
      ctx.strokePath(
        projection_->Project(projection.Viewcap(), 0.1).path());
    }

    // Refill the outline using the texture as a source, this will remove any
    // overdraw or slight numerical error from the edges of the projection.
    BLPattern texture(texture_.image());
    ctx.restore();
    ctx.save();
    ctx.setCompOp(BL_COMP_OP_SRC_COPY);
    ctx.setFillStyle(texture);
    ctx.fillPath(outline.path());
    ctx.restore();
  }

private:
  std::unique_ptr<Projection> projection_;
  vec2i pos_ = vec2i(0,0);
  Pixbuffer texture_;
};

}  // namespace w
