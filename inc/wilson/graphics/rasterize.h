// Copyright 2025 Google LLC
// Author: smcallis@google.com (Sean McAllister)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/log/check.h"
#include "absl/types/span.h"
#include "s2/r2.h"
#include "wilson/util/fixed.h"

namespace w {

// An implementation of a scan line polygon rasterization algorithm, akin to the
// algorithms in Anti-Grain Geometry (AGG), Blend2D, FreeType, and others. Takes
// 2D edges which form closed polygon loops and yields horizontal Spans defining
// coverage information for contiguous ranges of pixels.
//
// The coverage value in a Span may be greater than 1, indicating multiple loops
// intersected the span. Users must implement an appropriate fill rule to handle
// this for their particular use case (see even-odd vs non-zero fill rules).
//
// Although simple, this is a high quality software rasterizer that's capable of
// processing complex geometry at high frame rates on modern hardware.
//
// Usage:
// ‾‾‾‾‾‾
//   Rasterizer rasterizer;
//   for (int i = 0, size = points.size(); i < size - 1; ++i) {
//      rasterizer.AddEdge(points[i], points[i+1]);
//   }
//   rasterizer.AddEdge(points.back(), points[0]);
//
//   rasterizer.Sweep([](const Rasterizer::Span& span) {
//      // process each span of pixels.
//   });
//
// Theory
// ‾‾‾‾‾‾
// A good write up of the algorithm is available at:
//
//    https://projects.tuxee.net/cl-vectors/section-the-cl-aa-algorithm
//
// The rasterizer takes 2D edges and creates Cells from them. A Cell is a struct
// that represents the coverage information for a one pixel. It stores four bits
// of information: the x and y coordinates of the cell, a cover and an area:
//
//                            area
//                             |
//                            ┌───•─┐ ┬
//                            │█▛╱  │ │ cover
//                            │▛╱   │ │
//                          y └•────┘ ┴
//                            x
//                            ├─────┤
//                             width
//
// These terms are inherited from the original algorithm:
//
//    x,y   - The x,y coordinate of the cell (really just a pixel coordinate).
//    cover - The -signed- vertical span of a line segment within a cell.
//    area  - The -signed- area to the -left- of a line segment within a cell.
//
// To create Spans, we accumulate area and cover values by working left-to-right
// along each Y coordinate (a scan line). Area and cover are signed so they will
// naturally sum to zero on each scan line, assuming closed watertight loops.
//
// Summing yields a net cover and area value at each cell. We must convert these
// values into a final pixel coverage that can be used to shade the pixels. Note
// that width * cover is the total area of the piece of the pixel spanned by the
// segment:
//
//                            ┌─────┐
//                            │──•──│ ┬
//            width * cover ->│██│██│ │ cover
//                            └──•──┘ ┴
//                            ├─────┤
//                             width
//
// Since area is the area -left- of the segment, we can subtract from this value
// to get the area -right- of the segment: width * cover - area. Since area is a
// fractional value of the total pixel area, width is normalized to one, and the
// final coverage formula is just: cover - area.
//
// Given all this, the algorithm itself is simple:
//
//   1. AddEdge()   - Process each edge in the polygon, generating cells.
//   2. SortCells() - Sort the cells first by Y then by X.
//   3. Sweep()     - Scan along each Y coordinate generating Spans.
//
// The difficult parts of the algorithm are purely numeric. Given a closed loop,
// we need to ensure that the cells we generate cancel exactly to zero along any
// given scanline the loop intersects.

class Rasterizer {
 public:
  // We use a 24.8 fixed point value for coordinates and coverage. Pixel area is
  // a 16.16 fixed point value to prevent precision loss summing over scanlines.
  using fix32p8 = Fixed32<8>;
  using fix32p16 = Fixed32<16>;

 private:
  // Limit to total line length to prevent overflow in intermediate products.
  static constexpr fix32p8 kLengthLimit = 16384;

 public:
  // The sub-pixel precision in bits.
  static constexpr int kSubpixelShift = fix32p8::kPrecision;

  // Maximum absolute coordinate value, in pixels.
  static constexpr int kMinCoordinate = fix32p8::MinInteger();
  static constexpr int kMaxCoordinate = fix32p8::MaxInteger();

  // A Span is a contiguous set of pixels with constant Y value and coverage.
  struct Span {
    Span() = default;
    Span(int y, int32_t xbeg, int32_t xlen, fix32p8 coverage)
        : y(y), xbeg(xbeg), xlen(xlen), coverage(coverage) {}

    // A single pixel span.
    static Span Single(int32_t y, int32_t x, fix32p8 coverage) {
      return {y, x, 1, coverage};
    }

    template <typename Sink>
    friend void AbslStringify(Sink& sink, const Span& s) {
      absl::Format(  //
        &sink, "%d [%d, %d) - %v", s.y, s.xbeg, s.xbeg + s.xlen, s.coverage);
    }

    bool operator==(const Span& b) const {
      return y == b.y &&        //
             xbeg == b.xbeg &&  //
             xlen == b.xlen &&  //
             coverage == b.coverage;
    }

    int32_t y, xbeg, xlen;
    fix32p8 coverage;
  };

  // Resets rasterizer state for a new set of geometry.
  void Reset() {
    rows_.clear();
    cells_.clear();
    sorted_.clear();

    ymin_ = std::numeric_limits<int>::max();
    ymax_ = std::numeric_limits<int>::min();
  }

  // Processes a line segment into cells.
  void AddEdge(const R2Point& p0, const R2Point& p1);

  // Adds edges from a loop, which will be implicitly closed.
  void AddLoop(absl::Span<const R2Point> points) {
    for (int i = 0, size = points.size(); i < size - 1; ++i) {
      AddEdge(points[i], points[i+1]);
    }
    AddEdge(points.back(), points[0]);
  }

  // Processes a line segment int cells, operating on fix32p8 coordinates.
  void AddEdge(fix32p8 x0, fix32p8 y0, fix32p8 x1, fix32p8 y1);

  // Processes cells to generate Spans.
  template <typename Callable>
  void Sweep(Callable&& emit);

 private:
  friend class RasterizerTest;

  struct DivMod {
    fix32p8 quot;
    fix32p16 rem;
  };

  // Compute a * b / c and a * b % c while maintaining intermediate precision.
  //
  // Results are calculated s.t. modulus is always positive.
  static constexpr DivMod MulDivMod(  //
      fix32p8 fa, fix32p8 fb, fix32p8 fc) ABSL_ATTRIBUTE_ALWAYS_INLINE {
    DCHECK_LE(Abs(fa), 1);
    DCHECK_LE(Abs(fb), kLengthLimit);
    DCHECK_LE(0, fc);
    DCHECK_LE(fc, kLengthLimit);

    const int32_t a = fa.Raw();
    const int32_t b = fb.Raw();
    const int32_t c = fc.Raw();

    int32_t prod = a * b;
    int32_t quot = prod / c;
    int32_t rest = prod % c;
    if (rest < 0) {
      quot--;
      rest += c;
    }

    return {fix32p8::FromRaw(quot), fix32p16::FromRaw(rest)};
  };

  // A fixed point implementation of Bresenham's algorithm.
  //
  // Bresenham can operate with either x or y as the dominant axis. To avoid any
  // confusion when operating in the opposite mode, we name our axes differently
  // than the standard x/y. We take the convention that the dominant axis is the
  // 'A' axis and the dependent axis the 'B' axis (also called major and minor).
  //
  // The function iterates in the major axis and yields ranges with fractional B
  // values. Each range of B values thus produced is guaranteed to correspond to
  // a single A value.
  //
  // E.g.: An actual use for this function is to split a  into horizontal 'runs'
  // where the A axis is Y and B axis is X:
  //
  //
  //      A3 │                             ------------
  //      A2 │                   -----------
  //      A1 │            -------
  // Y/A  A0 │     --------
  //          ─────────────────────────────────────────
  //          X/B  B0     B1    B2         B3         B4
  //
  // The algorithm will produce four spans:
  //
  //         (A0, [B0, B1))
  //         (A1, [B1, B2))
  //         (A2, [B1, B2))
  //         (A3, [B1, B2))
  //
  // Theory
  // ‾‾‾‾‾‾
  // Given dA and dB representing the slope of a line segment, and a step in the
  // A direction we'll call Astep, then we can compute the corresponding step in
  // the B direction simply enough:
  //
  //                         Bstep = Astep * dB / dA
  //
  // This value will have some rounding error in it because of truncation in the
  // division. We want to be sure that we account for this error in later steps,
  // so we'll compute it explicitly as the remainder:
  //
  //                         error = Astep * dB % dA
  //
  // When working with 8 bit fixed-point values, the error term is actually a 16
  // bit value (because the fraction is doubled by the multiply). Thus the error
  // that we track is sub-sub-pixel in resolution.
  //
  // The core insight of Bresenham is that we can track the error of the current
  // line from the true line, and increment the dependent axis whenever we amass
  // too much error. We do that by starting with the negative of dA:
  //
  //                         error = -dA;
  //
  // At each A step, we add dB to the error. After dA/dB steps the error will be
  // positive, at which point we step by an extra subpixel in the B direction to
  // compensate, and reset the error:
  //
  //                        if (error >= 0) {
  //                           ++Bstep;
  //                           error -= dA;
  //                        }
  //
  template <typename Emit>
  void Bresenham(  //
      int32_t ib, fix32p8 a0, fix32p8 a1, fix32p8 b0, fix32p8 b1, fix32p8 da,
      fix32p8 db, DivMod db_div_da, Emit&& emit) ABSL_ATTRIBUTE_ALWAYS_INLINE {
    using std::swap;

    if (db == 0) {
      return;
    }

    // Negate values if we're moving backwards.
    fix32p8 astep = 1 - a0.Fract();
    fix32p8 alast = 0;
    fix32p8 anext = 1;
    int32_t istep = 1;
    if (da < 0) {
      swap(alast, anext);
      astep = 1 - astep;
      istep = -istep;
      da = -da;
    }

    int32_t ia = a0.ToInteger();
    fix32p8 bb = b0;

    // If entire segment fits in one pixel, just emit it.
    if (a0.Whole() == a1.Whole()) {
      emit(ia, ib, a0.Fract(), a1.Fract(), b0, b1);
      return;
    }

    // Compute the first (potentially partially) covered cell.
    auto [cover, error] = MulDivMod(astep, db, da);
    emit(ia, ib, a0.Fract(), anext, bb, bb + cover);
    ia += istep;
    bb += cover;

    const auto da16 = fix32p16::FromRaw(da.Raw());
    const int32_t ia1 = a1.ToInteger();

    if (ia != ia1) {
      // Compute step and fractional step for one pixel x increment.
      auto [step, frac] = db_div_da;
      error -= da16;

      while (ia != ia1) {
        cover = step;
        error += frac;

        // If error went positive, bump cover by one sub-pixel to compensate.
        if (error >= 0) {
          error -= da16;
          cover += fix32p8::Epsilon();
        }

        emit(ia, ib, alast, anext, bb, bb + cover);
        ia += istep;
        bb += cover;
      }
    }

    // Emit a final partial cell.
    emit(ia, ib, alast, a1.Fract(), bb, b1);
  };

  struct Cell {
    // NOTE: We explicitly avoid zero initialization of this struct for
    // performance reasons. A default instance will have undefined values.
    Cell() {};  // NOLINT
    Cell(int32_t x, int32_t y, fix32p8 cover, fix32p16 area)
        : area(area), cover(cover), x(x), y(y) {}

    template <typename Sink>
    friend void AbslStringify(Sink& sink, const Cell& c) {
      absl::Format(  //
        &sink, "(%d, %d) cover: %v area: %v", c.x, c.y, c.cover, c.area);
    }

    bool operator==(const Cell& c) const {
      return x == c.x && y == c.y && area == c.area && cover == c.cover;
    }

    fix32p16 area;
    fix32p8 cover;
    int32_t x, y;
  };

  // Represents a row of cells with constant Y coordinate.
  struct Row {
    int32_t offset = 0;
    int32_t length = 0;
  };

  // Sorts Cells into (y, x) order.
  void SortCells();

  std::vector<Cell> cells_;
  std::vector<Cell> sorted_;
  std::vector<Row> rows_;

  // Minimum and maximum cell y coordinate we've seen.
  int ymin_ = std::numeric_limits<int>::max();
  int ymax_ = std::numeric_limits<int>::min();
};

inline void Rasterizer::AddEdge(const R2Point& p0, const R2Point& p1) {
  AddEdge(  //
      fix32p8::FromFloat(p0.x()), fix32p8::FromFloat(p0.y()),
      fix32p8::FromFloat(p1.x()), fix32p8::FromFloat(p1.y()));
}

inline void Rasterizer::AddEdge(  //
    fix32p8 x0, fix32p8 y0, fix32p8 x1, fix32p8 y1) {
  using std::swap;

  // NOTE: Forcing this lambda to inline improves performance by over 50%.
  const auto AddXCell =  //
      [&](int cx, int cy, fix32p8 fx0, fix32p8 fx1, fix32p8 fy0, fix32p8 fy1)
          ABSL_ATTRIBUTE_ALWAYS_INLINE {
            if (fy0 == fy1) {
              return;
            }

            ymin_ = std::min(ymin_, cy);
            ymax_ = std::max(ymax_, cy);

            // It should be (fx0 + fx1) * dy / 2 for the area of a trapezoid.
            // Saving double the area lets us avoid the shift required to
            // divide.
            const fix32p8 dy = fy1 - fy0;
            cells_.emplace_back(cx, cy, dy, (fx0 + fx1).Mul(dy));
          };

  const auto AddYCell =  //
      [&](int cy, int cx, fix32p8 fy0, fix32p8 fy1, fix32p8 fx0, fix32p8 fx1)
          ABSL_ATTRIBUTE_ALWAYS_INLINE {
            AddXCell(cx, cy, fx0, fx1, fy0, fy1);
          };

  const fix32p8 dx = x1 - x0;
  const fix32p8 dy = y1 - y0;
  if (Abs(dx) > kLengthLimit || Abs(dy) > kLengthLimit) {
    // Note we divide before summing to avoid overflow when x0/y0 are near the
    // limits of representable coordinates.
    fix32p8 cx = (x0 >> 1) + (x1 >> 1);
    fix32p8 cy = (y0 >> 1) + (y1 >> 1);
    AddEdge(x0, y0, cx, cy);
    AddEdge(cx, cy, x1, y1);
    return;
  }

  const int32_t iy0 = y0.ToInteger();
  const int32_t iy1 = y1.ToInteger();

  // Horizontal lines don't change total cover so they can be ignored.
  if (dy == 0) {
    return;
  }

  // If the line is vertical, then just generate the cells directly.
  if (dx == 0) {
    // Adds a cell from only the y components.
    const auto AddCell = [&](int cy, fix32p8 fy0, fix32p8 fy1) {
      AddXCell(x0.ToInteger(), cy, x0.Fract(), x0.Fract(), fy0, fy1);
    };

    int32_t ystep = 1;
    fix32p8 ylast = 0;
    fix32p8 ynext = 1;
    if (dy < 0) {
      swap(ylast, ynext);
      ystep = -ystep;
    }

    // If the line is entirely within one cell we can just add it.
    if (iy0 == iy1) {
      AddCell(iy0, y0.Fract(), y1.Fract());
      return;
    }

    // Otherwise add each vertical piece.
    AddCell(iy0, y0.Fract(), ynext);
    for (int iy = iy0 + ystep; iy != iy1; iy += ystep) {
      AddCell(iy, ylast, ynext);
    }
    AddCell(iy1, ylast, y1.Fract());
    return;
  }

  // Compute these here to avoid recomputing them constantly.
  const DivMod dx_div_dy = MulDivMod(1, dx, Abs(dy));
  const DivMod dy_div_dx = MulDivMod(1, dy, Abs(dx));

  // If the line is entirely within one vertical pixel range, just add it.
  if (iy0 == iy1) {
    Bresenham(iy0, x0, x1, y0, y1, dx, dy, dy_div_dx, AddXCell);
    return;
  }

  // Use Bresenham to break out "runs" of pixels in the major axis (whichever is
  // longer). Each run is then broken out into individual cells. Using the major
  // axis first gives ~10% speedup.
  if (Abs(dx) > Abs(dy)) {
    Bresenham(
        0, y0, y1, x0, x1, dy, dx, dx_div_dy,
        [&](int cy, int, fix32p8 fy0, fix32p8 fy1, fix32p8 x0, fix32p8 x1) {
          Bresenham(cy, x0, x1, fy0, fy1, dx, dy, dy_div_dx, AddXCell);
        });
  } else {
    Bresenham(
        0, x0, x1, y0, y1, dx, dy, dy_div_dx,
        [&](int cx, int, fix32p8 fx0, fix32p8 fx1, fix32p8 y0, fix32p8 y1) {
          Bresenham(cx, y0, y1, fx0, fx1, dy, dx, dx_div_dy, AddYCell);
        });
  }
}

void Rasterizer::SortCells() {
  // Sort cells by Y coordinate, then by X coordinate. We have an advantage that
  // we know the range of Y that we actually saw, and it's a small finite set so
  // we can sort by Y using a bucketing sort. Then sort each bucket by X.
  //
  // For most inputs, there will only be a couple cells per Y bucket (often only
  // two). By doing a bucketing sort first, the final sort routine complexity is
  // much more like an O(n) sort than an O(nlog(n)) sort.
  if (cells_.empty()) {
    return;
  }
  rows_.clear();
  sorted_.clear();

  const auto CmpCellX = [](const Cell& a, const Cell& b) {
    return a.x < b.x;
  };

  // Count how many cells are at each y coordinate.
  rows_.resize(ymax_ - ymin_ + 1);
  for (const Cell& cell : cells_) {
    ++rows_[cell.y - ymin_].length;
  }

  // Integrate row lengths to get offsets in the sorted array.
  int offset = 0;
  for (Row& row : rows_) {
    row.offset = offset;
    offset += row.length;
    row.length = 0;  // Used as a running offset below.
  }

  // Scan the cell array, put each cell into the correct Y bucket.
  sorted_.resize(cells_.size());
  for (const Cell& cell : cells_) {
    Row& row = rows_[cell.y - ymin_];
    sorted_[row.offset + row.length] = cell;
    ++row.length;
  }

  // Now sort each Y bucket by X coordinate.
  Cell* cell = &sorted_[0];
  for (const Row& row : rows_) {
    std::sort(cell, cell + row.length, CmpCellX);
    cell += row.length;
  }
}

template <typename Callable>
void Rasterizer::Sweep(Callable&& emit) {
  static_assert(
      std::is_convertible_v<Callable, std::function<void(const Span&)>>,
      "emit function must take a Span");

  SortCells();

  for (const Row& row : rows_) {
    const Cell* cell = &sorted_[row.offset];
    const Cell* end = cell + row.length;

    // Running totals for the current row.
    fix32p16 area = 0;
    fix32p8 cover = 0;

    while (cell < end) {
      const int cx = cell->x;
      const int cy = cell->y;

      // Aggregate cells with the same X coordinate together.
      while ((cell < end) && (cell->x == cx)) {
        area += cell->area;
        cover += cell->cover;
        ++cell;
      }

      // ----------------
      // Emit a single-pixel span with coverage for the current cell.

      // We store area doubled in a 16.16 fixed point value. We need to round it
      // to 8 bits and divide by two, which we can do by rounding to 7 bits.
      const fix32p8 rounded_area = fix32p8::FromRaw(area.RoundTo<7>().Raw());

      // Area is to the -left- of the geometry as it comes through the cell. But
      // we want to compute alpha based on the area to the -right-. We know that
      // in the cell the line has a vertical extent of 'cover' (a fraction). The
      // fraction of the cell's area corresponding to cover is just 1px * cover,
      // from which we can subtract rounded_area to get the final cell coverage.
      //
      //  ┌──────────────────┐
      //  │                  ╱ ┬
      //  │░░░░░░░░░░░░░░░░░╱│ │
      //  │░░░░░░░░░░░░░░░░╱ │ │
      //  │░░░░░░░░░░░░░░░╱  │ │
      //  │░rounded_area░╱   │ │ cover
      //  │░░░░░░░░░░░░░╱    │ │
      //  │░░░░░░░░░░░░╱     │ │
      //  │░░░░░░░░░░░╱      │ │
      //  └──────────╱───────┘ ┴
      //  ├──────────────────┤
      //           1 px
      //
      const fix32p8 coverage = cover - rounded_area;
      if (coverage > 0) {
        emit(Span::Single(cy, cx, coverage));
      }

      // The coverage is constant now to the next cell.  If we're not at the end
      // of the scan line yet, and we have a non-zero coverage, then make a Span
      // containing the current coverage, extending until the next Cell.
      if (cover > 0 && cell < end && (cell->x - cx) > 1) {
        emit(Span(cy, cx + 1, cell->x - cx - 1, cover));
      }

      area = 0;
    }

    // Coverage should close to zero on each scanline.
    DCHECK_EQ(cover, 0);
  }
}

}  // namespace w
