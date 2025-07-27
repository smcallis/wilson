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
#include "wilson/graphics/rasterize.h"

#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

//#include "testing/base/public/benchmark.h"
#include "absl/log/check.h"
#include "absl/types/span.h"
#include "fuzztest/fuzztest.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "s2/r2.h"
#include "wilson/util/fixed.h"

namespace w {

using ::testing::ElementsAre;
using ::testing::ElementsAreArray;

class RasterizerTest : public ::testing::Test {
 public:
  // using Coordinate = Rasterizer::Coordinate;
  using Span = Rasterizer::Span;
  using fix32p8 = Rasterizer::fix32p8;
  using fix32p16 = Rasterizer::fix32p16;

  // Constructs a cell from a higher precision fixed value.
  Rasterizer::Cell Cell(int x, int y, fix32p8 cover, fix32p8 area) {
    return {x, y, cover, fix32p16::FromRaw(area.Raw() << 9)};
  }

  static constexpr fix32p8 kMaxCover = 1;
  static constexpr fix32p8 kMaxArea = 1;

  // Rasterizes the given polygon loops and return PixelRanges for them.
  void Rasterize(absl::Span<const std::vector<R2Point>> loops) {
    rasterizer_.Reset();
    for (const auto& vertices : loops) {
      for (int i = 0, size = vertices.size(); i < size - 1; ++i) {
        rasterizer_.AddEdge(vertices[i], vertices[i + 1]);
      }
    }
    rasterizer_.SortCells();
  }

  // Rasterize the given polygon loops after reflecting them over the y == x
  // line and returns PixelRanges for them.
  void RasterizeFlipped(absl::Span<const std::vector<R2Point>> loops) {
    const auto Flip = [](const R2Point& point) {
      return R2Point(point.y(), point.x());
    };

    rasterizer_.Reset();
    for (const auto& vertices : loops) {
      for (int i = vertices.size() - 1; i > 0; --i) {
        rasterizer_.AddEdge(Flip(vertices[i]), Flip(vertices[i - 1]));
      }
    }
    rasterizer_.SortCells();
  }

  // Returns the cells in the rasterizer.
  absl::Span<const Rasterizer::Cell> Cells() const {
    return rasterizer_.sorted_;
  }

  std::vector<Span> Sweep() {
    std::vector<Span> spans;
    rasterizer_.Sweep([&](const auto& span) { spans.emplace_back(span); });
    return spans;
  }

 private:
  Rasterizer rasterizer_;
};

TEST_F(RasterizerTest, VerticalLine) {
  // Rasterize a simple vertical line, upwards in y.
  Rasterize({{{0.5, 0.5}, {0.5, 2.5}}});
  EXPECT_THAT(Cells(), ElementsAre(Cell(0, 0, kMaxCover / 2, kMaxArea / 4),
                                   Cell(0, 1, kMaxCover / 1, kMaxArea / 2),
                                   Cell(0, 2, kMaxCover / 2, kMaxArea / 4)));

  // Rasterize downwards, cover and area signs should flip.
  Rasterize({{{0.5, 2.5}, {0.5, 0.5}}});
  EXPECT_THAT(Cells(), ElementsAre(Cell(0, 0, -kMaxCover / 2, -kMaxArea / 4),
                                   Cell(0, 1, -kMaxCover / 1, -kMaxArea / 2),
                                   Cell(0, 2, -kMaxCover / 2, -kMaxArea / 4)));

  // Rasterize upwards but in negative coordinates.
  Rasterize({{{0.5, -2.5}, {0.5, -0.5}}});
  EXPECT_THAT(Cells(), ElementsAre(Cell(0, -3, kMaxCover / 2, kMaxArea / 4),
                                   Cell(0, -2, kMaxCover / 1, kMaxArea / 2),
                                   Cell(0, -1, kMaxCover / 2, kMaxArea / 4)));

  // Rasterize downwards but in negative coordinates.
  Rasterize({{{0.5, -0.5}, {0.5, -2.5}}});
  EXPECT_THAT(Cells(), ElementsAre(Cell(0, -3, -kMaxCover / 2, -kMaxArea / 4),
                                   Cell(0, -2, -kMaxCover / 1, -kMaxArea / 2),
                                   Cell(0, -1, -kMaxCover / 2, -kMaxArea / 4)));

  // Rasterize upwards but straddling positive and negative.
  Rasterize({{{0.5, -1.5}, {0.5, +1.5}}});
  EXPECT_THAT(Cells(), ElementsAre(Cell(0, -2, kMaxCover / 2, kMaxArea / 4),
                                   Cell(0, -1, kMaxCover / 1, kMaxArea / 2),
                                   Cell(0, 0, kMaxCover / 1, kMaxArea / 2),
                                   Cell(0, +1, kMaxCover / 2, kMaxArea / 4)));

  // Rasterize upwards a single pixel.
  Rasterize({{{0.5, 0.0}, {0.5, 1.0}}});
  EXPECT_THAT(Cells(), ElementsAre(Cell(0, 0, +kMaxCover, +kMaxArea / 2)));

  // Rasterize downwards a single pixel.
  Rasterize({{{0.5, 1.0}, {0.5, 0.0}}});
  EXPECT_THAT(Cells(), ElementsAre(Cell(0, 0, -kMaxCover, -kMaxArea / 2)));

  // Rasterize upwards a single pixel in negative coordinates.
  Rasterize({{{0.5, -1.0}, {0.5, 0.0}}});
  EXPECT_THAT(Cells(), ElementsAre(Cell(0, -1, +kMaxCover, +kMaxArea / 2)));

  // Rasterize downwards a single pixel in negative coordinates.
  Rasterize({{{0.5, 0.0}, {0.5, -1.0}}});
  EXPECT_THAT(Cells(), ElementsAre(Cell(0, -1, -kMaxCover, -kMaxArea / 2)));
}

TEST_F(RasterizerTest, DiagonalPixel) {
  // Rasterize across the diagonal of a pixel.
  Rasterize({{{0.0, 0.0}, {1.0, 1.0}}});
  EXPECT_THAT(Cells(), ElementsAre(Cell(0, 0, +kMaxCover, +kMaxArea / 2)));

  Rasterize({{{1.0, 1.0}, {0.0, 0.0}}});
  EXPECT_THAT(Cells(), ElementsAre(Cell(0, 0, -kMaxCover, -kMaxArea / 2)));

  Rasterize({{{0.0, 1.0}, {1.0, 0.0}}});
  EXPECT_THAT(Cells(), ElementsAre(Cell(0, 0, -kMaxCover, -kMaxArea / 2)));

  Rasterize({{{1.0, 0.0}, {0.0, 1.0}}});
  EXPECT_THAT(Cells(), ElementsAre(Cell(0, 0, +kMaxCover, +kMaxArea / 2)));

  // Rasterize across the diagonal of a pixel in negative coordinates.
  Rasterize({{{0.0, 0.0}, {-1.0, -1.0}}});
  EXPECT_THAT(Cells(), ElementsAre(Cell(-1, -1, -kMaxCover, -kMaxArea / 2)));

  Rasterize({{{-1.0, -1.0}, {0.0, 0.0}}});
  EXPECT_THAT(Cells(), ElementsAre(Cell(-1, -1, +kMaxCover, +kMaxArea / 2)));

  Rasterize({{{0.0, -1.0}, {-1.0, 0.0}}});
  EXPECT_THAT(Cells(), ElementsAre(Cell(-1, -1, +kMaxCover, +kMaxArea / 2)));

  Rasterize({{{-1.0, 0.0}, {0.0, -1.0}}});
  EXPECT_THAT(Cells(), ElementsAre(Cell(-1, -1, -kMaxCover, -kMaxArea / 2)));

  // Rasterize from midpoint to midpoint of pixels along the diagonal.
  Rasterize({{{0.5, 0.5}, {2.5, 2.5}}});
  EXPECT_THAT(Cells(),
              ElementsAre(Cell(0, 0, +kMaxCover / 2, +3 * kMaxArea / 8),
                          Cell(1, 1, +kMaxCover / 1, +kMaxArea / 2),
                          Cell(2, 2, +kMaxCover / 2, +1 * kMaxArea / 8)));

  // Rasterize from midpoint to midpoint of pixels downwards;
  Rasterize({{{2.5, 2.5}, {0.5, 0.5}}});
  EXPECT_THAT(Cells(),
              ElementsAre(Cell(0, 0, -kMaxCover / 2, -3 * kMaxArea / 8),
                          Cell(1, 1, -kMaxCover / 1, -kMaxArea / 2),
                          Cell(2, 2, -kMaxCover / 2, -1 * kMaxArea / 8)));

  Rasterize({{{-2.5, -2.5}, {-0.5, -0.5}}});
  EXPECT_THAT(Cells(),
              ElementsAre(Cell(-3, -3, +kMaxCover / 2, +3 * kMaxArea / 8),
                          Cell(-2, -2, +kMaxCover / 1, +kMaxArea / 2),
                          Cell(-1, -1, +kMaxCover / 2, +1 * kMaxArea / 8)));

  Rasterize({{{-0.5, -0.5}, {-2.5, -2.5}}});
  EXPECT_THAT(Cells(),
              ElementsAre(Cell(-3, -3, -kMaxCover / 2, -3 * kMaxArea / 8),
                          Cell(-2, -2, -kMaxCover / 1, -kMaxArea / 2),
                          Cell(-1, -1, -kMaxCover / 2, -1 * kMaxArea / 8)));
}

TEST_F(RasterizerTest, SimpleShape) {
  // A diamond shape covering four pixels:
  //
  //       2 ┌─────┬─────┐
  //         │   ⟋ │⟍    │
  //         │ ⟋ ▟█│█▙⟍  │
  //       1 ├─────┼─────┤
  //         │ ⟍ ▜█│█▛⟋  │
  //         │   ⟍ │⟋    │
  //       0 └─────┴─────┘
  //         0     1     2
  //
  const std::vector<R2Point> kVertices = {
      {1, 2}, {2, 1}, {1, 0}, {0, 1}, {1, 2}};

  std::vector<Span> expected_ranges = {
      Span(0, 0, 1, kMaxArea / 2), Span(0, 1, 1, kMaxArea / 2),
      Span(1, 0, 1, kMaxArea / 2), Span(1, 1, 1, kMaxArea / 2)};

  // Flipping shouldn't affect the ranges since it's symmetric.
  Rasterize({kVertices});
  EXPECT_THAT(Sweep(), ElementsAreArray(expected_ranges));

  RasterizeFlipped({kVertices});
  EXPECT_THAT(Sweep(), ElementsAreArray(expected_ranges));
}

TEST_F(RasterizerTest, ShapeWithFullPixels) {
  // A diamond shape covering eight pixels, include four full ones.  The end
  // pixels should be separate ranges and the middle two pixels on each line
  // should be one range.
  //
  //       2 ┌─────┬─────┬─────┬─────┐
  //         │   ⟋ │█████│█████│⟍    │
  //         │ ⟋ ▟█│█████│█████│█▙⟍  │
  //       1 ├─────┼─────┼─────┼─────┤
  //         │ ⟍ ▜█│█████│█████│█▛⟋  │
  //         │   ⟍ │█████ █████│⟋    │
  //       0 └─────┴─────┴─────┴─────┘
  //         0     1     2     3     4
  //
  const std::vector<R2Point> kVertices = {{0, 1}, {1, 2}, {3, 2}, {4, 1},
                                          {3, 0}, {1, 0}, {0, 1}};

  // Rasterizing the regular geometry scans across rows in increasing Y.
  Rasterize({kVertices});
  EXPECT_THAT(  //
      Sweep(), ElementsAre(
                   // y == 0
                   Span(0, 0, 1, kMaxArea / 2), Span(0, 1, 2, kMaxArea),
                   Span(0, 3, 1, kMaxArea / 2),

                   // y == 1
                   Span(1, 0, 1, kMaxArea / 2), Span(1, 1, 2, kMaxArea),
                   Span(1, 3, 1, kMaxArea / 2)));

  // Rasterizing the flipped geometry scans up columns in increasing X.
  RasterizeFlipped({kVertices});
  EXPECT_THAT(  //
      Sweep(), ElementsAre(
                   // x == 0
                   Span(0, 0, 1, kMaxArea / 2), Span(0, 1, 1, kMaxArea / 2),

                   // x == 1
                   Span(1, 0, 1, kMaxArea), Span(1, 1, 1, kMaxArea),

                   // x == 2
                   Span(2, 0, 1, kMaxArea), Span(2, 1, 1, kMaxArea),

                   // x == 3
                   Span(3, 0, 1, kMaxArea / 2), Span(3, 1, 1, kMaxArea / 2)));
}

TEST_F(RasterizerTest, MultipleLoops) {
  // Two diamond shapes:
  //
  //       4 ┌─────┬─────┬─────┬─────┬─────┬─────┐
  //         │     │     │     │    ╱│╲    │     │
  //         │     │     │     │   ╱▟│▙╲   │     │
  //       3 ├─────┼─────┼─────┼─────┼─────┼─────┤
  //         │     │     │     │ ╱▟██│██▙╲ │     │
  //         │     │     │     │╱▟███│███▙╲│     │
  //       2 ├─────┴─────┼─────┼─────┼─────┼─────┤
  //         │   ⟋ │⟍    │     │╲▜███│███▛╱│     │
  //         │ ⟋ ▟█│█▙⟍  │     │ ╲▜██│██▛╱ │     │
  //       1 ├─────┼─────┼─────┼─────┼─────┼─────┤
  //         │ ⟍ ▜█│█▛⟋  │     │   ╲▜│▛╱   │     │
  //         │   ⟍ │⟋    │     │    ╲│╱    │     │
  //       0 └─────┴─────┴─────┴─────┴─────┴─────┘
  //         0     1     2     3     4     5     6
  //
  const std::vector<R2Point> kLoops[] = {
      {{0, 1}, {1, 2}, {2, 1}, {1, 0}, {0, 1}},
      {{3, 2}, {4, 4}, {5, 2}, {4, 0}, {3, 2}}};

  // Rasterizing the regular geometry scans across rows with increasing Y.
  Rasterize(kLoops);
  EXPECT_THAT(
      Sweep(),
      ElementsAre(
          // y == 0
          Span(0, 0, 1, kMaxArea / 2), Span(0, 1, 1, kMaxArea / 2),
          Span(0, 3, 1, kMaxArea / 4), Span(0, 4, 1, kMaxArea / 4),

          // y == 1
          Span(1, 0, 1, kMaxArea / 2), Span(1, 1, 1, kMaxArea / 2),
          Span(1, 3, 1, 3 * kMaxArea / 4), Span(1, 4, 1, 3 * kMaxArea / 4),

          // y == 2
          Span(2, 3, 1, 3 * kMaxArea / 4), Span(2, 4, 1, 3 * kMaxArea / 4),

          // y == 3
          Span(3, 3, 1, kMaxArea / 4), Span(3, 4, 1, kMaxArea / 4)));

  // Rasterizing the flipped geometry scans up columns in increasing X.
  RasterizeFlipped(kLoops);
  EXPECT_THAT(
      Sweep(),
      ElementsAre(
          // x == 0
          Span(0, 0, 1, kMaxArea / 2), Span(0, 1, 1, kMaxArea / 2),

          // x == 1
          Span(1, 0, 1, kMaxArea / 2), Span(1, 1, 1, kMaxArea / 2),

          // x == 3
          Span(3, 0, 1, kMaxArea / 4), Span(3, 1, 1, 3 * kMaxArea / 4),
          Span(3, 2, 1, 3 * kMaxArea / 4), Span(3, 3, 1, kMaxArea / 4),

          // x == 4
          Span(4, 0, 1, kMaxArea / 4), Span(4, 1, 1, 3 * kMaxArea / 4),
          Span(4, 2, 1, 3 * kMaxArea / 4), Span(4, 3, 1, kMaxArea / 4)));
}

TEST_F(RasterizerTest, PartiallyCoveredCells) {
  // A split diamond shape that covers half pixels in the middle:
  //
  //       2 ┌─────┬─────┬─────┬─────┬─────┬─────┐
  //         │   ⟋ │⟍ ___│_____│_____│____⟋│⟍    │
  //         │ ⟋ ▟█│█████│█████│█████│█████│█▙⟍  │
  //       1 ├─────┼─────┼─────┼─────┼─────┼─────┤
  //         │ ⟍ ▜█│█████│█████│█████│█████│█▛⟋  │
  //         │   ⟍ │⟋    │     │     │    ⟍│⟋    │
  //       0 └─────┴─────┴─────┴─────┴─────┴─────┘
  //         0     1     2     3     4     5     6
  //
  const std::vector<R2Point> kVertices{
      {0, 1}, {1, 2},     {1.5, 1.5}, {4.5, 1.5}, {5, 2}, {6, 1},
      {5, 0}, {4.5, 0.5}, {1.5, 0.5}, {1, 0},     {0, 1}};

  // Rasterizing the regular geometry scans across rows with increasing Y.
  Rasterize({kVertices});
  EXPECT_THAT(Sweep(),
              ElementsAre(
                  // y == 0
                  Span(0, 0, 1, kMaxArea / 2), Span(0, 1, 1, 5 * kMaxArea / 8),
                  Span(0, 2, 2, kMaxArea / 2), Span(0, 4, 1, 5 * kMaxArea / 8),
                  Span(0, 5, 1, kMaxArea / 2),

                  // y == 1
                  Span(1, 0, 1, kMaxArea / 2), Span(1, 1, 1, 5 * kMaxArea / 8),
                  Span(1, 2, 2, kMaxArea / 2), Span(1, 4, 1, 5 * kMaxArea / 8),
                  Span(1, 5, 1, kMaxArea / 2)));

  // Rasterizing the flipped geometry scans up columns in increasing X.
  RasterizeFlipped({kVertices});
  EXPECT_THAT(
      Sweep(),
      ElementsAre(
          // x == 0
          Span(0, 0, 1, kMaxArea / 2), Span(0, 1, 1, kMaxArea / 2),

          // x == 1
          Span(1, 0, 1, 5 * kMaxArea / 8), Span(1, 1, 1, 5 * kMaxArea / 8),

          // x == 2
          Span(2, 0, 1, kMaxArea / 2), Span(2, 1, 1, kMaxArea / 2),

          // x == 3
          Span(3, 0, 1, kMaxArea / 2), Span(3, 1, 1, kMaxArea / 2),

          // x == 4
          Span(4, 0, 1, 5 * kMaxArea / 8), Span(4, 1, 1, 5 * kMaxArea / 8),

          // x == 5
          Span(5, 0, 1, kMaxArea / 2), Span(5, 1, 1, kMaxArea / 2)));
}

TEST_F(RasterizerTest, LongSkinnyShape) {
  // A long narrow triangle.  This helps provides coverage when the x step is
  // much less than the y step and vice-versa.
  const std::vector<R2Point> kVertices{{0.5, 2}, {8.5, 1}, {0.5, 0}, {0.5, 2}};

  // Rasterizing the regular geometry scans across rows with increasing Y.
  Rasterize({kVertices});
  EXPECT_THAT(
      Sweep(),
      ElementsAre(
          // x == 0
          Span(0, 0, 1, 31 * kMaxArea / 64), Span(0, 1, 1, 7 * kMaxArea / 8),
          Span(0, 2, 1, 3 * kMaxArea / 4), Span(0, 3, 1, 5 * kMaxArea / 8),
          Span(0, 4, 1, kMaxArea / 2), Span(0, 5, 1, 3 * kMaxArea / 8),
          Span(0, 6, 1, kMaxArea / 4), Span(0, 7, 1, kMaxArea / 8),
          Span(0, 8, 1, kMaxArea / 64),

          // x == 1
          Span(1, 0, 1, 31 * kMaxArea / 64), Span(1, 1, 1, 7 * kMaxArea / 8),
          Span(1, 2, 1, 3 * kMaxArea / 4), Span(1, 3, 1, 5 * kMaxArea / 8),
          Span(1, 4, 1, kMaxArea / 2), Span(1, 5, 1, 3 * kMaxArea / 8),
          Span(1, 6, 1, kMaxArea / 4), Span(1, 7, 1, kMaxArea / 8),
          Span(1, 8, 1, kMaxArea / 64)));

  // Rasterizing the flipped geometry scans up columns in increasing X.
  RasterizeFlipped({kVertices});
  EXPECT_THAT(
      Sweep(),
      ElementsAre(
          // x == 0
          Span(0, 0, 1, 31 * kMaxArea / 64), Span(0, 1, 1, 31 * kMaxArea / 64),

          // x == 1
          Span(1, 0, 1, 7 * kMaxArea / 8), Span(1, 1, 1, 7 * kMaxArea / 8),

          // x == 2
          Span(2, 0, 1, 3 * kMaxArea / 4), Span(2, 1, 1, 3 * kMaxArea / 4),

          // x == 3
          Span(3, 0, 1, 5 * kMaxArea / 8), Span(3, 1, 1, 5 * kMaxArea / 8),

          // x == 4
          Span(4, 0, 1, kMaxArea / 2), Span(4, 1, 1, kMaxArea / 2),

          // x == 5
          Span(5, 0, 1, 3 * kMaxArea / 8), Span(5, 1, 1, 3 * kMaxArea / 8),

          // x == 6
          Span(6, 0, 1, kMaxArea / 4), Span(6, 1, 1, kMaxArea / 4),

          // x == 7
          Span(7, 0, 1, kMaxArea / 8), Span(7, 1, 1, kMaxArea / 8),

          // x == 8
          Span(8, 0, 1, kMaxArea / 64), Span(8, 1, 1, kMaxArea / 64)));
}

// Returns a domain over points with the given X and Y limits.
fuzztest::Domain<R2Point> R2PointDomain(double xmin, double xmax, double ymin,
                                        double ymax) {
  const auto MakeR2Point = [](double x, double y) { return R2Point(x, y); };

  return fuzztest::Map(MakeR2Point,                    //
                       fuzztest::InRange(xmin, xmax),  //
                       fuzztest::InRange(ymin, ymax));
}

void CoverageCloses(std::vector<R2Point> offsets) {
  constexpr int kPointLimit = 16384;

  // Need at least three vertices to form a closed loop.
  if (offsets.size() < 3) {
    return;
  }

  // The vertices are really (dx,dy) pairs, we need to sum them to turn them
  // into vertices.
  R2Point sum(0, 0);
  int sign = +1;
  for (R2Point& offset : offsets) {
    sum += sign * offset;

    if (sum.x() > +kPointLimit || sum.y() > +kPointLimit) {
      sign = -1;
    }
    if (sum.x() < -kPointLimit || sum.y() < -kPointLimit) {
      sign = +1;
    }

    offset = sum;
  }

  Rasterizer rasterizer;
  rasterizer.AddLoop(offsets);

  int count = 0;
  rasterizer.Sweep([&](auto) { ++count; });
  //benchmark::DoNotOptimize(count);  // Make sure Sweep is not optimized out.
}
FUZZ_TEST(RasterizerFuzzing, CoverageCloses)
    .WithDomains(fuzztest::VectorOf(R2PointDomain(1, +1024, 1, +1024)));

// // Benchmark with double precision points.
// void BM_RasterizeLionDouble(benchmark::State& state) {
//   // Collect the lion loops once and hold onto them.
//   std::vector<std::vector<R2Point>> loops;
//   VisitLionLoops([&](auto, absl::Span<const R2Loop> points) {
//     for (const auto& loop : points) {
//       loops.emplace_back(loop);
//     }
//   });

//   // Count the total boundary pixels to process.
//   int64_t boundary_pixels = 0;
//   for (const std::vector<R2Point>& loop : loops) {
//     for (int i = 0, size = loop.size(); i < size - 1; ++i) {
//       boundary_pixels += std::round((loop[i+1] - loop[i]).Norm());
//     }
//   }
//   CHECK_NE(boundary_pixels, 0);

//   Rasterizer rasterizer;
//   for (auto _ : state) {
//     for (const std::vector<R2Point>& loop : loops) {
//       rasterizer.Reset();
//       for (int i = 0, size = loop.size(); i < size - 1; ++i) {
//         rasterizer.AddEdge(loop[i], loop[i+1]);
//       }
//     }

//     int count = 0;
//     rasterizer.Sweep([&](auto) {
//       ++count;
//     });

//     benchmark::DoNotOptimize(count);
//   }
//   state.SetItemsProcessed(state.iterations() * boundary_pixels);
// }
// BENCHMARK(BM_RasterizeLionDouble);

// // Benchmark with fixed precision points.
// void BM_RasterizeLionFixed(benchmark::State& state) {
//   using Fixed = Rasterizer::fix32p8;

//   struct F2Point {
//     F2Point() = default;
//     F2Point(Fixed x, Fixed y) : x(x), y(y) {}
//     Fixed x, y;
//   };

//   // Collect the lion loops once and hold onto them.
//   std::vector<std::vector<F2Point>> loops;
//   VisitLionLoops([&](auto, absl::Span<const R2Loop> r2loops) {
//     std::vector<F2Point> loop;
//     for (const auto& r2loop : r2loops) {
//       for (const R2Point& point : r2loop) {
//         loop.emplace_back(  //
//           Fixed::FromDouble(point.x()), Fixed::FromDouble(point.y()));
//       }
//       loops.emplace_back(std::move(loop));
//     }
//   });

//   // Count the total boundary pixels to process.
//   int64_t boundary_pixels = 0;
//   for (const std::vector<F2Point>& loop : loops) {
//     for (int i = 0, size = loop.size(); i < size - 1; ++i) {
//       const double dx = ToDouble(loop[i+1].x - loop[i].x);
//       const double dy = ToDouble(loop[i+1].y - loop[i].y);
//       boundary_pixels += std::round(std::sqrt(dx*dx + dy*dy));
//     }
//   }
//   CHECK_NE(boundary_pixels, 0);

//   Rasterizer rasterizer;
//   for (auto _ : state) {
//     for (const std::vector<F2Point>& loop : loops) {
//       rasterizer.Reset();
//       for (int i = 0, size = loop.size(); i < size - 1; ++i) {
//         rasterizer.AddEdge(loop[i].x, loop[i].y, loop[i+1].x, loop[i+1].y);
//       }
//     }

//     int count = 0;
//     rasterizer.Sweep([&](auto) {
//       ++count;
//     });

//     benchmark::DoNotOptimize(count);
//   }
//   state.SetItemsProcessed(state.iterations() * boundary_pixels);
// }
// BENCHMARK(BM_RasterizeLionFixed);


}  // namespace w
