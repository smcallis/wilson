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

#include <vector>

#include "s2/r2.h"
#include "s2/util/bitmap/bitmap.h"
#include "absl/functional/function_ref.h"

#include "wilson/geometry/vertex_sink.h"

namespace w {

// A helper class to support stitching cut polygon loops back together.

class ChainStitcher : public R2VertexSink {
 private:
  static constexpr int kUnconnected = -1;

 public:
  // Clears the stitch buffer for a new shape.
  void Clear() override {
    nodes_.clear();
    nexts_.clear();
    Break();
  }

  // Returns true when the stitcher is waiting to start a new chain.
  bool Broken() const {
    return tail_ == kUnconnected;
  }

  // Breaks the current chain, the next vertex appended will be unconnected.
  void Break() override {
    tail_ = kUnconnected;
  }

  // Close the current chain, the next vertex appended will be unconnected.
  void Close() override {
    if (Broken()) {
      return;
    }

    // If the tail is a repeat of the head, pop the tail before connecting.
    if (tail_ > head_ && nodes_[tail_] == nodes_[head_]) {
      PopBack();
    }
    Connect(tail_, head_);
    Break();
  }

  // Appends a new vertex to the buffer, connecting it to the previous vertex.
  void Append(const R2Point& point) override {
    const int next = nodes_.size();

    if (Broken()) {
      head_ = next;  // Save start of new chain.
    } else {
      Connect(tail_, next);
    }

    nodes_.emplace_back(point);
    nexts_.emplace_back(kUnconnected);
    tail_ = next;
  }

  // Appends a span of vertices to the buffer.
  void Append(absl::Span<const R2Point> points) override {
    for (const R2Point& point : points) {
      Append(point);
    }
  }

  ssize_t Size() const override {
    return nodes_.size();
  }

  ssize_t ChainSize() const override {
    return Broken() ? 0 : Size() - head_;
  }

  // Returns the index of the last vertex (may be kUnconnected if there is none.
  int LastVertex() const {
    return Size() - 1;
  }

  // Returns the index the next vertex added will have.
  int NextVertex() const {
    return Size();
  }

  // Returns the last vertex appended.
  const R2Point& Back() const { return nodes_.back(); }

  // Returns the first vertex appended.
  const R2Point& Front() const { return nodes_.front(); }

  // Removes the last vertex added (if any).
  void PopBack() {
    nodes_.pop_back();
    nexts_.pop_back();
    tail_ = nodes_.size() - 1;
  }

  // Make vertex B the next vertex after A.  Returns B.
  int Connect(int a, int b) {
    return nexts_[a] = b;
  }

  // Emits each connected chain to the given callback.  Returns false if an
  // infinite loop is detected in the connected vertices, true otherwise.
  bool EmitChains(absl::FunctionRef<void(absl::Span<const R2Point>)> emit) {
    const size_t total_vertices = nodes_.size();
    util::bitmap::Bitmap64 unseen(total_vertices, true);

    size_t start;
    while (unseen.FindFirstSetBit(&start)) {
      scratch_.clear();

      // We'll try to emit vertices in-place from the vertices array.
      bool in_place = true;

      ssize_t cnt = 0;
      ssize_t cur = start;
      ssize_t beg = cur;
      ssize_t end = beg;

      do {
        if (in_place) {
          if (cur == end) {
            ++end;
          } else {
            // Disjoint vertex, punt and switch to out of place mode.
            in_place = false;
            for (ssize_t ii = beg; ii < end; ++ii) {
              scratch_.emplace_back(nodes_[ii]);
            }
          }
        }

        if (!in_place) {
          scratch_.emplace_back(nodes_[cur]);
        }

        // We should emit each vertex once so if we total up to more than the
        // total number of vertices, then we must have hit an infinite loop.
        if (++cnt > total_vertices) {
          return false;
        }

        unseen.Set(cur, false);
        cur = nexts_[cur];

      } while (cur != kUnconnected && cur != start);

      if (in_place) {
        emit(absl::MakeSpan(&nodes_[beg], end - beg));
      } else {
        emit(scratch_);
      }
    }

    return true;
  }

  const R2Point& operator[](int index) const {
    return nodes_[index];
  }

 private:
  int head_ = 0;
  int tail_ = kUnconnected;

  std::vector<R2Point> nodes_;
  std::vector<int>     nexts_;
  std::vector<R2Point> scratch_;
};

#ifdef DOCTEST_LIBRARY_INCLUDED

TEST_SUITE("ChainStitcher") {
  TEST_CASE("Default Instance") {
    ChainStitcher stitcher;
    TCHECK(stitcher.Empty());
    TCHECK(stitcher.Size() == 0);

    // When created there's no active chain.
    TCHECK(stitcher.Broken());
  }

  TEST_CASE("Appending Points") {
    ChainStitcher stitcher;

    std::vector<R2Point> points = {{0, 0}, {2, 2}, {1, 2}};
    stitcher.Append(points);
    TCHECK(!stitcher.Broken());
    TCHECK(!stitcher.Empty());
    TCHECK(stitcher.Size() == points.size());
    TCHECK(stitcher.ChainSize() == points.size());

    SUBCASE("Break Is Idempotent") {
      stitcher.Break();
      TCHECK(stitcher.Broken());
      TCHECK(!stitcher.Empty());
      stitcher.Break();
      TCHECK(stitcher.Broken());
    }

    SUBCASE("Can Clear") {
      stitcher.Clear();
      TCHECK(stitcher.Empty());
      TCHECK(stitcher.Size() == 0);
      TCHECK(stitcher.ChainSize() == 0);
    }

    SUBCASE("Not Naturally Closed") {
      TCHECK(!stitcher.Broken());
    }

    SUBCASE("Can Close Manually") {
      stitcher.Close();
      TCHECK(stitcher.Broken());
      TCHECK(stitcher.Size() == points.size());
      TCHECK(stitcher.ChainSize() == 0);
    }
  }

  TEST_CASE("Natural Closure") {
    ChainStitcher stitcher;

    std::vector<R2Point> points = {{0, 0}, {2, 2}, {1, 2}, {0, 0}};
    stitcher.Append(points);
    TCHECK(!stitcher.Broken());
    TCHECK(!stitcher.Empty());
    TCHECK(stitcher.Size() == points.size());
    TCHECK(stitcher.ChainSize() == points.size());

    stitcher.Close();
    TCHECK(stitcher.Size() == points.size() - 1);
    TCHECK(stitcher.ChainSize() == 0);

    stitcher.EmitChains([&](auto pnts) {
      TCHECK(pnts.size() == points.size() - 1);
    });
  }
}

#endif

}  // namespace w
