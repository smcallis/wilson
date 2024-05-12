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

#include "wilson/chain_sink.h"

namespace w {

// A helper class to support stitching cut polygon loops back together.

class ChainStitcher : public ChainSink {
 private:
  static constexpr int kUnconnected = -1;

 public:
  // Clears the stitch buffer for a new shape.
  void Clear() override {
    vertices_.clear();
    nexts_.clear();
    Break();
  }

  // Break the current chain, the next vertex appended will be unconnected.
  void Break() override{
    last_ = kUnconnected;
    chain_size_ = 0;
  }

  // Appends a new vertex to the buffer, connecting it to the previous vertex.
  void Append(const R2Point& point) override {
    const int curr = vertices_.size();
    vertices_.emplace_back(point);
    ++chain_size_;

    if (last_ != kUnconnected) {
      nexts_[last_] = curr;
    }

    nexts_.emplace_back(kUnconnected);
    last_ = curr;
  }

  // Appends a span of vertices to the buffer.
  void Append(absl::Span<const R2Point> points) override {
    if (points.empty()) {
      return;
    }

    // Append first vertex manually.  We definite have a last_ now.
    Append(points[0]);

    // Append the rest of the points.
    for (int i = 1, n = points.size(); i < n; ++i) {
      const int curr = vertices_.size();
      vertices_.emplace_back(points[i]);
      nexts_[last_] = curr;
      nexts_.emplace_back(kUnconnected);
      last_ = curr;
    }
    chain_size_ += (points.size() - 1);
  }

  ssize_t ChainSize() const {
    return chain_size_;
  }

  // Returns the current number of vertices appended.
  int size() const {
    return vertices_.size();
  }

  // Returns true if the buffer is empty.
  bool empty() const {
    return size() == 0;
  }

  // Returns true if we're at the start of a chain, i.e. the last vertex is the
  // unconnected vertex.
  bool start_of_chain() const {
    return last_ == kUnconnected;
  }

  // Returns the last vertex appended.
  const R2Point& back() const { return vertices_.back(); }

  // Removes the last vertex added (if any).
  void pop_back() {
    vertices_.pop_back();
    nexts_.pop_back();
    last_ = vertices_.size() - 1;
  }

  // Make vertex B the next vertex after A.  Returns B.
  int Connect(int a, int b) {
    return nexts_[a] = b;
  }

  // Emits each connected chain to the given callback.  Returns false if an
  // infinite loop is detected in the connected vertices, true otherwise.
  bool EmitChains(absl::FunctionRef<void(absl::Span<const R2Point>)> emit) {
    const size_t total_vertices = vertices_.size();
    util::bitmap::Bitmap64 unseen(total_vertices, true);

    size_t start;
    while (unseen.FindFirstSetBit(&start)) {
      scratch_.clear();

      //fprintf(stderr, "chain\n");

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
              scratch_.emplace_back(vertices_[ii]);
            }
          }
        }

        if (!in_place) {
          scratch_.emplace_back(vertices_[cur]);
        }

        // We should emit each vertex once so if we total up to more than the
        // total number of vertices, then we must have hit an infinite.
        if (++cnt > total_vertices) {
          return false;
        }

        // fprintf(stderr, "curr: %zd  cnt: %zd  next: %zd\n", cur, cnt, nexts_[cur]);

        unseen.Set(cur, false);
        cur = nexts_[cur];
      } while (cur != kUnconnected && cur != start);

      if (in_place) {
        emit(absl::MakeSpan(&vertices_[beg], end - beg));
      } else {
        emit(scratch_);
      }
    }

    return true;
  }

  const R2Point& operator[](int index) const {
    return vertices_[index];
  }

 private:
  int chain_size_ = 0;
  int last_ = kUnconnected;

  std::vector<R2Point> vertices_;
  std::vector<int> nexts_;
  std::vector<R2Point> scratch_;
};


}
