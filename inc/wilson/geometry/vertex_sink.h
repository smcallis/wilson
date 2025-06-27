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

#include "absl/types/span.h"

#include "s2/r2.h"
#include "s2/s2point.h"

namespace w {

// An abstract interface for something that can receive vertices and group them
// into chains.

template <typename Point>
class VertexSink {
 public:
  virtual ~VertexSink() = default;

  // Clears the buffer to its initial state, as though nothing has been added.
  virtual void Clear() = 0;

  // End the current chain without closing it.
  //
  // Does nothing if there is no current chain.
  virtual void Break() = 0;

  // Ends the current chain and closes it by moving back to the start.
  //
  // Does nothing if there is no current chain.
  virtual void Close() = 0;

  // Appends a single point to the current chain.
  virtual void Append(const Point&) = 0;

  // Appends a span of points to the current chain.
  virtual void Append(absl::Span<const Point>) = 0;

  // Returns the total number of vertices.
  virtual ssize_t Size() const = 0;

  // Returns the size of the current chain.
  virtual ssize_t ChainSize() const = 0;

  // Returns true if the current chain is empty.
  bool ChainEmpty() const {
    return ChainSize() == 0;
  }

  // Returns true if the sink is empty.
  bool Empty() const {
    return Size() == 0;
  }
};

using R2VertexSink = VertexSink<R2Point>;
using S2VertexSink = VertexSink<S2Point>;

// Adds a new chain to the sink that's a closed circle.  The circle is segmented
// into lines such that the maximum distance from any given line segment to the
// circle is less than `max_error`.
inline void AppendCircle(R2VertexSink* absl_nonnull out,  //
  const R2Point& center, double radius, double max_error) {

  // The sagitta of a chord subtending an angle θ is
  //
  //   h = radius*(1-cos(θ/2))
  //
  // Setting h to be our max_error and solving for θ:
  //
  //   θ = 2*std::acos(1-max_error/radius)
  //
  // Which is our step size in radians around the circle.
  const double step = 2*std::acos(1 - max_error/radius);
  const double cstep = std::cos(step);
  const double sstep = std::sin(step);

  // Multiplying repeatedly to rotate the point will accrue error, but not
  // enough to matter during a single rotation around the circle.
  const int steps = std::floor(2*M_PI/step);

  out->Break();
  R2Point pnt = {radius, 0};
  for (int i=0; i < steps; ++i) {
    out->Append(center + pnt);
    pnt = {
      pnt.x()*cstep - pnt.y()*sstep,
      pnt.x()*sstep + pnt.y()*cstep
    };
  }
  out->Close();
}


}  // namespace w
