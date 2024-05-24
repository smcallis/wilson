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

#include <cstdint>
#include <optional>

#include "s2/r2.h"
#include "s2/s2point.h"

namespace w {

// A pair of boundary identifiers.
struct BoundaryPair {
  constexpr BoundaryPair()
    : b0(0), b1(0) {}

  constexpr BoundaryPair(uint8_t b0, uint8_t b1)
    : b0(b0), b1(b1) {}

  // Returns a BoundaryPair with only b1 set.
  static constexpr BoundaryPair Snap1(uint8_t b1) {
    return BoundaryPair(0, b1);
  }

  // Returns a BoundaryPair with only b0 set.
  static constexpr BoundaryPair Snap0(uint8_t b0) {
    return BoundaryPair(b0, 0);
  }

  uint8_t b0;
  uint8_t b1;
};


// There's four possible processing paths for an edge:
//
//   DROP - The edge isn't visible and can be dropped.
//
//   KEEP - Keep the edge as-is, it can be subdivided normally.
//
//   SNAP - One vertex or the other needs to be snapped to a boundary of the
//     projection.  One field of boundary will be set with the boundary to
//     snap the respective vertex to.
//
//   CHOP - The edge has to be broken due to wrapping across the anti-meridian
//    in this case, isect is set to the intersection point where the edge
//    crossed the anti-meridian.  The edge should be broken into two segments:
//
//      {edge.v0, isect} - with isect snapped to boundary.b0
//      {isect, edge.v1} - with isect snapped to boundary.b1
//
struct ClipResult {
  enum Status {
    kKeep, kDrop, kSnap, kChop
  };

  // Returns a ClipResult indicating that the edge should be kept as-is.
  static constexpr ClipResult Keep() {
    return {};
  }

  // Returns a ClipResult indicating we can drop the edge altogether.
  static constexpr ClipResult Drop() {
    return Snap({1, 1});
  }

  // Returns a ClipResult indicating v0 and v1 should be snapped.
  static constexpr ClipResult Snap(BoundaryPair b) {
    ClipResult result;
    result.boundary = b;
    return result;
  }

  // Returns a ClipResult indicating v0 should be snapped to the boundary.
  static constexpr ClipResult Snap0(uint8_t b0 = 1) {
    return ClipResult::Snap(BoundaryPair::Snap0(b0));
  }

  // Returns a ClipResult indicating v1 should be snapped to the boundary.
  static constexpr ClipResult Snap1(uint8_t b1 = 1) {
    return ClipResult::Snap(BoundaryPair::Snap1(b1));
  }

  // Returns a ClipResult indicating the edge must be split in two.  A direction
  // flag is provided opaquely to help interpret the result.
  static constexpr ClipResult Chop(const S2Point& isect, int direction) {
    ClipResult result;
    result.isect = isect;
    result.direction = direction;
    return result;
  }

  // Returns a status code indicating how to process the edge.
  Status status() const {
    // Chop the edge in half.
    if (isect.has_value()) {
      return kChop;
    }

    bool snap = (boundary.b0 != 0 || boundary.b1 != 0);
    if (snap) {
      // If we have two boundaries but they're the same, drop the edge.
      if (boundary.b0 == boundary.b1) {
        return kDrop;
      }

      // They're not the same, snap the vertices.
      return kSnap;
    }

    // Keep the edge as-is.
    return kKeep;
  }

  std::optional<S2Point> isect;
  BoundaryPair boundary;
  int direction = 0;
};

}  // namespace w
