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

// A wrapper around BLPath that can be used to implement a 2D shape or store
// generic paths consistently.  The primary motivation behind this is to have a
// single abstraction to append path elements to.
//
// Rather than the moveTo(), lineTo() paradigm of other paths, we use a chain
// based approach where a new chain can be started by the calling code and then
// the first path element appended determines the starting point of that chain.
//
// This avoids the need for code that generates path elements to know when the
// cursor needs to be moved.
//
// This only supports linear path elements by Append()-ing points to the path.

#include <vector>

#include "blend2d.h"
#include "absl/types/span.h"
#include "absl/algorithm/container.h"

#include "s2/r2.h"

namespace w {

struct R2Shape {
  struct Edge {
    Edge() = default;
    Edge(const R2Point& v0, const R2Point& v1)
      : v0(v0), v1(v1) {}

    Edge Reversed() const {
      return {v1, v0};
    }

    bool operator==(const Edge& b) const {
      return v0 == b.v0 && v1 == b.v1;
    }

    bool operator!=(const Edge& b) const {
      return !(*this == b);
    }

    R2Point v0, v1;
  };

  struct Chain {
    int id;
    int offset;
    int length;
  };

  // Appends a point to the path.  If a new chain is created, results in a move
  // of the cursor, otherwise a line is drawn from the previous position to the
  // new position.
  void Append(R2Point pnt) {
    if (MaybeStartChain()) {
      path_.moveTo(BL(pnt));
    } else {
      path_.lineTo(BL(pnt));
    }
  }

  // Same as Append() above but takes a span of points.
  void Append(absl::Span<const R2Point> points) {
    for (const auto& pnt : points) {
      Append(pnt);
    }
  }

  void Append(const R2Shape& shape) {
    EndChain();
    for (int i=0; i < shape.nchains(); ++i) {
      Append(shape.chain_vertices(i));
      EndChain();
    }
  }

  // Similar to Append, but always adds the span of points as a separate chain.
  void AddChain(absl::Span<const R2Point> points) {
    EndChain();
    for (const auto& pnt : points) {
      Append(pnt);
    }
    EndChain();
  }

  // Adds a new chain that's a closed circle.  The circle is segmented into
  // lines such that the maximum distance from any given line segment to the
  // circle is at most tolerance.
  void AddCircle(R2Point center, double radius, double tolerance=0.25) {
    // The sagitta of a chord subtending an angle θ is
    //   h = radius*(1-cos(θ/2))
    //
    // Setting h to be our tolerance and solving for θ:
    //   θ = 2*std::acos(1-h/radius)
    //
    // Which is our step size in radians around the circle to achieve the
    // given tolerance.
    double theta = 2*std::acos(1-tolerance/radius);
    double c = std::cos(theta);
    double s = std::sin(theta);

    // Multiplying to rotate the point will accrue error as it goes, but not
    // enough to matter during a single rotation around the circle.
    int steps = std::ceil(2*M_PI/theta);

    EndChain();
    R2Point pnt = {radius,0};
    for (int i=0; i <= steps; ++i) {
      Append(pnt + center);
      pnt = {
        pnt.x()*c - pnt.y()*s,
        pnt.x()*s + pnt.y()*c
      };
    }
    EndChain();
  }

  // Return a reference to the underlying path object.
  const BLPath& path() const { return path_; }
  const R2Point* data() const { return
      // Both R2Point and BLPoint are a struct of two double fields.
      reinterpret_cast<const R2Point*>(path_.vertexData());
  }

  // Closes and ends the current chain.  The next command will start a new one.
  void CloseChain() {
    if (active_chain_ >= 0) {
      path_.close();
      active_chain_ = -1;
    }
  }

  // Ends the current chain.  The next command will start a new one.
  void EndChain() {
    if (active_chain_ >= 0) {
      active_chain_ = -1;
    }
  }

  // Returns true if there are no vertices.
  bool empty() const {
    return path_.empty();
  }

  // Returns the size of the path in vertices.
  ssize_t nvertices() const {
    return path_.size();
  }

  // Returns the number of chains in the path.
  ssize_t nchains() const {
    return chains_.size();
  }

  // Returns metadata about the given chain.
  Chain chain(int idx) const {
    Chain chain;
    chain.id = idx;
    chain.offset = chains_[idx];

    if (idx == chains_.size()-1) {
      chain.length = nvertices()-chain.offset;
    } else {
      chain.length = chains_[idx+1]-chain.offset;
    }
    return chain;
  }

  // Returns an edge from the given chain.
  Edge chain_edge(int chain_id, int offset) const {
    Chain c = chain(chain_id);

    const R2Point* pnts = data()+c.offset;
    if (offset == c.length-1) {
      return {pnts[offset], pnts[0]};
    }
    return {pnts[offset], pnts[offset+1]};
  }

  // Returns an iterable span over the vertices of the given chain.
  absl::Span<const R2Point> chain_vertices(int chain_id) const {
    Chain c = chain(chain_id);
    if (c.length == 0) {
      return {};
    }
    return {data()+c.offset, static_cast<size_t>(c.length)};
  }

  // Removes any segments from the path and resets it to empty.
  void clear() {
    path_.clear();
    chains_.clear();
    active_chain_ = -1;
  }

private:
  // Converts an R2Point into a BLPoint.
  BLPoint BL(const R2Point& pnt) {
    return {pnt.x(), pnt.y()};
  }

  // Starts a new chain if there isn't one.  If there's a current active chain
  // then degrades into a noop.
  bool MaybeStartChain() {
    if (active_chain_ < 0) {
      active_chain_ = chains_.size();
      chains_.emplace_back(path_.size());
      return true;
    }
    return false;
  }

  // Chain offsets and current active chain.
  int active_chain_ = -1;
  std::vector<int> chains_;

  // Underlying path storage.
  BLPath path_;
};

}
