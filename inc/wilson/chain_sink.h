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

namespace w {

// An abstract interface for something that can receive vertices and group them
// into chains.

class ChainSink {
 public:
  // Clears the buffer to its initial state, as though nothing has been added.
  virtual void Clear() = 0;

  // Starts a new chain.  If the current chain is empty, does nothing.
  virtual void Break() = 0;

  // Appends a single point to the current chain.
  virtual void Append(const R2Point&) = 0;

  // Appends a span of points to the current chain.
  virtual void Append(absl::Span<const R2Point>) = 0;

  // Returns the size of the current chain.
  virtual ssize_t ChainSize() const = 0;

  // Convenience method to test if the current chain is empty.
  bool ChainEmpty() const {
    return ChainSize() == 0;
  }
};

}