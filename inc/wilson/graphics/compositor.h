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

#include "wilson/graphics/pixbuffer.h"

namespace w {

// A compositing engine to composite pixbuffers together.
struct Compositor {
  // Composites a source pixbuffer and all its children into destination pixbuf.
  absl::Span<const region2i> composite(Pixbuffer dst, const Pixbuffer& src) {
    dirty_.clear();
    merge_dirty(src);

    // Nothing to composite.
    composited_ = false;
    if (dirty_.empty()) {
      return src.dirty_list();
    }

    // Blend each dirty region recursively.
    for (region2i region : dirty_) {
      blend(dst, src, region);
    }

    composited_ = true;
    return dirty_;
  }

  bool composited() const { return composited_; }

private:
  bool composited_;

  // Recursively merge dirty buffers into a single list.
  void merge_dirty(const Pixbuffer& pb, pnt2i offset={}) {
    for (const region2i dirty : pb.dirty_list()) {
      dirty_.emplace_back(dirty + offset);
    }

    for (const Pixbuffer& child : pb.children()) {
      merge_dirty(child, offset + child.offset());
    }
  }

  // Recursively blends pixbuffer into destination.
  void blend(
    Pixbuffer& dst, const Pixbuffer& src, region2i region, pnt2i offset={}) {

    // Blend this pixbuffer.
    w::blend(dst, src, region, offset);

    // And blend child pixbuffers.
    for (const Pixbuffer& child : src.children()) {
      if (!child.enabled()) {
        continue;
      }
      blend(dst, child, region, offset + child.offset());
    }
  }

  std::vector<region2i> dirty_;
};

} // namespace w
