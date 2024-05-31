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

#include <absl/types/span.h>
#include <blend2d.h>

#include <memory>
#include <mutex>
#include <vector>

#include <wilson/geometry/region.h>
#include <wilson/graphics/pixel.h>

namespace w {

// A pixbuffer is a wrapper around a BLImage that also contains a dirty list and
// may have one or more child pixbuffers that are positioned relative to it.
//
// When rendering regions to the pixbuffer, we can mark them dirty and composite
// them to the screen as needed to allow us to redraw just the portions of the
// screen that need updating.
//
// We always use 32-bit pre-multiplied ARGB for maximum compatibility.
//
// Pixbuffers implement aliasing semantics by default, which means that copying
// a pixbuffer through a normal copy:
//
//   Pixbuffer a;
//   Pixbuffer b = a;
//
// Will create a new reference to the same pixbuffer.  If an actual copy
// (copying the underlying image data and all) is required, then use the static
// Copy() constructor which will return a new instance.
//
struct Pixbuffer {
  // A default constructed instance must be resize()-ed before use.
  Pixbuffer()
    : pi(std::make_shared<Impl>()) {}

  Pixbuffer(int width, int height)
    : Pixbuffer() {
    resize(width, height);
  }

  // Two pixbuffers are equal if they're the same pixbuffer.
  bool operator==(const Pixbuffer& b) const {
    return b.pi == pi;
  }

  // A deep copy can be made explicitly from anything though.
  static Pixbuffer Copy(const Pixbuffer& pb) {
    Pixbuffer ans;
    ans.image().assignDeep(pb.image());
    return ans;
  }

  // Accessors for the enabled flag on the pixbuffer.
  bool enabled() const { return pi->enabled; }
  void enabled(bool flag) { pi->enabled = flag; }

  // Accessors for the pixbuffer relative offset.
  pnt2i offset() const { return pi->offset; }
  void offset(pnt2i off) { pi->offset = off; }
  pnt2i shift(pnt2i off) { return pi->offset += off; }

  // Returns the dimensions of the image.
  int width()  const { return pi->image.width(); }
  int height() const { return pi->image.height(); }

  // Returns the row stride of the image in pixels.
  int stride() const { return stride_bytes()/sizeof(pixel); }

  // Returns the row stride of the image in bytes.
  int stride_bytes() const { return image_data().stride;  }

  // Returns the bounds of the pixbuffer as a region.
  region2i bounds() const {
    return {0, 0, width(), height()};
  }

  // Resizes the pixbuffer.  Image contents after resizing are undefined.
  void resize(int width, int height) {
    pi->image.create(width, height, BL_FORMAT_PRGB32);
    pi->dirty.clear();
  }

  // Access the raw pixel data, starting at the given pixel coordinates.  If the
  // coordinates are out of bounds of the pixbuffer, the result is undefined.
  const pixel* data(int x=0, int y=0) const {
    auto data = reinterpret_cast<const pixel*>(image_data().pixelData);
    return data + y*stride() + x;
  }

  pixel* data(int x=0, int y=0) {
    auto data = reinterpret_cast<pixel*>(image_data().pixelData);
    return data + y*stride() + x;
  }

  // Returns the BLImageData from the build in BLImage.
  BLImageData image_data() const {
    BLImageData data;
    image().getData(&data);
    return data;
  }

  // Get a reference to the Blend2D image we can draw on.
        BLImage& image()       { return pi->image; }
  const BLImage& image() const { return pi->image; }

  // Clears a region in the pixbuffer to a given color without blending.
  void clear(region2i region, pixel color=pixel{}) {
    // Clip the region to the current pixbuffer bounds.
    region = intersect(region, bounds());

    const int inc = stride();

    pixel* ptr = data(region.lo().x(), region.lo().y());
    for (int y=0; y < region.height(); ++y) {
      for (int x=0; x < region.width(); ++x) {
        ptr[x] = color;
      }
      ptr += inc;
    }
  }

  // Clears the entire pixbuffer to a given color.
  void clear(pixel color=pixel{}) {
    clear(bounds(), color);
  }

  // Returns a span over the list of dirty regions.
  absl::Span<const region2i> dirty_list() const {
    return pi->dirty;
  }

  // Clear the dirty list.
  void clear_dirty_list() const {
    for (const Pixbuffer& pb : children()) {
      pb.clear_dirty_list();
    }
    pi->dirty.clear();
  }

  // Marks a region in the pixbuffer as dirty (and in need of re-blitting).
  void sully(region2i region) {
    pi->dirty.emplace_back(region);
  }

  // Marks the entire pixbuffer as dirty.
  void sully() {
    sully(bounds());
  }

  // Returns a span over the pixbuffers children.
  absl::Span<Pixbuffer> children() {
    return absl::MakeSpan(pi->children);
  }

  absl::Span<const Pixbuffer> children() const {
    return pi->children;
  }

  // Adds a new child pixbuffer and returns it.
  Pixbuffer add_child(int ww, int hh) {
    pi->children.emplace_back(ww, hh);
    return pi->children.back();
  }

  // Removes a child matching the given pixbuffer.  Returns true if a child was
  // found and removed, false otherwise.
  bool remove_child(const Pixbuffer& pb) const {
    using std::swap;

    for (int i=0; i < pi->children.size(); ++i) {
      if (pi->children[i] == pb) {
        swap(pi->children[i], pi->children.back());
        pi->children.pop_back();
        return true;
      }
    }
    return false;
  }

  // Removes all children from the pixbuffer.
  void clear_children() {
    pi->children.clear();
  }

private:
  struct Impl {
    std::vector<Pixbuffer> children;
    std::vector<region2i> dirty;
    BLImage image;
    bool enabled = true;
    pnt2i offset = {};
  };

  Pixbuffer(std::shared_ptr<Impl> impl)
    : pi(impl) {}

  std::shared_ptr<Impl> pi;
};

// Blends pixels from a region in a source pixbuffer to a destination pixbuffer.
//
// Offset specifies an offset in the destination at which to blend the pixels.
static inline region2i blend(
  Pixbuffer& dst, const Pixbuffer& src, region2i region, pnt2i offset={}) {

  // Clip regions to the two pixbuffers.
  region2i src_region = intersect(region,              src.bounds());
  region2i dst_region = intersect(src_region + offset, dst.bounds());

  BLContext ctx(dst.image());

  // Build a pattern from the source and translate it to where we want to blit.
  BLPattern pattern(src.image());
  pattern.translate(offset.x(), offset.y());
  ctx.setFillStyle(pattern);

  // And fill a rectangle of the right size.
  ctx.fillRect(
    dst_region.lo().x(), dst_region.lo().y(),
    dst_region.width(),  dst_region.height());
  ctx.end();

  return src_region;
}

} // namespace w
