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

#include <cmath>
#include <cstdint>

#include <blend2d.h>

namespace w {

using u8 = uint8_t;

// Accurate 8-bit multiplication, from "Imaging Compositing Fundamentals" by
// Alvy Ray Smith.
//
// This uses two terms of the expansion a/(1-r) = a + ar + ar^2 + ar^3 + ... to
// get a more accurate multiplication.  Without this, 255*255 = 254 which can
// cause slight transparency in cases where we didn't want it.
static inline constexpr uint8_t mul8x8(uint8_t a, uint8_t b) {
  return (((uint16_t)a*b + 0x80) + (((uint16_t)a*b + 0x80) >> 8)) >> 8;
}

// We only work with 32-bit ARGB pre-multiplied in a linear colorspace.  This
// is implicitly convertible to a BLRgba32 value for use with Blend2d.
struct pixel {
  pixel() = default;

  pixel(uint32_t c)
    : b(c), g(c >> 8), r(c >> 16), a(c >> 24) {
    multiply();
  }

  pixel(u8 r, u8 g, u8 b, u8 a=0xFF)
    : b(b), g(g), r(r), a(a) {
    multiply();
  }

  // Creates a pixel but skips pre-multiplication step.
  // be pre-multiplied.
  static pixel FromPremultiplied(u8 r, u8 g, u8 b, u8 a=0xFF) {
    pixel ans;
    ans.r = r;
    ans.g = g;
    ans.b = g;
    ans.a = a;
    return ans;
  }

  // Creates a pixel from an integer but ignores the alpha field, yielding an
  // opaque pixel.
  static pixel Opaque(uint32_t c) {
    pixel p(c);
    p.a = 0xFF;
    return p;
  }

  // Takes a color in sRGB and returns a pixel for it in linear space.
  static pixel FromSrgb(u8 r, u8 g, u8 b, u8 a=0xFF) {
    return pixel(
      srgb_to_linear(r),
      srgb_to_linear(g),
      srgb_to_linear(b),
      a
    );
  }

  // Takes a color in sRGB and returns a pixel for it in linear space.
  static pixel FromSrgb(uint32_t rgba) {
    return pixel(
      srgb_to_linear(rgba >> 16),
      srgb_to_linear(rgba >> 8),
      srgb_to_linear(rgba >> 0),
      rgba >> 24
    );
  }

  // Unmultiplies and remultiplies the alpha.  This can accrue error so its best
  // to use it once on a base pixel instance when e.g. adjusting color
  // transparency.
  void set_alpha(u8 alpha) {
    r = ((uint16_t)r*255)/a;
    g = ((uint16_t)g*255)/a;
    b = ((uint16_t)b*255)/a;
    a = alpha;
    multiply();
  }

  // Multiplies each field by a new alpha value to get a new transparent color.
  // If you start with a half-transparent vaule, then alpha = 0xFF will give you
  // that same color back, and lower values will shade to zero still.
  void mul_alpha(u8 alpha) {
    r = mul8x8(r, alpha);
    g = mul8x8(g, alpha);
    b = mul8x8(b, alpha);
    a = mul8x8(a, alpha);
  }

  // Returns true if the pixel is opaque.
  bool opaque() const { return a == 0xFF; }

  operator BLRgba32() const {
    return BLRgba32(r, g, b, a);
  }

  // Format is 0xAARRGGBB in memory for maximum compatibility.
  u8 b,g,r,a;

  // Blend two pre-multiplied pixels together.
  friend pixel blend(pixel under, pixel over) {
    if (over.a == 0)   return under;
    if (over.a == 255) return over;
    under.b = over.b + (((uint16_t)under.b*(255-over.a)) >> 8);
    under.g = over.g + (((uint16_t)under.g*(255-over.a)) >> 8);
    under.r = over.r + (((uint16_t)under.r*(255-over.a)) >> 8);
    under.a = over.a + (((uint16_t)under.a*(255-over.a)) >> 8);
    return under;
  }

private:
  static inline constexpr u8 srgb_to_linear(u8 v) {
    const float vf = v/255.0f;
    if (vf < 0.04045) {
      return static_cast<u8>(255*vf/12.92);
    } else {
      return static_cast<u8>(255*std::pow((vf+0.055)/1.055, 2.4));
    }
  }

  // Multiply fields by alpha to get into pre-multiplied format.
  void multiply() {
    b = mul8x8(b,a);
    g = mul8x8(g,a);
    r = mul8x8(r,a);
  }
};

}
