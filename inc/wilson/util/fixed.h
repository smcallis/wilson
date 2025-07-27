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
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

#include "absl/log/check.h"
#include "absl/numeric/int128.h"
#include "absl/strings/str_format.h"

namespace w {

// A General purpose fixed point type, parameterized on an integer type and
// fractional precision.

// There are several ways to define a fixed point number in terms of whole and
// fractional precision. We take the convention that Whole() is always the
// integer value <= the fixed point value, and thus the Fract() value is always
// non-negative.
//
// This means that negative values such as -1.25 will have a Whole() value of -2
// and Fract() value of 0.75, which may be surprising at first. The invariant we
// maintain is that Whole() + Fract() == value always. This has the benefit that
// Whole() and Fract() can be computed by simple masking and one needs never
// worry about negative fractional values when writing algorithms, simplifying
// the mental model.
//
template <typename TInteger, int kFractional>
class Fixed {
 public:
  static constexpr int kWidth = sizeof(TInteger) * 8;

  // Total width of the fixed point type, including the sign bit.
  static_assert(std::is_signed_v<TInteger>, "Integer type must be signed.");
  static_assert(kFractional <= kWidth, "Too much precision for integer type.");

  // Expose precision and number of whole integer bits available.
  static constexpr int kPrecision = kFractional;
  static constexpr int kWholeBits = kWidth - kPrecision;

  // Constants for manipulating the fraction via shifting, scaling, and masking.
  static constexpr int kShift = kPrecision;
  static constexpr TInteger kScale = TInteger{1} << kShift;
  static constexpr TInteger kMask = kScale - 1;

  // An integer type large enough to hold the product of two fixed point values.
  using TProduct = decltype([]() {
    if constexpr (kPrecision <= 8) {
      return int16_t{};
    } else if constexpr (kPrecision <= 16) {
      return int32_t{};
    } else if constexpr (kPrecision <= 32) {
      return int64_t{};
    } else {
      return absl::int128{};
    }
  }());

  // The maximum representable integer value.
  static consteval TInteger MaxInteger() {
    return std::numeric_limits<TInteger>::max() >> kShift;
  }

  // The minimum representable integer value.
  static consteval int64_t MinInteger() {
    return std::numeric_limits<TInteger>::min() >> kShift;
  }

  // The maximum double that won't overflow converting to fixed point.
  static constexpr double MaxDouble() {
    return std::nextafter(static_cast<double>(MaxInteger()), -INFINITY);
  }

  // The minimum double that won't overflow converting to fixed point.
  static constexpr double MinDouble() {
    return std::nextafter(static_cast<double>(MinInteger()), +INFINITY);
  }

  // Decimal precision to use when printing values.
  static constexpr int kDecimalPrecision = kPrecision * (M_LN2 / M_LN10) + 1;

  constexpr Fixed() = default;

  // Allow implicit conversion from integer values.

  // Equivalent to calling Fixed::FromInt(value), DCHECKs for overflow.
  constexpr Fixed(TInteger value) : v_(value << kShift) {  // NOLINT
    DCHECK(MinInteger() <= value && value <= MaxInteger());
  }

  // Allow construction from a lower-precision Fixed value since the conversion
  // will never risk loss of information.
  template <typename T, int F>
    requires(F <= kPrecision)
  explicit constexpr Fixed(Fixed<T, F> f)
      : v_(static_cast<TInteger>(f.v_) << (kPrecision - F)) {
    DCHECK(MinInteger() <= f.ToInteger() && f.ToInteger() <= MaxInteger());
  }

  template <typename Sink>
  friend void AbslStringify(Sink& sink, const Fixed& f) {
    absl::Format(&sink, "%.*f", kDecimalPrecision,
                 static_cast<double>(f.Raw()) / kScale);
  }

  // Returns the minimum representable increment for the fixed point value.
  static constexpr Fixed Epsilon() { return FromRaw(1); }

  // Creates a Fixed from an integer without any scaling.
  static constexpr Fixed FromRaw(TInteger x) {
    DCHECK(MinInteger() <= (x >> kShift) && (x >> kShift) <= MaxInteger());

    Fixed f;
    f.v_ = x;
    return f;
  }

  // Creates a Fixed from an integer.
  static constexpr Fixed FromInt(TInteger x) { return {x}; }

  // Creates a Fixed from a float.
  static constexpr Fixed FromFloat(float x) {
    DCHECK(MinDouble() <= x && x <= MaxDouble());
    return FromRaw(static_cast<TInteger>(x * kScale));
  }

  // Creates a Fixed from a double.
  static constexpr Fixed FromDouble(double x) {
    DCHECK(MinDouble() <= x && x <= MaxDouble());
    return FromRaw(static_cast<TInteger>(x * kScale));
  }

  // Returns this value truncated back to an integer.
  constexpr TInteger ToInteger() const { return v_ >> kShift; }

  // Returns this value converted to a float.
  constexpr float ToFloat() const { return static_cast<float>(v_) / kScale; }

  // Returns this value converted to a double.
  constexpr double ToDouble() const { return static_cast<double>(v_) / kScale; }

  // Rounds the value to the given bits of precision. Does nothing if precision
  // is greater than or equal to the current precision.
  template <int kBits>
  constexpr auto RoundTo() const {
    using Return = Fixed<TInteger, kBits>;

    // If the requested precision is higher, then we can just shift into place.
    if constexpr (kBits >= kPrecision) {
      return Return::FromRaw(Raw() << (kBits - kPrecision));
    }

    // Otherwise round and shift to the desired precision.
    constexpr int shift = kPrecision - kBits;
    constexpr TInteger scale = TInteger{1} << shift;
    constexpr TInteger mask = scale - 1;

    return Return::FromRaw(((v_ + scale / 2) & ~mask) >> shift);
  }

  // Returns the raw underlying integer.
  constexpr TInteger Raw() const { return v_; }

  // Returns a Fixed with only the integral part of this value.
  //
  // Note: In 2's complement, masking the fractional part away is equivalent to
  // Floor()-ing the value, so the Whole() part is always rounded to -infinity,
  // and the fractional part is always non-negative.
  //
  // The core invariant here is *this == Whole() + Fract() always.
  constexpr Fixed Whole() const { return Floor(); }

  // Returns a Fixed with only the fractional part of this value.
  //
  // Note: Always positive, see Whole().
  constexpr Fixed Fract() const { return *this - Whole(); }

  // Returns the absolute value of this value.
  constexpr Fixed Abs() const { return FromRaw(std::abs(v_)); }

  // Returns the nearest integral value by rounding (ties to nearest even).
  constexpr Fixed Round() const {
    constexpr TInteger half = kScale >> 1;
    const TInteger biased = v_ + (half - 1);
    const TInteger oddmask = (biased & kScale) != 0;
    return FromRaw(biased + oddmask).Floor();
  }

  // Returns the nearest integral value < this value.
  constexpr Fixed Below() const { return FromRaw(v_ - 1).Floor(); }

  // Returns the nearest integral value <= this value.
  constexpr Fixed Floor() const { return FromRaw(v_ & ~kMask); }

  // Returns the nearest integral value >= this value.
  constexpr Fixed Ceil() const { return FromRaw((v_ + kMask) & ~kMask); }

  // Returns the nearest integral value > this value.
  constexpr Fixed Above() const { return FromRaw(v_ + 1).Ceil(); }

  // Returns the nearest integral value I s.t. Abs(I) < Abs(this).
  constexpr Fixed AbsBelow() const {
    return (*this + ((v_ < 0) ? 1 + Epsilon() : 0)).Below();
  }

  // Returns the nearest integral value I s.t. Abs(I) <= Abs(this).
  constexpr Fixed AbsFloor() const {
    return (*this + ((v_ < 0) ? 1 - Epsilon() : 0)).Floor();
  }

  // Returns the nearest integral value I s.t. Abs(I) >= Abs(this).
  constexpr Fixed AbsCeil() const {
    return (*this - ((v_ < 0) ? 1 - Epsilon() : 0)).Ceil();
  }

  // Returns the nearest integral value I s.t. Abs(I) > Abs(this).
  constexpr Fixed AbsAbove() const {
    return (*this - ((v_ < 0) ? 1 + Epsilon() : 0)).Above();
  }

  // Multiplies two fixed values without rounding. Result has double precision.
  template <typename T = TInteger>
  constexpr auto Mul(Fixed b) const {
    DCHECK((Fixed::kPrecision <= Fixed<T, kPrecision>::kWidth / 2));
    using Result = Fixed<T, 2 * kPrecision>;
    return Result::FromRaw(Raw() * b.Raw());
  }

  // ---------------------------------------------------------------------------
  // Friend versions to give the option of free-function syntax, e.g. Abs(x);
  // ---------------------------------------------------------------------------
  constexpr friend Fixed RoundTo(int precision, Fixed a) {
    return a.RoundTo(precision);
  }

  constexpr friend TInteger ToInteger(Fixed a) { return a.ToInteger(); }
  constexpr friend float ToFloat(Fixed a) { return a.ToFloat(); }
  constexpr friend double ToDouble(Fixed a) { return a.ToDouble(); }
  constexpr friend Fixed Raw(Fixed a) { return a.Raw(); }
  constexpr friend Fixed Whole(Fixed a) { return a.Whole(); }
  constexpr friend Fixed Fract(Fixed a) { return a.Fract(); }
  constexpr friend Fixed Abs(Fixed a) { return a.Abs(); }

  // Rounding
  constexpr friend Fixed Round(Fixed a) { return a.Round(); }
  constexpr friend Fixed Below(Fixed a) { return a.Below(); }
  constexpr friend Fixed Floor(Fixed a) { return a.Floor(); }
  constexpr friend Fixed Ceil(Fixed a) { return a.Ceil(); }
  constexpr friend Fixed Above(Fixed a) { return a.Above(); }
  constexpr friend Fixed AbsBelow(Fixed a) { return a.AbsBelow(); }
  constexpr friend Fixed AbsFloor(Fixed a) { return a.AbsFloor(); }
  constexpr friend Fixed AbsCeil(Fixed a) { return a.AbsCeil(); }
  constexpr friend Fixed AbsAbove(Fixed a) { return a.AbsAbove(); }

  // Unary operators.
  constexpr Fixed operator+() const { return FromRaw(+v_); }
  constexpr Fixed operator-() const { return FromRaw(-v_); }

  // We define operators as inline friend methods for consistency. The general
  // rule is that we allow implicit Integer promotion to a Fixed term as long as
  // it's part of an expression involving another Fixed.
  constexpr friend Fixed operator<<(Fixed a, int k) {
    return FromRaw(a.v_ << k);
  }

  constexpr friend Fixed operator>>(Fixed a, int k) {
    return FromRaw(a.v_ >> k);
  }

  constexpr friend Fixed operator+(Fixed a, Fixed b) {
    return FromRaw(a.v_ + b.v_);
  }

  constexpr friend Fixed operator+(TInteger a, Fixed b) {
    return FromInt(a) + b;
  }

  constexpr friend Fixed operator+(Fixed a, TInteger b) {
    return a + FromInt(b);
  }

  constexpr friend Fixed operator-(Fixed a, Fixed b) {
    return FromRaw(a.v_ - b.v_);
  }

  constexpr friend Fixed operator-(TInteger a, Fixed b) {
    return FromInt(a) - b;
  }

  constexpr friend Fixed operator-(Fixed a, TInteger b) {
    return a - FromInt(b);
  }

  constexpr friend Fixed operator*(Fixed a, Fixed b) {
    return FromRaw(
        static_cast<TInteger>((static_cast<TProduct>(a.v_) * b.v_) >> kShift));
  }

  constexpr friend Fixed operator*(TInteger a, Fixed b) {
    return FromRaw(a * b.v_);
  }

  constexpr friend Fixed operator*(Fixed a, TInteger b) {
    return FromRaw(a.v_ * b);
  }

  constexpr friend Fixed operator/(Fixed a, Fixed b) {
    return FromRaw(
        static_cast<TInteger>((static_cast<TProduct>(a.v_) << kShift) / b.v_));
  }

  constexpr friend Fixed operator/(TInteger a, Fixed b) {
    return FromInt(a) / b;
  }

  constexpr friend Fixed operator/(Fixed a, TInteger b) {
    return FromRaw(a.v_ / b);
  }
  constexpr bool friend operator<(Fixed a, Fixed b) {
    return a.v_ < b.v_;
  }

  constexpr bool friend operator<(TInteger a, Fixed b) {
    return FromInt(a) < b;
  }

  constexpr bool friend operator<(Fixed a, TInteger b) {
    return a < FromInt(b);
  }

  constexpr bool friend operator<=(Fixed a, Fixed b) {
    return a.v_ <= b.v_;
  }

  constexpr bool friend operator<=(TInteger a, Fixed b) {
    return FromInt(a) <= b;
  }

  constexpr bool friend operator<=(Fixed a, TInteger b) {
    return a <= FromInt(b);
  }

  constexpr bool friend operator==(Fixed a, Fixed b) {
    return a.v_ == b.v_;
  }

  constexpr bool friend operator==(TInteger a, Fixed b) {
    return FromInt(a) == b;
  }

  constexpr bool friend operator==(Fixed a, TInteger b) {
    return a == FromInt(b);
  }

  constexpr bool friend operator>=(Fixed a, Fixed b) {
    return a.v_ >= b.v_;
  }

  constexpr bool friend operator>=(Fixed a, TInteger b) {
    return a >= FromInt(b);
  }

  constexpr bool friend operator>=(TInteger a, Fixed b) {
    return FromInt(a) >= b;
  }

  constexpr bool friend operator>(Fixed a, Fixed b) {
    return a.v_ > b.v_;
  }

  constexpr bool friend operator>(TInteger a, Fixed b) {
    return FromInt(a) > b;
  }

  constexpr bool friend operator>(Fixed a, TInteger b) {
    return a > FromInt(b);
  }

  constexpr bool friend operator!=(Fixed a, Fixed b) {
    return a.v_ != b.v_;
  }

  constexpr bool friend operator!=(TInteger a, Fixed b) {
    return FromInt(a) != b;
  }

  constexpr bool friend operator!=(Fixed a, TInteger b) {
    return a != FromInt(b);
  }

  constexpr friend Fixed& operator+=(Fixed& a, Fixed b) { return a = a + b; }

  constexpr friend Fixed& operator+=(Fixed& a, TInteger b) { return a = a + b; }

  constexpr friend Fixed& operator-=(Fixed& a, Fixed b) { return a = a - b; }

  constexpr friend Fixed& operator-=(Fixed& a, TInteger b) { return a = a - b; }

  constexpr friend Fixed& operator*=(Fixed& a, Fixed b) { return a = a * b; }

  constexpr friend Fixed& operator*=(Fixed& a, TInteger b) { return a = a * b; }

  constexpr friend Fixed& operator/=(Fixed& a, Fixed b) { return a = a / b; }

  constexpr friend Fixed& operator/=(Fixed& a, TInteger b) { return a = a / b; }

 private:
  template <typename T, int F>
  friend class Fixed;

  // Intentionally uninitialized to mimic default-initialization of built-ins.
  TInteger v_;
};

template <int kPrecision>
using Fixed32 = Fixed<int32_t, kPrecision>;

template <int kPrecision>
using Fixed64 = Fixed<int64_t, kPrecision>;

}  // namespace w
