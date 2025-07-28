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
#include "wilson/util/fixed.h"

#include <cmath>
#include <type_traits>

#include "fuzztest/fuzztest.h"
#include "gtest/gtest.h"

namespace w {
namespace {

using ::fuzztest::InRange;
using ::fuzztest::OneOf;
using fix32p8 = Fixed32<8>;
using fix32p16 = Fixed32<16>;

constexpr fix32p8 Fixed(double d) { return fix32p8::FromDouble(d); }

TEST(FixedTest, WholeFloorsValue) {
  // In two's complement, masking off the fractional part of a value is the same
  // as Floor()-ing it, rather than truncating.
  EXPECT_EQ(Fixed(+1.5).Whole(), Fixed(+1.0));
  EXPECT_EQ(Fixed(-1.5).Whole(), Fixed(-2.0));
  EXPECT_EQ(Whole(Fixed(+1.5)), Fixed(+1.0));
  EXPECT_EQ(Whole(Fixed(-1.5)), Fixed(-2.0));
}

TEST(FixedTest, FractIsRemainder) {
  // Fract should be the remainder of Whole().
  EXPECT_EQ(Fixed(+1.5).Fract(), Fixed(+0.5));
  EXPECT_EQ(Fixed(-1.5).Fract(), Fixed(+0.5));
  EXPECT_EQ(Fract(Fixed(+1.5)), Fixed(+0.5));
  EXPECT_EQ(Fract(Fixed(-1.5)), Fixed(+0.5));
}

TEST(FixedTest, AbsIsCorrect) {
  EXPECT_EQ(Fixed(+1.5).Abs(), Fixed(+1.5));
  EXPECT_EQ(Fixed(-1.5).Abs(), Fixed(+1.5));
  EXPECT_EQ(Abs(Fixed(+1.5)), Fixed(+1.5));
  EXPECT_EQ(Abs(Fixed(-1.5)), Fixed(+1.5));
}

TEST(FixedTest, RoundRoundsTiesToNearestEven) {
  EXPECT_EQ(Fixed(+2.5).Round(), Fixed(+2.0));
  EXPECT_EQ(Fixed(-2.5).Round(), Fixed(-2.0));
  EXPECT_EQ(Fixed(+1.5).Round(), Fixed(+2.0));
  EXPECT_EQ(Fixed(-1.5).Round(), Fixed(-2.0));
  EXPECT_EQ(Round(Fixed(+1.5)), Fixed(+2.0));
  EXPECT_EQ(Round(Fixed(-1.5)), Fixed(-2.0));
  EXPECT_EQ(Round(Fixed(+2.5)), Fixed(+2.0));
  EXPECT_EQ(Round(Fixed(-2.5)), Fixed(-2.0));
}

TEST(FixedTest, BelowRoundsDownIgnoresEqual) {
  // Below() should return the next lower integer if the value is integral.
  EXPECT_EQ(Fixed(+2.0).Below(), Fixed(+1.0));
  EXPECT_EQ(Fixed(+1.5).Below(), Fixed(+1.0));
  EXPECT_EQ(Fixed(-1.5).Below(), Fixed(-2.0));
  EXPECT_EQ(Fixed(-2.0).Below(), Fixed(-3.0));
  EXPECT_EQ(Below(Fixed(+2.0)), Fixed(+1.0));
  EXPECT_EQ(Below(Fixed(+1.5)), Fixed(+1.0));
  EXPECT_EQ(Below(Fixed(-1.5)), Fixed(-2.0));
  EXPECT_EQ(Below(Fixed(-2.0)), Fixed(-3.0));
}

TEST(FixedTest, FloorRoundsDownKeepsEqual) {
  // Floor() should round down or return the value if it's already integral.
  EXPECT_EQ(Fixed(+2.0).Floor(), Fixed(+2.0));
  EXPECT_EQ(Fixed(+1.5).Floor(), Fixed(+1.0));
  EXPECT_EQ(Fixed(-1.5).Floor(), Fixed(-2.0));
  EXPECT_EQ(Fixed(-2.0).Floor(), Fixed(-2.0));
  EXPECT_EQ(Floor(Fixed(+2.0)), Fixed(+2.0));
  EXPECT_EQ(Floor(Fixed(+1.5)), Fixed(+1.0));
  EXPECT_EQ(Floor(Fixed(-1.5)), Fixed(-2.0));
  EXPECT_EQ(Floor(Fixed(-2.0)), Fixed(-2.0));
}

TEST(FixedTest, CeilRoundsUpKeepsEqual) {
  // Ceil() should round up or return the value if it's already integral.
  EXPECT_EQ(Fixed(+2.0).Ceil(), Fixed(+2.0));
  EXPECT_EQ(Fixed(+1.5).Ceil(), Fixed(+2.0));
  EXPECT_EQ(Fixed(-1.5).Ceil(), Fixed(-1.0));
  EXPECT_EQ(Fixed(-2.0).Ceil(), Fixed(-2.0));
  EXPECT_EQ(Ceil(Fixed(+2.0)), Fixed(+2.0));
  EXPECT_EQ(Ceil(Fixed(+1.5)), Fixed(+2.0));
  EXPECT_EQ(Ceil(Fixed(-1.5)), Fixed(-1.0));
  EXPECT_EQ(Ceil(Fixed(-2.0)), Fixed(-2.0));
}

TEST(FixedTest, AboveRoundsUpIgnoresEqual) {
  // Above() should return the next higher integer if the value is integral.
  EXPECT_EQ(Fixed(+2.0).Above(), Fixed(+3.0));
  EXPECT_EQ(Fixed(+1.5).Above(), Fixed(+2.0));
  EXPECT_EQ(Fixed(-1.5).Above(), Fixed(-1.0));
  EXPECT_EQ(Fixed(-2.0).Above(), Fixed(-1.0));
  EXPECT_EQ(Above(Fixed(+2.0)), Fixed(+3.0));
  EXPECT_EQ(Above(Fixed(+1.5)), Fixed(+2.0));
  EXPECT_EQ(Above(Fixed(-1.5)), Fixed(-1.0));
  EXPECT_EQ(Above(Fixed(-2.0)), Fixed(-1.0));
}

TEST(FixedTest, AbsBelowRoundsTowardsZero) {
  EXPECT_EQ(Fixed(+2.0).AbsBelow(), Fixed(+1.0));
  EXPECT_EQ(Fixed(+1.5).AbsBelow(), Fixed(+1.0));
  EXPECT_EQ(Fixed(-1.5).AbsBelow(), Fixed(-1.0));
  EXPECT_EQ(Fixed(-2.0).AbsBelow(), Fixed(-1.0));
  EXPECT_EQ(AbsBelow(Fixed(+2.0)), Fixed(+1.0));
  EXPECT_EQ(AbsBelow(Fixed(+1.5)), Fixed(+1.0));
  EXPECT_EQ(AbsBelow(Fixed(-1.5)), Fixed(-1.0));
  EXPECT_EQ(AbsBelow(Fixed(-2.0)), Fixed(-1.0));
}

TEST(FixedTest, AbsFloorRoundsTowardsZero) {
  EXPECT_EQ(Fixed(+2.0).AbsFloor(), Fixed(+2.0));
  EXPECT_EQ(Fixed(+1.5).AbsFloor(), Fixed(+1.0));
  EXPECT_EQ(Fixed(-1.5).AbsFloor(), Fixed(-1.0));
  EXPECT_EQ(Fixed(-2.0).AbsFloor(), Fixed(-2.0));
  EXPECT_EQ(AbsFloor(Fixed(+2.0)), Fixed(+2.0));
  EXPECT_EQ(AbsFloor(Fixed(+1.5)), Fixed(+1.0));
  EXPECT_EQ(AbsFloor(Fixed(-1.5)), Fixed(-1.0));
  EXPECT_EQ(AbsFloor(Fixed(-2.0)), Fixed(-2.0));
}

TEST(FixedTest, AbsCeilRoundsAwayFromZero) {
  EXPECT_EQ(Fixed(+2.0).AbsCeil(), Fixed(+2.0));
  EXPECT_EQ(Fixed(+1.5).AbsCeil(), Fixed(+2.0));
  EXPECT_EQ(Fixed(-1.5).AbsCeil(), Fixed(-2.0));
  EXPECT_EQ(Fixed(-2.0).AbsCeil(), Fixed(-2.0));
  EXPECT_EQ(AbsCeil(Fixed(+2.0)), Fixed(+2.0));
  EXPECT_EQ(AbsCeil(Fixed(+1.5)), Fixed(+2.0));
  EXPECT_EQ(AbsCeil(Fixed(-1.5)), Fixed(-2.0));
  EXPECT_EQ(AbsCeil(Fixed(-2.0)), Fixed(-2.0));
}

TEST(FixedTest, AbsAboveRoundsAwayFromZero) {
  EXPECT_EQ(Fixed(+2.0).AbsAbove(), Fixed(+3.0));
  EXPECT_EQ(Fixed(+1.5).AbsAbove(), Fixed(+2.0));
  EXPECT_EQ(Fixed(-1.5).AbsAbove(), Fixed(-2.0));
  EXPECT_EQ(Fixed(-2.0).AbsAbove(), Fixed(-3.0));
  EXPECT_EQ(AbsAbove(Fixed(+2.0)), Fixed(+3.0));
  EXPECT_EQ(AbsAbove(Fixed(+1.5)), Fixed(+2.0));
  EXPECT_EQ(AbsAbove(Fixed(-1.5)), Fixed(-2.0));
  EXPECT_EQ(AbsAbove(Fixed(-2.0)), Fixed(-3.0));
}

TEST(FixedTest, MulReturnsExtraPrecision) {
  const fix32p8 epsilon = fix32p8::Epsilon();

  // Mul() should promote the type to double precision keep the precision.
  EXPECT_TRUE((std::is_same_v<decltype(epsilon.Mul(epsilon)), fix32p16>));
  EXPECT_EQ(epsilon.Mul(epsilon), fix32p16::Epsilon());
}

// Returns a domain of doubles that can be converted to Fixed without overflow
// or underflow. If margin values are given, the max and min values returned
// are adjusted accordingly to avoid coming within the margin of either extreme.
// This can be used to accommodate operations without overflowing.
auto DoubleDomainForFixed(double min_margin = 0.0, double max_margin = 0.0) {
  return InRange(  //
      fix32p8::MinDouble() + min_margin, fix32p8::MaxDouble() - max_margin);
}

// Returns a domain of doubles that avoids the region where we can't represent
// numbers in fixed point (because we lack the precision). If margin values are
// given, the max and min values returned are adjusted accordingly to avoid
// coming within the margin of either extreme.
auto DoubleDomainForFixedAvoidZero(double min_margin = 0.0,
                                   double max_margin = 0.0) {
  // Avoid the region near zero where we can't represent numbers in Fixed.
  return OneOf(InRange(1.0 / fix32p8::kScale, fix32p8::MaxDouble()),
               InRange(fix32p8::MinDouble(), -1.0 / fix32p8::kScale));
}

// Converting a double to Fixed and back should have a max error of 1 ulp.
void DoubleConvertsOneUlp(double d) {
  EXPECT_LE(  //
      std::fabs(ToDouble(Fixed(d)) - d), 1.0 / fix32p8::kScale);
}
FUZZ_TEST(FixedTests, DoubleConvertsOneUlp).WithDomains(DoubleDomainForFixed());

void RoundingModesAreCorrect(double d) {
  const fix32p8 value = Fixed(d);
  const fix32p8 whole = Fixed(d).Whole();
  const fix32p8 midway = whole + Fixed(0.5);

  // Fract() should be non-negative and Whole() + Fract() == value always.
  EXPECT_GE(value.Fract(), 0);
  EXPECT_EQ(value, value.Whole() + value.Fract());

  // Check that RoundNe rounds ties to nearest even value.
  {
    EXPECT_EQ(Abs(midway.Round()).ToInteger() % 2, 0);

    if (Abs(value) < Abs(midway)) {
      if (value.Fract() == 0) {
        EXPECT_EQ(Abs(value.Round()), Abs(value));
      } else {
        EXPECT_LT(Abs(value.Round()), Abs(value));
      }
    }

    if (Abs(value) > Abs(midway)) {
      if (value.Fract() == 0) {
        EXPECT_EQ(Abs(value.Round()), Abs(value));
      } else {
        EXPECT_GT(Abs(value.Round()), Abs(value));
      }
    }
  }

  // Check that ToInteger() floors the value.
  if (value.Fract() == 0) {
    EXPECT_EQ(ToInteger(value), value);
  } else {
    EXPECT_LT(ToInteger(value), value);
  }

  // Check that Below() returns a value < value.
  EXPECT_LT(Below(value), value);

  // Check that Floor() returns a value <= value.
  if (value.Fract() == 0) {
    EXPECT_EQ(Floor(value), value);
  } else {
    EXPECT_LT(Floor(value), value);
  }

  // Check that Ceil() returns a value >= value.
  if (value.Fract() == 0) {
    EXPECT_EQ(Floor(value), value);
  } else {
    EXPECT_LT(Floor(value), value);
  }

  // Check that Above() returns a value > value.
  EXPECT_GT(Above(value), value);

  // Check that AbsBelow rounds s.t. Abs(result) < Abs(value).
  EXPECT_LT(Abs(value.AbsBelow()), Abs(value));

  // Check that AbsFloor rounds s.t. Abs(result) <= Abs(value).
  if (value.Fract() == 0) {
    EXPECT_EQ(Abs(value.AbsFloor()), Abs(value));
  } else {
    EXPECT_LT(Abs(value.AbsFloor()), Abs(value));
  }

  // Check that AbsCeil rounds s.t. Abs(result) >= Abs(value).
  if (value.Fract() == 0) {
    EXPECT_EQ(Abs(value.AbsCeil()), Abs(value));
  } else {
    EXPECT_GT(Abs(value.AbsCeil()), Abs(value));
  }

  // Check that AbsAbove rounds s.t. Abs(result) > Abs(value).
  EXPECT_GT(Abs(value.AbsAbove()), Abs(value));
}
FUZZ_TEST(FixedTests, RoundingModesAreCorrect)  //
    .WithDomains(DoubleDomainForFixedAvoidZero(1, 1));

}  // namespace
}  // namespace w
