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

#include <utility>

#include "absl/base/optimization.h"
#include "s2/util/math/vector.h"

namespace w {

using vec2f = Vector2<float>;
using vec2  = Vector2<double>;
using vec2i = Vector2<int>;

using vec3f = Vector3<float>;
using vec3  = Vector3<double>;
using vec3i = Vector3<int>;

// Points are really equivalent to vectors, but we one implies a position and
// one a displacement, so we provide both sets of aliases for readability.
using pnt2f = Vector2<float>;
using pnt2  = Vector2<double>;
using pnt2i = Vector2<int>;

using pnt3f = Vector3<float>;
using pnt3  = Vector3<double>;
using pnt3i = Vector3<int>;


template <typename T>
struct _Interval {
  using type = T;

  _Interval() = default;
  _Interval(T v0, T v1)
    : v0_(v0), v1_(v1) {}

  // Converting constructor
  template <typename U>
  explicit _Interval(const _Interval<U> &b)
    : v0_(static_cast<T>(b.v0_)), v1_(static_cast<T>(b.v1_)) {}

  T   length() const { return v1_-v0_; }
  bool empty() const { return length() == 0; }

  T v0() const { return v0_; }
  T v1() const { return v1_; }

  // Clamps a value to the interval.
  T clamp(T v) const {
    return std::max(v0_, std::min(v1_, v));
  }

  // Arithmetic operators are applied element-wise to each endpoint.
  _Interval& operator +=(T v) { v0_ += v; v1_ += v;            return *this; }
  _Interval& operator -=(T v) { v0_ -= v; v1_ -= v;            return *this; }
  _Interval& operator *=(T v) { v0_ *= v; v1_ *= v; rectify(); return *this; }
  _Interval& operator /=(T v) { v0_ /= v; v1_ /= v; rectify(); return *this; }

  // Out-of-place versions.
  friend _Interval operator+(_Interval i, T v) { return i += v; }
  friend _Interval operator+(T v, _Interval i) { return i += v; }
  friend _Interval operator-(_Interval i, T v) { return i -= v; }
  friend _Interval operator*(_Interval i, T v) { return i *= v; }
  friend _Interval operator*(T v, _Interval i) { return i *= v; }
  friend _Interval operator/(_Interval i, T v) { return i /= v; }

  // Returns true if an interval contains a value in a half-open sense.
  bool contains(T v) const {
    return v0_ <= v && v < v1_;
  }

  // Returns true if an interval is contained.
  bool contains(_Interval b) const {
    return contains(b.v0_) && contains(b.v1_);
  }

  // Returns true if another intervals intersects this one.
  bool intersects(_Interval b) const {
    return !intersect(*this, b).empty();
  }

  // Splits the interval into upper and lower halves at the given point.
  std::pair<_Interval,_Interval> split(T v) const {
    if (v < v0_) {
      return std::make_pair(_Interval(), *this);
    }

    if (v > v1_) {
      return std::make_pair(*this, _Interval());
    }

    return std::make_pair(
      _Interval(v0_, std::max(v0_, std::min(v1_, v))),
      _Interval(std::min(v1_, std::max(v0_, v)), v1_)
    );
  }

  // Linearly interpolate between two intervals.  This will map a.v0 to b.v0 and
  // a.v1_ to b.v1_ with a linear ramp between them.  This ramp extends outside
  // the interval so the point is not required to be in either interval to
  // interpolate.
  friend T lerp(_Interval a, _Interval b, T v) {
    return b.v0_ + (v-a.v0_)*(b.length()/a.length());
  }

  // Shrinks an interval by a given amount on each end.  If the interval would
  // become inverted, returns an empty interval at the midpoint.
  friend _Interval erode(_Interval a, T d) {
    _Interval ans = {a.v0_ + d, a.v1_ - d};
    if (ans.v0_ >= ans.v1_) {
      T middle = (ans.v0_ + ans.v1_)/2;
      return _Interval(middle, middle);
    }
    return ans;
  }

  // Grows an interval by a given amount on each end.
  friend _Interval dilate(_Interval a, T d) {
    return {a.v0_ - d, a.v1_ + d};
  }

  // Rounds an interval to integer coordinates s.t. length() is >= old length.
  friend _Interval ceil(_Interval a) {
    return {std::floor(a.v0_), std::ceil(a.v1_)};
  }

  // Rounds an interval to integer coordinates s.t. length() is <= old length.
  friend _Interval floor(_Interval a) {
    return {std::ceil(a.v0_), std::floor(a.v1_)};
  }

  // Returns an interval representing the intersection of a and b.
  friend _Interval intersect(_Interval a, _Interval b) {
    _Interval ans { std::max(a.v0_, b.v0_), std::min(a.v1_, b.v1_) };
    if (ans.v1_ < ans.v0_) {
      return {};
    }
    return ans;
  }

  // Returns the interval that bounds a and b.
  friend _Interval bounding(_Interval a, _Interval b) {
    return { std::min(a.v0_, b.v0_), std::max(a.v1_, b.v1_) };
  }

  // Equality operators.
  bool operator !=(_Interval b) const { return !operator==(b); }
  bool operator ==(_Interval b) const { return v0_ == b.v0_ && v1_ == b.v1_; }


private:
  T v0_,v1_;

  // Forces interval to be upright if needed.
  void rectify() {
    *this = _Interval(std::min(v0_, v1_), std::max(v0_, v1_));
  }

};

using intervalf = _Interval<float>;
using interval  = _Interval<double>;
using intervali = _Interval<int>;

template <typename T, int _kDim, typename Vector, typename Derived>
struct _Region {
  static constexpr int kDim = _kDim;
  using type     = T;
  using interval = _Interval<T>;
  using vector   = Vector;

  _Region() = default;

  // Creates a region from two points specifying opposite corners.
  _Region(const vector& p0, const vector& p1) {
    for (int i=0; i < kDim; ++i) {
      intervals_[i] = {p0[i], p1[i]};
    }
  }

  // Converting constructor from another type of region.
  template <typename U>
  explicit _Region(const _Region<U, kDim, Vector, Derived>& region) {
    for (int i=0; i < kDim; ++i) {
      intervals_[i] = static_cast<_Interval<U>>(region.intervals_[i]);
    }
  }

  // Get extrema as points.
  vector lo() const {
    vector ans;
    for (int i=0; i < kDim; ++i) {
      ans[i] = intervals_[i].v0();
    }
    return ans;
  }

  vector hi() const {
    vector ans;
    for (int i=0; i < kDim; ++i) {
      ans[i] = intervals_[i].v1();
    }
    return ans;
  }


  bool operator!=(const _Region& a) const { return !(*this == a); }
  bool operator==(const _Region& a) const {
    bool equal = true;
    for (int i=0; i < kDim; ++i) {
      equal &= (intervals_[i] == a.intervals_[i]);
    }
    return equal;
  }

  const interval& operator[](int idx) const { return intervals_[idx];  }
  interval& operator[](int idx)       { return intervals_[idx]; }

  // Shifts region by an offset given by a vector.
  Derived& operator +=(const vector& v) {
    for (int i=0; i < kDim; ++i) {
      intervals_[i] += v[i];
    }
    return region();
  }

  Derived operator +(const vector& v) const {
    _Region ans = *this;
    ans += v;
    return ans.region();
  }

  // Shifts region by an offset given by a vector.
  Derived& operator -=(const vector& v) {
    for (int i=0; i < kDim; ++i) {
      intervals_[i] -= v[i];
    }
    return region();
  }

  Derived operator -(const vector& v) const {
    _Region ans = *this;
    ans -= v;
    return ans.region();
  }


  // Returns true if the region contains the given point.
  bool contains(const vector& pnt) const {
    bool contains = true;
    for (int i=0; i < kDim; ++i) {
      contains &= intervals_[i].contains(pnt[i]);
    }
    return contains;
  }

  // Returns true if the region intersects the other region.
  bool intersects(const _Region& region) const {
    bool intersects = true;
    for (int i=0; i < kDim; ++i) {
      intersects &= intervals_[i].intersects(region.intervals_[i]);
    }
    return intersects;
  }

  // Returns true if the region is empty.
  bool empty() const {
    bool empty = true;
    for (int i=0; i < kDim; ++i) {
      empty &= intervals_[i].empty();
    }
    return empty;
  }

  // Returns the length of a given dimension of the region.
  T length(int dim) const { return intervals_[dim].length(); }

  // Returns the measure of the region as the product of lengths.
  T measure() const {
    T measure = length(0);
    for (int i=1; i < kDim; ++i) {
      measure *= length(i);
    }
    return measure;
  }

  // Splits a region in the given dimension into two regions.
  std::pair<_Region, _Region> split(int dim, T p) const {
    auto [lower, upper] = intervals_[dim].split(p);
    _Region a = *this;
    _Region b = *this;

    a[dim] = lower;
    b[dim] = upper;
    return std::make_pair(a,b);
  }

  // Linearly interpolate between two regions.
  friend vector lerp(const _Region& a, const _Region& b, const vector& p) {
    vector ans;
    for (int i=0; i < kDim; ++i) {
      ans[i] = lerp(a[i], b[i], p[i]);
    }
    return ans;
  }

  // Returns the intersection of the two regions.
  friend Derived intersect(const Derived& a, const Derived& b) {
    Derived ans;
    for (int i=0; i < kDim; ++i) {
      ans[i] = intersect(a[i], b[i]);
    }
    return ans;
  }

  // Returns the region bounding the two regions.
  friend Derived bounding(const Derived& a, const Derived& b) {
    Derived ans;
    for (int i=0; i < kDim; ++i) {
      ans[i] = bounding(a[i], b[i]);
    }
    return ans;
  }

  // Returns a region rounded to integer coordinates s.t. the measure of the
  // region is >= its previous value.
  friend Derived ceil(const Derived& a) {
    Derived ans;
    for (int i=0; i < kDim; ++i) {
      ans[i] = ceil(a[i]);
    }
    return ans;
  }

  // Returns a region rounded to integer coordinates s.t. the measure of the
  // region is <= its previous value.
  friend Derived floor(const Derived& a) {
    Derived ans;
    for (int i=0; i < kDim; ++i) {
      ans[i] = floor(a[i]);
    }
    return ans;
  }

  // Returns a region shrunk by a given amount in each dimension.
  friend Derived erode(const Derived& a, T d) {
    Derived ans;
    for (int i=0; i < kDim; ++i) {
      ans[i] = erode(a[i], d);
    }
    return ans;
  }

  // Returns a region grown by a given amount in each dimension.
  friend Derived dilate(const Derived& a, T d) {
    Derived ans;
    for (int i=0; i < kDim; ++i) {
      ans[i] = dilate(a[i], d);
    }
    return ans;
  }

private:
  // This class will only be used as a base of Derived, so its safe to downcast.
        Derived& region()       { return *static_cast<Derived*>(this); }
  const Derived& region() const { return *static_cast<const Derived*>(this); }

  interval intervals_[kDim];
};


template <class T>
struct _Region1 : _Region<T, 1, T, _Region1<T>> {
  using base     = _Region<T, 1, T, _Region1<T>>;
  using interval = typename base::interval;
  using vector   = T;

  _Region1() = default;

  _Region1(T p0, T p1)
    : base(p0, p1) {}

  template <typename U>
  _Region1(const _Region1<U>& a)
    : base(static_cast<vector>(a.lo()), static_cast<vector>(a.hi())) {}

  // Splits the region into two regions along named axes.
  std::pair<_Region1, _Region1> splitx(T p) const { return split(0, p); }

  // Accesses the intervals by name.
  const interval& xinterval() const { return (*this)[0]; }
        interval& xinterval()       { return (*this)[0]; }

  // Names for x-oriented measures.
  T length() const { return (*this)[0].length(); }
};


template <class T>
struct _Region2 : _Region<T, 2, Vector2<T>, _Region2<T>> {
  using base     = _Region<T, 2, Vector2<T>, _Region2<T>>;
  using interval = typename base::interval;
  using vector   = Vector2<T>;

  _Region2() = default;

  _Region2(vector p0, vector p1)
    : base(p0, p1) {}

  _Region2(T x0, T y0, T xlen, T ylen)
    : base(vector(x0,y0), vector(x0+xlen,y0+ylen)) {}

  template <typename U>
  _Region2(const _Region2<U>& a)
    : base(static_cast<vector>(a.lo()), static_cast<vector>(a.hi())) {}

  // Returns a corner of the region.  For convenience the vertex index is used
  // modulo 4.
  vector GetVertex(int k) const {
    switch (k % 4) {
      case 0: return vector(xinterval().v0(), yinterval().v0());
      case 1: return vector(xinterval().v1(), yinterval().v0());
      case 2: return vector(xinterval().v1(), yinterval().v1());
      case 3: return vector(xinterval().v0(), yinterval().v1());
    }
    ABSL_UNREACHABLE();
  }

  // Splits the region into two regions along named axes.
  std::pair<_Region2, _Region2> splitx(T p) const { return split(0, p); }
  std::pair<_Region2, _Region2> splity(T p) const { return split(1, p); }

  // Accesses the intervals by name.
  const interval& xinterval() const { return (*this)[0]; }
        interval& xinterval()       { return (*this)[0]; }

  const interval& yinterval() const { return (*this)[1]; }
        interval& yinterval()       { return (*this)[1]; }

  // Names for x-oriented measures.
  T  width() const { return (*this)[0].length(); }
  T length() const { return (*this)[0].length(); }

  // Names for y-oriented and 2D measures.
  T height() const { return (*this)[1].length(); }
  T   area() const { return (*this).measure();   }
};


template <class T>
struct _Region3 : _Region<T, 3, Vector3<T>, _Region3<T>> {
  using base     = _Region<T, 3, Vector3<T>, _Region3<T>>;
  using interval = typename base::interval;
  using vector   = Vector2<T>;

  _Region3() = default;

  _Region3(vector p0, vector p1)
    : base(p0, p1) {}

  _Region3(T x0, T y0, T xlen, T ylen)
    : base(vector(x0,y0), vector(x0+xlen,y0+ylen)) {}

  template <typename U>
  _Region3(const _Region3<U>& a)
    : base(static_cast<vector>(a.lo()), static_cast<vector>(a.hi())) {}

  // Splits the region into two regions along named axes.
  std::pair<_Region3, _Region3> splitx(T p) const { return (*this).split(0, p); }
  std::pair<_Region3, _Region3> splity(T p) const { return (*this).split(1, p); }
  std::pair<_Region3, _Region3> splitz(T p) const { return (*this).split(2, p); }

  // Accesses the intervals by name.
  const interval& xinterval() const { return (*this)[0]; }
        interval& xinterval()       { return (*this)[0]; }

  const interval& yinterval() const { return (*this)[1]; }
        interval& yinterval()       { return (*this)[1]; }

  const interval& zinterval() const { return (*this)[2]; }
        interval& zinterval()       { return (*this)[2]; }

  // Names for x-oriented measures.
  T  width() const { return (*this)[0].length(); }
  T length() const { return (*this)[0].length(); }

  // Names for y-oriented measures.
  T height() const { return (*this)[1].length(); }

  // Names for z-oriented and 3D measures.
  T  depth() const { return (*this)[2].length(); }
  T volume() const { return (*this).measure();   }
};

using region1f = _Region1<float>;
using region1  = _Region1<double>;
using region1i = _Region1<int>;

using region2f = _Region2<float>;
using region2  = _Region2<double>;
using region2i = _Region2<int>;

using region3f = _Region3<float>;
using region3  = _Region3<double>;
using region3i = _Region3<int>;

#ifdef DOCTEST_LIBRARY_INCLUDED

TEST_CASE_TEMPLATE("Interval Checks", I, intervalf, interval, intervali) {
  REQUIRE(I{3,7}.length() == 4);

  REQUIRE(!I{3,7}.empty());
  REQUIRE(I{3,3}.empty());

  REQUIRE(I{0,10}.clamp(11) == 10);
  REQUIRE(I{0,10}.clamp(-2) ==  0);

  REQUIRE(I{0,10} == I{0,10});
  REQUIRE(I{0,10} != I{1,3});

  REQUIRE(I{0,10} + 3 == I{3,13});
  REQUIRE(3 + I{0,10} == I{3,13});

  REQUIRE(I{0,10} - 3  == I{-3,7});

  REQUIRE(I{3,7} * 5   == I{15, 35});
  REQUIRE(5 * I{3,7}   == I{15, 35});

  REQUIRE(I{15,35} / 5   == I{3,7});

  REQUIRE(I{0,10}.contains(0));
  REQUIRE(I{0,10}.contains(5));
  REQUIRE(!I{0,10}.contains(10));

  REQUIRE(I{0,10}.contains(0));
  REQUIRE(I{0,10}.contains(5));
  REQUIRE(!I{0,10}.contains(10));

  REQUIRE(I{0,10}.split(3)  == std::make_pair(I{0,3}, I{3, 10}));
  REQUIRE(I{0,10}.split(0)  == std::make_pair(I{0,0}, I{0, 10}));
  REQUIRE(I{0,10}.split(10) == std::make_pair(I{0,10},I{10,10}));

  REQUIRE(lerp(I{0,3}, I{2,8}, 0) == 2);
  REQUIRE(lerp(I{0,3}, I{2,8}, 3) == 8);
  REQUIRE(lerp(I{0,3}, I{2,8}, 2) == 6);

  REQUIRE(erode(I{0,10},  2) == I{2,8});
  REQUIRE(erode(I{0,10}, 10) == I{5,5});

  REQUIRE(dilate(I{0,10}, 2) == I{-2, 12});

  REQUIRE(intersect(I{0,10}, I{-5, -10}).empty());
  REQUIRE(intersect(I{0,10}, I{11,  20}).empty());
  REQUIRE(intersect(I{0,10}, I{2,5}) == I{2,5});
  REQUIRE(intersect(I{0,10}, I{-15,25}) == I{0,10});
  REQUIRE(intersect(I{0,10}, I{-3,5}) == I{0,5});
  REQUIRE(intersect(I{0,10}, I{8,12}) == I{8,10});

  REQUIRE(bounding(I{3,6}, I{7,8}) == I{3,8});
  REQUIRE(bounding(I{3,6}, I{0,1}) == I{0,6});
  REQUIRE(bounding(I{0,1}, I{3,6}) == I{0,6});
  REQUIRE(bounding(I{2,5}, I{3,6}) == I{2,6});
}

TEST_CASE_TEMPLATE("Floating Interval Checks", I, intervalf, interval) {
  REQUIRE( ceil(I{0.5, 9.5}) == I{0,10});
  REQUIRE(floor(I{0.5, 9.5}) == I{1,9});
}

TEST_CASE_TEMPLATE("1D Region Checks", R, region1f, region1, region1i) {
  R region;
  (void)region;
}

TEST_CASE_TEMPLATE("2D Region Checks", R, region2f, region2, region2i) {
  R region;
  (void)region;
}

TEST_CASE_TEMPLATE("3D Region Checks", R, region3f, region3, region3i) {
  R region;
  (void)region;
}

#endif

} // namespace w
