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

#include "s2/util/math/vector.h"
#include "wilson/geometry/transform.h"

template <typename T>
struct _quaternion {
  using type = T;
  using affine = _Affine3<type>;
  using vector = Vector3<type>;

  // Default quaternion is the unit quaternion representing no rotation.
  _quaternion()
    : v{1, 0, 0, 0} {}

  _quaternion(T s, T x, T y, T z)
    : v{s, x, y, z} {}

  _quaternion(T s, const vector& v)
    : v{s, v.x(), v.y(), v.z()} {}

  // Creates a quaternion that rotates by a given angle around a given axis.
  _quaternion(vector axis, T angle)
    : _quaternion(std::cos(angle/2), std::sin(angle/2)*axis) {}

  // Creates a quaternion that rotates the from vector into the to vector.
  _quaternion(vector from, vector to) {
    // const vector axis = from.CrossProd(to);
    // const T angle = std::atan(axis.Norm()/from.DotProd(to));

    // const T c = std::cos(angle/2);
    // const T s = std::sin(angle/2);
    // v = {c, s*axis.x(), s*axis.y(), s*axis.z()};

    const T d = from.DotProd(to);
    const vector w = from.CrossProd(to);
    v = Vector4<T>(d + sqrt(d*d + w.DotProd(w)), w.x(), w.y(), w.z()).Normalize();
  }

  // Multiplies this quaternion by another and returns the result.
  _quaternion operator*(const _quaternion& s) const {
    _quaternion ans = *this;
    ans *= s;
    return ans;
  }

  _quaternion& operator*=(const _quaternion& s) {
    _quaternion& r = *this;

    Vector4<T> v = {
      r[0]*s[0] - r[1]*s[1] - r[2]*s[2] - r[3]*s[3],
      r[0]*s[1] + r[1]*s[0] - r[2]*s[3] + r[3]*s[2],
      r[0]*s[2] + r[1]*s[3] + r[2]*s[0] - r[3]*s[1],
      r[0]*s[3] - r[1]*s[2] + r[2]*s[1] + r[3]*s[0]
    };
    r.v = v;
    return r;
  }

  // Returns the inverse of this quaternion.
  _quaternion Inverse() const {
    return {v[0], -v[1], -v[2], -v[3]};
  }

  // Returns the field of the quaternion at the given index.
  T operator[](int idx) const {
    return v[idx];
  }

  // Converts quaternion to an affine transform representing the rotation.
  affine ToTransform() const {
    T w = v[0];
    T x = v[1];
    T y = v[2];
    T z = v[3];

    T m2 = v.DotProd(v);
    return affine(
      vector(w*w+x*x-y*y-z*z, 2*x*y - 2*w*z,   2*x*z + 2*w*y)/m2,
      vector(2*x*y + 2*w*z,   w*w-x*x+y*y-z*z, 2*y*z - 2*w*x)/m2,
      vector(2*x*z - 2*w*y,   2*y*z + 2*w*x,   w*w-x*x-y*y+z*z)/m2
    );
  }

  Vector4<T> v;
};

using Quaternion  = _quaternion<double>;
using Quaternionf = _quaternion<float>;
