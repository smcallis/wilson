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

#include "s2/r2.h"
#include "s2/util/math/vector.h"

// Three dimensional transform represented as a 4x4 affine matrix composed of
// four 3-vectors, u, v, w, and t:
//
// u.x u.y u.z t.x
// v.x v.y v.z t.y
// w.x w.y w.z t.z
// 0   0   0   1
//

template <typename T>
struct _Affine3 {
  using type   = T;
  using point  = Vector3<T>;
  using vector = Vector3<T>;

  _Affine3() = default;

  // Builds a transform from two or more vectors.
  _Affine3(vector u, vector v, vector w={}, vector t={})
    : u(u), v(v), w(w), t(t) {}

  bool operator!=(const _Affine3 &b) const { return !(*this == b); }
  bool operator==(const _Affine3 &b) const {
    return u == b.u && v == b.v && w == b.w && t == b.t;
  }

  // Multiplies a vector by this transform.
  friend vector operator*(const _Affine3& a, const vector& vec) {
    return vector(a.u.DotProd(vec), a.v.DotProd(vec), a.w.DotProd(vec)) + a.t;
  }

  // Multiplies a two-vector by this transform.  The point is implicitly
  // converted to a 3d point by setting z=0, and the resulting z value is
  // dropped.
  friend Vector2<T> operator*(const _Affine3& a, const Vector2<T>& vec) {
    vector pnt(vec.x(), vec.y(), 0);
    vector mul = a*pnt;
    return {mul.x(), mul.y()};
  }

  // Multiplies another transform by this one.
  _Affine3 operator*(const _Affine3& b) const {
    // Transpose the transform part of B so we can dot it.
    vector bx(b.u.x(), b.v.x(), b.w.x());
    vector by(b.u.y(), b.v.y(), b.w.y());
    vector bz(b.u.z(), b.v.z(), b.w.z());
    vector bt = b.t;

    return _Affine3(
      vector(u.DotProd(bx), u.DotProd(by), u.DotProd(bz)),
      vector(v.DotProd(bx), v.DotProd(by), v.DotProd(bz)),
      vector(w.DotProd(bx), w.DotProd(by), w.DotProd(bz)),
      vector(u.DotProd(bt), v.DotProd(bt), w.DotProd(bt)) + t
    );
  }

  // Scale this transform by constant value.
  _Affine3 operator*(T scale) const {
    _Affine3 ans = *this;
    ans *= scale;
    return ans;
  }

  _Affine3& operator*=(T scale) {
    u *= scale;
    v *= scale;
    w *= scale;
    t *= scale;
    return *this;
  }

  // Returns a transform that permutes axes by rotating the set of basis vectors.
  //
  // E.g. if {i,j,k} are the basis vectors, then Permute(1) returns {j,k,i} as
  // the transform.  The shift is taken mod 3 so Permute(3) == Permute(0);
  static _Affine3 Permute(int shift) {
    while (shift < 0) shift += 3;
    shift = shift % 3;

    if (shift == 0) {
      return Eye();
    }

    static const vector bases[3] = {
      {1,0,0},
      {0,1,0},
      {0,0,1}
    };

    return _Affine3(
      bases[(0 + shift) % 3],
      bases[(1 + shift) % 3],
      bases[(2 + shift) % 3]
    );
  }

  // Returns the identity transform.
  static _Affine3 Eye() {
    return _Affine3(
      {1,0,0},
      {0,1,0},
      {0,0,1},
      {0,0,0}
    );
  }

  // Returns a rotation around an arbitrary axis by given angle (radians).
  //   see: https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
  static _Affine3 Rotate(T angle, vector u) {
    T c = std::cos(angle);
    T s = std::sin(angle);
    T mc = 1-c;

    u = u.Normalize();
    return _Affine3(
      vector(u.x()*u.x()*mc + c,       u.x()*u.y()*mc - u.z()*s, u.x()*u.z()*mc + u.y()*s),
      vector(u.y()*u.x()*mc + u.z()*s, u.y()*u.y()*mc + c,       u.y()*u.z()*mc - u.x()*s),
      vector(u.z()*u.x()*mc - u.y()*s, u.z()*u.y()*mc + u.x()*s, u.z()*u.z()*mc + c)
    );
  }

  // Returns a rotation by the given angle (radians) around the x axis.
  static _Affine3 RotateX(T angle) {
    const T c = std::cos(angle);
    const T s = std::sin(angle);
    return _Affine3(
      {1,  0,  0},
      {0, +c, -s},
      {0, +s, +c}
    );
  }

  // Returns a rotation by the given angle (radians) around the y axis.
  static _Affine3 RotateY(T angle) {
    const T c = std::cos(angle);
    const T s = std::sin(angle);
    return _Affine3(
      {+c, 0, +s},
      { 0, 1,  0},
      {-s, 0, +c}
    );
  }

  // Returns a rotation by the given angle (radians) around the z axis.
  static _Affine3 RotateZ(T angle) {
    const T c = std::cos(angle);
    const T s = std::sin(angle);
    return _Affine3(
      {+c, -s, 0},
      {+s, +c, 0},
      { 0,  0, 1}
    );
  }

  // Returns a scale matrix which scales by each component of the given vector.
  static _Affine3 Scale(const vector& s) {
    _Affine3 eye = Eye();
    eye.u *= s.x();
    eye.v *= s.y();
    eye.w *= s.z();
    return eye;
  }

  // Returns a scale matrix which scales by a constant value in all axes.
  static _Affine3 Scale(const T& scale) {
    return Eye()*scale;
  }

  // Returns a translation by a given vector.
  static _Affine3 Translate(const vector& p) {
    _Affine3 eye = Eye();
    eye.t = p;
    return eye;
  }

  // Generates an orthographic projection matrix that maps the volume inside the
  // given clipping planes in each axis to the unit cube.  The zoom parameter can
  // be used to zoom the view conveniently without scaling every parameter.
  static _Affine3 Orthographic(T xl, T xh, T yl, T yh, T zl, T zh, T zoom=1) {
    T xp = xh+xl, xm = xh-xl;
    T yp = yh+yl, ym = yh-yl;
    T zp = zh+zl, zm = zh-zl;

    return _Affine3(
      {zoom*2/xm, 0, 0},
      {0, zoom*2/ym, 0},
      {0, 0, -zoom*2/zm},
      {-zoom*xp/xm, -zoom*yp/ym, -zoom*zp/zm}
    );
  }

  vector u,v,w,t;
};

typedef _Affine3<float> Affine3f;
typedef _Affine3<double> Affine3;
