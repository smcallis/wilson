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

#include <optional>

#include "absl/base/thread_annotations.h"
#include "absl/base/nullability.h"
#include "absl/container/inlined_vector.h"
#include "absl/functional/function_ref.h"
#include "absl/synchronization/mutex.h"
#include "s2/r2.h"
#include "s2/s2cap.h"
#include "s2/s2cell_union.h"
#include "s2/s2lax_polyline_shape.h"
#include "s2/s2point.h"
#include "s2/s2region_coverer.h"
#include "s2/s2shape.h"
#include "s2/s2edge_distances.h"
#include "s2/s2contains_point_query.h"

#include "wilson/graphics/pixel.h"
#include "wilson/chain_sink.h"
#include "wilson/chain_stitcher.h"
#include "wilson/quaternion.h"
#include "wilson/r2shape.h"
#include "wilson/region.h"
#include "wilson/transform.h"
#include "wilson/projector.h"

namespace w {

// A definition of an abstract Projection class to represent arbitrary map
// projections.  We define the behavior of projections somewhat differently than
// other map programs.  Rather than allowing the user to view any part of a
// projection by zooming in on it, changing the visible region is accomplished
// by applying a rotation to the unit sphere -before- it's projected.  This has
// a couple of advantages:

//   * It's more generic.  By and large the only thing projections have in
//     common is that they operate on the unit sphere.  By rotating the sphere
//     we can reposition the viewport in a way that works for -all- projections.
//
//   * It's more separable.  The individual projection implementations only have
//     to focus on their projection-specific details relative to a unit sphere,
//     without having to worry about zoom and rotation on their own.
//
// The downside is that some projections seem 'odd' when the underlying sphere
// has been rotated.  An easy example is the Plate carrée projection, which is
// -highly- distorted at the poles.  As the unit sphere is rotated, this
// distortion is readily apparent.  But, this is a feature of the projection
// itself and shouldn't be disregarded.
//
// Class Structure
// ───────────────
//
// For algorithms such as edge subdivision, the core logic is the same for any
// projection, only the details of projecting to screen space change.  We'd
// prefer not to dispatch virtually for every single call to Project().
//
// Normally we could use CRTP directly for this but we still want a virtual
// interface to use projections through so that we can slot them in as needed.
//
// So we define a base Projection interface for projections which defines the
// basic interface that a projection must implement.
//
// Then we define the type that derived projections can inherit from, which is
// Projection<T>.  This uses CRTP to provide default implementations for
// pieces of the Projection API with the benefit of knowing the derived class
// type.
//
// Projections can then just inherit from Projection<T> with CRTP and they'll
// receive the default implementations for things like edge subdivision.  They
// may still be overridden in derived classes as appropriate (e.g. Gnomonic
// projections turn edges into straight lines so edge subdivision isn't useful
// there).
//
//   +-------------+
//   | Projection  |
//   +-------------+
//          |
//          v
//  +-------------------+
//  | Projection<T>     |<--------------+
//  +-------------------+               |
//          |                           |
//          v                           |
//  +---------------------------------------------+
//  |   Orthographic : Projection<Orthographic>   |
//  +---------------------------------------------+
//
//
// Details
// ───────
//
// We define three coordinate spaces for geometry:
//
//   world  - Cartesian space in three dimensions after rotation is applied.
//   unit   - Cartesian space centered on 0,0 without scaling, after projecting.
//   screen - Cartesian space in pixels, translated and scaled to the window.
//
// The visible region in unit space is always a rectangular region centered on
// (0, 0).  This region can be any width or height, but the aspect ratio is
// always the same as the window.  The smaller of the two dimensions is taken to
// map to the range [-1, 1], with the other dimension scaled equivalently.
//
// As an example, the Plate carrée projection has an aspect ratio of 2 (its
// width is twice its height).  So the height is mapped to the range [-1, 1]
// and the width to [-2, 2]:
//
//     +------+------+------+
//     |      |      |      |
//     |      | Unit |      |
//     |      |      |      |
//     +------+------+------+
//
//     ├────────────────────┤
//             Screen
//
// Everything in the screen space centered about the origin is ultimately
// visible in the window.  As the projection is zoomed in by scaling, the
// projected image of the unit sphere will grow in unit space, but the visible
// region won't, giving the zoom effect.
//
// We define two transformations that are composed to project to the window:
//
//   * WorldToUnit - Converts a rotated point from world space to unit space.
//   * UnitToScreen - Converts from unit space centered on the origin to screen
//   space by translating and shifting.
//
// Of the two, Projection implementations are only required to define the
// WorldToUnit transformation (via implementations of Project()).
//

// Builds a function that takes an S2Point and returns whether a particular
// shape in an S2ShapeIndex contains it.  The input index must live at least
// as long as the returned function.
static inline absl::AnyInvocable<bool(const S2Point&)> IndexedContains(
  const S2ShapeIndex& index, int shape_id) {
  const S2Shape& shape = *index.shape(shape_id);
  S2ContainsPointQuery<S2ShapeIndex> query(&index);

  return [&, query = std::move(query)](const S2Point& point) mutable {
    return query.ShapeContains(shape, point);
  };
}

class IProjection {
public:
  // The default distance-squared error (in pixels) when subdividing.
  static constexpr double kDefaultProjectionErrorSq = 0.125*0.125;

  // Returns true if a shape contains a given point, false otherwise.
  using ContainsPointFn = absl::AnyInvocable<bool(const S2Point&)>;

  // A simple struct to store scale and rotation values for the projection.
  struct Transform {
    Transform() = default;

    Transform(double scale, Quaternion rotation)
      : scale_(scale), rotation_(rotation) {
      rotate_fwd_ = rotation_.ToTransform();
      rotate_inv_ = rotation_.Inverse().ToTransform();
    }

    // Returns the current scale factor.
    double Scale() const { return scale_; }

    // Returns the current rotation.
    Quaternion Rotation() const { return rotation_; }

    // Rotate a point by the forward rotation set by the user.
    S2Point RotateFwd(S2Point p) const {
      return rotate_fwd_*p;
    }

    // Rotate a point by the inverse rotation.
    S2Point RotateInv(S2Point p) const {
      return rotate_inv_*p;
    }

  private:
    double scale_;
    Quaternion rotation_;

    // Cached affined transformations for the rotation.
    Affine3 rotate_fwd_;
    Affine3 rotate_inv_;
  };

  virtual ~IProjection() = default;

  //--------------------------------------------------------------------------
  // Configuration API
  //--------------------------------------------------------------------------

  // Sets the maximum squared error (in pixels) for edge tessellation.
  // Default: kDefaultProjectionErrorSq
  void SetMaxSqError(double error) {
    max_sq_error_ = error;
  }

  // Returns the maximum squared error (in pixels) for edge tessellation.
  double MaxSqError() const {
    return max_sq_error_;
  }

  // Sets the rotation to apply to points before projection.
  void SetRotation(const Quaternion& q) {
    SetTransform({transform_.Scale(), q});
  }

  // Returns a quaternion representing the current rotation.
  Quaternion Rotation() const {
    return transform_.Rotation();
  }

  // Sets the scale to apply when projecting points.
  void SetScale(double scale) {
    SetTransform(Transform(scale, transform_.Rotation()));
  }

  // Returns the current scale factor.
  double Scale() const {
    return transform_.Scale();
  }

  // Returns the un-rotated image of the point (1, 0, 0).

  // This point is the center of the projection of the unit sphere and will
  // always be mapped to (0, 0) in unit space.
  S2Point Nadir() const {
    return nadir_;
  }

  // Pushes the current transform onto a stack; must pair with PopTransform().
  void PushTransform() {
    transforms_.emplace_back(transform_);
  }

  // Pops a transform off of the stack, restoring the projection state.
  bool PopTransform() {
    if (transforms_.empty()) {
      return false;
    }

    SetTransform(transforms_.back());
    transforms_.pop_back();
    return true;
  }

  //--------------------------------------------------------------------------
  // Size API
  //--------------------------------------------------------------------------

  // Resizes screen space to the given dimensions.
  virtual void    Resize(int width, int height) = 0;

  // Returns the current width of screen space.
  virtual double  Width() const = 0;

  // Returns the current height of screen space.
  virtual double  Height() const = 0;

  // Returns a 2D region representing screen space.
  virtual region2 Screen() const = 0;

  // Returns the scale factor between unit and screen space.
  //
  // Any unit vector in unit space will have this length in screen space.
  virtual double UnitScale() const = 0;

  //--------------------------------------------------------------------------
  // Utility API
  //--------------------------------------------------------------------------

  // Appends an outline for the projection in screen space to the given sink.
  //
  // Does not clear the sink before appending.  The resulting outline is
  // suitable for filling with a background color for the projection as a whole.
  virtual void MakeOutline(absl::Nonnull<ChainSink*> out) const = 0;

  // Appends a graticule showing lines of latitude and longitude to a sink.
  //
  // Does not clear the sink before appending.
  virtual void MakeGraticule(absl::Nonnull<ChainSink*> out) const = 0;

  // Returns an S2Cap covering the viewport.  This is a coarse covering but
  // generally much faster than getting a full viewport covering via Viewport().
  virtual S2Cap Viewcap() const = 0;

  // Returns an S2CellUnion cell covering for the visible region of the sphere.
  virtual const S2CellUnion& Viewport() const;

  //--------------------------------------------------------------------------
  // Transformation API
  //--------------------------------------------------------------------------

  // Rotates a point using the current Rotation().
  S2Point Rotate(const S2Point& point) const {
    return transform_.RotateFwd(point);
  }

  // Rotates a point using the inverse of the current Rotation().
  S2Point Unrotate(const S2Point& point) const {
    return transform_.RotateInv(point);
  }

  // Converts a point from world space to unit space.
  virtual R2Point WorldToUnit(const S2Point& point) const = 0;

  // Converts a point from unit space to screen space.
  virtual R2Point UnitToScreen(const R2Point& point) const = 0;

  // Converts a point from screen space to unit space.
  virtual R2Point ScreenToUnit(const R2Point&) const = 0;

  // Converts a point from unit space back to world space.  If the inverse
  // projection of the point hits the unit sphere, then that point is returned.
  //
  // Otherwise, if 'nearest' is true, then the point is project onto the closest
  // visible point on the unit sphere.
  virtual bool UnitToWorld( //
    absl::Nonnull<S2Point*> out, const R2Point& point, bool nearest) const = 0;


  // Projects a point from world space to screen space.
  //
  // Returns true if the point is visible on screen, false otherwise.
  virtual bool Project(absl::Nonnull<R2Point*> out, const S2Point&) const = 0;


  // Transforms a point from screen space back to world space, if possible.
  //
  // If unprojecting the point onto the unit sphere is possible, then the
  // unprojected point stored in out.  If the point can't be unprojected but
  // 'nearest' is true, then the nearest visible point is stored and true
  // returned.
  //
  // Otherwise, returns false.
  virtual bool Unproject(  //
    absl::Nonnull<S2Point *> out, const R2Point& point, bool nearest) const = 0;


  // Projects an edge from world space to screen space.  The edge is subdivided,
  // respecting the current MaxSqError() and appended to the given ChainSink.
  //
  // Does not clear the sink before appending.  May add breaks to the ChainSink.
  virtual void Project( //
    absl::Nonnull<ChainSink*> out, const S2Shape::Edge& edge) const = 0;


  // Projects a shape of 0 or 1 dimensions into screen space.  Any edges are
  // subdivided, honoring the current MaxSqError(), and appended to the given
  // ChainSink.
  //
  // Does not clear the sink before appending.  May add breaks to the ChainSink.
  virtual void Project(  //
    absl::Nonnull<ChainSink*> out, const S2Shape& shape) const = 0;


  // Projects a polygon into screen space.  0 and 1 dimensional shapes should
  // call Project() above.  Any edges are subdivided, honoring the current
  // MaxSqError(), and appended to the given ChainSink.
  //
  // A ChainStitcher instance must be provided to be used to stitch polygon
  // chains together.
  //
  // When clipping polygons, a callback must be provided that can be used to
  // test whether a single point is contained in the polygon.  For shapes that
  // are indexed, IndexedContains() will automatically generate a suitable
  // function for a given shape.
  //
  // Does not clear the sink before appending.  May add breaks to the ChainSink.
  virtual void Project(  //
      absl::Nonnull<ChainSink*> out, absl::Nonnull<ChainStitcher*> stitcher,
      const S2Shape&, ContainsPointFn contains) const = 0;


  // Projects an S2Cap into screen space.  The cap is subdivided, honoring the
  // current MaxSqError() and the resulting points are appended to the given
  // ChainSink.
  //
  // A ChainStitcher instance must be provided to be used to stitch polygon
  // chains together.
  //
  // Does not clear the sink before appending.  May add breaks to the ChainSink.
  virtual void Project(  //
      absl::Nonnull<ChainSink*> out, absl::Nonnull<ChainStitcher*> stitcher,
      const S2Cap& cap) const = 0;

  // --------------------------------------------------------------------------
  // Overloads using default values.
  // We can't use default parameters with virtual methods.
  // --------------------------------------------------------------------------

  bool UnitToWorld(absl::Nonnull<S2Point*> out, const R2Point& point) const {
    return UnitToWorld(out, point, false);
  }

  bool Unproject(absl::Nonnull<S2Point*> out, const R2Point& point) const {
    return Unproject(out, point, false);
  }

protected:
  // Called on changes to scale, size, and rotation.
  virtual void UpdateTransforms() {}

  // An optional transformation to make to the geometry before rotation.
  virtual S2Point PreRotate(const S2Point& point) const { return point; }

private:
  // Sets the current transform for the projection.
  void SetTransform(Transform transform) {
    transform_ = std::move(transform);
    nadir_ = transform_.RotateInv(S2Point(1,0,0));

    {
      // Mark the viewport covering dirty.
      absl::WriterMutexLock lock(&viewport_lock_);
      viewport_dirty_ = true;
    }

    // And notify derived classes that transforms changed.
    UpdateTransforms();
  }

  // A stack of transforms and the current transform.
  absl::InlinedVector<Transform, 1> transforms_;
  Transform transform_ = {Transform(1/1.1, {})};

  // The center point of the projection.
  S2Point nadir_ = S2Point(1, 0, 0);

  // Maximum squared error (in pixels) when tessellating edges.
  double max_sq_error_ = kDefaultProjectionErrorSq;

  // A cell covering for the viewport, guarded by a mutex so that we can
  // generate it on-demand.
  mutable absl::Mutex viewport_lock_;
  mutable bool viewport_dirty_ = true; ABSL_GUARDED_BY(viewport_lock_);
  mutable S2CellUnion viewport_ ABSL_GUARDED_BY(viewport_lock_);
};

// A partial implementation of IProjection that provides default implementations
// for things such as Subdivide() and Project() with the benefit of knowing the
// derived type.  The compiler can use that knowledge to inline function calls
// and improve performance, but will still get the benefit of a virtually
// dispatched interface.
//
// Individual projection implementations should inherit from this class.
template <typename D>
class Projection : public IProjection {
public:
  using IProjection::Project;
  using IProjection::Unproject;

  // The aspect ratio of the projection is defined as width/height.  Projection
  // implementations can define their own static constant if they need a ratio
  // other than the default 1:1.
  static constexpr double kAspectRatio = 1.0;

  // The natural scale of a projection is a multiple of the space needed to fit
  // a unit circle on screen, along a single dimension.  E.g. if we project onto
  // the faces of a cube, then we may need sqrt(2) additional scale to fit the
  // cube within the unit circle.
  //
  // Subclasses may define their own constant which will be used when defining
  // the unit_scale().
  static constexpr double kNaturalScale = 1.0;

  // Size API implementations.
  //
  // These are in the Projection class because we need more information from
  // the derived class to implement them (i.e. aspect ratio and natural scale).
  void    Resize(int width, int height) final;
  double  Width()  const final { return width_;  }
  double  Height() const final { return height_; }
  region2 Screen() const final {
    return region2(0, 0, Width(), Height());
  }

  double UnitScale() const final {
    return unit_scale_;
  }

  R2Point UnitToScreen(const R2Point& p) const final {
    return unit_to_screen_ * p;
  }

  R2Point ScreenToUnit(const R2Point& p) const final {
    return screen_to_unit_ * p;
  }

  virtual S2Cap Viewcap() const override;

  virtual void Project(absl::Nonnull<ChainSink*> out,
    absl::Nonnull<ChainStitcher*> stitcher, const S2Cap& cap) const override;

  virtual bool Unproject(
    absl::Nonnull<S2Point*> out, const R2Point& point, bool nearest) const override;

private:
  // Cast back to the Derived class.
  const D& projection() const { return *static_cast<const D*>(this); }
  D& projection() { return *static_cast<D*>(this); }

  // Estimate the max distance of an R2 edge in screen space to a point.
  S1ChordAngle R2EdgeDistance(
    R2Point v0, R2Point v1, S2Point pnt, S1ChordAngle dist) const;

  int width_;
  int height_;

  // Current transformation matrices.
  double unit_scale_;
  Affine3 unit_to_screen_;
  Affine3 screen_to_unit_;
};

// Recursively bisect an unprojected edge to find the distance from a point.
template <typename Derived>
inline S1ChordAngle Projection<Derived>::R2EdgeDistance(
  R2Point v0, R2Point v1, S2Point center, S1ChordAngle dist) const {
  S2Point p0, p1;
  bool got0 = Unproject(&p0, v0, true);
  bool got1 = Unproject(&p1, v1, true);

  if (got0) dist = std::max(dist, S1ChordAngle::Radians(p0.Angle(center)));
  if (got1) dist = std::max(dist, S1ChordAngle::Radians(p1.Angle(center)));

  // Check the center point, see if it's further than dist.
  R2Point vc = v0 + (v1-v0)*0.5;
  S2Point pc;

  if (Unproject(&pc, vc)) {
    S1ChordAngle vc_dist = S1ChordAngle::Radians(pc.Angle(center));
    if (vc_dist > dist) {
      dist = vc_dist;
      dist = R2EdgeDistance(v0, vc, center, dist);
      dist = R2EdgeDistance(vc, v1, center, dist);
    }
  }
  return dist;
}


template <typename Derived>
inline S2Cap Projection<Derived>::Viewcap() const {
  S1ChordAngle radius = S1ChordAngle::Zero();
  for (int side=0; side < 4; ++side) {
    const R2Point v0 = Screen().GetVertex(side);
    const R2Point v1 = Screen().GetVertex(side+1);

    radius = R2EdgeDistance(v0, v1, Nadir(), radius);
  }
  return S2Cap(Nadir(), radius);
}


template <typename Derived>
inline void Projection<Derived>::Resize(int width, int height) {
  width_ = width;
  height_ = height;

  unit_scale_ = height*Derived::kAspectRatio/Derived::kNaturalScale;
  if (width < unit_scale_) {
    unit_scale_ = width;
  }
  unit_scale_ /= 2.0;

  double clip_width  = width/unit_scale_;
  double clip_height = height/unit_scale_;

  unit_to_screen_ =
    Affine3::Scale(unit_scale_)
    *Affine3::Translate({clip_width/2, clip_height/2, 0});

  screen_to_unit_ =
    Affine3::Translate({-clip_width/2, -clip_height/2, 0})
    *Affine3::Scale(1/unit_scale_);

  UpdateTransforms();
}

template <typename Derived>
bool Projection<Derived>::Unproject(
  absl::Nonnull<S2Point*> out, const R2Point& pnt, bool nearest) const {
  if (projection().UnitToWorld(out, projection().ScreenToUnit(pnt), nearest)) {
    *out = projection().Unrotate(*out);
    return true;
  }
  return false;
}

template <typename Derived>
void Projection<Derived>::Project(absl::Nonnull<ChainSink*> out,
  absl::Nonnull<ChainStitcher*> stitcher, const S2Cap& cap) const {

  ABSL_UNREACHABLE();

  // Find the center point of the cap and UV vectors forming a basis for points
  // along the cap boundary.
  const S2Point center = (1-cap.height())*cap.center();
  const S2Point u = center.CrossProd({0,0,1}).Normalize();
  const S2Point v = u.CrossProd(center).Normalize();

  // Get a point on the cap parameterized by angle.
  const double radius = std::sqrt(1-(1-cap.height())*(1-cap.height()));
  auto CapPoint = [&](double ang) {
    return (radius*u)*std::cos(ang) + (radius*v)*std::sin(ang) + center;
  };

  // Subdivide the cap into segments suitable for use as geodesic edges.  Stop
  // when the edges are short enough that projecting edges to screen space has
  // acceptable error from the actual cap boundary.
  std::vector<S2Point> points;
  const auto SubdivideCap = [&](
    double a0, S2Point p0, double a1, S2Point p1, auto&& self) -> void {

    // Find the midpoint between the two points by interpolating and directly
    // from the parametric form of the cap boundary defined above.
    double amid = (a0+a1)/2;
    S2Point pmid = CapPoint(amid);
    S2Point interp = S2::Interpolate(p0, p1, 0.5);

    // When we project the two different mid points, recurse if they're too far
    // apart in screen space.
    R2Point proj0, proj1;
    Project(&proj0, pmid);
    Project(&proj1, interp);

    if ((proj0-proj1).Norm2() > MaxSqError()) {
      self(a0, p0, amid, pmid, self);
      self(amid, pmid, a1, p1, self);
    } else {
      points.emplace_back(p0);
    }
  };

  // Subdivide quadrants.
  for (int q=0; q < 4; ++q) {
    double a0 = (q+0)*M_PI/2;
    double a1 = (q+1)*M_PI/2;
    SubdivideCap(a0, CapPoint(a0), a1, CapPoint(a1), SubdivideCap);
  }
  points.emplace_back(CapPoint(0));

  Project(out, stitcher, S2LaxPolylineShape(points), [](const S2Point& pnt) { return false; });
}


} // namespace w
