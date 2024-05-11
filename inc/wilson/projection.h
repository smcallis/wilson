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

#include "wilson/graphics/pixel.h"
#include "wilson/quaternion.h"
#include "wilson/r2shape.h"
#include "wilson/region.h"
#include "wilson/transform.h"

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

class IProjection {
public:
  // The default distance-squared error (in pixels) when subdividing.
  static constexpr double kDefaultProjectionErrorSq = 0.25*0.25;

  // A simple struct to store scale and rotation values for the projection.
  struct Transform {
    Transform() = default;

    Transform(double scale, Quaternion rotation)
      : scale_(scale), rotation_(rotation) {
      rotate_fwd_ = rotation_.ToTransform();
      rotate_inv_ = rotation_.Inverse().ToTransform();
    }

    double scale() const { return scale_; }
    Quaternion rotation() const { return rotation_; }

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

  using EdgeList = absl::InlinedVector<S2Shape::Edge, 2>;

  virtual ~IProjection() = default;

  // Resizes screen space to the given dimensions.
  virtual void Resize(int width, int height) = 0;

  // Returns the current width of screen space.
  virtual double width()  const = 0;

  // Returns the current height of screen space.
  virtual double height() const = 0;

  // Returns a 2D region representing screen space.
  virtual region2 screen() const = 0;


  // Returns an S2Cap covering the viewport.  This is a coarse covering but
  // generally much faster than getting a full viewport covering via Viewport().
  virtual S2Cap Viewcap() const = 0;


  // Returns a S2CellUnion cell covering for the visible region of the sphere.
  const S2CellUnion& Viewport() const;


  // Returns a scale factor between unit and screen space.
  //
  // Any unit vector in unit space will have this length in screen space.
  virtual double unit_scale() const = 0;


  // Returns the un-rotated image of the point (1, 0, 0).  This is the center
  // point of the projection on the unit sphere.  It will always be mapped to
  // (0, 0) in unit space.
  S2Point nadir() const {
    return nadir_;
  }


  // Sets the rotation to apply to points before projection.  This has the
  // effect of moving the nadir() point.
  void set_rotation(const Quaternion& q) {
    set_transform(Transform(transform_.scale(), q));
  }

  // Returns the current rotation.
  Quaternion rotation() const { return transform_.rotation(); }


  // Sets the scale factor to apply when projecting points.  This has the effect
  // of 'zooming' around the nadir() point.
  void set_scale(double scale) {
    set_transform(Transform(scale, transform_.rotation()));
  }

  // Returns the current scale factor.
  double scale() const { return transform_.scale(); }


  // Pushes the current transform onto a stack; must pair with PopTransform().
  void PushTransform() {
    transforms_.emplace_back(transform_);
  }


  // Pops a transform off of the stack, restoring the projection state.
  bool PopTransform() {
    if (transforms_.empty()) {
      return false;
    }

    set_transform(transforms_.back());
    transforms_.pop_back();
    return true;
  }


  // An optional transformation to make to the geometry before rotation.
  virtual S2Point BeforeRotate(S2Point point) const = 0;

  // Rotate or unrotate a point using the current rotation().  These are meant
  // to be inverses i.e.
  //
  //       Rotate(Unrotate(point)) == point
  //   and Unrotate(Rotate(point)) == point
  //
  // Though slight numerical error might make the results slightly non-equal.
  S2Point Rotate(S2Point point) const { return transform_.RotateFwd(point); }
  S2Point Unrotate(S2Point point) const { return transform_.RotateInv(point); }


  // Converts a point from world space to unit space.
  virtual R2Point WorldToUnit(S2Point) const = 0;

  // Converts a point from unit space back to world space.  If the inverse
  // projection of the point hits the unit sphere, then that point is returned.
  //
  // Otherwise, if 'nearest' is true, then the point is project onto the closest
  // visible point on the unit sphere.
  virtual bool UnitToWorld( //
    absl::Nonnull<S2Point *> out, R2Point point, bool nearest) const = 0;

  // Converts a point between unit and screen space.
  virtual R2Point UnitToScreen(R2Point) const = 0;
  virtual R2Point ScreenToUnit(R2Point) const = 0;


  // Clears a provided R2Shape and fills it with the outline of the projected
  // unit sphere in screen space.
  //
  // Returns a reference to the passed R2Shape.
  virtual R2Shape& MakeOutline(absl::Nonnull<R2Shape*> out) const = 0;


  // Clears a provided R2Shape and fills it with a graticule showing lines of
  // latitude and longitude.
  //
  // Returns a reference to the passed R2Shape.
  virtual R2Shape& MakeGraticule(absl::Nonnull<R2Shape*> out) const = 0;


  // Projects a point from world space to screen space.
  //
  // Does not distinguish between visible and non-visible points.  Geometry
  // should be Clip()-ed to the projection boundary to ensure only visible
  // points are projected.
  virtual R2Point Project(S2Point) const = 0;


  // Projects an entire S2Shape into screen space, returning a polygon with its
  // edges clipped, stitched and subdivided as needed.
  //
  // Implementations must maintain polygon closure when interiors are clipped
  // across projection boundaries by Stitch()-ing edges that cross the boundary
  // together.
  //
  // max_sq_error is the maximum -squared- error between the true projection of
  // the shape's edges and the edges in screen space, in pixels.
  //
  // The projected geometry is appended to the provided R2Shape.
  //
  // Returns a reference to the passed R2Shape.
  virtual R2Shape& Project(absl::Nonnull<R2Shape *> out,  //
    const S2Shape& shape, double max_sq_error) const = 0;


  // Projects an S2Cap into screen space, returning a polygon with its edges
  // clipped, stitched and subdivided as needed.
  //
  // max_sq_error is the maximum -squared- error between the true projection of
  // the shape's edges and the edges in screen space, in pixels.
  //
  // The projected geometry is appended to the provided R2Shape.
  //
  // Returns a reference to the passed R2Shape.
  virtual R2Shape& Project(absl::Nonnull<R2Shape *> out,  //
    const S2Cap& cap, double max_sq_error) const = 0;


  // Transforms a point from screen space back to world space, if possible.
  //
  // If unprojecting the point onto the unit sphere is possible, then the
  // unprojected point is returned.
  //
  // Otherwise, if 'nearest' is true, stores the nearest visible point on the
  // sphere and returns true.
  //
  // Otherwise, returns false.
  virtual bool Unproject(absl::Nonnull<S2Point *> out,  //
    R2Point point, bool nearest) const = 0;


  // Clips a single S2Point to the visible portion of the sphere.  Returns true
  // if the point is visible and false otherwise.
  virtual bool Clip(S2Point) const = 0;


  // Clips an S2Shape edge to the visible portion of the sphere.  If need be,
  // the edge is broken into multiple disconnected parts and added to the given
  // vector.
  //
  // Returns a reference to the passed EdgeList.
  virtual EdgeList& Clip(absl::Nonnull<EdgeList *> edges,  //
    const S2Shape::Edge& edge) const = 0;


  // Takes an edge and vertex that are presumed to have been clipped via Clip(),
  // converts them into screen space and connects edge.v1 to v0 with a series of
  // lines along the projection boundary.
  //
  // v0 isn't added to the path so that this function may be used to stitch
  // together a chain of edges during projection.
  //
  // The stitched geometry is appended to the provided R2Shape.
  //
  // Returns a reference to the passed R2Shape.
  virtual R2Shape& Stitch(absl::Nonnull<R2Shape *> out,  //
    const S2Shape::Edge& edge, const S2Point& v0) const = 0;


  // Takes an S2Shape::Edge, and subdivides it as needed to achieve a maximum
  // error in post-projection distortion due to edge curvature.
  //
  // The edges is assumed to have been produced by Clip().  The edge is then
  // recursively subdivided Norm2() distance from the actual edge in screen
  // space is less than max_sq_error, storing projected points into the output
  // as it goes.
  //
  // By default the last vertex of the edge is not added to the shape so that
  // this function can be used to tessellate a chain of edges.  This can be
  // overridden by passing add_last = true.
  //
  // max_sq_error is the maximum -squared- error between the true projection of
  // the shape's edges and the edges in screen space, in pixels.
  //
  // The subdivided geometry is appended to the provided R2Shape.
  // Returns a reference to the passed R2Shape.
  virtual R2Shape& Subdivide(absl::Nonnull<R2Shape *> out,
    const S2Shape::Edge&, bool add_last, double max_sq_error) const = 0;


  // Overloads that provide default values for parameters.  We can't use default
  // parameters with virtual methods so we provide these instead.
  bool UnitToWorld(absl::Nonnull<S2Point *> out, R2Point point) const {
    return UnitToWorld(out, point, false);
  }

  R2Shape& Project(absl::Nonnull<R2Shape *> out, const S2Shape &shape) const {
    return Project(out, shape, kDefaultProjectionErrorSq);
  }

  R2Shape& Project(absl::Nonnull<R2Shape *> out, const S2Cap &cap) const {
    return Project(out, cap, kDefaultProjectionErrorSq);
  }

  bool Unproject(absl::Nonnull<S2Point *> out, R2Point point) const {
    return Unproject(out, point, false);
  }

  R2Shape& Subdivide(absl::Nonnull<R2Shape *> out,  //
    const S2Shape::Edge& edge, bool add_last = false) const {
    return Subdivide(out, edge, add_last, kDefaultProjectionErrorSq);
  }

protected:
  // Called on changes to scale, size, and rotation.
  virtual void UpdateTransforms() {}

private:
  // Sets the current transform for the projection.
  void set_transform(Transform transform) {
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
  S2Point nadir_ = S2Point(1,0,0);

  // Current viewport cell union and a mutex so we can generate it lazily.
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

  // Gets and sets the dimensions of screen space.
  void    Resize(int width, int height) final;
  double  width()  const final { return width_;  }
  double  height() const final { return height_; }
  region2 screen() const final {
    return region2(0, 0, width(), height());
  }

  double unit_scale() const final { return unit_scale_; }
  R2Point UnitToScreen(R2Point p) const final { return unit_to_screen_*p; }
  R2Point ScreenToUnit(R2Point p) const final { return screen_to_unit_*p; }


  virtual S2Cap Viewcap() const override;
  virtual S2Point BeforeRotate(S2Point point) const { return point; }
  virtual R2Point Project(S2Point pnt) const override;

  virtual R2Shape& Project(absl::Nonnull<R2Shape *> out,
    const S2Shape& shape, double max_sq_error) const override;

  virtual R2Shape& Project(absl::Nonnull<R2Shape *> out,
    const S2Cap& cap, double max_sq_error) const override;

  virtual bool Unproject(
    absl::Nonnull<S2Point *>, R2Point point, bool nearest) const override;

  virtual R2Shape& Subdivide(absl::Nonnull<R2Shape *> out,  //
    const S2Shape::Edge&, bool add_last, double max_sq_error) const override;

private:
  // Cast back to the Derived class.
  const D& projection() const { return *static_cast<const D*>(this); }
  D& projection() { return *static_cast<D*>(this); }

  // Estimate the max distance of an R2 edge in screen space to a point.
  S1ChordAngle R2EdgeDistance(
    R2Point v0, R2Point v1, S2Point pnt, S1ChordAngle dist) const;

  // Subdivides the given edge, provided projected endpoints are given for it.
  //
  // By passing the projected endpoints as parameters, we're able to eliminate
  // the need to re-project points
  void Subdivide(absl::Nonnull<R2Shape*> out,  //
    S2Shape::Edge s2edge, R2Shape::Edge r2edge, double max_sq_error) const;

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
    const R2Point v0 = screen().GetVertex(side);
    const R2Point v1 = screen().GetVertex(side+1);

    radius = R2EdgeDistance(v0, v1, nadir(), radius);
  }
  return S2Cap(nadir(), radius);
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
inline R2Shape& Projection<Derived>::Subdivide(absl::Nonnull<R2Shape*> out,
  const S2Shape::Edge& edge, bool add_last, double max_sq_error) const {

  R2Point p0 = projection().Project(edge.v0);
  R2Point p1 = projection().Project(edge.v1);

  out->Append(p0);
  if ((p1-p0).Norm2() > max_sq_error) {
    Subdivide(out, edge, {p0, p1}, max_sq_error);
  }

  if (add_last) {
    out->Append(p1);
  }

  return *out;
}


template <typename Derived>
inline void Projection<Derived>::Subdivide(absl::Nonnull<R2Shape*> out,
  S2Shape::Edge s2edge, R2Shape::Edge r2edge, double max_sq_error) const {

  // Compute a point halfway along the edge.
  S2Point v2 = S2::Interpolate(s2edge.v0, s2edge.v1, 0.5);
  R2Point p2 = projection().Project(v2);

  // Compute the distance from the projected point to the line from p0 to p1.
  R2Point vp = p2-r2edge.v0;
  R2Point vn = r2edge.v1-r2edge.v0;
  double dist2 = (vp - (vp.DotProd(vn)*vn)/vn.Norm2()).Norm2();

  // If we're too far from the line, recursively subdivide the first half and
  // the second half.
  if (dist2 > max_sq_error) {
    Subdivide(out, {s2edge.v0, v2}, {r2edge.v0, p2}, max_sq_error);
    out->Append(p2);
    Subdivide(out, {v2, s2edge.v1}, {p2, r2edge.v1}, max_sq_error);
  }
}


template <typename Derived>
inline R2Point Projection<Derived>::Project(S2Point pnt) const {
  return projection().UnitToScreen(
    projection().WorldToUnit(
      projection().Rotate(
        projection().BeforeRotate(pnt))));
}


template <typename Derived>
bool Projection<Derived>::Unproject(absl::Nonnull<S2Point*> out, R2Point pnt, bool nearest) const {
  if (projection().UnitToWorld(out, projection().ScreenToUnit(pnt), nearest)) {
    *out = projection().Unrotate(*out);
    return true;
  }
  return false;
}


template <typename Derived>
R2Shape& Projection<Derived>::Project(absl::Nonnull<R2Shape*> out, const S2Shape& shape, double max_sq_error) const {
  for (int chain_id=0; chain_id < shape.num_chains(); ++chain_id) {
    S2Shape::Chain chain = shape.chain(chain_id);

    // The head of the chain and the last edge processed.
    std::optional<S2Shape::Edge> head;
    std::optional<S2Shape::Edge> last;

    IProjection::EdgeList edges;
    for (int i=0; i < chain.length; ++i) {
      Clip(&edges, shape.chain_edge(chain_id, i));
      for (const S2Shape::Edge &edge : edges) {
        if (!head) {
          head = edge;
        }

        // Edges should be connected if they don't cross the projection
        // boundary.  So we should have a chain of edges something like [(v0,
        // v1), (v1, v2), (v2, v3)] where vertices are shared with the previous
        // and next edges.
        //
        // If the tail of the current chain doesn't match the start of this
        // edge, then we must have left the projection and come back in.
        //
        // Stitch around the boundary between the vertices to close it.
        if (last && last->v1 != edge.v0) {
          Stitch(out, *last, edge.v0);
        }

        Subdivide(out, edge, false, max_sq_error);
        last = edge;
      }
    }

    if (head) {
      // If we didn't land back on the first vertex after going around the
      // chain, it's because we went over the clip edge, stitch a path between
      // them to close the polygon.
      if (last->v1 != head->v0) {
        Stitch(out, *last, head->v0);
      } else {
        out->Append(Project(last->v1));
      }
    }

    out->EndChain();
  }

  return *out;
}


template <typename Derived>
R2Shape& Projection<Derived>::Project(
  absl::Nonnull<R2Shape*> out, const S2Cap& cap, double max_sq_error) const {

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
    if ((Project(pmid)-Project(interp)).Norm2() > max_sq_error) {
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

  // Now project the S2Shape into screen space wholesale.
  return Project(out, S2LaxPolylineShape(points));
}


} // namespace w
