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

#include <algorithm>
#include <cassert>
#include <cmath>
#include <optional>

#include "absl/base/optimization.h"
#include "blend2d.h"
#include "s2/s2cap.h"
#include "s2/s2cell.h"
#include "s2/s2edge_crossings.h"
#include "s2/s2point.h"
#include "s2/s2predicates.h"
#include "s2/s2edge_distances.h"

#include "wilson/plane.h"
#include "wilson/quaternion.h"
#include "wilson/r2shape.h"
#include "wilson/projection/clipping.h"
#include "wilson/projection/projection.h"

namespace w {

// Implementation of logic for an orthographic projection.  This projection just
// projects everything down to the view plane at a 90 degree angle with no
// perspective transformation or coordinate shifting.
struct Orthographic final : Projection<Orthographic> {
  using IProjection::Unproject;
  using Projection::Project;

  Orthographic() {
    UpdateTransforms();
  }

  S2Cap Viewcap() const override {
    // Note we cheat here and return a little less than 90 degrees to avoid
    // drawing the cap -exactly- on the clip horizon.
    S1ChordAngle radius = S1ChordAngle::Degrees(90 - 1e-6);

    S2Point corner;
    if (Unproject(&corner, Screen().GetVertex(0))) {
      radius = S1ChordAngle::Radians(Nadir().Angle(corner));
    }
    return S2Cap(Nadir(), radius);
  }

  // Populates a path representing the outline of the sphere on screen.  May
  // encompass the entire screen.
  void MakeOutline(absl::Nonnull<ChainSink*> out) const override {
    out->Break();

    // Find basis vectors for nadir plane that correspond to the axes of
    // the ellipse of the Earth in the projection.
    const S2Point n = Unrotate({0,0,1});
    const S2Point v = n.CrossProd(Nadir()).Normalize();

    const R2Point np = Project(Nadir());
    const R2Point vp = Project(v);
    AppendCircle(out, np, (vp - np).Norm(), std::sqrt(MaxSqError()));
  }

  // Populates a path with a graticule with lines of latitude and longitude.
  void MakeGraticule(absl::Nonnull<ChainSink*> out) const override {
    //out->Clear();
    //GenerateGraticule(out);
  }

  // Projects a point from world space to unit space.
  R2Point WorldToUnit(const S2Point& p) const override {
    return static_cast<R2Point>(world_to_unit_*p);
  }

  // Attempts to convert a point from unit space back to world space.  If
  // `nearest` is true then the closest point on the projection is returned,
  // otherwise false is returned if the point is out of bounds.
  bool UnitToWorld(  //
    absl::Nonnull<S2Point*> out, const R2Point& proj, bool nearest) const override {
    S2Point pnt = unit_to_world_ * S2Point(proj.x(), proj.y(), 0);

    // Unprojecting a point gives us Y and Z coordinates in 3 space.  Since X is
    // free to vary, this defines a three dimensional line.  If |(y,z)| <= 1
    // then the point is within the unit sphere and we can set the X coordinate
    // to the closest of the two points that the line intersects.
    double mag2 = pnt.Norm2();
    if (mag2 <= 1) {
      pnt.x(std::sqrt(1 - mag2));
      *out = pnt.Normalize();
      return true;
    } else if (nearest) {
      // We missed but nearest is set. Return a unit vector in the X==0 plane.
      pnt.x(0);
      *out = pnt.Normalize();
      return true;
    }
    return false;
  }

  // Projects a point from world space to screen space unconditionally.
  R2Point Project(const S2Point& point) const override {
    return UnitToScreen(WorldToUnit(Rotate(PreRotate(point))));
  }

  // Projects a point into screen space.  Returns true if it's visible.
  bool Project(absl::Nonnull<R2Point*> out, const S2Point& point) const override {
    // Orthographic projections can see half the sphere.  So check that we're on
    // the positive side of the plane defined by the nadir.
    if (s2pred::SignDotProd(Nadir(), point) > 0) {
      *out = Project(point);
      return true;
    }
    return false;
  }

  // Projection functions.
  void Project(absl::Nonnull<ChainSink*> out, const S2Shape::Edge&) const override;
  void Project(absl::Nonnull<ChainSink*> out, const S2Shape&) const override;
  void Project(absl::Nonnull<ChainSink*> out, absl::Nonnull<ChainStitcher*>, const S2Shape&, ContainsPointFn contains) const override;

protected:
  // Updates the transformation matrices.
  void UpdateTransforms() override {
    world_to_unit_ =
      Affine3::Permute(1)                        // Align x and y with screen.
      *Affine3::Orthographic(-1,+1,-1,+1,-1,+1)  // Drop the z coordinate.
      *Affine3::Scale(Scale());                  // Zoom in on sphere.

    // Inverse projection is the reverse sequence of inverted transforms, but
    // without the orthographic matrix.  We can't fully unproject because the
    // orthographic matrix isn't invertible, but we can take it the rest of the
    // way manually in ClipToWorld().
    //
    // Note that the orthographic matrix naturally flip the z coordinate (which
    // aligns us with screen space), so we need to negate it when going back.
    unit_to_world_ =
      Affine3::Scale({1/Scale(), 1/Scale(), -1/Scale()})
      *Affine3::Permute(-1);

    nadir_plane_ = Plane(Nadir());
  }

 private:
  // A crossing where an edge crossed the clip plane.
  struct Crossing {
    Crossing() = default;

    Crossing(const S2Point& point, int vertex, int direction)
        : point(point), vertex(vertex), direction(direction) {}

    static Crossing Incoming(const S2Point& point, int vertex) {
      return Crossing(point, vertex, +1);
    }

    static Crossing Outgoing(const S2Point& point, int vertex) {
      return Crossing(point, vertex, -1);
    }

    bool incoming() const { return direction > 0; }
    bool outgoing() const { return direction < 0; }

    S2Point point;
    int vertex;
    int direction;  // -1 for outgoing and +1 for incoming.
  };

  using CrossingVector = absl::InlinedVector<Crossing, 16>;

  // Subdivides an edge and appends it to the output.
  //
  // Expects that the edge has been properly clipped so that projecting will not
  // wrap in screen space, which will lead to unpredictable results.
  void Subdivide(  //
    ChainSink* out, const S2Shape::Edge& edge) const {

    const R2Point p0 = Project(edge.v0);
    const R2Point p1 = Project(edge.v1);

    if (out->ChainEmpty()) {
      out->Append(p0);
    }

    if ((p1-p0).Norm2() > MaxSqError()) {
      Subdivide(out, edge, {p0, p1});
    }
    out->Append(p1);
  }

  // Helper method that recursively subdivides an edge.
  void Subdivide(absl::Nonnull<ChainSink*> out,
    const S2Shape::Edge& s2edge, const R2Shape::Edge& r2edge) const {

    // Compute a point halfway along the edge.
    S2Point v2 = S2::Interpolate(s2edge.v0, s2edge.v1, 0.5);
    R2Point p2 = Project(v2);

    // Compute the distance from the projected point to the line from p0 to p1.
    R2Point vp = p2 - r2edge.v0;
    R2Point vn = r2edge.v1 - r2edge.v0;
    double dist2 = (vp - (vp.DotProd(vn)*vn)/vn.Norm2()).Norm2();

    // If we're too far from the line, recursively subdivide each half.
    if (dist2 > MaxSqError()) {
      Subdivide(out, {s2edge.v0, v2}, {r2edge.v0, p2});
      out->Append(p2);          //
      Subdivide(out, {v2, s2edge.v1}, {p2, r2edge.v1});
    }
  }

  // Regenerate the graticule path with the current transform.
  void GenerateGraticule(absl::Nonnull<R2Shape*> out) const {
    // // Draw lines of longitude.
    // for (int i=0; i < 36; ++i) {
    //   double lon = -M_PI + (M_PI/180)*10*i;

    //   // Use +/- 80 for non-meridian lines.
    //   double limit = 80;
    //   if (i % 9 == 0) {
    //     limit = 90;
    //   }
    //   GenerateMeridian(out, lon, M_PI/180.0*limit);
    //   out->Break();
    // }
    // GenerateParallels(out);
  }

  // // Generate meridian curves.
  // void GenerateMeridian(absl::Nonnull<R2Shape*> out, double lon, double maxlat) const {
  //   double clon = std::cos(lon);
  //   double clat = std::cos(maxlat);
  //   double slon = std::sin(lon);
  //   double slat = std::sin(maxlat);

  //   // Endpoints of line.  v1 is on the equator to ensure the meridian is under
  //   // 180 degrees in length, even if it stretches from pole to pole.
  //   S2Point v0 = S2Point(clat*clon, clat*slon, +slat);
  //   S2Point v1 = S2Point(clon, slon, 0);
  //   S2Point v2 = S2Point(clat*clon, clat*slon, -slat);

  //   IProjection::EdgeList edges;
  //   for (const auto& edge : Clip(&edges, {v0, v1})) {
  //     Subdivide(out, edge, 0.25);
  //   }

  //   for (const auto& edge : Clip(&edges, {v1, v2})) {
  //     Subdivide(out, edge, 0.25);
  //   }
  // }

  // Takes four points, converts them to unit space, and draws a cubic bezier
  // that goes through all four.  v1 and v2 must be interpolated between the two
  // endpoints at 0.25 and 0.75 of the curve length, respectively.
  // void points_to_bezier(BLPath& path, S2Point v0, S2Point v1, S2Point v2, S2Point v3) const {
  //   R2Point s0 = Project(v0);
  //   R2Point s1 = Project(v1);
  //   R2Point s2 = Project(v2);
  //   R2Point s3 = Project(v3);

  //   R2Point P1 = (-10*s0 + 24*s1 -  8*s2 +  3*s3)*(1/9.0);
  //   R2Point P2 = (  3*s0 -  8*s1 + 24*s2 - 10*s3)*(1/9.0);

  //   path.moveTo(r2b(s0));
  //   path.cubicTo(r2b(P1), r2b(P2), r2b(s3));
  // }

  // static BLPoint r2b(const R2Point& pnt) {
  //   return BLPoint(pnt.x(), pnt.y());
  // }

  // void GenerateParallels(absl::Nonnull<R2Shape*> out) const;


  // Processes an edge and returns a ClipResult to use to process it.
  ClipResult ClipEdge(const S2Shape::Edge& edge) const {
    // Check for antipodal vertices, which should never occur.
    DCHECK(edge.v0 != -edge.v1);

    // Compare signs across the plane defined by the nadir.
    int sign0 = s2pred::SignDotProd(edge.v0, Nadir());
    int sign1 = s2pred::SignDotProd(edge.v1, Nadir());

    // The vertex signs are the same.
    if (ABSL_PREDICT_TRUE(sign0 == sign1)) {
      // The edge is entirely on one side of the nadir plane, so just keep it if
      // it's positive and drop it if it's negative.  This is by far the most
      // common case, so most testing will end here.
      if (sign0 > 0) {
        return ClipResult::Keep();
      }
      return ClipResult::Drop();
    }

    // The vertex signs are different.  If either vertex is exactly on the nadir
    // plane, then mark it as needing snapped on the given vertex.
    if (sign0 == 0) {
      if (sign1 > 0) {
        return ClipResult::Snap0();
      }
      return ClipResult::Drop();
    }

    if (sign1 == 0) {
      if (sign0 > 0) {
        return ClipResult::Snap1();
      }
      return ClipResult::Drop();
    }

    // Signs are different and neither one is zero, so the combination must be
    // (+, -) or (-, +).
    S2Point cross = S2::RobustCrossProd(edge.v0, edge.v1).Normalize();
    S2Point isect = S2::RobustCrossProd(cross, Nadir()).Normalize();
    if (sign1 - sign0 > 0) {
      isect = -isect;
    }

    return ClipResult::Chop(isect, sign1 - sign0);
  }

  // Forward and reverse transformation matrices.
  Affine3 world_to_unit_, unit_to_world_;
  Plane nadir_plane_;
};

// void Orthographic::GenerateParallels(absl::Nonnull<R2Shape*> out) const {
//   // Generate lines of latitude.
//   //
//   // This is more complex because we have to find the limb points for each line
//   // of latitude so we don't draw the backside of the globe.  The code below
//   // finds 0 or 2 intersection points for each circle of constant latitude and
//   // then draws a couple beziers between them.
//   //
//   // The nadir point defines a clipping plane where everything on the far side
//   // is invisible to us, so we clip against that plane to find the limb points.
//   for (double lat=-80; lat < +90; lat += 10) {
//     double beg = 0;
//     double len = 2*M_PI;

//     const double coslat = std::cos(M_PI/180.0*lat);
//     const double sinlat = std::sin(M_PI/180.0*lat);

//     // If the nadir passes through a pole, then the planes are parallel so we
//     // can just clip by latitude.  Anything on the opposite side of 0 in Z is
//     // invisible to us.
//     if (nadir() == S2Point(0,0,1) || nadir() == S2Point(0,0,-1)) {
//       if ((nadir().z() < 0 && lat >= 0) || (nadir().z() > 0 && lat <= 0)) {
//         continue;
//       }
//     } else {
//       // Cross product gives us the direction vector for the intersection of the
//       // nadir plane and the circle plane.  We can just ignore the z-coordinate
//       // to project it into the circle plane.
//       S2Point n = S2Point(0,0,1).CrossProd(nadir()).Normalize();
//       n.z(0);

//       // Solve for a point in the intersection of the two planes at x=0:
//       //
//       // The normal of a plane dotted with any point in the plane is 0.  Let N
//       // be the normal of the circle plane (0,0,1) and O be the offset of the
//       // plane in the Z direction (0, 0, sin(lat)), then:
//       //
//       //   nadir*p = 0 and N*(p-O) = 0  (plane equation)
//       //
//       // We want to solve the system of equations that results, but we have two
//       // equations and three variables, so we can discard one by setting it to
//       // zero.
//       //
//       // Let p.x == 0:
//       //
//       //     nadir*p = nadir.y*p.y + nadir.z*p.z = 0
//       // and N*(p-O) = 0*p.y + p.z-sin(lat)      = 0
//       //
//       //  so
//       //     p.z = sin(lat)
//       //     p.y = (-nadir.z*sin(lat))/nadir.y;
//       //
//       // In the event that the y component of the nadir is zero, we set p.y=0
//       // instead and solve that system.  The only time x and y can both be zero
//       // is when the nadir is on a pole, which we handle above.
//       S2Point p0,p1;
//       if (nadir().y() != 0) {
//         p0 = S2Point(0, -nadir().z()*sinlat/nadir().y(), 0);
//         p1 = p0 + n;
//       } else {
//         p0 = S2Point(-nadir().z()*sinlat/nadir().x(), 0, 0);
//         p1 = p0 + n;
//       }

//       // The radius of a circle of latitude is the cosine of the latitude.
//       const double radius = coslat;

//       // Find the intersection points between the line defined by (p0,p1) and
//       // the circle of constant latitude.
//       //
//       // See: https://mathworld.wolfram.com/Circle-LineIntersection.html
//       double dx = p1.x()-p0.x();
//       double dy = p1.y()-p0.y();
//       double mag2 = (p1-p0).Norm2();
//       double det = p0.x()*p1.y() - p1.x()*p0.y();
//       double disc = radius*radius*mag2 - det*det;

//       // If the discriminant is negative, there are no intersections between the
//       // line and circle, so the whole circle is visible or not visible.
//       //
//       // We can just check whether the circle faces the nadir point or not with
//       // a dot product.
//       //
//       // We check the discriminant here with an epsilon to avoid flickering as
//       // we cross zero.
//       if (disc <= 1e-9) {
//         // Check the point at longitude 0 to see if the circle is visible.
//         S2Point on_circle(coslat, 0, sinlat);
//         if (on_circle.DotProd(nadir()) < 0) {
//           continue;
//         }
//       } else if (disc > 0) {
//         // There are two intersection points, compute them.
//         double x0 = (det*dy - ((dy < 0) ? -1 : 1)*dx*sqrt(disc))/mag2;
//         double y0 = (-det*dx - std::abs(dy)*sqrt(disc))/mag2;
//         double x1 = (det*dy + ((dy < 0) ? -1 : 1)*dx*sqrt(disc))/mag2;
//         double y1 = (-det*dx + std::abs(dy)*sqrt(disc))/mag2;

//         // Embed them back into 3D space.
//         p0 = S2Point(x0, y0, sinlat).Normalize();
//         p1 = S2Point(x1, y1, sinlat).Normalize();

//         // Compute the angle of (x0,y0) to start.  If p0->p1 doesn't sweep
//         // clockwise through the nadir point, then swap them.
//         beg = std::atan2(y0,x0);
//         if (!s2pred::OrderedCCW(p0, nadir(), p1, S2Point(0,0,0))) {
//           beg = std::atan2(y1,x1);
//         }

//         // Compute the angle between the vectors.
//         auto p0_proj = S2Point(p0.x(), p0.y(), 0).Normalize();
//         auto p1_proj = S2Point(p1.x(), p1.y(), 0).Normalize();

//         double prod = std::min(1.0, std::max(-1.0, p0_proj.DotProd(p1_proj)));
//         len = std::acos(prod);

//         // If the intersection points are on the far side of the origin
//         // relative to the nadir point, then we have to draw the long way
//         // around the circle.
//         S2Point nadir_proj = S2Point(nadir().x(), nadir().y(), 0);
//         if (p0_proj.DotProd(nadir_proj) < 0) {
//           len = 2*M_PI-len;
//         }
//       }
//     }

//     // XXX: broken
//     (void)beg;
//     // int npiece = std::ceil(std::abs(len)/(M_PI/3));
//     // for (int i=0; i < npiece; ++i) {
//     //   const double lon0 = beg + len/npiece*(i+0);
//     //   const double lon3 = beg + len/npiece*(i+1);
//     //   const double lon1 = 0.75*lon0 + 0.25*lon3;
//     //   const double lon2 = 0.25*lon0 + 0.75*lon3;

//     //   const double latrad = M_PI/180.0*lat;
//     //   points_to_bezier(path,
//     //     S2LatLng::FromRadians(latrad, lon0).ToPoint(),
//     //     S2LatLng::FromRadians(latrad, lon1).ToPoint(),
//     //     S2LatLng::FromRadians(latrad, lon2).ToPoint(),
//     //     S2LatLng::FromRadians(latrad, lon3).ToPoint()
//     //   );
//     // }
//   }
// }

inline void Orthographic::Project(absl::Nonnull<ChainSink*> out,
  const S2Shape::Edge& edge) const {

  const ClipResult ans = ClipEdge(edge);

  switch (ans.status()) {
    // The edge was dropped, we can just ignore it.
    case ClipResult::kDrop:
      return;

    // The edge was kept as-is, we can just subdivide it normally.
    case ClipResult::kKeep:
      Subdivide(out, edge);
      return;

    // The edge needs to be chopped in half since it wrapped.
    case ClipResult::kChop: {
      if (ans.direction < 0) {  // outgoing
        Subdivide(out, {edge.v0, *ans.isect});
        out->Break();
      }

      if (ans.direction > 0) {  // incoming
        Subdivide(out, {*ans.isect, edge.v1});
      }
      return;
    }

      // One or the other vertex needs to be snapped to a boundary.
    case ClipResult::kSnap: {
      if (ans.boundary.b0) {  // incoming
        out->Break();
        Subdivide(out, edge);
      }

      if (ans.boundary.b1) {  // outgoing
        Subdivide(out, edge);
        out->Break();
      }

      break;
    }

    default:
      ABSL_UNREACHABLE();
  }
  ABSL_UNREACHABLE();
}


inline void Orthographic::Project(  //
  absl::Nonnull<ChainSink*> out, const S2Shape& shape) const {
  DCHECK_LT(shape.dimension(), 2);

  // Points don't need anything fancy, just project them.
  out->Clear();
  if (shape.dimension() == 0) {
    for (int chain = 0; chain < shape.num_chains(); ++chain) {
      for (int i = 0, n = shape.chain(chain).length; i < n; ++i) {
        R2Point proj;
        if (!Project(&proj, shape.chain_edge(chain, i).v0)) {
          continue;
        }
        out->Append(proj);
        out->Break();
      }
    }
    return;
  }

  // Subdivide edges and split chains as needed.
  for (int chain = 0; chain < shape.num_chains(); ++chain) {
    out->Break();

    for (int i = 0, n = shape.chain(chain).length; i < n; ++i) {
      const S2Shape::Edge& edge = shape.chain_edge(chain, i);

      ClipResult ans = ClipEdge(edge);

      switch (ans.status()) {
        // The edge was dropped, ignore it.
        case ClipResult::kDrop:
          break;

        // The edge was kept as-is, just subdivide normally.
        case ClipResult::kKeep:
          Subdivide(out, edge);
          break;

        // We have to chop the edge into two pieces.
        case ClipResult::kChop: {
          if (ans.direction < 0) {  // outgoing
            Subdivide(out, {edge.v0, *ans.isect});
            out->Break();
          }

          if (ans.direction > 0) {  // incoming
            Subdivide(out, {*ans.isect, edge.v1});
          }
          break;
        }

        // One or the other vertex needs to be snapped to a boundary.
        case ClipResult::kSnap: {
          if (ans.boundary.b0) {  // incoming
            out->Break();
            Subdivide(out, edge);
          }

          if (ans.boundary.b1) {  // outgoing
            Subdivide(out, edge);
            out->Break();
          }

          break;
        }

        default:
          ABSL_UNREACHABLE();
      }
    }
  }
}

inline void Orthographic::Project(absl::Nonnull<ChainSink*> out,
  absl::Nonnull<ChainStitcher*> stitcher, const S2Shape& shape, ContainsPointFn contains) const {

  // Adds a chain of vertices to the output.
  const auto AddChain = [&](absl::Span<const R2Point> vertices) {
    out->Append(vertices);
    out->Close();
  };

  // Delegate points and polygons to the other Project().
  if (shape.dimension() != 2) {
    return Project(out, shape);
  }

  CrossingVector crossings;
  stitcher->Clear();

  // Stitch an edge along the projection boundary.  We may have to stitch up to
  // a full 360 degrees, so we test the edge length around the nadir and split
  // the edge before subdividing each piece.
  //
  // If needed, the angle between the vectors is subtracted from 2*PI to stitch
  // from v0 to v1 counter-clockwise around the nadir().  This can be forced by
  // setting `complement` to true.  This is particularly useful if the start and
  // end points are equal and we want to stitch the entire circle.
  const auto Stitch = [&](const S2Shape::Edge& edge, bool complement = false) {
    // Split at slightly less than 180 degrees to avoid numerical issues.
    constexpr double kSplitThreshold = 0.95 * M_PI;

    // Find the oriented angle between the vertices around the nadir() vector.
    double angle = edge.v0.Angle(edge.v1);
    if (complement || s2pred::Sign(Nadir(), edge.v0, edge.v1) < 0) {
      angle = 2*M_PI - angle;
    }

    if (angle >= kSplitThreshold) {
      const S2Point u = edge.v0;
      const S2Point v = S2::RobustCrossProd(Nadir(), u).Normalize();

      angle /= 3;
      const S2Point a = edge.v0;
      const S2Point b = (u*std::cos(1*angle) + v*std::sin(1*angle)).Normalize();
      const S2Point c = (u*std::cos(2*angle) + v*std::sin(2*angle)).Normalize();
      const S2Point d = edge.v1;

      Subdivide(stitcher, {a, b});
      Subdivide(stitcher, {b, c});
      Subdivide(stitcher, {c, d});
    } else {
      Subdivide(stitcher, edge);
    }
  };

  // Subdivide edges and split chains as needed.
  for (int chain = 0; chain < shape.num_chains(); ++chain) {
    stitcher->Break();

    const int start = stitcher->NextVertex();
    for (int i = 0, n = shape.chain(chain).length; i < n; ++i) {
      const S2Shape::Edge& edge = shape.chain_edge(chain, i);

      const ClipResult ans = ClipEdge(edge);

      switch (ans.status()) {
        // The edge was dropped, we can just ignore it.
        case ClipResult::kDrop:
          continue;

        // The edge was kept as-is, we can just subdivide it normally.
        case ClipResult::kKeep:
          Subdivide(stitcher, edge);
          break;

          // The edge crossed the clip plane and needs to be chopped in half.
        case ClipResult::kChop:
          if (ans.direction < 0) {  // outgoing
            Subdivide(stitcher, {edge.v0, *ans.isect});
            stitcher->Break();

            crossings.push_back(  //
              Crossing::Outgoing(*ans.isect, stitcher->LastVertex()));
          }

          if (ans.direction > 0) {  // incoming
            crossings.push_back(  //
              Crossing::Incoming(*ans.isect, stitcher->NextVertex()));

            stitcher->Break();
            Subdivide(stitcher, {*ans.isect, edge.v1});
          }
          break;

        // One or the other vertex needs to be snapped to a boundary.
        case ClipResult::kSnap: {
          if (ans.boundary.b0) {  // incoming
            crossings.push_back(  //
              Crossing::Incoming(edge.v0, stitcher->NextVertex()));

            stitcher->Break();
            Subdivide(stitcher, edge);
          }

          if (ans.boundary.b1) {  // outgoing
            Subdivide(stitcher, edge);
            stitcher->Break();

            crossings.push_back(  //
              Crossing::Outgoing(edge.v1, stitcher->LastVertex()));
          }

          break;
        }

        default:
          ABSL_UNREACHABLE();
      }
    }

    // If the chain closed, remove the repeat point and connect to the start.
    if (stitcher->Size() > start && stitcher->Back() == (*stitcher)[start]) {
      stitcher->PopBack();
      stitcher->Connect(stitcher->LastVertex(), start);
    }
  }

  // Since we use exact predicates, the number of crossings should be even.
  DCHECK(crossings.size() % 2 == 0);

  // Usually there's no crossings.  The polygon must entirely contain or not
  // contain the boundary of the projection.  Test the north pole to break the
  // tie.  If the boundary is contained, emit the outline as a shell first, then
  // we can emit the polygon chains.
  if (ABSL_PREDICT_TRUE(crossings.empty())) {
    if (contains(Nadir().Ortho())) {
      MakeOutline(out);
    }

    if (!stitcher->EmitChains(AddChain)) {
      fprintf(stderr, "[Equirectangular] Saw infinite loop splicing chains!\n");
    }
    return;
  }

  const S2Point& reference_point = Nadir().Ortho();
  bool duplicates = false;
  absl::c_sort(crossings, [&](const Crossing& a, const Crossing& b) {
      // OrderedCCW returns true when any two input points are equal, and we
      // want it to be false because we want a < comparison here, not <=.
      if (a.point == b.point) {
        duplicates = true;
        return false;
      }
      return s2pred::OrderedCCW(reference_point, a.point, b.point, Nadir());
    });

  if (duplicates) {
    // Returns one past the end of the duplicate points starting at index.
    const auto DuplicateEnd = [&](int index) {
      DCHECK_LT(index, crossings.size());
      const S2Point& curr = crossings[index + 0].point;

      int end = index;
      for (; end < crossings.size(); ++end) {
        if (curr != crossings[end].point) {
          break;
        }
      }
      return end;
    };

    // If we have two crossings at -exactly- the same point, we can't order the
    // crossings by just sorting.  We'll walk around the projection from the
    // bottom-right corner (the south pole) counter-clockwise.  The crossings
    // should alternate between incoming and outgoing.

    // This should happen -very- rarely, so we don't have to worry about being
    // efficient here.  When we run into duplicates then we'll just swap
    // elements in the duplicate range to make the order consistent.
    bool outgoing = !contains(reference_point);
    for (int i = 0; i < crossings.size(); ++i) {
      if (crossings[i].outgoing() != outgoing) {
        const int end = DuplicateEnd(i);

        int j = i + 1;
        for (; j < end; ++j) {
          if (crossings[j].outgoing() == outgoing) {
            std::swap(crossings[j], crossings[i]);
            break;
          }
        }

        // This should never happen, but the world is a strange place.
        if (j == end) {
          fprintf(stderr, "[Orthographic] - Couldn't permute crossings!\n");
          return;
        }
      }
      outgoing = !crossings[i].outgoing();
    }
  }

  // Now stitch crossings together.
  for (int ii = 0, n = crossings.size(); ii < n; ++ii) {
    // Skip to an outgoing crossing.
    if (crossings[ii].incoming()) {
      continue;
    }
    const Crossing& outgoing = crossings[ii];
    const Crossing& incoming = crossings[(ii + 1) % n];
    DCHECK(incoming.incoming());

    // Subdivide the stitch since it's a geodesic edge.
    const int start = stitcher->NextVertex();
    if (outgoing.point != incoming.point) {
      stitcher->Break();
      Stitch({outgoing.point, incoming.point});
    }

    // And splice the subdivided edge into the loop.
    if (stitcher->Size() - start == 0) {
      stitcher->Connect(outgoing.vertex, incoming.vertex);
    } else {
      stitcher->Connect(outgoing.vertex, start);
      stitcher->Connect(stitcher->LastVertex(), incoming.vertex);
    }
  }

  if (!stitcher->EmitChains(AddChain)) {
    fprintf(stderr, "[Orthographic] Saw infinite loop splicing chains!\n");
  }
}

} // namespace w
