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
#include <cmath>
#include <optional>
#include <utility>

#include "blend2d.h"
#include "s2/s2cap.h"
#include "s2/s2cell.h"
#include "s2/s2point.h"
#include "s2/s2latlng.h"
#include "s2/s2edge_distances.h"
#include "s2/s2wedge_relations.h"

#include "wilson/chain_stitcher.h"
#include "wilson/plane.h"
#include "wilson/projection.h"
#include "wilson/quaternion.h"
#include "wilson/r2shape.h"

namespace w {

struct Gnomonic final : Projection<Gnomonic> {
  static constexpr double kDefaultViewAngle = 2*M_PI/3;

  // Include project function explicitly to have visibility with our overloads.
  using Projection::Project;

  Gnomonic() {
    SetViewAngle(kDefaultViewAngle);
    UpdateTransforms();
  }

  S2Cap Viewcap() const override {
    return S2Cap(
      nadir(),
      S1ChordAngle::Radians(
        std::min(ViewAngle()/2, 2*std::atan(1/scale()))));
  }

  // Edges are always straight lines in gnomonic projections, so we can simplify
  // the subdivision algorithm by not doing it.
  void Subdivide(absl::Nonnull<ChainSink *> out, const S2Shape::Edge& edge, double) const override {
    if (out->ChainEmpty()) {
      out->Append(Project(edge.v0));
    }
    out->Append(Project(edge.v1));
  }

  // Sets the angle of the cone that's visible in the viewport.  Only the
  // absolute value of the angle is considered, and it must be less than 180
  // degrees.  Angles larger than that are ignored.
  void SetViewAngle(double angle) {
    angle = std::abs(angle);
    if (angle >= M_PI) {
      return;
    }

    view_angle_ = angle;

    // Compute the sin and cosine of the complementary angle PI/2-angle/2, which
    // is just the cosine and sine of angle/2.
    sin_angle_ = std::cos(angle/2);
    cos_angle_ = std::sin(angle/2);

    UpdateTransforms();
  }

  // Return the current view angle.
  double ViewAngle() const {
    return view_angle_;
  }

  // Populates a path representing the outline of the sphere on screen.  May
  // encompass the entire screen.
  R2Shape& MakeOutline(absl::Nonnull<R2Shape *> out) const override {
    out->Clear();

    // The gnomonic projection is always a unit circle, we can just multiply the
    // radius by the scale to get the correct size.
    R2Point origin = UnitToScreen({0,0});
    R2Point radius = UnitToScreen({scale(), 0});

    out->AddCircle(origin, (radius - origin).x());
    return *out;
  }

  // Populates a path with a graticule with lines of latitude and longitude.
  R2Shape& MakeGraticule(absl::Nonnull<R2Shape *> out) const override {
    out->Clear();
    // generate_graticule(path);
    return *out;
  }

  // Clips a single S2Point to the visible portion of the sphere.
  // Returns true if the point is visible and false otherwise.
  bool Clip(S2Point point) const override {
    return clip_plane_.Sign(point) > 0;
  }

  EdgeList& Clip(absl::Nonnull<EdgeList*> edges, const S2Shape::Edge& edge) const override {
    // The visible area of a gnomonic projection is < half a hemisphere, so its
    // not possible for an edge to leave one side and re-enter on the other.
    // Thus we never have to split edges into multiple parts, so we can just
    // clip the edge once if needed and return it if it's not occluded.
    edges->clear();
    S2Shape::Edge copy = edge;
    if (clip_plane_.ClipEdgeOnSphere(copy)) {
      edges->emplace_back(copy);
    }
    return *edges;
  };

  void Stitch(absl::Nonnull<R2Shape *> out, const S2Shape::Edge& edge, const S2Point& v0) const override {
    // The visible region is always < a hemisphere in gnomonic projection.  The
    // visible circle is a unit circle so we can stitch two vertices with edges
    // along that circle.

    // Overdraw the projection outline by ~10% to avoid artifacts from
    // linearizing the arc along the edge.
    constexpr double kOverdraw = 0.10;

    R2Point cc = Project(nadir()); // Center of the circle.
    R2Point p0 = Project(edge.v1)-cc;
    R2Point p1 = Project(v0)-cc;

    // P0 and P1 are on the circle so the radius is just their magnitude.
    // Expand the radius by the overdraw to give ourselves some wiggle room.
    double rr = p0.Norm();

    // The signed angle between p0 and p1.
    double sweep = std::atan2(p0.CrossProd(p1), p0.DotProd(p1));

    // The sagitta of a chord subtending an angle theta is
    //    h = rr*(1-cos(θ/2)), so we can solve for theta given a desired
    // saggita:
    //    θ = 2*std::acos(1-h/rr)
    //
    // We'll require the height be half the overdraw.
    double h = kOverdraw/2*rr;
    rr *= (1+kOverdraw);

    // We can just divide this into the sweep to solve for the number of steps.
    int nsteps = std::ceil(std::fabs(sweep)/(2*std::acos(1-h/rr)));
    nsteps += 2; // Two steps for the endpoints.

    double beg = std::atan2(p0.y(), p0.x());
    double step = sweep/(nsteps-1);

    for (int i=0; i < nsteps; ++i) {
      R2Point pnt(std::cos(beg + i*step), std::sin(beg + i*step));
      out->Append(cc + rr*pnt);
    }
  }

  R2Point WorldToUnit(S2Point p) const override {
    return (scale()*sin_angle_/cos_angle_)*R2Point(p.y()/p.x(), -p.z()/p.x());
  }

  bool UnitToWorld(absl::Nonnull<S2Point*> out, R2Point proj, bool nearest=false) const override {
    R2Point pnt = (cos_angle_/(sin_angle_*scale()))*proj;

    double rr = pnt.Norm2();
    double cot = cos_angle_/sin_angle_;
    if (rr <= cot*cot) {
      *out = S2Point(1, pnt.x(), -pnt.y()).Normalize();
      return true;
    }

    // Missed the sphere, if nearest is true find a vector of the appropriate
    // radius in the clip plane as the closest point on the sphere.
    if (nearest) {
      double radius = cos_angle_;
      pnt = pnt.Normalize()*radius;
      *out = S2Point(sin_angle_, pnt.x(), -pnt.y()).Normalize();
      return true;
    }

    return false;
  }

  R2Shape& Project(absl::Nonnull<R2Shape *> out, absl::Nonnull<ChainStitcher*>, const S2Shape&, double max_sq_error) const override;

protected:
  void UpdateTransforms() override {
    // Recompute the origin of the clip plane from the new view angle.
    //
    // The clip plane is offset by the sin of the complementary angle towards
    // the nadir, so just scale the nadir vector to get the origin.
    clip_plane_ = Plane(nadir(), nadir()*sin_angle_);
  }

 private:
  // A crossing where an edge crossed the clip plane.
  struct Crossing {
    Crossing() = default;

    static Crossing Incoming(const S2Point& point, int vertex) {
      Crossing crossing;
      crossing.point = point;
      crossing.vertex = vertex;
      crossing.incoming = true;
      return crossing;
    }

    static Crossing Outgoing(const S2Point& point, int vertex) {
      Crossing crossing = Incoming(point, vertex);
      crossing.incoming = false;
      return crossing;
    }

    S2Point point;
    int vertex;
    bool incoming;
  };

  using CrossingVector = absl::InlinedVector<Crossing, 16>;

 // Regenerate the graticule path with the current transform.
  void generate_graticule(BLPath& path) const {
    // Draw lines of longitude.
    for (int i=0; i < 36; ++i) {
      double lon = -M_PI + (M_PI/180)*10*i;

      // Use +/- 80 for non-meridian lines.
      double limit = 80;
      if (i % 9 == 0) {
        limit = 90;
      }
      generate_meridian(path, lon, M_PI/180.0*limit);
    }

    generate_parallels(path);
  }

  // Generate a meridian as piece wise cubic beziers.
  void generate_meridian(BLPath& path, double lon, double maxlat) const {
    double clon = std::cos(lon);
    double clat = std::cos(maxlat);
    double slon = std::sin(lon);
    double slat = std::sin(maxlat);

    // Endpoints of line.
    S2Point v0 = S2Point(clat*clon, clat*slon, +slat);
    S2Point v1 = S2Point(clat*clon, clat*slon, -slat);

    // Break the lines of longitude into equal parts.
    constexpr int kNpiece = 4;
    S2Point points[kNpiece-1];
    for (int j=0; j < kNpiece-1; ++j) {
      double lat = maxlat - 2*maxlat/kNpiece*(+1);
      double clat = std::cos(lat);
      double slat = std::sin(lat);

      points[j] = S2Point(clat*clon, clat*slon, slat);
    }

    // Create curves.
    edge_to_path(path, {v0, points[0]});
    for (int j=0; j < kNpiece-2; ++j) {
      edge_to_path(path, {points[j], points[j+1]});
    }
    edge_to_path(path, {points[kNpiece-2], v1});
  }

  void edge_to_path(BLPath& path, S2Shape::Edge edge) const {
    absl::InlinedVector<S2Shape::Edge, 1> edges;
    if (!clip_plane_.ClipEdgeOnSphere(edge)) {
      return;
    }

    R2Point p0 = Project(edge.v0);
    path.moveTo(p0.x(), p0.y());

    R2Point p1 = Project(edge.v1);
    path.lineTo(p1.x(), p1.y());
  }

  // Takes four points, converts them to unit space, and draws a cubic bezier
  // that goes through all four.  v1 and v2 must be interpolated between the two
  // endpoints at 0.25 and 0.75 of the curve length, respectively.
  void points_to_bezier(BLPath& path, S2Point v0, S2Point v1, S2Point v2, S2Point v3) const {
    R2Point s0 = Project(v0);
    R2Point s1 = Project(v1);
    R2Point s2 = Project(v2);
    R2Point s3 = Project(v3);

    R2Point P1 = (-10*s0 + 24*s1 -  8*s2 +  3*s3)*(1/9.0);
    R2Point P2 = (  3*s0 -  8*s1 + 24*s2 - 10*s3)*(1/9.0);

    path.moveTo(r2b(s0));
    path.cubicTo(r2b(P1), r2b(P2), r2b(s3));
  }

  static BLPoint r2b(const R2Point& pnt) {
    return BLPoint(pnt.x(), pnt.y());
  }

  void generate_parallels(BLPath& path) const;

  // Subdivide an edge.
  void subdivide(
    std::vector<R2Point>& points,
    S2Point v0, S2Point v1,
    R2Point s0, R2Point s1) const;

  // We have to limit the field of view with gnomonic projection to avoid
  // distortion becoming too large.  We can achieve this by clipping geometry
  // against a plane, whose normal is the current nadir() point, and which is
  // offset towards the viewer by sin((π-θ)/2) in the direction of the nadir:
  //
  //  Visible
  //  ├─────┤
  //   nadir
  // ----•----      ┬
  //   ╲   ╱        │ sin((π-θ)/2)
  //    ╲θ╱ (π-θ)/2 │
  // ---------      ┴
  double view_angle_;
  double sin_angle_;
  double cos_angle_;

  Plane clip_plane_;
};


void Gnomonic::generate_parallels(BLPath& path) const {
  // Generate lines of latitude.
  //
  // This is more complex because we have to find the limb points for each line
  // of latitude so we don't draw the backside of the globe.  The code below
  // finds 0 or 2 intersection points for each circle of constant latitude and
  // then draws a couple beziers between them.
  //
  // The nadir point defines a clipping plane where everything on the far side
  // is invisible to us, so we clip against that plane to find the limb points.
  for (double lat=-80; lat < +90; lat += 10) {
    double beg = 0;
    double len = 2*M_PI;

    const double coslat = std::cos(M_PI/180.0*lat);
    const double sinlat = std::sin(M_PI/180.0*lat);

    // If the nadir passes through a pole, then the planes are parallel so we
    // can just clip by latitude.  Anything on the opposite side of 0 in Z is
    // invisible to us.
    if (nadir() == S2Point(0,0,1) || nadir() == S2Point(0,0,-1)) {
      if ((nadir().z() < 0 && lat >= 0) || (nadir().z() > 0 && lat <= 0)) {
        continue;
      }
    } else {
      // Cross product gives us the direction vector for the intersection of the
      // nadir plane and the circle plane.  We can just ignore the z-coordinate
      // to project it into the circle plane.
      S2Point n = S2Point(0,0,1).CrossProd(nadir()).Normalize();
      n.z(0);

      // Solve for a point in the intersection of the two planes at x=0:
      //
      // The normal of a plane dotted with any point in the plane is 0.  Let N
      // be the normal of the circle plane (0,0,1) and O be the offset of the
      // plane in the Z direction (0, 0, sin(lat)), then:
      //
      //   nadir*p = 0 and N*(p-O) = 0  (plane equation)
      //
      // We want to solve the system of equations that results, but we have two
      // equations and three variables, so we can discard one by setting it to
      // zero.
      //
      // Let p.x == 0:
      //
      //     nadir*p = nadir.y*p.y + nadir.z*p.z = 0
      // and N*(p-O) = 0*p.y + p.z-sin(lat)      = 0
      //
      //  so
      //     p.z = sin(lat)
      //     p.y = (-nadir.z*sin(lat))/nadir.y;
      //
      // In the event that the y component of the nadir is zero, we set p.y=0
      // instead and solve that system.  The only time x and y can both be zero
      // is when the nadir is on a pole, which we handle above.
      S2Point p0,p1;
      if (nadir().y() != 0) {
        p0 = S2Point(0, -nadir().z()*sinlat/nadir().y(), 0);
        p1 = p0 + n;
      } else {
        p0 = S2Point(-nadir().z()*sinlat/nadir().x(), 0, 0);
        p1 = p0 + n;
      }

      // The radius of a circle of latitude is the cosine of the latitude.
      const double radius = coslat;

      // Find the intersection points between the line defined by (p0,p1) and
      // the circle of constant latitude.
      //
      // See: https://mathworld.wolfram.com/Circle-LineIntersection.html
      double dx = p1.x()-p0.x();
      double dy = p1.y()-p0.y();
      double mag2 = (p1-p0).Norm2();
      double det = p0.x()*p1.y() - p1.x()*p0.y();
      double disc = radius*radius*mag2 - det*det;

      // If the discriminant is negative, there are no intersections between the
      // line and circle, so the whole circle is visible or not visible.
      //
      // We can just check whether the circle faces the nadir point or not with
      // a dot product.
      //
      // We check the discriminant here with an epsilon to avoid flickering as
      // we cross zero.
      if (disc <= 1e-9) {
        // Check the point at longitude 0 to see if the circle is visible.
        S2Point on_circle(coslat, 0, sinlat);
        if (on_circle.DotProd(nadir()) < 0) {
          continue;
        }
      } else if (disc > 0) {
        // There are two intersection points, compute them.
        double x0 = (det*dy - ((dy < 0) ? -1 : 1)*dx*sqrt(disc))/mag2;
        double y0 = (-det*dx - std::abs(dy)*sqrt(disc))/mag2;
        double x1 = (det*dy + ((dy < 0) ? -1 : 1)*dx*sqrt(disc))/mag2;
        double y1 = (-det*dx + std::abs(dy)*sqrt(disc))/mag2;

        // Embed them back into 3D space.
        p0 = S2Point(x0, y0, sinlat).Normalize();
        p1 = S2Point(x1, y1, sinlat).Normalize();

        // Compute the angle of (x0,y0) to start.  If p0->p1 doesn't sweep
        // clockwise through the nadir point, then swap them.
        beg = std::atan2(y0,x0);
        if (!s2pred::OrderedCCW(p0, nadir(), p1, S2Point(0,0,0))) {
          beg = std::atan2(y1,x1);
        }

        // Compute the angle between the vectors.
        auto p0_proj = S2Point(p0.x(), p0.y(), 0).Normalize();
        auto p1_proj = S2Point(p1.x(), p1.y(), 0).Normalize();

        double prod = std::min(1.0, std::max(-1.0, p0_proj.DotProd(p1_proj)));
        len = std::acos(prod);

        // If the intersection points are on the far side of the origin
        // relative to the nadir point, then we have to draw the long way
        // around the circle.
        S2Point nadir_proj = S2Point(nadir().x(), nadir().y(), 0);
        if (p0_proj.DotProd(nadir_proj) < 0) {
          len = 2*M_PI-len;
        }
      }
    }

    int npiece = std::ceil(std::abs(len)/(M_PI/3));
    for (int i=0; i < npiece; ++i) {
      const double lon0 = beg + len/npiece*(i+0);
      const double lon3 = beg + len/npiece*(i+1);
      const double lon1 = 0.75*lon0 + 0.25*lon3;
      const double lon2 = 0.25*lon0 + 0.75*lon3;

      const double latrad = M_PI/180.0*lat;
      points_to_bezier(path,
        S2LatLng::FromRadians(latrad, lon0).ToPoint(),
        S2LatLng::FromRadians(latrad, lon1).ToPoint(),
        S2LatLng::FromRadians(latrad, lon2).ToPoint(),
        S2LatLng::FromRadians(latrad, lon3).ToPoint()
      );
    }
  }
}

void Gnomonic::subdivide(
  std::vector<R2Point>& points, S2Point v0, S2Point v1, R2Point s0, R2Point s1) const {

  // Compute a point halfway along the edge.
  S2Point v2 = S2::Interpolate(v0, v1, 0.5);
  R2Point p0 = Project(v2);

  // Compute the projection distance onto the line segment in screen space.
  R2Point vp = p0-s0;
  R2Point vn = s1-s0;
  double dist2 = (vp - (vp.DotProd(vn)*vn/vn.Norm2())).Norm2();

  if (dist2 > 1) {
    subdivide(points, v0, v2, s0, p0);
    points.emplace_back(p0);
    subdivide(points, v2, v1, p0, s1);
  }
}

// void Gnomonic::Project(BLPath& path, const S2Shape& shape) const {
//   // Gnomonic projection has the advantage that less than half of the sphere is
//   // visible at any given time, so for an edge to leave across one boundary, and
//   // appear on the other, it would have to be > 180 degrees, which isn't
//   // allowed.  Thus we don't have to worry about splitting edges into multiple
//   // parts.
//   path.clear();

//   for (int i=0; i < shape.num_chains(); ++i) {
//     S2Point first;
//     S2Point last;
//     bool have_last = false;

//     bool move = true;
//     for (int j=0; j < shape.chain(i).length; ++j) {
//       S2Shape::Edge edge = shape.chain_edge(i, j);
//       if (!clip_plane_.ClipEdgeOnSphere(edge)) {
//         continue;
//       }

//       if (!have_last) {
//         have_last = true;
//         first = edge.v0;
//         last = edge.v0;
//       }

//       R2Point p0 = Project(last);
//       if (move) {
//         move = false;
//         path.moveTo(p0.x(), p0.y());
//       } else {
//         path.lineTo(p0.x(), p0.y());
//       }

//       if (last != edge.v0) {
//         //subdivide(r2points, last, v0, Project(last), Project(v0));
//         Stitch(path, last, edge.v0);

//         // R2Point c0 = ScreenToUnit(Project(last));
//         // R2Point c1 = ScreenToUnit(Project(edge.v0));

//         // R2Point dir = (c1-c0);
//         // for (int k=0; k < 100; ++k) {
//         //   R2Point p = UnitToScreen(scale()*(dir*k/100.0 + c0).Normalize());
//         //   path.lineTo(p.x(), p.y());
//         // }
//       }
//       //subdivide(r2points, v0, v1, Project(v0), Project(v1));
//       last = edge.v1;
//     }

//     if (have_last) {
//       R2Point p0 = Project(last);
//       path.lineTo(p0.x(), p0.y());
//       //r2points.emplace_back(Project(last));

//       if (last != first) {
//         Stitch(path, last, first);

//         // R2Point c0 = ScreenToUnit(Project(last));
//         // R2Point c1 = ScreenToUnit(Project(first));

//         // R2Point dir = (c1-c0);
//         // for (int k=0; k < 100; ++k) {
//         //   R2Point p = UnitToScreen(scale()*(dir*k/100.0 + c0).Normalize());
//         //   path.lineTo(p.x(), p.y());
//         // }
//       }

//       // Subdivide and add points between the last vertex and the first to close
//       // the chain with interpolated points.
//       //subdivide(r2points, last, first, Project(last), Project(first));
//       //simplify(r2points, 1);

//       //r2shape.add_chain(r2points);
//     }
//   }
// }


inline R2Shape& Gnomonic::Project(absl::Nonnull<R2Shape *> out,
  absl::Nonnull<ChainStitcher*> stitcher, const S2Shape& shape, double max_sq_error) const {

  CrossingVector crossings;
  stitcher->Clear();

  // If an edge is too long (close to 180 degrees), it's possible that we'll
  // flip and close it the wrong way around the sphere.  Check if the edge is
  // too long and sub-sample it before subdivision.
  const auto SampleAndSubdivide = [&](const S2Shape::Edge& edge) {
    const double angle = edge.v0.Angle(edge.v1);
    if (angle >= 0.95*M_PI) {
      const S2Point u = edge.v0;
      const S2Point v = nadir().CrossProd(u);

      const S2Point a = edge.v0;
      const S2Point b = u*std::cos(angle/2) + v*std::sin(angle/2);
      const S2Point c = edge.v1;

      Subdivide(stitcher, {a, b}, max_sq_error);
      Subdivide(stitcher, {b, c}, max_sq_error);
    } else {
      Subdivide(stitcher, {edge.v0, edge.v1}, max_sq_error);
    }
  };

  // Subdivide edges and split chains as needed.
  for (int chain = 0; chain < shape.num_chains(); ++chain) {
    stitcher->Break();

    int start = stitcher->size();
    for (int i = 0, n = shape.chain(chain).length; i < n; ++i) {
      const S2Shape::Edge& edge = shape.chain_edge(chain, i);
      S2Shape::Edge copy = edge;
      if (!clip_plane_.ClipEdgeOnSphere(copy)) {
        continue;
      }

      // Next vertex is an incoming crossing.
      if (copy.v0 != edge.v0) {
        crossings.push_back(Crossing::Incoming(copy.v0, stitcher->size()));
      }

      Subdivide(stitcher, copy, max_sq_error);

      // Last vertex is an outgoing crossing.
      if (copy.v1 != edge.v1) {
        crossings.push_back(Crossing::Outgoing(copy.v1, stitcher->size() - 1));
        stitcher->Break();
      }
    }

    // Ensure that polygon chains are closed properly if need be.
    if (shape.dimension() == 2) {
      if (stitcher->size() > start && (*stitcher)[start] == stitcher->back()) {
        stitcher->pop_back();
        stitcher->Connect(stitcher->size()-1, start);
      }
    }
  }

  if (!crossings.empty()) {
    const S2Point first = crossings[0].point;

    // Sort crossings CCW around the projection boundary.
    absl::c_sort(crossings, [&](const Crossing& a, const Crossing& b) {
      return !s2pred::OrderedCCW(b.point, a.point, first, nadir());
    });

    // Now stitch crossings together.
    const int ncrossings = crossings.size();
    for (int ii = 0; ii < ncrossings; ++ii) {
      DCHECK_NE(crossings[ii].vertex, -1);

      // Skip to an outgoing crossing.
      const Crossing& outgoing = crossings[ii];
      if (outgoing.incoming) {
        continue;
      }

      // Find the next incoming crossing.
      int jj = (ii + 1) % ncrossings;
      for (; jj != ii; jj = (jj + 1) % ncrossings) {
        if (crossings[jj].incoming) {
          break;
        }
      }
      DCHECK_NE(ii, jj);
      const Crossing& incoming = crossings[jj];

      // Subdivide the stitch since it's a geodesic edge.
      const int beg = stitcher->size();
      stitcher->Break();
      SampleAndSubdivide({outgoing.point, incoming.point});
      const int end = stitcher->size();

      // And splice the subdivided edge into the loop.
      stitcher->Connect(outgoing.vertex, beg);
      stitcher->Connect(end - 1, incoming.vertex);
    }
  }

  const auto AddChain = [&](absl::Span<const R2Point> vertices) {
    out->AddChain(vertices, shape.dimension() == 2);
  };

  if (!stitcher->EmitChains(AddChain)) {
    fprintf(stderr, "Detected infinite loop splicing chains\n");
  }

  return *out;
}


} // namespace w
