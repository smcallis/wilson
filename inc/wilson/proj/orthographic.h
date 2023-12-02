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

#include "blend2d.h"
#include "s2/s2cap.h"
#include "s2/s2cell.h"
#include "s2/s2point.h"
#include "s2/s2edge_distances.h"

#include "wilson/plane.h"
#include "wilson/projection.h"
#include "wilson/quaternion.h"
#include "wilson/r2shape.h"

namespace w {

// Implementation of logic for an orthographic projection.  This projection just
// projects everything down to the view plane at a 90 degree angle with no
// perspective transformation or coordinate shifting.
struct Orthographic final : ProjectionBase<Orthographic> {
  // Include project function explicitly to have visibility with our overloads.
  using ProjectionBase::Project;
  using Projection::Subdivide;

  Orthographic() {
    UpdateTransforms();
  }

  S2Cap Viewcap() const override {
    S1ChordAngle radius = S1ChordAngle::Right();

    S2Point corner;
    if (Unproject(corner, screen().GetVertex(0))) {
      radius = S1ChordAngle::Radians(nadir().Angle(corner));
    }
    return S2Cap(nadir(), radius);
  }

  // Populates a path representing the outline of the sphere on screen.  May
  // encompass the entire screen.
  R2Shape Outline(R2Shape shape={}) const override {
    shape.clear();

    // Find basis vectors for nadir plane that correspond to the axes of
    // the ellipse of the Earth in the projection.
    S2Point n = Unrotate(S2Point(0,0,1));
    S2Point v = n.CrossProd(nadir()).Normalize();
    //S2Point u = nadir().CrossProd(v).Normalize();

    R2Point np = Project(nadir());
    //R2Point up = Project(u);
    R2Point vp = Project(v);

    shape.AddCircle(np, (vp-np).Norm());
    return shape;
    //shape.addEllipse(BLEllipse(np.x(), np.y(), (vp-np).Norm(), (up-np).Norm()));
  }

  // Populates a path with a graticule with lines of latitude and longitude.
  R2Shape MakeGraticule(R2Shape shape={}) const override {
    shape.clear();
    GenerateGraticule(shape);
    return shape;
  }

  EdgeList Clip(S2Shape::Edge edge, EdgeList edges={}) const override {
    edges.clear();
    if (Plane(nadir()).ClipEdgeOnSphere(edge)) {
      edges.emplace_back(edge);
    }
    return edges;
  }

  void Stitch(R2Shape& out, const S2Point& v1, const S2Point& v0) const override {
    // Orthographic projection is always a circle on the screen and the clip
    // bisects the sphere (making it a great circle).  So we can just subdivide
    // an edge normally to connect them.
    Subdivide(out, {v1, v0});
  }

  R2Point WorldToUnit(const S2Point& p) const override {
    S2Point proj = world_to_unit_*p;
    return {proj.x(), proj.y()};
  }

  bool UnitToWorld(S2Point& out, const R2Point& proj, bool nearest=false) const override {
    S2Point pnt = unit_to_world_*S2Point(proj.x(), proj.y(), 0);

    // Unprojecting the point gives us the y/z coordinates of a line in 3 space.
    // If |(y,z)| <= 1 then the point is within the unit sphere and we can set
    // the X coordinate to the closest of the two points that the line
    // intersects.
    double mag2 = pnt.Norm2();
    if (mag2 <= 1) {
      pnt.x(sqrt(1-mag2));
      out = pnt.Normalize();
      return true;
    }

    // Otherwise we missed the sphere.  Return a unit vector in the X==0 plane.
    if (nearest) {
      pnt.x(0);
      out = pnt.Normalize();
      return true;
    }

    return false;
  }

protected:
  // Updates the transformation matrices.
  void UpdateTransforms() override {
    world_to_unit_ =
      Affine3::Permute(1)                        // Align x and y with screen.
      *Affine3::Orthographic(-1,+1,-1,+1,-1,+1)  // Drop the z coordinate.
      *Affine3::Scale(scale());                  // Zoom in on sphere.

    // Inverse projection is the reverse sequence of inverted transforms, but
    // without the orthographic matrix.  We can't fully unproject because the
    // orthographic matrix isn't invertible, but we can take it the rest of the
    // way manually in ClipToWorld().
    //
    // Note that the orthographic matrix naturally flip the z coordinate (which
    // aligns us with screen space), so we need to negate it when going back.
    unit_to_world_ =
      Affine3::Scale({1/scale(), 1/scale(), -1/scale()})
      *Affine3::Permute(-1);
  }

private:
  // Regenerate the graticule path with the current transform.
  void GenerateGraticule(R2Shape& shape) const {
    // Draw lines of longitude.
    for (int i=0; i < 36; ++i) {
      double lon = -M_PI + (M_PI/180)*10*i;

      // Use +/- 80 for non-meridian lines.
      double limit = 80;
      if (i % 9 == 0) {
        limit = 90;
      }
      GenerateMeridian(shape, lon, M_PI/180.0*limit);
      shape.EndChain();
    }
    GenerateParallels(shape);
  }

  // Generate meridian curves.
  void GenerateMeridian(R2Shape& shape, double lon, double maxlat) const {
    double clon = std::cos(lon);
    double clat = std::cos(maxlat);
    double slon = std::sin(lon);
    double slat = std::sin(maxlat);

    // Endpoints of line.  v1 is on the equator to ensure the meridian is under
    // 180 degrees in length, even if it stretches from pole to pole.
    S2Point v0 = S2Point(clat*clon, clat*slon, +slat);
    S2Point v1 = S2Point(clon, slon, 0);
    S2Point v2 = S2Point(clat*clon, clat*slon, -slat);

    for (const auto& edge : Clip({v0, v1})) {
      Subdivide(shape, edge, 0.25, true);
    }

    for (const auto& edge : Clip({v1, v2})) {
      Subdivide(shape, edge, 0.25, true);
    }
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

  void GenerateParallels(R2Shape& shape) const;

  // Forward and reverse transformation matrices.
  Affine3 world_to_unit_, unit_to_world_;
};

void Orthographic::GenerateParallels(R2Shape& shape) const {
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

    // XXX: broken
    (void)beg;
    // int npiece = std::ceil(std::abs(len)/(M_PI/3));
    // for (int i=0; i < npiece; ++i) {
    //   const double lon0 = beg + len/npiece*(i+0);
    //   const double lon3 = beg + len/npiece*(i+1);
    //   const double lon1 = 0.75*lon0 + 0.25*lon3;
    //   const double lon2 = 0.25*lon0 + 0.75*lon3;

    //   const double latrad = M_PI/180.0*lat;
    //   points_to_bezier(path,
    //     S2LatLng::FromRadians(latrad, lon0).ToPoint(),
    //     S2LatLng::FromRadians(latrad, lon1).ToPoint(),
    //     S2LatLng::FromRadians(latrad, lon2).ToPoint(),
    //     S2LatLng::FromRadians(latrad, lon3).ToPoint()
    //   );
    // }
  }
}

} // namespace w
