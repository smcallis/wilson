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

#include "blend2d.h"
#include "s2/s2cell.h"

#include "wilson/projection.h"
#include "wilson/plane.h"

namespace w {

// Populates an R2Shape with an S2 graticule where the grid lines are the
// boundaries of S2Cells up to a certain level.  E.g. S2Graticule(0) will only
// return the outlines of the faces, S2Graticule(1) includes level 1 cell
// boundaries.
//
// Any existing path information is cleared.
inline void S2Graticule(absl::Nonnull<R2Shape *> out, const IProjection &proj,
                        int level) {
  // Clip edges to the spherical cap bounding the viewport first thing.  This
  // well prevent us from trying to subdivide parts of edges that aren't even
  // visible.
  S2Cap cap = proj.Viewcap();
  Plane clip {cap.center(), (1-cap.height())*cap.center()};
  bool clip_cap = cap.radius() < S1ChordAngle::Right();

  IProjection::EdgeList edges;

  // Projects and subdivides an S2Shape edge and appends it to the path.
  const auto AppendEdgeToShape = [&](S2Shape::Edge edge) {
    if (!clip_cap || clip.ClipEdgeOnSphere(edge)) {
      proj.Clip(&edges, edge);
      for (const S2Shape::Edge& edge : edges) {
        proj.Subdivide(out, edge, true, 0.25);
        out->EndChain();
      }
    }
  };

  // A recursive lambda to add the boundaries of all the children of a cell that
  // are at the target level.
  const auto AddCellBoundaries = [&](const S2Cell& cell, auto&& self) -> void {
    if (cell.level() < (level-1)) {
      S2Cell children[4];
      cell.Subdivide(children);
      for (const S2Cell& child : children) {
        self(child, self);
      }
      return;
    }

    // We don't know what order our children are in because they're in
    // traversal order.  But, we do know that each one will share _one_ vertex
    // with us, so we can scan the vertices to determine which of 4 positions
    // the child is in.
    //
    // We'll grab vertices from the children lying on each of the four sides
    // of this cell, making up a cross shape that divides the cell.
    //
    // We could add cell edges directly rather than using their children to
    // bisect the cell, but doing this ensures we don't end up with duplicate
    // edges (e.g. from neighboring children), which don't look very good when
    // we go to render them.
    S2Cell children[4];
    cell.Subdivide(children);

    S2Point vertices[4];
    for (int i=0; i < 4; ++i) {
      const S2Cell& child = children[i];
      for (int j=0; j < 4; ++j) {
        if (child.GetVertex(j) == cell.GetVertex(j)) {
          vertices[j] = child.GetVertex(j+1);
        }
      }
    }

    AppendEdgeToShape({vertices[0], vertices[2]});
    AppendEdgeToShape({vertices[1], vertices[3]});
  };

  out->clear();
  if (level <= 0) {
    // Add bottom/left edges of faces 0 and 1
    for (int face=0; face <= 1; ++face) {
      S2Cell cell = S2Cell::FromFace(face);
      AppendEdgeToShape({cell.GetVertex(3), cell.GetVertex(4)});
      AppendEdgeToShape({cell.GetVertex(0), cell.GetVertex(1)});
    }

    // Add bottom/right edges of faces 3 and 4
    for (int face=3; face <= 4; ++face) { //
      S2Cell cell = S2Cell::FromFace(face);
      AppendEdgeToShape({cell.GetVertex(0), cell.GetVertex(1)});
      AppendEdgeToShape({cell.GetVertex(1), cell.GetVertex(2)});
    }

    // Add all edges of face 2 to complete the grid.
    S2Cell cell = S2Cell::FromFace(2);
    for (int k=0; k < 4; ++k) {
      AppendEdgeToShape({cell.GetVertex(k), cell.GetVertex(k+1)});
    }
  } else {
    for (int face=0; face < 6; ++face) {
      S2Cell cell = S2Cell::FromFace(face);
      AddCellBoundaries(cell, AddCellBoundaries);
    }
  }
}

inline R2Shape S2Graticule(const IProjection& proj, int level) {
  R2Shape out;
  S2Graticule(&out, proj, level);
  return out;
}

} // namespace w
