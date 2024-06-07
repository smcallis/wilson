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

#include <cstdint>
#include <ctime>
#include <fstream>

// External requirements
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_set.h"
#include "absl/strings/match.h"
#include "absl/strings/str_format.h"
#include "absl/time/time.h"
#include "blend2d.h"
#include "imgui/imgui.h"
#include "imgui/imgui_internal.h"
#include "imgui/imgui_impl_sdl2.h"
#include "imgui/imgui_impl_sdlrenderer2.h"
#include "imgui/font_inconsolata_semibold.h"
#include "s2/r2.h"
#include "s2/s2cell.h"
#include "s2/s2cell_iterator_join.h"
#include "s2/s2edge_distances.h"
#include "s2/s2edge_crossings.h"
#include "s2/s2point.h"
#include "s2/s2predicates.h"
#include "s2/s2lax_polygon_shape.h"
#include "s2/s2shapeutil_coding.h"
#include "s2/mutable_s2shape_index.h"
#include "s2/s2region_coverer.h"
#include "s2/s2text_format.h"
#include "s2/s2shape_index_region.h"
#include "s2/util/bitmap/bitmap.h"
#include "s2/util/coding/coder.h"

// Project requirements
#include "wilson/generated/land.h"
#include "wilson/graphics/inset.h"
#include "wilson/graticule.h"
#include "wilson/geometry/quaternion.h"
#include "wilson/geometry/simplify.h"
#include "wilson/geometry/sierpinski.h"
#include "wilson/strutil.h"
#include "wilson/timing.h"
#include "wilson/geometry/transform.h"
#include "wilson/graphics/pixbuffer.h"
#include "wilson/graphics/sdlapp.h"
#include "wilson/gui/colors.h"
#include "wilson/projection/equirectangular.h"
#include "wilson/projection/all.h"

#include "third_party/dash.h"
#include "third_party/monomath.h"

using namespace w;

static constexpr double kScaleFactor = 1.2;
static constexpr double kMinScale = 1/1.1;
static constexpr double kMaxScale = 1e6;

BLPoint r2b(const R2Point& pnt) {
  return BLPoint(pnt.x(), pnt.y());
}

namespace w {

// This is an ADL override for the monomath header to blend pixels.
pixel blend(pixel dst, pixel src, long amt) {
  if (amt > 255) {
    amt = 255;
  }
  src.mul_alpha(amt);
  return blend(dst, src);
}

}

// Helper to display a little (?) mark which shows a tooltip when hovered.
// In your own code you may want to display an actual icon if you are using a merged icon fonts (see docs/FONTS.md)
static void HelpMarker(const char* desc) {
  ImGui::TextDisabled("(?)");
  if (ImGui::BeginItemTooltip()) {
      ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
      ImGui::TextUnformatted(desc);
      ImGui::PopTextWrapPos();
      ImGui::EndTooltip();
    }
}

class Wilson : public SDLApplication {
public:
  enum : uint32_t { kNumVertices = 100 };

  Wilson() : SDLApplication(), projection_(new Orthographic()) {
    Resize();
    InitImGui();
    inset_.Resize(200, 200);
  }

  Wilson(int ww, int hh, absl::string_view title="")
    : SDLApplication(ww, hh, title), projection_(new Orthographic()) {
    Resize();
    InitImGui();
    inset_.Resize(200, 200);
  }

  ~Wilson() {
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
  }

  // Called periodically to request a new frame be displayed.  Any pending
  // updates can be coalesced and reified here when they're needed rather than
  // immediately.
  void OnFrame(double elapsed) override {
    // angle_ += M_PI / 6 * elapsed;
    // while (angle_ > 2 * M_PI) {
    //   angle_ -= 2 * M_PI;
    // }

    // // Quaternion rotation(Vector3_d(3, -7, 0).Normalize(), angle_);
    // // Quaternion rotation(Vector3_d(0, -1, 0).Normalize(), angle_);
    // Quaternion rotation(Vector3_d(+1, 0, 0).Normalize(), angle_);
    // projection_->SetRotation(rotation);
    // inset_.SetRotation(rotation);
    // dirty_.SetAll(true);

    // Start a new ImGui frame.
    ImGui_ImplSDLRenderer2_NewFrame();  // Renderer backend
    ImGui_ImplSDL2_NewFrame();          // SDL Backend
    ImGui::NewFrame();                  // ImGui core

    if (show_sidebar_) {
      const ImGuiViewport* main_viewport = ImGui::GetMainViewport();

      const ImGuiWindowFlags window_flags = \
        ImGuiWindowFlags_NoDocking
        | ImGuiWindowFlags_NoDecoration
        | ImGuiWindowFlags_NoMove
        | ImGuiWindowFlags_NoBringToFrontOnFocus
        //| ImGuiWindowFlags_NoMouseInputs
        | ImGuiWindowFlags_NoNav;

      ImGui::SetNextWindowPos(ImVec2(main_viewport->WorkSize.x-400, main_viewport->WorkPos.y), ImGuiCond_Always);
      ImGui::SetNextWindowSize(ImVec2(400, main_viewport->WorkSize.y), ImGuiCond_Always);
      ImGui::SetNextWindowViewport(main_viewport->ID);

      // Disable rounded corners, padding and any border for the dock space.
      ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
      ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
      //ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

      ImGui::Begin("Sidebar", nullptr, window_flags);
      {
        ImGui::PopStyleVar(3);
        ImGui::SeparatorText(absl::StrFormat("Status (%.2f)", fps()).c_str());
        ImGui::Indent();

        S2Point pnt;
        if (projection_->Unproject(&pnt, mouse_)) {
          ImGui::Text("Latitude:  %+11.6f", S2LatLng(pnt).lat().degrees());
          ImGui::SameLine(250); ImGui::Text("X: %+f", pnt.x());

          ImGui::Text("Longitude: %+11.6f", S2LatLng(pnt).lng().degrees());
          ImGui::SameLine(250); ImGui::Text("Y: %+f", pnt.y());

          ImGui::NewLine();
          ImGui::SameLine(250); ImGui::Text("Z: %+f", pnt.z());
        } else {
          ImGui::Text("Latitude:  ---");
          ImGui::SameLine(250); ImGui::Text("X: ---");

          ImGui::Text("Longitude: ---");
          ImGui::SameLine(250); ImGui::Text("Y: ---");

          ImGui::NewLine();
          ImGui::SameLine(250); ImGui::Text("Z: ---");
        }
        ImGui::Unindent();

        ImGui::SeparatorText("Config");

        ImGui::Indent();
        {
          ImGui::Text("Background");
          ChooseColor("Background", bg_color_);

          ImGui::Text("Land");
          {
            ImGui::SliderFloat("Thickness", &stroke_width_, 0.0f, 4.0f, "%.1f");
            ChooseColor("Land Border", stroke_color_);
            ImGui::SameLine();
            ChooseColor("Land Fill", fill_color_);
          }
        }
        ImGui::Unindent();

        // Display selectable list of projections.
        if (ImGui::CollapsingHeader("Projection")) {
          ImGui::Indent();

          EachProjection([&](const ProjectionInfo& info) {
            if (ImGui::Selectable(info.name.data())) {
              projection_ = info.factory(
                projection_->Rotation(), projection_->Scale());
              Resize();
            }
          });

          ImGui::Unindent();
        }
      }

      if (ImGui::CollapsingHeader("Timing")) {
        static ImGuiTableFlags flags =
          ImGuiTableFlags_ScrollY |
          ImGuiTableFlags_BordersV |
          ImGuiTableFlags_BordersOuterH |
          ImGuiTableFlags_RowBg |
          ImGuiTableFlags_NoBordersInBody;

        if (ImGui::BeginTable("Timings", 2, flags)) {
          ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_NoHide);
          ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, ImGui::CalcTextSize("A").x * 12.0f);

          VisitTimings(
            [&](absl::string_view name, absl::string_view tip, double time) {
              const float TEXT_BASE_WIDTH = ImGui::CalcTextSize("A").x;
              int indent = (int)(TEXT_BASE_WIDTH/2 * absl::c_count(name, '.'));

              if (indent) {
                ImGui::Indent(indent);
              }
              ImGui::TableNextRow();
              ImGui::TableSetColumnIndex(0);
              ImGui::Text("%s", name.data());
              if (!tip.empty()) {
                ImGui::SameLine(); HelpMarker(tip.data());
              }

              ImGui::TableSetColumnIndex(1);
              ImGui::Text("%s", engnot(time, "s", "%6.1f").c_str());

              if (indent) {
                ImGui::Unindent(indent);
              }
          });

          ImGui::EndTable();
        }
      }

      ImGui::End();
    }

    //ImGui::ShowDemoWindow();

    // Start a new rendering context and clear the screen.
    BLContext& ctx = context_;
    ctx.begin(texture_.image());

    // Clear the on-screen outline of the sphere.
    outline_.Clear();
    projection_->AppendOutline(&outline_);
    ctx.setFillStyle(bg_color_);
    ctx.fillPath(outline_.path());

    //s2cell cell(S2CellId::FromToken("14"));
    //S2Cell cell(S2CellId::FromToken("0b"));
    S2Cell cell(S2CellId::FromToken("0b"));

    // // Draw back cell.
    // R2Shape clipped1;
    // projection_->Subdivide(clipped1, {-cell.GetVertex(0), -cell.GetVertex(1)}, 0.25);
    // projection_->Subdivide(clipped1, {-cell.GetVertex(1), -cell.GetVertex(2)}, 0.25);
    // projection_->Subdivide(clipped1, {-cell.GetVertex(2), -cell.GetVertex(3)}, 0.25);
    // projection_->Subdivide(clipped1, {-cell.GetVertex(3), -cell.GetVertex(0)}, 0.25);

    // ctx.setFillStyle(pixel(0xffe6b3b3));
    // ctx.fillPath(clipped1.path());
    S2CellUnion viewport;

    timeit("OnFrame", [&]() {
      // XXX: reproject graticules.
      timeit("graticules", "Time to reproject graticules", [&]() {
        Reproject();
      });

      if (index_ != nullptr) {
        // Lookup the shape ids that intersect the viewport.
        absl::flat_hash_set<int> shapes;

        timeit("join", "Time to lookup the visible geometry", [&]() {

          // Cover the viewport with an S2CellUnion.
          timeit("cover", "Time to cover viewport with S2 cells", [&]() {
            viewport = projection_->Viewport();
          });

          // And use that cell union to lookup the visible geometry.
          MakeS2CellIteratorJoin(&viewport, index_).Join([&](auto, auto iter) {
            for (const S2ClippedShape& clipped : iter.cell().clipped_shapes()) {
              shapes.insert(clipped.shape_id());
            }
            return true;
          });
        });

        timeit("drawing", [&]() {
          ctx.setFillStyle(fill_color_);
          ctx.setStrokeStyle(stroke_color_);
          ctx.setStrokeWidth(stroke_width_);

          timeit("reproject", "Time to reproject out of date geometry", [&]() {
            for (int shape_id : shapes)
            //int shape_id = 0;
            {
              ProjectedShape& pair = projected_shapes_[shape_id];
              if (dirty_.Get(shape_id)) {
                dirty_.Set(shape_id, false);

                pair.r2shape.Clear();
                projection_->Project(&pair.r2shape, &chain_stitcher_,
                                     *pair.s2shape, IndexedContains(*index_, shape_id));

                R2Shape simplified;
                Simplify(&simplified, pair.r2shape);
                pair.r2shape = simplified;
              }
            }
          });

          timeit("fill", "Time to fill geometry interiors", [&]() {
            for (int shape_id : shapes) {
              ProjectedShape& pair = projected_shapes_[shape_id];
              ctx.fillPath(pair.r2shape.path());
            }
          });

          timeit("stroke", "Time to stroke geometry borders", [&]() {
            if (stroke_width_ > 0) {
              for (int shape_id : shapes) {
                ProjectedShape& pair = projected_shapes_[shape_id];
                ctx.strokePath(pair.r2shape.path());
              }
            }
          });
        });
      }
    });

    // Draw front cell.
    std::vector<std::vector<S2Point>> loops;
    loops.push_back({
        cell.GetVertex(0),
        cell.GetVertex(1),
        cell.GetVertex(2),
        cell.GetVertex(3)
      });

    // S2LaxPolygonShape shape0;
    // shape0.Init(loops);

    // R2Shape clipped0 = projection_->Project(shape0);
    // ctx.setFillStyle(pixel(0xff8fcc66));
    // ctx.fillPath(clipped0.path());

    const auto DrawNiceCircle = [&](S2Point normal) {
      R2Shape shape;
      const auto AddGreatCircle = [&](S2Point normal, bool flip=false) {
        using std::swap;

        // If we're aligned with the poles, then choose the X axis to find
        // our orthonormal basis.
        S2Point up(0,0,1);
        if (normal.DotProd({1,0,0}) == 0) {
          up = S2Point(1,0,0);
        }

        S2Point u = up.CrossProd(normal).Normalize();
        S2Point v = normal.CrossProd(u).Normalize();

        S2Point v0 = std::cos(0*2*M_PI/3)*u + std::sin(0*2*M_PI/3)*v;
        S2Point v1 = std::cos(1*2*M_PI/3)*u + std::sin(1*2*M_PI/3)*v;
        S2Point v2 = std::cos(2*2*M_PI/3)*u + std::sin(2*2*M_PI/3)*v;

        absl::InlinedVector<S2Point,4> vertices;

        // S2Shape::Edge edges[] = {{
        //     S2LatLng::FromDegrees(0,0).ToPoint(),
        //     S2LatLng::FromDegrees(60,0).ToPoint()
        //   }};
        // for (const S2Shape::Edge& edge : edges) {
        //   for (S2Shape::Edge clip : projection_->Clip(edge)) {
        //     if (flip) {
        //       clip.v0 = -clip.v0;
        //       clip.v1 = -clip.v1;
        //     }
        //     // printf("  clip: %f %f %f  %f %f %f\n",
        //     //   clip.v0.x(), clip.v0.y(), clip.v0.z(),
        //     //   clip.v1.x(), clip.v1.y(), clip.v1.z());

        //     projection_->Subdivide(shape, clip, 0.25, true);
        //     shape.EndChain();
        //     //clipped.emplace_back(clip);
        //   }
        // }

        // printf("\n");
        // S2Shape::Edge edges[] = {{v0,v1}, {v1,v2}, {v2,v0}};
        // int i=0;
        // for (const S2Shape::Edge& edge : edges) {
        //   printf("piece %d\n", ++i);

        //   IProjection::EdgeList edges;
        //   for (S2Shape::Edge clip : projection_->Clip(&edges, edge)) {
        //     if (flip) {
        //       clip.v0 = -clip.v0;
        //       clip.v1 = -clip.v1;
        //     }
        //     // printf("  clip: %f %f %f  %f %f %f\n",
        //     //   clip.v0.x(), clip.v0.y(), clip.v0.z(),
        //     //   clip.v1.x(), clip.v1.y(), clip.v1.z());

        //     projection_->Subdivide(&shape, clip, 0.25);
        //     shape.EndChain();
        //     //clipped.emplace_back(clip);
        //   }
        // }

        // if (clipped.empty()) {
        //   return;
        // }

        // vertices.emplace_back(clipped[0].v0);
        // vertices.emplace_back(clipped[0].v1);
        // swap(clipped[0], clipped.back());
        // clipped.pop_back();

        // while (!clipped.empty()) {
        //   for (int i=0; i < clipped.size();) {
        //     const S2Shape::Edge& curr = clipped[i];

        //     // printf("\n");
        //     // printf("vertices[0]: %f %f %f    curr.v1: %f %f %f\n",
        //     //   vertices[0].x(), vertices[0].y(), vertices[0].z(),
        //     //   curr.v1.x(), curr.v1.y(), curr.v1.z());

        //     // printf("curr.v0: %f %f %f    vertices[-1]: %f %f %f\n",
        //     //   curr.v0.x(), curr.v0.y(), curr.v0.z(),
        //     //   vertices.back().x(), vertices.back().y(), vertices.back().z());

        //     // printf("dist0: %e  dist1: %e\n",
        //     //   (vertices[0]-curr.v1).Norm2(),
        //     //   (curr.v0-vertices.back()).Norm2());

        //     if ((curr.v0-vertices.back()).Norm2() < 1e-6) {
        //       //printf("  appending (dist1)\n");
        //       vertices.emplace_back(curr.v1);
        //     } else if ((vertices[0]-curr.v1).Norm2() < 1e-6) {
        //       //printf("  prepending (dist0)\n");
        //       vertices.insert(vertices.begin(), curr.v0);
        //     } else {
        //       ++i;
        //       continue;
        //     }

        //     // Swap and pop the edge, don't increment the counter so we
        //     // check the new edge at this index.
        //     swap(clipped[i], clipped.back());
        //     clipped.pop_back();
        //   }
        // }
        // //   int j=0;
        // //   for (S2Shape::Edge clipped : projection_->Clip(edge)) {
        // //     printf("  piece %d\n", j++);

        // //     if (flip) {
        // //       clipped.v0 = -clipped.v0;
        // //       clipped.v1 = -clipped.v1;
        // //     }

        // //     if (vertices.empty()) {
        // //       vertices.emplace_back(clipped.v0);
        // //       vertices.emplace_back(clipped.v1);
        // //     } else {
        // //       printf("vertices[0]: %f %f %f    clipped.v1: %f %f %f\n",
        // //         vertices[0].x(), vertices[0].y(), vertices[0].z(),
        // //         clipped.v1.x(), clipped.v1.y(), clipped.v1.z());
        // //       printf("clipped.v0: %f %f %f    vertices[-1]: %f %f %f\n",
        // //         clipped.v0.x(), clipped.v0.y(), clipped.v0.z(),
        // //         vertices.back().x(), vertices.back().y(), vertices.back().z());

        // //       printf("dist0: %e  dist1: %e\n",
        // //         (vertices[0]-clipped.v1).Norm2(),
        // //         (clipped.v0-vertices.back()).Norm2());

        // //       if ((vertices[0]-clipped.v1).Norm2() < 1e-6) {
        // //         printf("  prepending (dist0)\n");
        // //         vertices.insert(vertices.begin(), clipped.v0);
        // //       } else if ((clipped.v0-vertices.back()).Norm2() < 1e-6) {
        // //         printf("  appending (dist1)\n");
        // //         vertices.emplace_back(clipped.v1);
        // //       }
        // //     }
        // //}

        // if (!vertices.empty()) {
        //   for (int i=0; i < vertices.size()-1; ++i) {
        //     projection_->Subdivide(shape, {vertices[i], vertices[i+1]}, 0.25, i == (vertices.size()-2));
        //   };
        //   shape.EndChain();
        // }
      };

      // Draw forward part of great circle.
      ctx.setStrokeWidth(2);
      ctx.setStrokeStyle(pixel(0xff434C5E));
      AddGreatCircle(normal);
      ctx.strokePath(shape.path());

      // And draw the backward half dashed.
      shape.Clear();
      AddGreatCircle(normal, true);

      if (!shape.Empty()) {
        std::vector<double> pt_nodes_x;
        std::vector<double> pt_nodes_y;
        for (int i=0; i < shape.nchains(); ++i) {
          for (const R2Point& pnt : shape.chain_vertices(i)) {
            pt_nodes_x.push_back(pnt.x());
            pt_nodes_y.push_back(pnt.y());
          }
        }

        std::vector<double> dash = { 5,10 };
        double dash_offset = 0.0;

        std::vector<double> xp;
        std::vector<double> yp;
        double r = 0;
        makeDashedPolyline(pt_nodes_x,pt_nodes_y,dash,dash_offset,xp,yp,r);

        ctx.setStrokeWidth(1.5);
        ctx.setStrokeStyle(pixel(0xff8e95a4));
        for (int i=0; i < xp.size()/2; ++i) {
          ctx.strokeLine(xp[2*i], yp[2*i], xp[2*i+1], yp[2*i+1]);
        }
      }
    };

    //S2Cell cell = S2Cell::FromFace(0);
    // DrawNiceCircle(cell.GetEdge(0));
    // DrawNiceCircle(cell.GetEdge(1));
    // DrawNiceCircle(cell.GetEdge(2));
    // DrawNiceCircle(cell.GetEdge(3));

    ctx.setStrokeStyle(pixel(0xffd0d0d0));
    int nlevel = graticules_.size();
    for (int l=nlevel-1; l >= 0; --l) {
      ctx.setStrokeWidth(3-2.5*(double)l/(nlevel-1));
      ctx.strokePath(graticules_[l].path());
    }

    // Use the texture canvas as a compositing source and draw the outline
    // again.  This will clip out any overdraw or slight numerical error visible
    // around the edge of the projection.
    ctx.end();

    BLPattern texture(texture_.image());
    ctx.begin(surface().image());
    surface().clear(pixel(0xff000000));

    ctx.save();
    ctx.setCompOp(BL_COMP_OP_SRC_COPY);
    ctx.setFillStyle(texture);
    ctx.fillPath(outline_.path());
    ctx.restore();

    // draw any final overlay text we need last, so that it's on top.
    // DrawText(20, 20, pixel(0xffffcc00), absl::StrFormat("FPS: %.2f", fps()));

    // Draw lat/lon if we have it.
    S2Point pnt;
    if (projection_->Unproject(&pnt, mouse_, true)) {
      // DrawText(20, height()-25, pixel(0xffffcc00),
      //   absl::StrFormat("Lat: %+10.6f  Lon: %+11.6f  (%f %f %f)",
      //     S2LatLng(pnt).lat().degrees(),
      //     S2LatLng(pnt).lng().degrees(), pnt.x(), pnt.y(), pnt.z()
      //   ));

      R2Point proj;
      if (projection_->Project(&proj, pnt)) {
        ctx.setFillStyle(BLRgba32(0xFFCC8899u));
        ctx.fillCircle(proj.x(), proj.y(), 3.0);
      }

      if (draw_rule_) {
        R2Shape shape;
        projection_->Project(&shape, {rule_start_, pnt});

        ctx.setStrokeStyle(pixel(0xaa64389f));
        ctx.setStrokeWidth(3);
        ctx.strokePath(shape.path());
      }
    }

    // red: v1: (0.706867 -0.706867 -0.026045)[4]
    // orange: v0: (0.244553 -0.685636 0.685636)[4]

    // // Red
    // R2Point p0 = projection_->Project(S2Point(0.706867, -0.706867, -0.026045));
    // ctx.setFillStyle(BLRgba32(0xFFFF0000));
    // ctx.fillCircle(p0.x(), p0.y(), 3);

    // // Orange
    // p0 = projection_->Project(S2Point(0.244553, -0.685636, 0.685636));
    // ctx.setFillStyle(BLRgba32(0xFFFFA500));
    // ctx.fillCircle(p0.x(), p0.y(), 3);

    // // Yellow
    // p0 = projection_->Project(S2Point(0.319845, -0.669962, -0.669962));
    // ctx.setFillStyle(BLRgba32(0xFFFFFF00));
    // ctx.fillCircle(p0.x(), p0.y(), 3);

    // // Green
    // p0 = projection_->Project(S2Point(0.649617, -0.649617, -0.394962));
    // ctx.setFillStyle(BLRgba32(0xFF00FF00));
    // ctx.fillCircle(p0.x(), p0.y(), 3);

    // // Blue
    // p0 = projection_->Project(S2Point(-0.589198, 0.589198, -0.552894));
    // ctx.setFillStyle(BLRgba32(0xFF0000FF));
    // ctx.fillCircle(p0.x(), p0.y(), 3);

    // // Indigo
    // p0 = projection_->Project(S2Point(-0.609768, 0.506326, -0.609768));
    // ctx.setFillStyle(BLRgba32(0xFF4B0082));
    // ctx.fillCircle(p0.x(), p0.y(), 3);

    timeit("inset", "Time to redraw inset.", [&]() {
      inset_.Redraw(ctx, *projection_);
    });

    ctx.end();
    surface().sully();
  }

  // Called after a frame is drawn.
  void AfterDraw() override {
    // Render ImGui commands into vertex lists, and pass that data to the
    // backend to actual render it to the window.
    ImGui::Render();
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());
  }

  void OnEvent(const SDL_Event& event) override {
    ImGui_ImplSDL2_ProcessEvent(&event);

    // Filter out any capture mouse events.
    const ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse) {
      switch (event.type) {
        case SDL_MOUSEMOTION:     // fallthrough
        case SDL_MOUSEWHEEL:      // fallthrough
        case SDL_MOUSEBUTTONDOWN: // fallthrough
        case SDL_MOUSEBUTTONUP:   // fallthrough
          return;
        default:
          break;
      }
    }

    // Filter out any capture keyboard events.
    if (io.WantCaptureKeyboard) {
      switch (event.type) {
        case SDL_KEYDOWN:     // fallthrough
        case SDL_KEYUP:       // fallthrough
        case SDL_TEXTEDITING: // fallthrough
        case SDL_TEXTINPUT:   // fallthrough
          return;
        default:
          break;
      }
    }

    if (event.type == SDL_WINDOWEVENT) {
      switch (event.window.event) {
        case SDL_WINDOWEVENT_SIZE_CHANGED:
          Resize();
          break;
      }
      return;
    }

    if (event.type == SDL_KEYUP) {
      if (event.key.keysym.sym == SDLK_ESCAPE) {
        RequestQuit();
      }

      if (event.key.keysym.sym == SDLK_UP) {
        angle_ += .1;
      }

      if (event.key.keysym.sym == SDLK_0) {
        // Reset rotation to default.
        Quaternion rotation({0, 1, 0}, 0.0);
        projection_->SetScale(kMinScale);
        projection_->SetRotation(rotation);
        inset_.SetRotation(rotation);
        dirty_.SetAll(true);
      }

      return;
    }

    // Calls Projection::Unproject but temporarily sets a new rotation.  This
    // lets us find rotations relative to a starting reference frame.
    const auto Unproject = [&](S2Point& out, R2Point pnt, Quaternion rot) {
      projection_->PushTransform();
      projection_->SetRotation(rot);
      bool hit = projection_->Unproject(&out, pnt);
      projection_->PopTransform();
      return hit;
    };

    // Handle scroll wheel events to adjust the zoom.
    if (event.type == SDL_MOUSEWHEEL) {
      S2Point prev, curr;
      if (projection_->Unproject(&prev, mouse_)) {
        double scale = projection_->Scale();

        // Be sure to use .preciseY here because when compiling for WASM
        // browsers often hold the wheel position as floating point, so if we
        // use the regular .y value then it's often zero which leads to
        // stuttering zooming in and out.
        scale = scale*pow(kScaleFactor, event.wheel.preciseY);
        scale = std::min(std::max(scale, kMinScale), kMaxScale);

        // Adjust the projection with the new scale.
        projection_->SetScale(scale);
        dirty_.SetAll(true);

        // Grab the new point under the mouse (if any) and rotate the old
        // position to be under the new position.  This will make it appear
        // we're zooming in-and-out around the mouse point.
        if (projection_->Unproject(&curr, mouse_)) {
          Quaternion rotation(prev, curr);
          rotation *= projection_->Rotation();

          projection_->SetRotation(rotation);
          inset_.SetRotation(rotation);
        }
      }
      return;
    }

    if (event.type == SDL_MOUSEBUTTONDOWN) {
      if (event.button.button == SDL_BUTTON_LEFT) {
        if (!dragging_ && projection_->Unproject(&mouse_start_, mouse_)) {
          dragging_ = true;
          rot_start_ = projection_->Rotation();
        }
      }
      return;
    }

    if (event.type == SDL_MOUSEBUTTONUP) {
      switch (event.button.button) {
        case SDL_BUTTON_RIGHT:
          draw_rule_ = !draw_rule_ && projection_->Unproject(&rule_start_, mouse_);
          break;

        case SDL_BUTTON_LEFT:
          if (dragging_) {
            dragging_ = false;

            S2Point current;
            if (Unproject(current, mouse_, rot_start_)) {
              // Compose final rotation.
              auto rotation = Quaternion(mouse_start_, current)*rot_start_;
              projection_->SetRotation(rotation);
              inset_.SetRotation(rotation);
              dirty_.SetAll(true);
            }
          }
          break;
      }
      return;
    }

    if (event.type == SDL_TEXTINPUT) {
      absl::string_view text = event.text.text;
      if (text == "S" || text == "s") {
        Snapshot();
      }

      if (text == "M" || text == "m") {
        show_sidebar_ = !show_sidebar_;
      }
      return;
    }

    if (event.type == SDL_MOUSEMOTION) {
      mouse_ = R2Point(event.motion.x, event.motion.y);

      // If we're left-dragging.
      if (dragging_) {
        if (event.motion.state & SDL_BUTTON_LMASK) {
          S2Point current;
          if (Unproject(current, mouse_, rot_start_)) {
            auto rotation = Quaternion(mouse_start_, current)*rot_start_;
            projection_->SetRotation(rotation);
            inset_.SetRotation(rotation);
            dirty_.SetAll(true);
          }
        }
      }
      return;
    }
  }

  void Snapshot() {
    std::string name = "wilson-" + absl::FormatTime(absl::Now()) + ".png";
    BLImageCodec codec;
    codec.findByName("PNG");
    surface().image().writeToFile(name.c_str(), codec);
    printf("Saved snapshot as '%s'\n", name.c_str());
  };

  // Adds an S2ShapeIndex to be drawn to the sphere display.
  void AddIndex(const S2ShapeIndex& index) {
    index_ = &index;
    dirty_.Resize(index_->num_shape_ids());
    dirty_.SetAll(true);

    for (int i=0; i < index_->num_shape_ids(); ++i) {
      projected_shapes_.push_back({R2Shape{}, index_->shape(i)});
    }
  }

private:
  // Updates internal state to accomodate a window resize.
  void Resize() {
    projection_->Resize(width(), height());
    texture_.resize(width(), height());
    dirty_.SetAll(true);
  }

  // Initializes ImGui.
  void InitImGui() {
    // Initialize the SDL2 backend, we'll be using an SDL renderer to draw.
    if (!ImGui_ImplSDL2_InitForSDLRenderer(window(), renderer())) {
      fprintf(stderr, "Error initializing imgui\n");
      exit(-1);
    }

    // Initialize the actual rendering backend with our renderer().
    if (!ImGui_ImplSDLRenderer2_Init(renderer())) {
      fprintf(stderr, "Error initializing imgui\n");
      exit(-1);
    }
  }

  // Font to draw text with.  This needs to be a member because computing the
  // signed distance fields are expensive, so we create one and re-use it.
  monomath::Font font_;

  // Draws text to the given position with given color using default settings.
  void DrawText(int x, int y, pixel color, absl::string_view text) {
    monomath::Options options;
    options.width  = 8;
    options.height = 16;
    options.weight = 1.25;
    options.spacing = 0.4;

    font_.draw(
      surface().data(),
      surface().stride(),
      surface().height(),
      color, x, y,
      text.data(), options
    );
  }

  struct ProjectedShape {
    R2Shape r2shape;
    const S2Shape* s2shape;
  };

  Inset inset_;

  std::vector<ProjectedShape> projected_shapes_;
  std::vector<R2Shape> graticules_;

  pixel bg_color_ = pixel(0xffe6ecee);
  pixel fill_color_ = pixel(0xFF5BA668);
  pixel stroke_color_ = pixel(0xFF2E3238);
  float stroke_width_ = 1.0;
  S2Shape* shape_ = nullptr;

  // Generates projected shape pairs through the current projection.
  void Reproject() {
    // if (shape_ == nullptr) {
    //   std::vector<std::vector<S2Point>> points;
    //   points.emplace_back();
    //   S2Point center = S2Cell::FromFace(0).GetVertex(2);
    //   S2Point u = center.CrossProd(S2Cell::FromFace(2).GetVertex(2)).Normalize();
    //   S2Point v = center.CrossProd(u).Normalize();

    //   for (int i=0; i < 6; ++i) {
    //     S2Point pp = u*std::cos(2*M_PI/6*i) + v*std::sin(2*M_PI/6*i);
    //     double radius = i % 2 == 1 ? 0.1 : 0.5;
    //     points.back().emplace_back((radius*pp+center).Normalize());
    //   }
    //   shape_ = new S2LaxPolygonShape(points);
    // }

    // if (projected_shapes_.empty() && !indices_.empty()) {
    //   //projected_shapes_.push_back({R2Shape(), shape_});
    //   for (const S2Shape* shape : *indices_[0]) {
    //     projected_shapes_.push_back({R2Shape(), shape});
    //   }
    // }

    // for (auto& pair : projected_shapes_) {
    //   pair.r2shape = projection_->Project(*pair.s2shape, std::move(pair.r2shape));
    //   pair.r2shape = Simplify(pair.r2shape);
    // }

    // Generate the S2 cell graticule.
    graticules_.clear();

    constexpr int kNlevel = 4;
    for (int level=0; level < kNlevel; ++level) {
      //graticules_.emplace_back(projection_->MakeGraticule());
      graticules_.emplace_back(S2Graticule(*projection_.get(), level));
    }
  }

  ChainStitcher chain_stitcher_;
  std::unique_ptr<IProjection> projection_;
  const S2ShapeIndex* index_;
  util::bitmap::Bitmap64 dirty_;

  bool show_sidebar_ = false;
  double angle_ = 0;
  // Drawing context for 2D graphics.
  BLContext context_;
  Pixbuffer texture_;

  // Current mouse coordinates.
  R2Point mouse_;

  // Starting and pending rotations.
  Quaternion rot_start_;

  bool dragging_ = false;
  S2Point mouse_start_;

  bool draw_rule_ = false;
  S2Point rule_start_;

  // Transform from world-space to unit square.
  Affine3 project_fwd_ = Affine3::Eye();
  Affine3 project_inv_ = Affine3::Eye();
  Affine3 invrot_;

  // Scratch space for projected geometry.
  R2Shape outline_;
};

// Land geometry.
static MutableS2ShapeIndex LoadLandIndex() {
  Decoder decoder(LandIndex().data(), LandIndex().size());
  MutableS2ShapeIndex index;
  index.Init(&decoder, s2shapeutil::FullDecodeShapeFactory(&decoder));
  return index;
}

static const MutableS2ShapeIndex& GetLandIndex() {
  static const MutableS2ShapeIndex index = LoadLandIndex();
  return index;
}

int main(int argc, char* argv[]) {
  absl::ParseCommandLine(argc, argv);

  // Setup Dear ImGui context.
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags &= ~ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  io.Fonts->AddFontFromMemoryCompressedTTF(
    inconsolata_semibold_compressed_data,
    inconsolata_semibold_compressed_size, 15);

  ImGuiStyle& style = ImGui::GetStyle();
  style.GrabRounding = 6.0f;
  style.FrameRounding = 12.0f;
  style.WindowRounding = 6.0f;

  Wilson wilson(1024, 1024, "Wilson");

  //auto index = s2textformat::MakeIndexOrDie("## 1:-40, 1:40, 40:0");
  S2Polygon fractal = Sierpinski(6, false);

  // auto index = s2textformat::MakeIndexOrDie(
  //   "## 41.643516896669674:-144.77627950461175,42.94370748528561:-43.87784200461175,42.94370748528561:57.37215799538825,41.90570152627342:163.54403299538825,25.96218157501415:-162.7059670046117,16.4721071207355:163.54403299538825,2.640384861525707:-165.1669045046117,-12.378434176571048:162.84090799538825,-28.454987792176478:-162.3544045046117,-41.415262067488264:158.62215799538825,-56.10063597597255:-154.9715920046117");

  // Mr. Friendly.  Does not contain the poles.
  auto index = s2textformat::MakeIndexOrDie(
    "## 14.218719424393095:-139.04493922809414,22.874523271599866:-160.84181422809414,-3.209494247301447:-138.69337672809414,8.360560091398495:-162.95118922809414,-17.353660502031545:-136.23243922809414,-8.801354410789957:-160.84181422809414,-25.842299026955686:-134.12306422809414,-24.569987918626186:-159.78712672809414,-35.78473924572461:-139.04493922809414,-36.91729573578281:-157.67775172809414,-44.6240076404484:-138.6177252890106,-45.86161710843011:-162.1724127890106,-50.319429809809975:-146.7036627890106,-51.64725873602859:-78.14897528901061,-49.64122759535099:12.202587210989394,-50.543375122592195:109.58539971098939,-54.190008075990896:157.39789971098938,-51.42858864582373:-153.38335028901054,-39.132516286175175:168.29633721098938,-42.067957244234535:-154.43803778901054,-26.15827993925154:167.24164971098938,-33.47074915461003:-154.78960028901054,-10.534132882777719:167.94477471098938,-21.660370875972657:-154.43803778901054,8.360560091398495:166.18696221098938,-6.01260805092413:-154.78960028901054,23.198050611199644:166.18696221098938,13.194081901557816:-157.25053778901054,38.23569318555331:164.42914971098938,25.44041544526596:-154.43803778901054,50.033939944397154:107.47602471098939,52.880480514013065:9.843435709163883,54.134841473590974:-81.91437679083612,53.09212268552355:-168.04718929083612");

// Mr. Friendly's Reverse.  Does contain the poles.
//   auto index = s2textformat::MakeIndexOrDie(
// "## 53.09212268552355:-168.04718929083612,54.134841473590974:-81.91437679083612,52.880480514013065:9.843435709163883,50.033939944397154:107.47602471098939,25.44041544526596:-154.43803778901054,38.23569318555331:164.42914971098938,13.194081901557816:-157.25053778901054,23.198050611199644:166.18696221098938,-6.01260805092413:-154.78960028901054,8.360560091398495:166.18696221098938,-21.660370875972657:-154.43803778901054,-10.534132882777719:167.94477471098938,-33.47074915461003:-154.78960028901054,-26.15827993925154:167.24164971098938,-42.067957244234535:-154.43803778901054,-39.132516286175175:168.29633721098938,-51.42858864582373:-153.38335028901054,-54.190008075990896:157.39789971098938,-50.543375122592195:109.58539971098939,-49.64122759535099:12.202587210989394,-51.64725873602859:-78.14897528901061,-50.319429809809975:-146.7036627890106,-45.86161710843011:-162.1724127890106,-44.6240076404484:-138.6177252890106,-36.91729573578281:-157.67775172809414,-35.78473924572461:-139.04493922809414,-24.569987918626186:-159.78712672809414,-25.842299026955686:-134.12306422809414,-8.801354410789957:-160.84181422809414,-17.353660502031545:-136.23243922809414,8.360560091398495:-162.95118922809414,-3.209494247301447:-138.69337672809414,22.874523271599866:-160.84181422809414,14.218719424393095:-139.04493922809414");


  wilson.AddIndex(GetLandIndex());
  //wilson.AddIndex(*index);
  //wilson.AddIndex(fractal.index());

  //wilson.set_tick_rate(2);
  wilson.Run();
}
