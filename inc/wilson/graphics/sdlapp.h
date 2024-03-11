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

#include <absl/strings/string_view.h>
#include <blend2d.h>
#include <SDL.h>

#ifdef EMSCRIPTEN
#include <emscripten/emscripten.h>
#endif

#include <unistd.h>

#include "wilson/graphics/compositor.h"
#include "wilson/graphics/pixbuffer.h"
#include "wilson/timing.h"

namespace w {

#ifdef EMSCRIPTEN
inline void emscripten_main(void* arg);
#endif

// A class to inherit from that provides a main loop for an application via SDL.
class SDLApplication {
public:
  SDLApplication() { CreateWindow(800, 800); }
  SDLApplication(int ww, int hh, absl::string_view title="") {
    CreateWindow(ww, hh, title);
  }

  ~SDLApplication() {
    DestroyWindow();
  }

  // Sets the title text for the window.
  void set_title(absl::string_view title) {
    SDL_SetWindowTitle(window_, title.data());
  }

  // Sets the rate of calls to OnFrame (default is 60/second).
  void set_tick_rate(double rate) {
    tick_rate_ = rate;
  }

  // Returns the current tick interval.
  double tick_rate() const { return tick_rate_; }

  // Returns the current value of the frame rate estimator.
  double fps() const { return frame_rate_; }

  // Gets the width and height of the window.
  int width() const { return width_; }
  int height() const { return height_; }

  // Main loop that processes and dispatches events.
  int Run() {
#ifndef EMSCRIPTEN
    while (RunOne());
#else
    // We have to use a trampoline to call our main loop in Emscripten so that
    // the browser's event loop can run too.  emscripten_main will call RunOne()
    // for us.
    emscripten_set_main_loop_arg(emscripten_main, this, -1, 1);
#endif
    return exitcode_;
  }

  bool RunOne() {
    SDL_Event event;

    bool sleep = true;
    while (!quitting_ && SDL_PollEvent(&event)) {
      HandleEvent(event);
      sleep = false;
    }

    const double elapsed = stopwatch(last_frame_);
    if (elapsed*tick_rate_ >= 1) {
      last_frame_ += elapsed;

      // Exponentially average to estimate the current frame rate.
      frame_rate_ = .05*(1/elapsed) + .95*frame_rate_;

      OnFrame(elapsed);
      sleep = false;
    }

    if (sleep) {
      // Nothing happened, sleep briefly to avoid pinning the CPU.
      if (1/tick_rate_-elapsed > 2e-3) {
        usleep(1000);
      }
    } else {
      DrawUpdates();
    }

    return !quitting_;
  }

  // Callback interface.
  virtual void OnCreate() {};
  virtual void OnEvent(const SDL_Event&) {};
  virtual void OnFrame(double elapsed) {};
  virtual void AfterDraw() {};

protected:
  void RequestQuit(bool flag=true) { quitting_ = flag; }
  Pixbuffer& surface() { return primary_; }
  const Pixbuffer& surface() const { return primary_; }

  // Returns a pointer to the underlying SDL window.
  SDL_Window* window() { return window_; }

  // Returns a pointer to the underlying SDL renderer.
  SDL_Renderer* renderer() { return renderer_; }

private:
  // Creates a new window and displays it.
  void CreateWindow(int w, int h, absl::string_view title = "");

  // Destroys the window and associated renderer.
  void DestroyWindow();

  // Processes an event and passes it on via OnEvent.
  void HandleEvent(const SDL_Event& event);

  // Creates new rendering surfaces with the given dimensions.  The old surfaces
  // (if any) are destroyed.
  bool PrepareSurfaces(int w, int h);

  // Destroys the texture surface we're rendering to.
  void DestroySurfaces();

  // Draw and updates to the screen.
  void DrawUpdates();

  // Handles for the primary window, the renderer and the texture we use to blit
  // pixels to the screen.
  SDL_Window* window_ = nullptr;
  SDL_Renderer* renderer_ = nullptr;
  SDL_Texture* texture_ = nullptr;

  // A compositor to blend changes together from the pixbuffer stack.  They're
  // blended into the composite_ buffer.
  Compositor compositor_;
  Pixbuffer primary_;
  Pixbuffer composite_;

  int width_, height_;

  double tick_rate_ = 60.0;
  double frame_rate_ = 0;
  double last_frame_ = 0;

  bool quitting_ = false;
  int exitcode_ = 0;
};

inline void SDLApplication::CreateWindow(int w, int h, absl::string_view title) {
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS | SDL_INIT_TIMER) != 0) {
    fprintf(stderr, "Failed to initialize SDL: %s\n", SDL_GetError());
    exit(-1);
  }

  fprintf(stderr, "video driver: %s\n", SDL_GetCurrentVideoDriver());

  window_ = SDL_CreateWindow(
    title.data(),
    SDL_WINDOWPOS_UNDEFINED,
    SDL_WINDOWPOS_UNDEFINED,
    w, h,
    SDL_WINDOW_RESIZABLE
  );

  if (window_ == nullptr) {
    fprintf(stderr, "Failed to create SDL window: %s\n", SDL_GetError());
    exit(-1);
  }

  // Create a renderer, and set it's blend mode to SDL_BLENDMODE_NONE.  This
  // will ensure we blit pixels directly without any alpha blending.
  renderer_ = SDL_CreateRenderer(
    window_, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_TARGETTEXTURE
  );

  if (renderer_ == nullptr) {
    fprintf(stderr, "Failed to create SDL renderer: %s\n", SDL_GetError());
    DestroyWindow();
    exit(-1);
  }
  SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_NONE);

  if (SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 255) < 0) {
    fprintf(stderr, "Error!: %s\n", SDL_GetError());
    exit(-1);
  }

  // Clear the entire screen to white
  if(SDL_RenderClear(renderer_) < 0) {
    fprintf(stderr, "Error setting color: %s\n", SDL_GetError());
    exit(-1);
  }
  SDL_RenderPresent(renderer_);

  if (!PrepareSurfaces(w, h)) {
    DestroyWindow();
    exit(-1);
  }

  width_ = w;
  height_ = h;
  OnCreate();
}

inline void SDLApplication::DestroyWindow() {
  DestroySurfaces();

  if (renderer_) {
    SDL_DestroyRenderer(renderer_);
    renderer_ = nullptr;
  }

  if (window_) {
    SDL_DestroyWindow(window_);
    window_ = nullptr;
  }
}

inline bool SDLApplication::PrepareSurfaces(int w, int h) {
  DestroySurfaces();

  texture_ = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, w, h);
  if (!texture_) {
    fprintf(stderr, "Failed to create texture: %s\n", SDL_GetError());
    return false;
  }
  SDL_SetTextureBlendMode(texture_, SDL_BLENDMODE_NONE);

  primary_.resize(w, h);
  composite_.resize(w, h);
  return true;
}

inline void SDLApplication::DestroySurfaces() {
  if (texture_) {
    SDL_DestroyTexture(texture_);
    texture_ = nullptr;
  }
}

inline void SDLApplication::HandleEvent(const SDL_Event& event) {
  if (event.type == SDL_QUIT) {
    RequestQuit();
  }

  if (event.type == SDL_WINDOWEVENT) {
    switch (event.window.event) {
      case SDL_WINDOWEVENT_SIZE_CHANGED:
        int neww = event.window.data1;
        int newh = event.window.data2;

        if (neww != width() || newh != height()) {
          PrepareSurfaces(neww, newh);
        }

        width_ = neww;
        height_ = newh;
        break;
    }
  }

  // Pass event down to subclass.
  OnEvent(event);
}

// Draws any regions of the primary pixbuffer or its children to the screen.
inline void SDLApplication::DrawUpdates() {
  // Nothing to do if there are no dirty regions.
  if (primary_.dirty_list().empty()) {
    AfterDraw();
    return;
  }

  for (region2i region : compositor_.composite(composite_, primary_)) {
    SDL_Rect rect;
    rect.x = region.lo().x();
    rect.y = region.lo().y();
    rect.w = region.width();
    rect.h = region.height();

    pixel* dst;
    int pitch;

    SDL_LockTexture(texture_, &rect, (void**)&dst, &pitch);
    pitch /= sizeof(pixel);

    // If we didn't have to composite down then we can blit directly from the
    // primary pixbuffer instead of the composite pixbuffer.
    const Pixbuffer& source = compositor_.composited() ? composite_ : primary_;

    // Blit the pixels to the texture.
    const pixel* src = source.data(rect.x, rect.y);
    for (int y=0; y < rect.h; ++y) {
      memcpy(dst, src, rect.w*sizeof(pixel));
      dst += pitch;
      src += source.stride();
    }

    SDL_UnlockTexture(texture_);
  }

  // SDL_UpdateTexture(texture_, nullptr, source.data(), source.stride_bytes());
  SDL_RenderCopy(renderer_, texture_, nullptr, nullptr);

  primary_.clear_dirty_list();
  AfterDraw();
  SDL_RenderPresent(renderer_);
}

#ifdef EMSCRIPTEN
inline void emscripten_main(void* arg) {
  if (!static_cast<SDLApplication*>(arg)->RunOne()) {
    emscripten_cancel_main_loop();
  }
}
#endif

} // namespace w
