#pragma once

// Copyright (c) 2016-2018 Sean McAllister

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// system
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <X11/cursorfont.h>
#include <X11/extensions/Xdbe.h>
#include <X11/extensions/sync.h>

// c++
#include <memory>
#include <vector>

// c
#include <cassert>
#include <stdexcept>

// local
#include <wilson/geometry/region.h>
#include <wilson/graphics/compositor.h>
#include <wilson/graphics/pixbuffer.h>
#include <wilson/graphics/pixel.h>

namespace w {

/*******************************************************************************
 * Event structures
 *******************************************************************************/

// event enumeration of different events emitted
typedef enum {
  WINDOW_RESIZE, WINDOW_CLOSE, WINDOW_MOVE, // window events
  MOUSE_ENTER,   MOUSE_LEAVE,  MOUSE_MOVE,  // mouse events
  MOUSE_DOWN,    MOUSE_DRAG,   MOUSE_UP,
  KEY_PRESS                                 // keyboard events
} event_type_e;


// enumeration of mouse buttons to abstract away numeric number
typedef enum {
  MOUSE_NONE=0,
  MOUSE_LEFT,
  MOUSE_MIDDLE,
  MOUSE_RIGHT,
  MOUSE_MIDDLE_UP,
  MOUSE_MIDDLE_DOWN
} mouse_btn_e;


// struct representing instance of event
struct xwin_event {
  event_type_e type;
  mouse_btn_e  mouse;

  int x,y,w,h;  // positional variables
  int c;        // decoded character, or mouse button

  unsigned mod; // modifiers for key press
  KeySym   sym; // raw symbol for key press
};


// define flags for creating the window
static const int32_t XWIN_32BITVISUAL = 1; // try to get a 32-bit ARGB visual (compositing window manager required for actual transparency)
static const int32_t XWIN_DONTMAP     = 2; // don't map the window automatically (user must do it)
static const int32_t XWIN_NODBUFFER   = 4; // don't try to enable double buffering (useful for testing)
static const int32_t XWIN_FULLSCREEN  = 8; // create window full screen before mapping


/*******************************************************************************
 * X11 window
 *******************************************************************************/
struct xwindow {
  typedef XSetWindowAttributes attributes;

  // create default (null) window
  xwindow(
  ) : pi(new _window()) {}

  // construct window with given parent and dimensions
  xwindow(
    const xwindow &parent,
    ssize_t width,   ssize_t height,
    ssize_t left =0, ssize_t top=0,
    int32_t flags=0
  ) : pi(new _window(parent.pi->window_, width, height, left, top, flags)) {}

  // create window with root window as parent
  xwindow(
    ssize_t width,  ssize_t height,
    ssize_t left=0, ssize_t top=0,
    int32_t flags=0
  ) : pi(new _window(0, width, height, left, top, flags)) {}

  // construct window with given parent, no position
  xwindow(
    const xwindow &parent,
    ssize_t width, ssize_t height,
    int32_t flags=0
  ) : pi(new _window(parent.pi->window_, width, height, 0, 0, flags)) {}

  // map/unmap window (show/hide)
  void show() {   XMapRaised(pi->dpy_, pi->window_); }
  void hide() { XUnmapWindow(pi->dpy_, pi->window_); }

  // set the title that's displayed on the window
  void set_title(const char* title) {
    XStoreName(pi->dpy_, pi->window_, title);
  }

  // change window dimensions
  void resize(size_t ww, size_t hh) {
    XResizeWindow(pi->dpy_, pi->window_, ww, hh);
  }

  // change window position
  void move(int x, int y) {
    XMoveWindow(pi->dpy_, pi->window_, x, y);
  }

  // unset/set minimum size through size hints
  void unset_min_size() { pi->update_size_hint_field(PMinSize, false); }
  void   set_min_size(int minw, int minh) {
    pi->size_hints_.min_width  = minw;
    pi->size_hints_.min_height = minh;
    pi->update_size_hint_field(PMinSize, true);
  }

  // unset/set maximum size through size hints
  void unset_max_size() { pi->update_size_hint_field(PMaxSize, false); }
  void   set_max_size(int maxw, int maxh) {
    pi->size_hints_.max_width  = maxw;
    pi->size_hints_.max_height = maxh;
    pi->update_size_hint_field(PMaxSize, true);
  }


  // unset/set min/max aspect ratio through size hints
  void unset_aspect() { pi->update_size_hint_field(PAspect, false); }
  void   set_aspect(int min_num, int min_den, int max_num, int max_den) {
    pi->size_hints_.min_aspect.x = min_num;
    pi->size_hints_.min_aspect.y = min_den;
    pi->size_hints_.max_aspect.x = max_num;
    pi->size_hints_.max_aspect.y = max_den;
    pi->update_size_hint_field(PAspect, true);
  }


  // unset/set resize increment through size hints
  void unset_size_inc() { pi->update_size_hint_field(PResizeInc, false); }
  void   set_size_inc(int incw, int inch) {
    pi->size_hints_.width_inc  = incw;
    pi->size_hints_.height_inc = inch;
    pi->update_size_hint_field (PResizeInc, true);
  }

  // toggle fullscreen mode
  void toggle_fullscreen() {
    pi->set_fullscreen(!pi->fullscreen_);
  }

  // properties
  int      depth()      const { return pi->visinfo_.depth; }
  bool     fullscreen() const { return pi->fullscreen_;    }
  region2i bounds()     const { return pi->bounds();       }

  // free friend functions
  friend ssize_t  width(const xwindow &win) { return win.pi->width_;  }
  friend ssize_t height(const xwindow &win) { return win.pi->height_; }

  // get pixbuffer
        Pixbuffer& canvas()       { return pi->drawbuf_; }
  const Pixbuffer& canvas() const { return pi->drawbuf_; }

  // set background color for window
  void set_background(pixel color) {
    pi->set_background(color);
  }

  // set foreground color for window
  void set_foreground(pixel color) {
    pi->set_foreground(color);
  }

  // Takes any dirty regions from the pixbuffer, possibly composes them into the
  // composite buffer, and draws them to the screen.
  void draw_pending() {
    pi->draw_pending();
  }

  // Tries to get an event from the event queue.  Returns true if event is
  // available, otherwise returns false
  bool event(xwin_event &evt) {
    return pi->event(evt);
  }

private:
  // window state we can ship around via shared_ptr
  struct _window {
    // default constructor (creates null window)
    _window() =default;

    // create new window with given parent, dimensions, and flags
    _window(
      Window  parent,
      ssize_t width, ssize_t height,
      ssize_t left,  ssize_t top,
      int32_t flags=0
    );

    // cleanup window state
    ~_window();

    // non-copyable/non-moveable
    _window(const _window&)           =delete;
    _window(      _window&&)          =delete;
    _window& operator=(const _window) =delete;

    // flags
    bool fullscreen_   = false; // are we in fullscreen mode?
    bool have_sync_    = false; // do we have the SYNC extension?
    bool dbl_buffer_   = false; // are we actually double buffering if so?
    bool first_resize_ = true;  // indicates a resize is the first we've seen

    // dimensions
    ssize_t width_  = 0;
    ssize_t height_ = 0;
    ssize_t left_   = 0;
    ssize_t top_    = 0;

    // background color
    pixel background_ = pixel(0);

    // compositing buffers
    Pixbuffer    compbuf_; // pixbuffer we can composite drawbuf_ into before depth conversion
    Pixbuffer    drawbuf_; // pixbuffer we can expose to the user for drawing
    std::vector<char> pixmap_;  // pixmap to do pixel depth conversion in before blending

    // info about pending mouse drag
    struct {
      mouse_btn_e mouse;
      int x,y;
    } drag_ = { MOUSE_NONE, 0, 0};

    // info for syncing with window manager during resize
    struct {
      bool     pending;
      uint32_t request_lo;
      uint32_t request_hi;
    } sync_ = { false, 0, 0 };

    // last event type handles
    int last_event_ = 0;

    // atoms for communicating with window manager
    struct {
      Atom wm_protocols;            // atom for window manager protocol
      Atom wm_delete_window;        // atom for close window message
      Atom wm_normal_hints;         // atom for size hints to window manager

      Atom net_supported;           // list of supported protocols
      Atom net_wm_state;            // atom for list of atoms regarding window state
      Atom net_wm_state_fullscreen; // atom to indicate we want full screen
      Atom net_wm_sync_request;     // atom for syncing repaint events with window manager
      Atom net_wm_sync_request_ctr; // atom to hold counter for above
      Atom cardinal;                // cardinal type for properites
    } atoms_;

    // X11 specific variables
    Display       *dpy_    = nullptr; // X11 connection handle
    Window         window_ = 0;       // our window handle
    Window         parent_ = 0;       // parent window handle
    GC             gc_     = 0;       // graphics context
    int            screen_;           // screen to draw to
    XdbeBackBuffer backbuf_;          // backbuffer if we're double buffering
    XVisualInfo    visinfo_;          // current visual we're using
    Colormap       colormap_;         // colormap is required for 32-bit visuals (why?)
    XSyncCounter   counter_;          // counter for syncing with window manager
    XSizeHints     size_hints_;       // size hints, initially nulled out

    // API
    void set_background(pixel color); // set default background color for window
    void set_foreground(pixel color); // set default foreground color for window

    // set fullscreen mode
    void set_fullscreen(bool fullscreen);

    // get event from event queue if available. Populate event parameter
    // and return true if an event is available, otherwise return false.
    //
    bool event(xwin_event &evt);

    // take any dirty regions from pixbuffer, composite them and then
    // draw them to the screen.
    //
    void draw_pending();

    // return bounds of window
    //
    region2i bounds() const {
      return region2i(
        pnt2i(0, 0),
        pnt2i(width_, height_)
      );
    }

    // enable/disable field in size hints and set them for the window
    //
    void update_size_hint_field(unsigned field, bool set);

  private:
    typedef std::shared_ptr<XImage> pXImage;

    // accessors for rendering
    bool    fast_path()                    const; // return true if we can blit ARGB data directly to screen
    void*   curr_data()                    const; // return current pixel source for drawing to screen
    pXImage make_image()                   const; // build XImage wrapper around curr_data
    void    draw_image(XImage*, region2i) const; // draw a region from an XImage to the screen

    // general bookkeeping
    void    make_atoms();                        // populate atoms for communicating with window manager
    void    size_buffers(ssize_t w, ssize_t h);  // resize internal buffers to new dimensions
    void    swap_buffers();                      // if double buffer, swap buffers, else noop

    // return bytes per pixel based on visual depth
    size_t bytes_per_pixel() const {
      if (visinfo_.depth > 16) return 4;
      if (visinfo_.depth >  8) return 2;
      return 1;
    }
  };
  std::shared_ptr<_window> pi;
};


/*******************************************************************************
 * public _window API
 *******************************************************************************/

// create new window
xwindow::_window::_window(Window parent, ssize_t width, ssize_t height, ssize_t left, ssize_t top, int32_t flags)
  : width_(width), height_(height), left_(left), top_(top), parent_(parent) {

  // helpers
  struct h {
    // @param list of VisualIDs that support double buffering
    // @param id visual id to check
    //
    // @return true if given visual ID supported double buffering
    //
    static bool double_buffered(std::vector<VisualID> &visuals, VisualID id) {
      for (VisualID vid : visuals) {
        if (id == vid) { return true; }
      }
      return false;
    }

    // find the best available visual to use.  By default, the preference is a 24-bit
    // XRGB (non-transparent) visual, as this will allow us to copy pixels directly
    // without an intermediate depth conversion.
    //
    // If allow_argb is set, then a 32-bit visual will be sought first.  If neither
    // is possible, then the visual of the parent is returned.
    //
    // In any event, the visual is checked for double buffering support before returning
    // and the dbuffer parameter is set to indicate double buffer support.
    //
    // Note that 32-bit visuals require a compositing window manager to actually enable
    // transparency over background windows.
    //
    // @param dpy         pointer to display to query
    // @param screen      screen on dpy to query
    // @param allow_argb  allow 32-bit visuals (required for transparency)
    // @param dbuffer     on entry, whether to see a double buffered visual
    //                    on exit, whether the returned visual is actually double buffered
    //
    // @return XVisualInfo for visual we found
    //
    static XVisualInfo find_visual(
      Display *dpy, int screen, Window parent, bool allow_argb, bool &dbuffer
    ) {
      // enumerate double buffered visuals if enabled
      std::vector<VisualID> dbvisuals;
      if (dbuffer) {
        int nscreen=1;
        XdbeScreenVisualInfo *dbinfo = XdbeGetVisualInfo(dpy, &parent, &nscreen);
        {
          dbvisuals.resize(dbinfo->count);
          for (ssize_t ii=0; ii < dbinfo->count; ii++) {
            dbvisuals[ii] = dbinfo->visinfo[ii].visual;
          }
          std::sort(dbvisuals.begin(), dbvisuals.end());
        }
        XdbeFreeVisualInfo(dbinfo);
      }

      // define a template visual with "standard" 8-bit RGB fields
      XVisualInfo vis;
      vis.screen     = screen;
      vis.red_mask   = 0xFF0000;
      vis.green_mask = 0x00FF00;
      vis.blue_mask  = 0x0000FF;
      vis.c_class    = TrueColor;

      ////////////////////////////////////////////////////////////////////////////////
      // 1) if enabled, search for a transparent visual first (may not succeed)
      int nvisuals = 0;
      if (allow_argb) {
        vis.depth = 32;
        std::shared_ptr<XVisualInfo> xvs = std::shared_ptr<XVisualInfo>(
          XGetVisualInfo(dpy,
            VisualRedMaskMask | VisualGreenMaskMask | VisualBlueMaskMask |
            VisualScreenMask  | VisualClassMask     | VisualDepthMask,
            &vis, &nvisuals
          ),
          XFree
        );

        if (nvisuals > 0) {
          // try to return the first that supports double buffering
          for (ssize_t ii=0; ii < nvisuals; ii++) {
            XVisualInfo *xvp = xvs.get();
            if (double_buffered(dbvisuals, xvp[ii].visualid)) {
              dbuffer = true;
              return xvp[ii];
            }
          }

          // failed to find double buffered visual, just return first one
          dbuffer = false;
          return *xvs;
        }
      }


      ////////////////////////////////////////////////////////////////////////////////
      // 2) if not ARGB, or that failed, look for a 24-bit RGB visual
      vis.depth = 24;
      std::shared_ptr<XVisualInfo> xvs = std::shared_ptr<XVisualInfo>(
        XGetVisualInfo(dpy,
          VisualRedMaskMask | VisualGreenMaskMask | VisualBlueMaskMask |
          VisualScreenMask  | VisualClassMask     | VisualDepthMask,
          &vis, &nvisuals
        ),
        XFree
      );

      if (nvisuals > 0) {
        // try to return the first that supports double buffering
        for (ssize_t ii=0; ii < nvisuals; ii++) {
          XVisualInfo *xvp = xvs.get();
          if (double_buffered(dbvisuals, xvp[ii].visualid)) {
            dbuffer = true;
            return xvp[ii];
          }
        }

        // failed to find double buffered visual, just return first one
        dbuffer = false;
        return *xvs;
      }


      ////////////////////////////////////////////////////////////////////////////////
      // 3) if those fail, then return our parent's visual, this should always succeed

      // query parent attributes
      XWindowAttributes attr;
      XGetWindowAttributes(dpy, parent, &attr);

      // get visual info from ID
      vis.visualid = XVisualIDFromVisual(attr.visual);
      xvs = std::shared_ptr<XVisualInfo>(
        XGetVisualInfo(dpy,
          VisualIDMask,
          &vis, &nvisuals
        ),
        XFree
      );

      if (nvisuals > 0) {
        // try to return the first that supports double buffering
        for (ssize_t ii=0; ii < nvisuals; ii++) {
          XVisualInfo *xvp = xvs.get();
          if (double_buffered(dbvisuals, xvp[ii].visualid)) {
            dbuffer = true;
            return xvp[ii];
          }
        }

        // failed to find double buffered visual, just return first one
        dbuffer = false;
        return *xvs;
      }

      throw std::runtime_error("no X11 visual found, shouldn't happen, parent doesn't have a visual?!?");
    }
  };

  // clear out size hints
  size_hints_ = XSizeHints{};

  // open connection to X11 server
  dpy_ = XOpenDisplay(NULL);
  if (!dpy_) {
    return;
  }
  screen_ = DefaultScreen(dpy_);

  // now that we have a display connection, populate atom lists
  make_atoms();

  // query extensions that we support
  int major, minor;
  bool have_dbe =  XdbeQueryExtension(dpy_, &major, &minor);

  // initialize sync extension if available
  if (XSyncQueryExtension(dpy_, &major, &minor)) {
    have_sync_ = XSyncInitialize(dpy_, &major, &minor);
  }

  // set root window as parent if none specified
  parent_ = parent_ ? parent_ : RootWindow(dpy_, screen_);

  // find the best visual that we can, enable double buffering if DBE extension
  // is present and it hasn't been explcitly disabled.
  dbl_buffer_ = have_dbe && ((flags & XWIN_NODBUFFER) == 0);
  visinfo_ = h::find_visual(
    dpy_, screen_, parent_,
    flags & XWIN_32BITVISUAL,
    dbl_buffer_
  );

  // allocate buffer space until we get first resize event
  size_buffers(width_, height_);

  // for some reason, 32-bit visuals require the presence of an allocated
  // colormap. We'll just allocate one in all cases, it may or may not be used.
  colormap_ = XCreateColormap(
    dpy_, XDefaultRootWindow(dpy_), visinfo_.visual, AllocNone);

  // set window attributes
  xwindow::attributes attr;
  attr.border_pixel     = BlackPixel(dpy_, screen_);
  attr.background_pixel = BlackPixel(dpy_, screen_);
  attr.backing_store    = Always;
  attr.save_under       = False;
  attr.bit_gravity      = NorthWestGravity;
  attr.colormap         = colormap_;

  unsigned long attr_mask = \
    CWColormap  | CWBorderPixel  |
    CWBackPixel | CWBackingStore |
    CWSaveUnder | CWBitGravity;

  // tell X11 to create the window
  window_ = XCreateWindow(
    dpy_, parent_, left_, top_, width_, height_, 0, visinfo_.depth, InputOutput, visinfo_.visual, attr_mask, &attr
  );

  // create back buffer if we're double buffering. XdbeCopied here indicates
  // that when swapping, we want the new backbuffer to have the contents of
  // the old backbuffer, so we can continually draw updates to the screen
  // without re-blitting the entire thing.
  if (dbl_buffer_) {
    backbuf_ = XdbeAllocateBackBufferName(dpy_, window_, XdbeCopied);
  }

  // Get graphics context for window, no EXPOSE events on XCopy* functions
  gc_ = XCreateGC(dpy_, window_, 0, 0);
  XSetGraphicsExposures(dpy_, gc_, False);

  // create list of window manager protocols to comply with
  size_t natom = 0;
  Atom wmatoms[50];
  wmatoms[natom++] = atoms_.wm_delete_window;    // for window manager close events (ie 'x' clicked)
  wmatoms[natom++] = atoms_.net_wm_sync_request; // for synchronizing resizes

  XSetWMProtocols(dpy_, window_, (Atom*)wmatoms, natom);

  // setup structures for SYNC protocol
  if (have_sync_) {
    // this counter will be set in response to ConfigureNotify events
    // from window manager to indicate that we're done repainting
    XSyncValue value = {0,0};
    counter_ = XSyncCreateCounter(dpy_, value);
    XChangeProperty(dpy_, window_,
      atoms_.net_wm_sync_request_ctr,
      atoms_.cardinal,
      32,
      PropModeReplace,
      (const unsigned char*)&counter_, 1
    );
  }

  // define events that we want to receive
  XSelectInput(dpy_, window_,
    KeyPressMask      |
    ButtonPressMask   | ButtonReleaseMask |
    EnterWindowMask   | LeaveWindowMask   |
    PointerMotionMask | ExposureMask      | StructureNotifyMask
  );

  // map window (put it on the screen) if not disabled
  if (!(flags & XWIN_DONTMAP)) {
    XMapRaised(dpy_, window_);
  }

  // if we actually have a left/right, window manager might have ignored it,
  // so move window to be sure
  if (left_ || top_) {
    XMoveWindow(dpy_, window_, left_, top_);
  }

  // set window to fullscreen if flag was set
  if (flags & XWIN_FULLSCREEN) {
    set_fullscreen(true);
  }

  // set default colors
  set_foreground(pixel::Opaque(0xFFFFFF));
  set_background(pixel::Opaque(0x000000));

  // set a nice default cursor
  XDefineCursor(dpy_, window_, XCreateFontCursor(dpy_, XC_tcross));
  XSync(dpy_, false);
}


// destroy window
xwindow::_window::~_window() {
  if (dpy_) {
    XLockDisplay(dpy_);
    {
      if (dbl_buffer_) {
        XdbeDeallocateBackBufferName(dpy_, backbuf_);
      }
      XFreeGC        (dpy_, gc_);
      XFreeColormap  (dpy_, colormap_);
      XDestroyWindow (dpy_, window_);
    }
    XUnlockDisplay(dpy_);
    XCloseDisplay (dpy_);
  }
}


// set background color for window
void xwindow::_window::set_background(pixel color) {
  background_ = color;

  XColor xcolor;
  xcolor.flags = DoRed | DoGreen | DoBlue;
  xcolor.red   = (uint16_t)color.r << 8;
  xcolor.green = (uint16_t)color.g << 8;
  xcolor.blue  = (uint16_t)color.b << 8;
  XAllocColor         (dpy_, colormap_, &xcolor);
  XSetBackground      (dpy_, gc_,        xcolor.pixel);
  XSetWindowBackground(dpy_, window_,    xcolor.pixel);

  // Changes in background attributes don't take effect until the next
  // expose event for the window.  Have to explicitly clear and flush
  // to make them take effect immediately.
  XClearWindow(dpy_, window_);
  XFlush      (dpy_);
}


// set foreground color for window
void xwindow::_window::set_foreground(pixel color) {
  XColor xcolor;
  xcolor.flags = DoRed | DoGreen | DoBlue;
  xcolor.red   = (uint16_t)color.r << 8;
  xcolor.green = (uint16_t)color.g << 8;
  xcolor.blue  = (uint16_t)color.b << 8;
  XAllocColor   (dpy_, colormap_, &xcolor);
  XSetForeground(dpy_, gc_,        xcolor.pixel);
}


// set fullscreen mode on window
void xwindow::_window::set_fullscreen(bool enable) {
  XEvent xev = {};
  xev.type                 = ClientMessage;
  xev.xclient.window       = window_;
  xev.xclient.message_type = atoms_.net_wm_state;
  xev.xclient.format       = 32;
  xev.xclient.data.l[0]    = enable ? 1 : 0;
  xev.xclient.data.l[1]    = atoms_.net_wm_state_fullscreen;
  xev.xclient.data.l[2]    = 0;

  XSendEvent(dpy_, DefaultRootWindow(dpy_), False,
    SubstructureRedirectMask | SubstructureNotifyMask, &xev
  );

  fullscreen_ = enable;
}


// if event is available, populate parameter and return true.
// otherwise return false.
//
bool xwindow::_window::event(xwin_event &evt) {
  // if we have a pending sync with the window manager and we just sent
  // the resize to the client, then tell the window manager we can handle more.
  if (have_sync_ && sync_.pending && last_event_ == ConfigureNotify) {
    sync_.pending = false;

    XSyncValue value = { (int)sync_.request_hi, sync_.request_lo };
    XSyncSetCounter(dpy_, counter_, value);
  }


  while (XPending(dpy_)) {
    // grab next event
    XEvent event;
    XNextEvent(dpy_, &event);

    last_event_ = event.type;

    switch (event.type) {
      /* resize, move, etc events */
      case ConfigureNotify: {
        int     xx = event.xconfigure.x,      yy = event.xconfigure.y;
        ssize_t ww = event.xconfigure.width,  hh = event.xconfigure.height;

        // if dimensions changed, or we haven't sent initial resize event
        if ((width_ != ww) || (height_ != hh) || first_resize_) {
          first_resize_ = false;

          size_buffers(ww, hh);

          left_  = xx;  top_    = yy;
          width_ = ww;  height_ = hh;

          evt.type = WINDOW_RESIZE;
          evt.x = xx;  evt.y = yy;
          evt.w = ww;  evt.h = hh;
          return true;
        } else if (left_ != (ssize_t)xx || top_ != (ssize_t)yy) {
          left_ = xx;
          top_  = yy;

          // only moved, no change in geometry
          evt.type = WINDOW_MOVE;
          evt.x    = xx;
          evt.y    = yy;
          return true;
        }
        break;
      }


        /* Part of the window was revealed, copy data from our pixmap */
      case Expose: {
        // get wrapped image data
        pXImage image = make_image();

        // create event point
        pnt2i epnt(
          event.xexpose.x,
          event.xexpose.y
        );

        // draw exposed region back to window
        draw_image(
          image.get(),
          region2i(
            epnt,
            pnt2i(
              epnt.x() + event.xexpose.width,
              epnt.y() + event.xexpose.height
            )
          )
        );

        // make changes visible
        swap_buffers();
        break;
      }


        /* handle messages from window manager */
      case ClientMessage: {
        Atom type = event.xclient.message_type;
        Atom msg  = event.xclient.data.l[0];

        // if window manager requested a close, forward to client
        if ((type == atoms_.wm_protocols) && (msg == atoms_.wm_delete_window)) {
          evt.type = WINDOW_CLOSE;
          return true;
        }

        // sync message received, will be receiving a ConfigureNotify soon
        if ((type == atoms_.wm_protocols) && (msg == atoms_.net_wm_sync_request)) {
          sync_.pending    = true;
          sync_.request_lo = event.xclient.data.l[2];
          sync_.request_hi = event.xclient.data.l[3];
        }

        break;
      }


        /* Notify of key presses */
      case KeyPress: {
        char   buf[2];
        KeySym sym;
        if (XLookupString(&event.xkey, buf, 2, &sym, NULL) == 0) {
          buf[0] = 0;
        }

        evt.type = KEY_PRESS;
        evt.c    = buf[0];
        evt.mod  = event.xkey.state;
        evt.sym  = sym;
        evt.x    = event.xkey.x;
        evt.y    = event.xkey.y;
        return true;
        break;
      }


        /* Notify of mouse enter */
      case EnterNotify: {
        evt.type = MOUSE_ENTER;
        evt.x    = event.xcrossing.x;
        evt.y    = event.xcrossing.y;
        return true;
        break;
      }


        /* Notify of mouse exit */
      case LeaveNotify: {
        evt.type = MOUSE_LEAVE;
        evt.x    = event.xcrossing.x;
        evt.y    = event.xcrossing.y;
        return true;
        break;
      }


        /* Notify of mouse down event */
      case ButtonPress: {
        evt.type  = MOUSE_DOWN;
        evt.x     = event.xbutton.x;
        evt.y     = event.xbutton.y;
        evt.mouse = (mouse_btn_e)event.xbutton.button;

        if (drag_.mouse == MOUSE_NONE) {
          drag_.mouse = evt.mouse;
          drag_.x     = evt.x;
          drag_.y     = evt.y;
        }
        return true;
        break;
      }


        /* Notify of mouse motion */
      case MotionNotify: {
        evt.type = MOUSE_MOVE;
        evt.x    = event.xmotion.x;
        evt.y    = event.xmotion.y;

        // if the mouse is pressed, emit a drag event.
        if (drag_.mouse != MOUSE_NONE) {
          evt.type  = MOUSE_DRAG;
          evt.w     = evt.x - drag_.x;
          evt.h     = evt.y - drag_.y;
          evt.mouse = (mouse_btn_e)drag_.mouse;
        }
        return true;
      }


        /* Notify of mouse up event */
      case ButtonRelease: {
        evt.type    = MOUSE_UP;
        evt.x       = event.xbutton.x;
        evt.y       = event.xbutton.y;
        evt.mouse   = (mouse_btn_e)event.xbutton.button;
        drag_.mouse = MOUSE_NONE;
        return true;
        break;
      }

      default:
        break;
    }
  } // while (XPending...)

  return false;
}


// enable/disable field in size hints and set them for the window
void xwindow::_window::update_size_hint_field(unsigned field, bool set) {
  size_hints_.flags &= ~field;
  if (set) {
    size_hints_.flags |= field;
  }
  XSetWMNormalHints(dpy_, window_, &size_hints_);
}


/*******************************************************************************
 * private _window API
 *******************************************************************************/

// populate atom values required by ICCM and EWMH.
//
void xwindow::_window::make_atoms() {
  // ICCCM required atoms
  atoms_.wm_protocols            = XInternAtom(dpy_, "WM_PROTOCOLS",                 1);
  atoms_.wm_delete_window        = XInternAtom(dpy_, "WM_DELETE_WINDOW",             0);
  atoms_.wm_normal_hints         = XInternAtom(dpy_, "WM_NORMAL_HINTS",              0);

  // EWMH required atoms
  atoms_.net_supported           = XInternAtom(dpy_, "_NET_SUPPORTED",               0);
  atoms_.net_wm_state            = XInternAtom(dpy_, "_NET_WM_STATE",                0);
  atoms_.net_wm_state_fullscreen = XInternAtom(dpy_, "_NET_WM_STATE_FULLSCREEN",     0);
  atoms_.net_wm_sync_request     = XInternAtom(dpy_, "_NET_WM_SYNC_REQUEST",         0);
  atoms_.net_wm_sync_request_ctr = XInternAtom(dpy_, "_NET_WM_SYNC_REQUEST_COUNTER", 0);
  atoms_.cardinal                = XInternAtom(dpy_, "CARDINAL",                     0);
}


// resize internal buffers reflect window size
//
void xwindow::_window::size_buffers(ssize_t ww, ssize_t hh) {
  // resize draw buffer
  drawbuf_.resize(ww, hh);
  drawbuf_.clear(background_);

  // resize composite buffer
  compbuf_.resize(ww, hh);
  compbuf_.clear(background_);

  // if we're not blitting to an RGB visual, then resize pixmap memory
  if (!fast_path()) {
    pixmap_.resize(ww*hh*bytes_per_pixel());
  }
}


// swap double buffer if double buffering enabled, noop otherwise
void xwindow::_window::swap_buffers() {
  if (dbl_buffer_) {
    XdbeSwapInfo info;
    info.swap_window = window_;
    info.swap_action = XdbeCopied;
    XdbeSwapBuffers(dpy_, &info, 1);
  }
}

// return true if we're rending to an ARGB compatible visual, meaning we don't
// have to do any depth conversion before drawing to screen.
//
bool xwindow::_window::fast_path() const {
  return \
    (visinfo_.red_mask   == 0xFF0000) &&
    (visinfo_.green_mask == 0x00FF00) &&
    (visinfo_.blue_mask  == 0x0000FF);
}


// return current pixel source for window, may change as children/etc are added
// to drawbuf_ so call frequently.
//
void* xwindow::_window::curr_data() const {
  if (!fast_path()) {
    return (void*)pixmap_.data();
  }

  if (drawbuf_.children().empty()) {
    return (void*)drawbuf_.data();
  } else {
    return (void*)compbuf_.data();
  }
}


// wrap curr_data() in an XImage suitable for copying to screen
//
std::shared_ptr<XImage> xwindow::_window::make_image() const {
  struct h {
    static void free_ximage(XImage *image) {
      image->data = nullptr;
      XDestroyImage(image);
    }
  };

  return std::shared_ptr<XImage>(
    XCreateImage(
      dpy_,
      visinfo_.visual,
      visinfo_.depth,
      ZPixmap,
      0,
      (char*)curr_data(),
      width_,
      height_,
      bytes_per_pixel()*8,
      0
    ),
    h::free_ximage
  );
}


// draw region from image to screen
//
void xwindow::_window::draw_image(XImage *image, region2i region) const {
  XPutImage(dpy_,
    dbl_buffer_ ? backbuf_ : window_,
    gc_,
    image,
    region.lo().x(), region.lo().y(), // source      (x,y)
    region.lo().x(), region.lo().y(), // destination (x,y)
    region.width(),
    region.height()
  );
}


// Convert a frame of data from a pixbuffer to a target buffer through simple
// color truncation.  This just drops the least significant bits off the 8-bit
// color channels.
//
// @param visinfo XVisualInfo containing info about pixel format being converted to
// @param region  rectangular region to convert
// @param pixbuf  reference to pixbuffer containing source data
// @param dptr    destination pointer to write pixels to, assumed to be same
//                width as pixbuf.
//
template <typename T>
static void convert_frame(XVisualInfo visinfo, region2i region, const Pixbuffer &pixbuf, T* dptr) {
  // helper functions
  struct h {
    // generate mask/shift values for pulling out color fields and building new color
    static void gen_shifts(uint32_t mask, uint8_t *down, uint8_t *up) {
      // move mask down to mask out single byte
      *up = 0;
      while (mask && (mask & 0x1) == 0) {
        mask >>= 1;
        (*up)++;
      }

      *down = 8;
      while (mask && (mask & 0x1) == 1) {
        mask >>= 1;
        (*down)--;
      }
    }
  };

  // intersect and round region to make sure we have integral coordinates inside the pixbuf
  const region2i cregion = intersect(region, pixbuf.bounds());
  const int rw = cregion.width();
  const int rh = cregion.height();

  // compute shifts
  uint8_t rdown, rup, gdown, gup, bdown, bup;
  h::gen_shifts(visinfo.red_mask,   &rdown, &rup);
  h::gen_shifts(visinfo.green_mask, &gdown, &gup);
  h::gen_shifts(visinfo.blue_mask,  &bdown, &bup);

  // pointers to start of rectangular region in source and destination
  const pixel *src = pixbuf.data(cregion.lo().x(), cregion.lo().y());
  T *dst = dptr + (size_t)cregion.lo().y()*pixbuf.stride() + (size_t)cregion.lo().x();

  for (int yy=0; yy < rh; yy++) {
    for (int xx=0; xx < rw; xx++) {
      dst[xx] = \
        (T)(src[xx].r >> rdown) << rup |
        (T)(src[xx].g >> gdown) << gup |
        (T)(src[xx].b >> bdown) << bup;
    }
    src += pixbuf.stride();
    dst += pixbuf.stride();
  }
}


// take any dirty regions from the source pixbuffer, compose them into the
// blend buffer, and draw them to the screen.
//
void xwindow::_window::draw_pending() {
  // determine pixbuffer source, if draw buffer has children
  // then we have to composite down into the composite buffer before
  // we can depth convert
  absl::Span<const region2i> dirty;
  Pixbuffer *source;

  Compositor compositor;

  // composite down if draw buffer has children
  if (!drawbuf_.children().empty()) {
    source = &compbuf_;
    dirty  = compositor.composite(compbuf_, drawbuf_);
  } else {
    source = &drawbuf_;
    dirty  =  drawbuf_.dirty_list();
  }

  // do nothing if we have no dirty regions.
  if (dirty.size() == 0) {
    return;
  }

  // get XImage to draw with
  std::shared_ptr<XImage> image = make_image();

  // process each region
  for (const region2i& region : dirty) {
    // if we can't blit pixels directly, then depth convert the source buffer into the pixmap.
    if (!fast_path()) {
      assert(pixmap_.size() >= width_*height_*bytes_per_pixel() && "pixmap not large enough");

      if      (visinfo_.depth > 16) convert_frame<uint32_t>(visinfo_, region, *source, (uint32_t*)pixmap_.data());
      else if (visinfo_.depth >  8) convert_frame<uint16_t>(visinfo_, region, *source, (uint16_t*)pixmap_.data());
      else                          convert_frame< uint8_t>(visinfo_, region, *source, ( uint8_t*)pixmap_.data());
    }

    // draw region to screen
    draw_image(image.get(), region);
  }

  // clear pending dirty regions.
  drawbuf_.clear_dirty_list();

  // make changes visible
  swap_buffers();
}

// Sorry everyone
#undef Status
#undef None

} // namespace w
