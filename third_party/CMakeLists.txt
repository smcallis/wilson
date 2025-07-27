# Dependency install path
set(DEPS_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/opt)

# Build static libraries for dependencies if we can.
set(BUILD_SHARED_LIBS OFF)

# CPM packages
CPMAddPackage(
  NAME abseil
  GITHUB_REPOSITORY abseil/abseil-cpp
  GIT_TAG 20250512.1
  OPTIONS "ABSL_PROPAGATE_CXX_STD ON" "ABSL_ENABLE_INSTALL ON" "BUILD_TESTING OFF"
)

# Required for Blend2d
CPMAddPackage(
  NAME asmjit
  DOWNLOAD_ONLY YES # Blend2D will configure it.
  GITHUB_REPOSITORY asmjit/asmjit
  GIT_TAG master
  OPTIONS "ABSL_PROPAGATE_CXX_STD ON"
)

if (EMSCRIPTEN)
  set(BLEND2D_EXTRA_OPTS "BLEND2D_NO_JIT ON")
endif()

CPMAddPackage(
  NAME blend2d
  GITHUB_REPOSITORY blend2d/blend2d
  GIT_TAG master
  OPTIONS
    "ABSL_PROPAGATE_CXX_STD ON"
    "BLEND2D_STATIC ON"
    "ASMJIT_DIR ${asmjit_SOURCE_DIR}"
    "ASMJIT_STATIC ON"
    "${BLEND2D_EXTRA_OPTS}"
)

CPMAddPackage(
  NAME s2
  GIT_TAG v0.11.1
  VERSION v0.11.1
  GITHUB_REPOSITORY google/s2geometry
  OPTIONS "WITH_GFLAGS off" "WITH_GLOG off" "BUILD_TESTS off"
)

CPMAddPackage(
  NAME googletest
  GITHUB_REPOSITORY google/googletest
  GIT_TAG v1.14.0
  VERSION v1.14.0
  OPTIONS "INSTALL_GTEST OFF" "gtest_force_shared_crt"
)

# Only compile SDL2 if we're not building for Emscripten.  If we are then we
# should install its port with embuilder.
if (EMSCRIPTEN)
  find_package(SDL2 REQUIRED)
else()
  CPMAddPackage(
    NAME SDL2
    GIT_TAG release-2.28.5
    VERSION 2.28.5
    GITHUB_REPOSITORY libsdl-org/SDL

    # SDL will walk down its list of backends and try them one by one which is a
    # problem if e.g. X11 and Wayland don't work for some reason.  It'll get to
    # the DRM backend and error out because we likely don't have permission by
    # default, or it'll get to the offscreen backend and -just silently not
    # display a window-, which never should have been default behavior.  Disable
    # them both.
    #
    # Disable dynamic loading of X11 support since it doesn't look for the library
    # files in the right place (which causes it to fail and then see above...)
    OPTIONS "SDL_KMSDRM OFF" "SDL_KMSDRM_SHARED OFF" "SDL_OFFSCREEN OFF" "SDL_X11_SHARED OFF"
  )
endif()

# When building for newer X11 versions, we need to add these pre-processor
# defines to avoid SDL forward-declaring some X11 structs that then conflict
# when compiling.
if (SDL2_ADDED)
  # If we're compiling for OSX, then we need to make sure the macports path is
  # include to find libiconv.
  if (MACOSX)
    target_include_directories(sdl-build-options INTERFACE "/opt/local/include")
    target_link_directories(sdl-build-options INTERFACE "/opt/local/lib")
  else()
    target_link_libraries(sdl-build-options INTERFACE Xrender xcb)
  endif()
  target_compile_definitions(sdl-build-options INTERFACE SDL_VIDEO_DRIVER_X11_SUPPORTS_GENERIC_EVENTS SDL_VIDEO_DRIVER_X11_CONST_PARAM_XEXTADDDISPLAY)
endif()

add_subdirectory(imgui)
add_subdirectory(fuzztest)
