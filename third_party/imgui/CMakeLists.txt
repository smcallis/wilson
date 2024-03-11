add_library(imgui STATIC
  # Headers
  imconfig.h
  imgui.h
  imgui_freetype.h
  imgui_impl_sdl2.h
  imgui_impl_sdlrenderer2.h
  imgui_internal.h
  imgui_stdlib.h
  imstb_rectpack.h
  imstb_textedit.h
  imstb_truetype.h

  # Source
  imgui.cpp
  imgui_demo.cpp
  imgui_draw.cpp
  imgui_freetype.h
  imgui_impl_sdl2.cpp
  imgui_impl_sdlrenderer2.cpp
  imgui_stdlib.cpp
  imgui_tables.cpp
  imgui_widgets.cpp
)
target_link_libraries(imgui SDL2::SDL2)
target_include_directories(imgui PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../)
