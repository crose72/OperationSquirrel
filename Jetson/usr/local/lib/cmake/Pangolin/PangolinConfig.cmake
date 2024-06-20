# Compute paths
get_filename_component( PROJECT_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH )

# Library dependencies (contains definitions for IMPORTED targets)
if( NOT Pangolin_BINARY_DIR )
  include( "${PROJECT_CMAKE_DIR}/PangolinTargets.cmake" )
endif()

SET( Pangolin_CMAKEMODULES /home/cameroncrose72/Documents/GitHub/Pangolin/cmake )
SET( Pangolin_LIBRARIES    pango_core;pango_display;pango_geometry;pango_glgeometry;pango_image;pango_opengl;pango_packetstream;pango_plot;pango_python;pango_scene;pango_tools;pango_vars;pango_video;pango_windowing;tinyobj )
SET( Pangolin_LIBRARY      "${Pangolin_LIBRARIES}" )

include(CMakeFindDependencyMacro)
find_dependency(Eigen3)

if (UNIX)
  find_dependency(Threads)
endif()
