// CMake generated file. Do Not Edit.

#pragma once

namespace pangolin {

  // Forward declarations
  bool RegisterX11WindowFactory();
  bool RegisterWaylandWindowFactory();
  bool RegisterHeadlessWindowFactory();


  inline bool RegisterFactoriesWindowInterface() {
    bool success = true;
    success &= RegisterX11WindowFactory();
    success &= RegisterWaylandWindowFactory();
    success &= RegisterHeadlessWindowFactory();
    return success;
  }


} // pangolin
