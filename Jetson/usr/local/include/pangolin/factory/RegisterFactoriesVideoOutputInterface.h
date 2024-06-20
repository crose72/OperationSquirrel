// CMake generated file. Do Not Edit.

#pragma once

namespace pangolin {

  // Forward declarations
  bool RegisterImagesVideoOutputFactory();
  bool RegisterPangoVideoOutputFactory();
  bool RegisterFfmpegVideoOutputFactory();


  inline bool RegisterFactoriesVideoOutputInterface() {
    bool success = true;
    success &= RegisterImagesVideoOutputFactory();
    success &= RegisterPangoVideoOutputFactory();
    success &= RegisterFfmpegVideoOutputFactory();
    return success;
  }


} // pangolin
