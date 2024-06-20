// CMake generated file. Do Not Edit.

#pragma once

namespace pangolin {

  // Forward declarations
  bool RegisterTestVideoFactory();
  bool RegisterImagesVideoFactory();
  bool RegisterSplitVideoFactory();
  bool RegisterTruncateVideoFactory();
  bool RegisterPangoVideoFactory();
  bool RegisterDebayerVideoFactory();
  bool RegisterShiftVideoFactory();
  bool RegisterTransformVideoFactory();
  bool RegisterUnpackVideoFactory();
  bool RegisterPackVideoFactory();
  bool RegisterJoinVideoFactory();
  bool RegisterMergeVideoFactory();
  bool RegisterJsonVideoFactory();
  bool RegisterMjpegVideoFactory();
  bool RegisterThreadVideoFactory();
  bool RegisterV4lVideoFactory();
  bool RegisterFfmpegVideoFactory();
  bool RegisterFfmpegVideoFactory();
  bool RegisterFfmpegVideoConvertFactory();


  inline bool RegisterFactoriesVideoInterface() {
    bool success = true;
    success &= RegisterTestVideoFactory();
    success &= RegisterImagesVideoFactory();
    success &= RegisterSplitVideoFactory();
    success &= RegisterTruncateVideoFactory();
    success &= RegisterPangoVideoFactory();
    success &= RegisterDebayerVideoFactory();
    success &= RegisterShiftVideoFactory();
    success &= RegisterTransformVideoFactory();
    success &= RegisterUnpackVideoFactory();
    success &= RegisterPackVideoFactory();
    success &= RegisterJoinVideoFactory();
    success &= RegisterMergeVideoFactory();
    success &= RegisterJsonVideoFactory();
    success &= RegisterMjpegVideoFactory();
    success &= RegisterThreadVideoFactory();
    success &= RegisterV4lVideoFactory();
    success &= RegisterFfmpegVideoFactory();
    success &= RegisterFfmpegVideoFactory();
    success &= RegisterFfmpegVideoConvertFactory();
    return success;
  }


} // pangolin
