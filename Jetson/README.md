# Description

This folder contains any code, libraries, etc. for the jetson.

## usr

This contains the contents of the `/usr/local/include`, `/usr/local/bin`, and `/usr/local/lib` folders from my jetson.  It contains compiled libraries for ORB_SLAM3 and jetson-inference and jetson-utils that only need to be copied to the same location on your jetson.  Then you will be able to compile programs that use these libraries.

- Follow the instructions here to update the linker path <https://stackoverflow.com/questions/480764/linux-error-while-loading-shared-libraries-cannot-open-shared-object-file-no-s>
- Copy all .so files from `SquirrelDefender/lib/` to `/usr/local/lib/` on the jetson
- Copy all folders from `SquirrelDefender/inc/` to `/usr/local/include/` on the jetson
- Updated the shared library cache with `sudo ldconfig -v`
