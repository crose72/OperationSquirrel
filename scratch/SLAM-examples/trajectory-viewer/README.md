# How to run this example

After installing pangolin, you may need to add the library path so the linker can link the shared object files with the code.  In the terminal do the following:
`LD_LIBRARY_PATH=/usr/local/lib`
`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib`

Now you should be able to compile the example.
`mkdir build`
`cd build`
`cmake ..`
`make`
