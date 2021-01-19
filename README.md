Environment map remapping tool
==============================

This software is a very rough utility to transform environment maps from a parametrization to another.


Compilation
-----------

You need CMake.

```bash
mkdir build
cd build
cmake ..
```

Usage
-----

```bash
./bin/envremap -i rnl_probe.exr -s probe -o pano.exr -t pano -w 1024
```