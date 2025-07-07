# Wilson

This repository contains a proof of concept visualization tool built on top of
two excellent tools: [S2](https://github.com/google/s2geometry), a library from
Google operating on spherical geometry, and [Blend2D](https://blend2d.com/), a
JIT-ed 2D vector graphics engine.

See a live example here (press 'm' for a menu):

[![Wilson Screenshot](images/wilson-screenshot-1.png)](https://smcallis.github.io/wilson/)


# Compiling

The code is in a semi-proof of concept state, but should still compile and run
on a minimally capable Linux distribution or OSX (possibly with slightly more
work):

``` shell
git clone git@github.com:smcallis/wilson.git
cd wilson
mkdir build
cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release
make -j8
```

