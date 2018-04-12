## PolyTraj 
PolyTraj is a C++ trajectory generation library for autonomous vehicle motion planning based on ["Reactive Nonholonomic Trajectory Generation via Parametric Optimal Control"](http://frc.ri.cmu.edu/~alonzo/pubs/papers/ijrr02TrajGen.pdf).

### Installation
1. Install [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html).
2. Install [GTest](https://github.com/google/googletest) if you want to run the tests.

```
git clone --recurse-submodules -j2 [https://github.com/jsford/PolyTraj]()
cd PolyTraj
mkdir build
cd build
cmake ..
make
sudo make install
```

### Examples

```
cd PolyTraj/build/examples/
./path_example
./trajectory_example
```

### Tests
cd PolyTraj/build/test/<br>
./run_tests

### Contact
Jordan Ford<br>
[jordan@andrew.cmu.edu](mailto:jordan@andrew.cmu.edu)<br>


