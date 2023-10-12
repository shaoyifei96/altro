# altro
A Fast Solver for Constrained Trajectory Optimization

This is a fork that works for a quadrotor in a ros noetic environment. This is tested on a Falcon 250 from KumarLab.

See [kr_ilqr_optimizer](https://github.com/KumarRobotics/kr_ilqr_optimizer) for integration with the kr_autonomous_flight stack.

# Installation

## CMake FetchContent
One of the best modern ways to include a dependency in a CMake project is through the
`FetchContent` module. To bring altro into your CMake project, add these lines into 
your `CMakeLists.txt` file:
```cmake
list(APPEND CMAKE_MESSAGE_CONTEXT altro)
FetchContent_Declare(altro
  GIT_REPOSITORY https://github.com/bjack205/altro
  GIT_TAG 9819e66a6d20daf39e23bd9b45444b7f5aeb3322 
  )
FetchContent_MakeAvailable(altro)
list(POP_BACK CMAKE_MESSAGE_CONTEXT)
```
Then link against the `altro::altro` target, same as if you had brought it in via
`find_package` (described below). Note that this clones the repo into your build 
folder and builds all of the `altro` targets as part of your CMake project, ensuring
uniform compilation and dependency management. Note this will also bring in the 
`Eigen` and `fmt` libraries (also through `FetchContent`).


## CMake Install
To install altro locally onto your computer, simply build the install CMake target:
```shell
git clone https://github.com/bjack205/altro
cd altro
mkdir build
cd build   # TIP: you can shortcut these two with `take build` with zsh
cmake -D BUILD_SHARED_LIBS=ON -D CMAKE_INSTALL_PREFIX=/desired/install/location ..
cmake --build . -j4
cmake --install .
```
If the `--prefix` flag is left off (or `CMAKE_INSTALL_PREFIX` is unset), it will install to 
`/opt/rexlab/altro` by default.

To use the installed version, use the `find_package` command. Here's a 
minimal working example:
```cmake
cmake_minimum_required(VERSION 3.19)
project(AltroCMakeExample)

find_package(altro 0.1 REQUIRED)
find_package(fmt REQUIRED)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE altro::altro fmt::fmt Eigen3::Eigen)
```

Note that altro internally uses the `fmt` and `Eigen` libraries. To link against the same
versions used by altro, use the `fmt::fmt` and `Eigen3::Eigen` targets brought in 
automatically when importing altro.

Also note that `CMAKE_PREFIX_PATH` should be set to whatever directory you used in the install.

## Arduino
This shows how to compile the code to use on a Teensy microcontroller.

1. Install Arduino CLI
```shell
cd ~/.local
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
```
2. Change directory back to altro root
```shell
cd <altro/root/directory>
```
3. Install Teensy Arduino libraries and Teensy rules
```shell
sudo cp resources/00-teensy.rules /etc/udev/rules.d/
arduino-cli core install teensy:avr --additional-urls https://www.pjrc.com/teensy/td_156/package_teensy_index.json
```
4.  If needed, add yourself to the dialout and tty groups
```shell
sudo usermod -a -G tty $USER 
sudo usermod -a -G dialout $USER 
```

