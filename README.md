# UTTG

## Table of Contents

- [Description](#description)
- [Dependencies](#dependencies)
- [Build](#build)
- [Installation](#installation)
- [Usage](#usage)

## Description

An universal teleoperation trajectory generator (UTTG) is a library for generating smooth and within joint limits
trajectories for teleoperation tasks. The library provides a C++ interface for trajectory generation and a Python
interface for easy integration with other Python packages.

## Dependencies

### Basic Environment

- `cmake >= 3.21`

### Binary Packages

The following binary packages are required: `eigen3`, `yaml-cpp`.

These binary packages can be installed via:

```shell
sudo apt-get install libeigen3-dev libyaml-cpp-dev
```

### IRMV Packages

The following custom packages are required: `irmv_core`,`imc`.

These packages need to be obtained from the IRMV Lab.

## Build

```shell
cmake -B build . -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Installation

You can install the provided package directly:

```shell
cd build && sudo make install
```

or you can create a Debian package and install it for convenient deployment:

```shell
cd build && cpack
sudo dpkg -i UTTG-xxxx.deb
```

## Usage

### C++ Usage

```c++
#include <imc/bot_servo/ArmServoModeInterface.h>
#include <iostream>
#include <vector>

int main() {
    // Create an instance of ArmServoModeInterface
    ArmServoModeInterface armServo;

    // Initialize the interface with a configuration path and dummy functions
    armServo.init("path/to/planner_config.yaml", 
                  []() -> std::vector<double> { return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; }, 
                  [](const std::vector<double>& cmd) { std::cout << "Sending command: "; for (auto c : cmd) std::cout << c << " "; std::cout << std::endl; });

    // Set servo parameters
    ServoInterfaceParameters params;
    params.servo_rate = 100;
    params.desired_rate = 100;
    params.maxVelFactor = 1.0;
    params.maxAccFactor = 1.0;
    params.maxJerkFactor = 1.0;
    params.considerLimits = true;
    armServo.setServoParams(params);

    // Generate servo commands to move to a target joint position
    std::vector<double> targetJointValues = {1.0, 0.5, -0.5, 1.0, -1.0, 0.5};
    armServo.servoToPoint(targetJointValues);

    // Query the current command
    auto currentCommand = armServo.queryCurrentCommand();
    std::cout << "Current command: ";
    for (auto c : currentCommand) std::cout << c << " ";
    std::cout << std::endl;

    return 0;
}
```

One can compile the code via:

```cmake
find_package(UTTG REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example UTTG::UTTG_interface)
```

### Python Usage

We provide a Python interface for the trajectory generator. Users should first install the Python package (Pybind11 are required):

One can get the Pybind11 package via:

```shell
git submodule update --init --recursive
```

Then, users can install the Python package via

**Notice that python install must be done after the build and install process**

```shell
cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCOMPILE_UTTG_PYBINDING=ON
make -j$(nproc) && sudo make install
```

```shell
pip install .
```
Then, users can use the Python interface according to `example/example.py`

```python
from UTTG_interface_py import ArmServoModeInterface, ServoInterfaceParameters

# Create an instance of ArmServoModeInterface
arm_servo = ArmServoModeInterface()

# Initialize the interface with a configuration path and dummy functions
arm_servo.init("path/to/planner_config.yaml",
               lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
               lambda cmd: print(f"Sending command: {cmd}"))

# Set servo parameters
params = ServoInterfaceParameters()
params.servo_rate = 50
params.desired_rate = 100
params.maxVelFactor = 1.0
params.maxAccFactor = 1.0
params.maxJerkFactor = 1.0
params.considerLimits = True
arm_servo.setServoParams(params)

# Generate servo commands to move to a target joint position
target_joint_values = [1.0, 0.5, -0.5, 1.0, -1.0, 0.5]
arm_servo.servoToPoint(target_joint_values)

# Query the current command
current_command = arm_servo.queryCurrentCommand()
print(f"Current command: {current_command}")
```

## Citation

If you use this code in your research, please cite the us as follows:

```bibtex
@misc{ZHOU2025UTTG,
    author = {Shengjian Fang, Yixuan Zhou, Yu Zheng, Pengyu Jiang, Siyuan Liu and Hesheng Wang},
    title = {UTTG: A Universal Teleoperation Approach via Online Trajectory Generation},
    year = {2025},
    eprint = {arXiv:2504.19736},
    url = {https://arxiv.org/abs/2504.19736},
}
```