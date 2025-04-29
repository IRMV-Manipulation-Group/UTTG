/*
  ************************************************************************\

                               C O P Y R I G H T

    Copyright © 2024 IRMV lab, Shanghai Jiao Tong University, China.
                          All Rights Reserved.

    Licensed under the Creative Commons Attribution-NonCommercial 4.0
    International License (CC BY-NC 4.0).
    You are free to use, copy, modify, and distribute this software and its
    documentation for educational, research, and other non-commercial purposes,
    provided that appropriate credit is given to the original author(s) and
    copyright holder(s).

    For commercial use or licensing inquiries, please contact:
    IRMV lab, Shanghai Jiao Tong University at: https://irmv.sjtu.edu.cn/

                               D I S C L A I M E R

    IN NO EVENT SHALL TRINITY COLLEGE DUBLIN BE LIABLE TO ANY PARTY FOR
    DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING,
    BUT NOT LIMITED TO, LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE
    AND ITS DOCUMENTATION, EVEN IF TRINITY COLLEGE DUBLIN HAS BEEN ADVISED OF
    THE POSSIBILITY OF SUCH DAMAGES.

    TRINITY COLLEGE DUBLIN DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
    PURPOSE. THE SOFTWARE PROVIDED HEREIN IS ON AN "AS IS" BASIS, AND TRINITY
    COLLEGE DUBLIN HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
    ENHANCEMENTS, OR MODIFICATIONS.

    The authors may be contacted at the following e-mail addresses:

            YX.E.Z yixuanzhou@sjtu.edu.cn

    Further information about the IRMV and its projects can be found at the ISG
  web site :

           https://irmv.sjtu.edu.cn/

  \*************************************************************************
 */

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  // For STL container support
#include <vector>

#include "imc/bot_servo/ArmServoModeInterface.h"

namespace py = pybind11;

void bind_arm_servo_mode_interface(py::module& m) {
    // Define the ServoInterfaceParameters struct in Python
    py::class_<ServoInterfaceParameters>(m, "ServoInterfaceParameters")
            .def(py::init<>())
            .def_readwrite("servo_rate", &ServoInterfaceParameters::servo_rate, "The rate of the given commands")
            .def_readwrite("desired_rate", &ServoInterfaceParameters::desired_rate, "The rate of the desired commands")
            .def_readwrite("maxVelFactor", &ServoInterfaceParameters::maxVelFactor, "The factor of the maximum velocity")
            .def_readwrite("maxAccFactor", &ServoInterfaceParameters::maxAccFactor, "The factor of the maximum acceleration")
            .def_readwrite("maxJerkFactor", &ServoInterfaceParameters::maxJerkFactor, "The factor of the maximum jerk")
            .def_readwrite("considerLimits", &ServoInterfaceParameters::considerLimits, "Whether to consider the limits");

    // Define the ArmServoModeInterface class in Python
    py::class_<ArmServoModeInterface>(m, "ArmServoModeInterface")
            .def(py::init<>())
            .def("init", &ArmServoModeInterface::init, py::arg("planner_config_path"), py::arg("getFunc"), py::arg("sendFunc") = nullptr,
                 "Initialize the servo mode interface with the configuration file path, the function to get the current joint values, "
                 "and the function to send the servo command (optional). If not provided, the user should maintain own servo send thread in the desired rate "
                 "and call queryCurrentCommand to get the current command.")
            .def("servoToPoint", &ArmServoModeInterface::servoToPoint, py::arg("q"),
                 "Generate the C2 continuous servo commands to reach the given target joint values.")
            .def("servoToPose", &ArmServoModeInterface::servoToPose, py::arg("p"),
                 "Generate the C2 continuous servo commands to reach the given target pose. Format: [x, y, z, qx, qy, qz, qw]")
            .def("setServoParams", &ArmServoModeInterface::setServoParams, py::arg("params"),
                 "Set the servo parameters.")
            .def("queryCurrentCommand", &ArmServoModeInterface::queryCurrentCommand,
                 "Query the current servo command. Should be called after servoToPoint or servoToPose has been called.");
}

// In your module definition
PYBIND11_MODULE(UTTG_interface_py, m) { bind_arm_servo_mode_interface(m); }

