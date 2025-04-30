
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

#ifndef UTTG_ARMSERVOMODEINTERFACE_H
#define UTTG_ARMSERVOMODEINTERFACE_H

#include <functional>
#include <memory>
#include <string>

namespace bot_servo {
class ArmServoModeBase;
typedef std::shared_ptr<ArmServoModeBase> ArmServoModeBasePtr;
}  // namespace bot_servo


typedef std::function<std::vector<double>()>
    GetCurrentJointValues_interface_func_t;

typedef std::function<std::pair<int, std::string>(const std::vector<double>&)>
    SendServo_interface_func_t;

struct ServoInterfaceParameters {
  double servo_rate = 30.;        ///< The rate of the given commands;
  double desired_rate = 200.;     ///< The rate of the desired commands;
  double maxVelFactor = 1.0;      ///< The factor of the maximum velocity;
  double maxAccFactor = 1.0;      ///< The factor of the maximum acceleration;
  double maxJerkFactor = 1.0;     ///< The factor of the maximum jerk;
  bool considerLimits = false;    ///< Whether to consider the limits;
};

/**
 * @brief The ArmServoModeInterface class
 */
class ArmServoModeInterface {
 public:
  /**
   * constructor
   */
  ArmServoModeInterface() = default;

  /**
   * destructor
   */
  ~ArmServoModeInterface() = default;

 public:
    /**
     * @brief Given the configuration file path, the function to get the current joint values,
     *        and the function to send the servo command (optional), initialize the servo mode interface.
     * @param servo_config_path The path of the configuration file.
     * @param getFunc The function to get the current joint values, which will be called in the desired rate.
     * @param sendFunc The function to send the servo command.(optional).
     * If not provided, the user should maintain own servo send thread in the desired rate
     * and call @queryCurrentCommand to get the current command
     * @return error code and error message.
     */
  std::pair<int, std::string> init(
      const std::string& servo_config_path,
      const GetCurrentJointValues_interface_func_t& getFunc,
      const SendServo_interface_func_t& sendFunc = nullptr);

  /**
   * @brief Given the joint values, the interface will generate the C2 continuous servo commands tor reach the given target.
   * @param q The given target joint values.
   * @return error code and error message.
   */
  std::pair<int, std::string> servoToPoint(const std::vector<double>& q);

  /**
   * @brief Given the pose, the interface will generate the C2 continuous servo commands tor reach the given target.
   * @param p The given target pose. Format: [x, y, z, qx, qy, qz, qw]
   * @return error code and error message.
   */
  std::pair<int, std::string> servoToPose(const std::array<double, 7>& p);

  /**
   * @brief Set the servo parameters.
   * @param params The given servo parameters. see @ServoInterfaceParameters.
   */
  void setServoParams(const ServoInterfaceParameters& params);

  /**
   * @brief Query the current servo command. Should be called after @servoToPoint or @servoToPose has been called.
   * @return The current servo command.
   */
  std::vector<double> queryCurrentCommand();

 protected:
  bot_servo::ArmServoModeBasePtr m_impl;
};

#endif  // UTTG_ARMSERVOMODEINTERFACE_H
