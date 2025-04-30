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

#include "imc/bot_servo/ArmServoModeInterface.h"

#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "irmv/bot_math/utils/utils.hpp"
#include "imc/bot_servo/ArmServoModeDirect.h"
#include "imc/bot_servo/ArmServoModeTrajectory.h"

std::pair<int, std::string> ArmServoModeInterface::init(
    const std::string &servo_config_path,
    const GetCurrentJointValues_interface_func_t &getFunc,
    const SendServo_interface_func_t &sendFunc) {
  try {
    YAML::Node doc = YAML::LoadFile(servo_config_path);
    auto group = doc["Group"].as<std::string>();
    std::filesystem::path m_path(servo_config_path);
    m_path = m_path.parent_path() / ("planner_" + group + ".yml");
    bot_planner::PlannerPtr pl =
        std::make_shared<bot_planner::PlannerBase>(m_path.string());

    auto type = static_cast<bot_servo::ServoType>(doc["Method"].as<int>());

    bot_servo::GetCurrentJointValues_func_t real_get_func = [getFunc]() {
      std::vector<double> ret = getFunc();
      Eigen::VectorXd r_ret(ret.size());
      memcpy(r_ret.data(), ret.data(), ret.size() * sizeof(double));
      return r_ret;
    };
    bot_servo::SendServo_func_t real_send_func = nullptr;
    if (sendFunc != nullptr) {
      real_send_func = [sendFunc](const Eigen::VectorXd &q) {
        std::vector<double> dst(q.size());
        memcpy(dst.data(), q.data(), q.size() * sizeof(double));
        std::pair<int, std::string> ret = sendFunc(dst);
        return bot_common::ErrorInfo{
            static_cast<bot_common::ErrorCode>(ret.first), ret.second};
      };
    }

    switch (type) {
      case bot_servo::Trajectory: {
        m_impl = std::move(bot_servo::ArmServoModeTrajectory::create(
            pl, real_get_func, real_send_func));
        break;
      }
      case bot_servo::Direct: {
        m_impl = std::move(bot_servo::ArmServoModeDirect::create(
            pl, real_get_func, real_send_func));
        break;
      }
      default: {
        m_impl = std::move(bot_servo::ArmServoModeTrajectory::create(
            pl, real_get_func, real_send_func));
        break;
      }
    }
  } catch (const YAML::Exception &e) {
    return {-1, ("Cannot parse " + servo_config_path + " for " + e.what())};
  }
  return {0, "all ok"};
}

std::pair<int, std::string> ArmServoModeInterface::servoToPoint(
    const std::vector<double> &q) {
  Eigen::VectorXd r_ret(q.size());
  memcpy(r_ret.data(), q.data(), q.size() * sizeof(double));
  bot_common::ErrorInfo ret = m_impl->servoToPoint(r_ret);
  return {ret.error_code(), ret.error_msg()};
}

std::pair<int, std::string> ArmServoModeInterface::servoToPose(
    const std::array<double, 7> &p) {
  auto ret = m_impl->servoToPose(utils::convertToIsometry3d(p.data(), 7));
  return {ret.first.error_code(), ret.first.error_msg()};
}

void ArmServoModeInterface::setServoParams(
    const ServoInterfaceParameters &params) {
  bot_servo::ServoParametersPtr parm_ptr =
      std::make_shared<bot_servo::ServoParameters>();
  parm_ptr->considerLimits = params.considerLimits;
  parm_ptr->maxVelFactor = params.maxVelFactor;
  parm_ptr->maxAccFactor = params.maxAccFactor;
  parm_ptr->maxJerkFactor = params.maxJerkFactor;
  parm_ptr->servo_rate = params.servo_rate;
  parm_ptr->desired_rate = params.desired_rate;
  m_impl->setServoParameters(std::move(parm_ptr));
}

std::vector<double> ArmServoModeInterface::queryCurrentCommand() {
  auto ret = m_impl->queryCurrentCommand();
  std::vector<double> dst(ret.size());
  memcpy(dst.data(), ret.data(), ret.size() * sizeof(double));
  return dst;
}