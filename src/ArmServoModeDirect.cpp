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

    Further information about the IRMV and its projects can be found at the ISG web site :

           https://irmv.sjtu.edu.cn/

  \*************************************************************************
 */

#include <imc/bot_traj_planner/trajectory_spline_iterative.hpp>
#include <future>
#include "imc/bot_servo/ArmServoModeDirect.h"

namespace bot_servo {
    ArmServoModeDirect::ArmServoModeDirect(bot_planner::PlannerPtr pl, bot_executor::ExecutorPtr ext)
            : ArmServoModeBase(std::move(pl), std::move(ext)) {

    }

    ArmServoModeDirect::ArmServoModeDirect(bot_planner::PlannerPtr pl,
                                                   bot_servo::GetCurrentJointValues_func_t getFunc,
                                                   bot_servo::SendServo_func_t sendFunc) : ArmServoModeBase(
            std::move(pl), std::move(getFunc), std::move(sendFunc)) {

    }

    ArmServoModeUniquePtr
    ArmServoModeDirect::create(bot_planner::PlannerPtr pl, bot_servo::GetCurrentJointValues_func_t getFunc,
                                   bot_servo::SendServo_func_t sendFunc) {
        return bot_common::AlgorithmFactory<ArmServoModeBase, bot_planner::PlannerPtr, bot_servo::GetCurrentJointValues_func_t,
                bot_servo::SendServo_func_t>::CreateAlgorithm(
                ArmServoModeDirectName, std::move(pl), std::move(getFunc), std::move(sendFunc));
    }

    ArmServoModeUniquePtr ArmServoModeDirect::create(bot_planner::PlannerPtr pl, bot_executor::ExecutorPtr ext) {
        return std::make_unique<ArmServoModeDirect>(std::move(pl), std::move(ext));
    }

    void ArmServoModeDirect::servoImpl(bot_common::ErrorInfo &ret) {
        double T;
        T = 1. / m_param->servo_rate;
        auto expected = std::chrono::steady_clock::now() + std::chrono::duration<double>(T);
        auto front = buffer.pop_front();
        send_data.set(front.joint_data);
        std::this_thread::sleep_until(expected);
    }
}
