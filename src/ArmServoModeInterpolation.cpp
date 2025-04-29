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
            SJ Fang fang20021005@sjtu.edu.cn

    Further information about the IRMV and its projects can be found at the ISG
  web site :

           https://irmv.sjtu.edu.cn/

  \*************************************************************************
 */

#include "imc/bot_servo/ArmServoModeInterpolation.h"
#include <imc/bot_traj_planner/trajectory_spline_iterative.hpp>

namespace bot_servo {
    ArmServoModeInterpolation::ArmServoModeInterpolation(bot_planner::PlannerPtr pl, bot_executor::ExecutorPtr ext)
            : ArmServoModeBase(std::move(pl), std::move(ext)) {}

    ArmServoModeInterpolation::ArmServoModeInterpolation(bot_planner::PlannerPtr pl,
            GetCurrentJointValues_func_t getFunc,
            SendServo_func_t sendFunc)
            : ArmServoModeBase(std::move(pl), std::move(getFunc), std::move(sendFunc)) {}

    ArmServoModeUniquePtr
    ArmServoModeInterpolation::create(bot_planner::PlannerPtr pl, bot_servo::GetCurrentJointValues_func_t getFunc,
                                    bot_servo::SendServo_func_t sendFunc) {
            return bot_common::AlgorithmFactory<ArmServoModeBase, bot_planner::PlannerPtr, bot_servo::GetCurrentJointValues_func_t,
                    bot_servo::SendServo_func_t>::CreateAlgorithm(
                    ArmServoModeInterpolationName, std::move(pl), std::move(getFunc), std::move(sendFunc));
    }

    ArmServoModeUniquePtr ArmServoModeInterpolation::create(bot_planner::PlannerPtr pl, bot_executor::ExecutorPtr ext) {
            return std::make_unique<ArmServoModeInterpolation>(std::move(pl), std::move(ext));
    }

    void ArmServoModeInterpolation::servoImpl (bot_common::ErrorInfo &ret) {
        using namespace bot_common;
        double freq = m_param->servo_rate;
        // std::chrono::milliseconds INTERVAL((size_t)(1e3 / freq));
        // wait at least buffer size points to come up;
        execute_ret = bot_common::ErrorInfo::OK();
        //this must be ensured that the interpolation method is nearby
        const auto N = send_data.get().size();

        Eigen::VectorXd qk(N), q_goal(N), q_goal_next(N), vk(N), v_goal(N), ak(N), a_goal(N), qd(N), joint_finish(
                N), m_delta_q(N), qd_pre(N);
        //todo: should consider the passed-in function: see ArmServoModeBase(line 338-342)
        if (m_getCurrentJointValues) {
            qk = m_getCurrentJointValues();
        }else {
            qk = ext_->getCurrentJointValues();
        }
        vk.setZero();

        double Ts = 1. / freq;
        const auto TS = std::chrono::nanoseconds(size_t(Ts * 1e9));
        const double kDeltaQMotionFinished = 1e-6;
        double T, delay_time = 0;// count the number of the points
        while (startServo) {
            //first point special handle, this must be ensured that the interpolation is accomplished nearby
            auto start_time = std::chrono::steady_clock::now();
            joint_finish.setZero();
            //200hz sendServo once;
            if (!buffer.empty()) {
                q_goal = buffer.front().joint_data;
            	m_delta_q = q_goal - qk;
                if (buffer.size() > 1) {
                    q_goal_next = buffer[1].joint_data;
                    T = double((buffer[1].stamp - buffer[0].stamp).count()) / 1e9;
                    v_goal = (q_goal_next - qk) / 2 / T;
                } else {
                    PLOGD << "buffer size is just 1";
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    if (buffer.size() == 1) {
                        v_goal.setZero();
                        PLOGD << "the last point";
                        PLOGD << "delay time = " << delay_time;
                    } else {
                        PLOGD << "not last point";
                        q_goal_next = buffer[1].joint_data;
                        T = double((buffer[1].stamp - buffer[0].stamp).count()) / 1e9;
                        v_goal = (q_goal_next - qk) / 2 / T;
                    }
                }
                auto start_execution = std::chrono::steady_clock::now();
                while (true) {
                    qd.setZero();
                    auto loop_start_time = std::chrono::steady_clock::now();
                    double t = double((std::chrono::steady_clock::now() - start_execution).count()) / 1e9;
                    if (t >= T) {
                        joint_finish.setOnes();
                    } else {
                        for (int i = 0; i < N; i++) {
                            if (std::abs(q_goal[i] - qd[i]) <= kDeltaQMotionFinished) {
                                joint_finish[i] = 1;
                            } else {
                                Eigen::VectorXd x(6);
                                m_delta_q[i] = q_goal[i] - qk[i];
                                x << (12. * m_delta_q[i] - 6. * (vk[i] + v_goal[i]) * T) /
                                     (2. * std::pow(T, 5)),
                                        (-30. * m_delta_q[i] + (14. * v_goal[i] + 16. * vk[i]) * T) /
                                        (2. * std::pow(T, 4)),
                                        (20. * m_delta_q[i] - (8. * v_goal[i] + 12. * vk[i]) * T) /
                                        (2. * std::pow(T, 3)),
                                        0, vk[i], qk[i];
                                qd[i] = x[5] + x[4] * t + x[3] * std::pow(t, 2) + x[2] * std::pow(t, 3) + x[
                                                                                                                  1] *
                                                                                                          std::pow(
                                                                                                                  t,
                                                                                                                  4) +
                                        x[0] * std::pow(t, 5);
                            }
                            // PLOGD <<"delta q"<<i<<" = "<<std::abs(q_goal[i] - qd[i]);
                        }
                        send_data.set(qd);
                        auto expected_time = loop_start_time + TS;
                        if (std::chrono::steady_clock::now() < expected_time) {
                            std::this_thread::sleep_until(expected_time);
                        }
                    }
                    if (joint_finish.all())//update qk vk ak
                    {
                        qk = q_goal;
                        vk = v_goal;
                        buffer.pop_front();
                        delay_time = delay_time +
                                     double((std::chrono::steady_clock::now() - start_execution).count()) /
                                     1e9 - T;
                        break;
                    }
                }

            } else {
                //ext_->sendServo(ext_->getCurrentJointValues());
                auto expected_end = start_time + TS;
                if (std::chrono::steady_clock::now() < expected_end) {
                    std::this_thread::sleep_until(expected_end);
                }
            }
        }
        if (ext_) execute_ret = ext_->waitForFinish();
        PLOGD << "point finish";
    }
}