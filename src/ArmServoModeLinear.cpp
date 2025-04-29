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

#include "imc/bot_servo/ArmServoModeLinear.h"
#include <imc/bot_traj_planner/trajectory_spline_iterative.hpp>

namespace bot_servo {
    ArmServoModeLinear::ArmServoModeLinear(bot_planner::PlannerPtr pl, bot_executor::ExecutorPtr ext)
        : ArmServoModeBase(std::move(pl), std::move(ext)) {}

    ArmServoModeLinear::ArmServoModeLinear(bot_planner::PlannerPtr pl,
        GetCurrentJointValues_func_t getFunc,
        SendServo_func_t sendFunc)
        : ArmServoModeBase(std::move(pl), std::move(getFunc), std::move(sendFunc)) {}

    ArmServoModeUniquePtr
    ArmServoModeLinear::create(bot_planner::PlannerPtr pl, bot_servo::GetCurrentJointValues_func_t getFunc,
                                bot_servo::SendServo_func_t sendFunc) {
        return bot_common::AlgorithmFactory<ArmServoModeBase, bot_planner::PlannerPtr, bot_servo::GetCurrentJointValues_func_t,
            bot_servo::SendServo_func_t>::CreateAlgorithm(
            ArmServoModeLinearName, std::move(pl), std::move(getFunc), std::move(sendFunc));
    }

    ArmServoModeUniquePtr ArmServoModeLinear::create(bot_planner::PlannerPtr pl, bot_executor::ExecutorPtr ext) {
        return std::make_unique<ArmServoModeLinear>(std::move(pl), std::move(ext));
    }

    void ArmServoModeLinear::servoImpl (bot_common::ErrorInfo &ret) {
        using namespace bot_common;
        double freq = m_param->servo_rate;
    	double execute_freq = m_param->desired_rate;
        execute_ret = bot_common::ErrorInfo::OK();
    	auto kin_ = pl_->getKinematicsPtr();
    	const auto &ML = kin_->getJointMotionLimits();
    	const auto dq_max = ML.col(2) * m_param->maxVelFactor;
    	const auto ddq_max = ML.col(3) * m_param->maxAccFactor;

        const auto N = send_data.get().size();
    	Eigen::VectorXd delta_t_2 = Eigen::VectorXd::Zero(N);
    	Eigen::VectorXd t_1 = Eigen::VectorXd::Zero(N);
    	Eigen::VectorXi sign_delta_q(N), joint_finish(N);
    	Eigen::VectorXd delta_q(N), qd(N), dq_max_sync(N), t_1_sync(N), t_2_sync(N), delta_t_2_sync(N), t_f_sync(N), t_d(N), q_1(N);
		const double kDeltaQMotionFinished = 1e-6;
    	double Ts = 1. / freq;
    	double T_execute = 1. / execute_freq;
    	Eigen::VectorXd qk(N), q_goal(N);
        if (m_getCurrentJointValues) {
            qk = m_getCurrentJointValues();
        }else {
            qk = ext_->getCurrentJointValues();
        }

        while (startServo) {
        	auto start_time = std::chrono::steady_clock::now();
        	joint_finish.setZero();
        	if (!buffer.empty()) {
        		q_goal = buffer.pop_front().joint_data;
        		delta_q = q_goal - qk;
        		sign_delta_q = delta_q.cwiseSign().cast<int>();
        		for (Eigen::Index i = 0; i < 7; i++) {
        			if (std::abs(delta_q[i]) > kDeltaQMotionFinished) {
        				double a = 1.5 / 2.0 * (ddq_max[i] + ddq_max[i]);
        				double b = -1.0 * Ts * ddq_max[i] * ddq_max[i];
        				double c =
							std::abs(delta_q[i]) * ddq_max[i] * ddq_max[i];
        				double delta = b * b - 4.0 * a * c;
        				if (delta < 0.0) {
        					delta = 0.0;
        				}
        				dq_max_sync[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
        				t_1_sync[i] = 1.5 * dq_max_sync[i] / ddq_max[i];
        				delta_t_2_sync[i] = 1.5 * dq_max_sync[i] / ddq_max[i];
        				t_f_sync[i] = (t_1_sync)[i] / 2.0 + delta_t_2_sync[i] / 2.0 +
									   std::abs(delta_q[i] / dq_max_sync[i]);
        				t_2_sync[i] = (t_f_sync)[i] - delta_t_2_sync[i];
        				t_d[i] = t_2_sync[i] - t_1_sync[i];
        				q_1[i] = (dq_max_sync)[i] * sign_delta_q[i] * (0.5 * (t_1_sync)[i]);
        			}
        		}//compute t1, t2, delta t
        		auto start_execution_time = std::chrono::steady_clock::now();
        		while (true) {
        			auto loop_start_time = std::chrono::steady_clock::now();
        			double t = double((std::chrono::steady_clock::now() - start_execution_time).count()) / 1e9;
        			if (t >= Ts) {
        				joint_finish.setOnes();
        			}else {
						for (Eigen::Index i = 0; i < 7; i++) {
							if (std::abs(delta_q[i]) < kDeltaQMotionFinished) {
								joint_finish[i] = 1;
							}else {
								if (t < t_1_sync[i]) {
									delta_q[i] = -1.0 / std::pow(t_1_sync[i], 3.0) * dq_max_sync[i] *
												 sign_delta_q[i] * (0.5 * t - t_1_sync[i]) *
												 std::pow(t, 3.0);
								} else if (t >= t_1_sync[i] && t < t_2_sync[i]) {
									delta_q[i] =
										q_1[i] + (t - t_1_sync[i]) * dq_max_sync[i] * sign_delta_q[i];
								} else if (t >= t_2_sync[i] && t < t_f_sync[i]) {
									delta_q[i] =
										delta_q[i] +
										0.5 *
											(1.0 / std::pow(delta_t_2_sync[i], 3.0) *
												 (t - t_1_sync[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
												 std::pow((t - t_1_sync[i] - t_d[i]), 3.0) +
											 (2.0 * t - 2.0 * t_1_sync[i] - delta_t_2_sync[i] -
											  2.0 * t_d[i])) *
											dq_max_sync[i] * sign_delta_q[i];
								} else {
									joint_finish[i] = 1;
								}
							}
						}
        				send_data.set(qk + delta_q);
        			}
        			if (joint_finish.all()) {
        				if (m_getCurrentJointValues) {
        					qk = m_getCurrentJointValues();
        				}else {
        					qk = ext_->getCurrentJointValues();
        				}
        				break;
        			}
        		}
        	}
        }
    }
}