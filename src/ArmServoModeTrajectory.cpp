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
#include "imc/bot_servo/ArmServoModeTrajectory.h"
namespace bot_servo {
    ArmServoModeTrajectory::ArmServoModeTrajectory(bot_planner::PlannerPtr pl, bot_executor::ExecutorPtr ext)
            : ArmServoModeBase(std::move(pl), std::move(ext)) {

    }

    ArmServoModeTrajectory::ArmServoModeTrajectory(bot_planner::PlannerPtr pl,
                                                   bot_servo::GetCurrentJointValues_func_t getFunc,
                                                   bot_servo::SendServo_func_t sendFunc) : ArmServoModeBase(
            std::move(pl), std::move(getFunc), std::move(sendFunc)) {

    }

    ArmServoModeUniquePtr
    ArmServoModeTrajectory::create(bot_planner::PlannerPtr pl, bot_servo::GetCurrentJointValues_func_t getFunc,
                                   bot_servo::SendServo_func_t sendFunc) {
        return bot_common::AlgorithmFactory<ArmServoModeBase, bot_planner::PlannerPtr, bot_servo::GetCurrentJointValues_func_t,
                bot_servo::SendServo_func_t>::CreateAlgorithm(
                ArmServoModeTrajectoryName, std::move(pl), std::move(getFunc), std::move(sendFunc));
    }

    ArmServoModeUniquePtr ArmServoModeTrajectory::create(bot_planner::PlannerPtr pl, bot_executor::ExecutorPtr ext) {
        return std::make_unique<ArmServoModeTrajectory>(std::move(pl), std::move(ext));
    }

    void ArmServoModeTrajectory::servoImpl(bot_common::ErrorInfo &ret) {
        using namespace bot_common;
        //here we reach the first point, actually start servo
        auto kin_ = pl_->getKinematicsPtr();
        const auto &ML = kin_->getJointMotionLimits();
        //first convert to stamped joint path;
        std::vector<Eigen::VectorXd> raw_stamped_path;
        extractStampedJointPath(raw_stamped_path); //at least we got the last send command as start;

        auto next_traj = std::make_unique<bot_planner::TrajectoryIterativeSpline>(true, false, false);
        ret = next_traj->init(raw_stamped_path,
                              ML.col(2) * m_param->maxVelFactor,
                              ML.col(3) * m_param->maxAccFactor,
                              ML.col(4) * m_param->maxJerkFactor);

        if (!ret.IsOK()) {
            PLOGW << "cannot plan trajectory for current store servo points";
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            return;
        } else if (next_traj->getDuration() < std::numeric_limits<double>::epsilon()) {
            //PLOGD << "reach the current point, erase the current";
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            if(startServo)
                return;
            else{
                if (!buffer.empty())
                    buffer.pop_front();
            }
        } else {
            while (true) {
                //to fetch renew time, typically we take the first buffer point that should be sent
                double renew_time;
                const auto &time_stamps = next_traj->getImpl()->getTimeStamps();
                if (raw_stamped_path.size() > 4)
                    renew_time = time_stamps[1];
                else if (raw_stamped_path.size() == 4 || raw_stamped_path.size() == 3)
                    renew_time = time_stamps[2];
                else if (raw_stamped_path.size() == 2)
                    renew_time = time_stamps[4];


                bot_traj_planner::TrajectoryPtr current_traj = std::move(next_traj);

                //ideally, this will execute the trajectory past the second waypoint (the first point after current);
                std::future<ErrorInfo> future_ret = std::async(std::launch::async, [&]() {
                    return executeCurrentTrajectory(current_traj, renew_time);
                });
                // then start the consecutive mode

                //then compute the next trajectory// parallel computing
                // in this case, we believe that the given first waypoint is reached
                buffer.pop_front();

                Eigen::VectorXd forcast_point = current_traj->computePositionAt(renew_time);
                Eigen::VectorXd forcast_vel = current_traj->computeVelocityAt(renew_time);
                Eigen::VectorXd forcast_acc = current_traj->computeAccelerationAt(renew_time);

                //re-fetch raw stamped path
                raw_stamped_path.clear();
                Eigen::VectorXd data_stamped(forcast_point.size() + 1);
                data_stamped << 0., forcast_point;
                raw_stamped_path.emplace_back(data_stamped);

                for (auto iter = buffer.cbegin(); iter != buffer.cend(); ++iter) {
                    double d = (double) std::distance(buffer.cbegin(), iter) + 1;
                    data_stamped << 1. / m_param->servo_rate * d, iter->joint_data;
                    raw_stamped_path.emplace_back(data_stamped);
                    if (raw_stamped_path.size() > 50)
                        break;
                }

                next_traj = std::make_unique<bot_planner::TrajectoryIterativeSpline>(true,
                                                                                     false, false);
                next_traj->init(raw_stamped_path, std::vector<Eigen::VectorXd> {forcast_vel, forcast_acc},
                                std::vector<Eigen::VectorXd> {Eigen::VectorXd::Zero(ML.rows()),
                                                              Eigen::VectorXd::Zero(ML.rows())},
                                ML.col(2) * m_param->maxVelFactor,
                                ML.col(3) * m_param->maxAccFactor,
                                ML.col(4) * m_param->maxJerkFactor,
                                m_param->considerLimits);

                //wait for last send finished
                execute_ret = future_ret.get();

                if (next_traj->getDuration() < std::numeric_limits<double>::epsilon()) {
                    execute_ret = ErrorInfo::OK();
                    break;
                }
                if (execute_ret.error_code() == ErrorCode::RobotConnectFailed ||
                    execute_ret.error_code() == ErrorCode::TrajectoryPlanningFailed) {
                    PLOGE << "quit servo mode";
                    break;
                }
            }
        }
    }
}