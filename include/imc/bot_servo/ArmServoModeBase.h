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

#ifndef DUAL_ARM_APP_SERVOMODE_HPP
#define DUAL_ARM_APP_SERVOMODE_HPP

#include "imc/bot_planner/planner_base.hpp"
#include "imc/bot_traj_planner/trajectory_base.hpp"
#include "imc/bot_executor/ExecutorBase.hpp"
#include "irmv/bot_common/state/error_code.h"
#include <shared_mutex>
#include <thread>
#include <atomic>
#include "irmv/bot_utils/multithread/multi_thread_version.hpp"
#include "irmv/bot_utils/multithread/thread_safe_container/thread_safe_deque.hpp"

/**
 * \brief Base class for different types of filters.
 * \tparam T Type of data the filter processes.
 */
template<typename T>
class FilterBase;

/**
 * \brief Shared pointer type for FilterBase.
 * \tparam T Type of data the filter processes.
 */
template<typename T>
using FilterBasePtr = std::shared_ptr<FilterBase<T>>;

namespace bot_servo {

    /**
     * \brief Enumeration of different servo types.
     */
    enum ServoType {
        Trajectory,    ///< Servo type for trajectory following.
        Direct,        ///< Servo type for direct control.
        Interpolation, ///< Servo type for interpolation control.
    };

    /**
     * \brief Struct representing a joint state with a timestamp.
     */
    struct StampedJoint {
    public:
        /**
         * \brief Constructor initializing joint data.
         * \param joint Joint data as Eigen::VectorXd.
         */
        StampedJoint(const Eigen::VectorXd &joint);

    public:
        Eigen::VectorXd joint_data; ///< Joint data.
        std::chrono::steady_clock::time_point stamp; ///< Timestamp of the joint data.
    };

    /// Function type for getting current joint values.
    typedef std::function<Eigen::VectorXd()> GetCurrentJointValues_func_t;

    /// Function type for sending servo commands.
    typedef std::function<bot_common::ErrorInfo(const Eigen::VectorXd &)> SendServo_func_t;

    /**
     * \brief Struct representing servo parameters.
     */
    struct ServoParameters{
        double servo_rate = 30.; ///< Servo rate in Hz.
        double desired_rate = 200.; ///< Desired rate in Hz.
        double maxVelFactor = 1.0; ///< Maximum velocity factor.
        double maxAccFactor = 1.0; ///< Maximum acceleration factor.
        double maxJerkFactor = 1.0; ///< Maximum jerk factor.
        bool considerLimits = false; ///< Flag to consider joint limits.
    };

    /// Shared pointer type for ServoParameters.
    typedef std::shared_ptr<ServoParameters> ServoParametersPtr;

    /**
     * \brief Base class for arm servo modes.
     */
    class ArmServoModeBase {
    public:
        /**
         * \brief Constructor initializing with planner and executor.
         * \param pl Planner pointer.
         * \param ext Executor pointer.
         */
        ArmServoModeBase(bot_planner::PlannerPtr pl, bot_executor::ExecutorPtr ext);

        /**
         * \brief Constructor initializing with planner and function pointers.
         * \param pl Planner pointer.
         * \param getFunc Function to get current joint values.
         * \param sendFunc Function to send servo commands.
         */
        ArmServoModeBase(bot_planner::PlannerPtr pl, GetCurrentJointValues_func_t getFunc,
                         SendServo_func_t sendFunc = nullptr);

        /// Destructor.
        virtual ~ArmServoModeBase();

    protected:
        bot_planner::PlannerPtr pl_; ///< Planner pointer.
        bot_executor::ExecutorPtr ext_; ///< Executor pointer.

        std::atomic_bool startServo {false}; ///< Atomic flag to start servo.

        ThreadSafeDeque<StampedJoint> buffer; ///< Thread-safe buffer for joint states.

        std::mutex buffer_mutex; ///< Mutex for buffer.
        std::condition_variable buffer_condition; ///< Condition variable for buffer.

        bot_common::ErrorInfo execute_ret; ///< Execution return status.

        std::thread servo_worker, maintain_worker, watch_dog; ///< Threads for servo operations.

        int checked_counter = 0; ///< Counter for checks.

        FilterBasePtr<Eigen::VectorXd> m_filter; ///< Filter pointer.

        GetCurrentJointValues_func_t m_getCurrentJointValues; ///< Function to get current joint values.

        SendServo_func_t m_sendServo; ///< Function to send servo commands.

        MultiThreadVersion<Eigen::VectorXd> send_data; ///< Multi-threaded version of send data.

        std::string method_name; ///< Name of the method.

        ServoParametersPtr m_param; ///< Servo parameters pointer.

    public:
        /**
         * \brief Set servo parameters.
         * \param param Servo parameters pointer.
         * \return Error information.
         */
        bot_common::ErrorInfo setServoParameters(ServoParametersPtr param);

        /**
         * \brief Servo to a specific joint position.
         * \param q Joint position as Eigen::VectorXd.
         * \return Error information.
         */
        bot_common::ErrorInfo servoToPoint(const Eigen::VectorXd &q);

        /**
         * \brief Servo to a specific pose.
         * \param p Pose as Eigen::Isometry3d.
         * \return Pair of error information and joint position.
         */
        std::pair<bot_common::ErrorInfo, Eigen::VectorXd> servoToPose(const Eigen::Isometry3d &p);

        /**
         * \brief End the servo mode.
         * \return Error information.
         */
        bot_common::ErrorInfo endServoMode();

        /**
         * \brief Check if the servo is halted.
         * \return True if halted, false otherwise.
         */
        bool isServoHalt();

        /**
         * \brief Get the name of the method.
         * \return Method name as string.
         */
        const std::string &getMethodName();

        /**
         * \brief Query the current command.
         * \return Current command as Eigen::VectorXd.
         */
        const Eigen::VectorXd &queryCurrentCommand();

    protected:
        /// Servo frame function.
        void servoFrame();

        /**
         * \brief Servo implementation function.
         * \param ret Error information.
         */
        virtual void servoImpl(bot_common::ErrorInfo &ret) = 0;

        /// Worker function for sending servo commands.
        void servoSendWorker();

        /**
         * \brief Extract joint path from the buffer.
         * \param path Vector to store the joint path.
         */
        void extractStampedJointPath(std::vector<Eigen::VectorXd> &path);

        /**
         * \brief Execute the current trajectory.
         * \param current_traj Current trajectory pointer.
         * \param renew_time Time to renew the trajectory.
         * \return Error information.
         */
        bot_common::ErrorInfo executeCurrentTrajectory(const bot_traj_planner::TrajectoryPtr &current_traj, double renew_time);

        /// Boost the watchdog thread.
        void boostWatchDog();

        /// Boost the servo thread.
        void boostServo();
    };

    /// Shared pointer type for ArmServoModeBase.
    typedef std::shared_ptr<ArmServoModeBase> ArmServoModePtr;

    /// Unique pointer type for ArmServoModeBase.
    typedef std::unique_ptr<ArmServoModeBase> ArmServoModeUniquePtr;
}

#endif //DUAL_ARM_APP_SERVOMODE_HPP