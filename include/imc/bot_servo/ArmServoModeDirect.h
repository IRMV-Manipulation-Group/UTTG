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

//
// Created by zyx on 24-11-4.
//

#ifndef UTTG_ARMSERVOMODEDIRECT_H
#define UTTG_ARMSERVOMODEDIRECT_H

#include "ArmServoModeBase.h"
#include "irmv/bot_common/alg_factory/algorithm_factory.h"

namespace bot_servo {
    /// \brief Constant for the name of the ArmServoModeDirect class.
    constexpr char ArmServoModeDirectName[] = "ArmServoModeDirectName";

    /**
     * \class ArmServoModeDirect
     * \brief A class for direct control of arm servo modes.
     *
     * This class inherits from ArmServoModeBase and provides direct control
     * mechanisms for arm servos.
     */
    class ArmServoModeDirect : public ArmServoModeBase {
    public:
        /**
         * \brief Constructor with planner and executor.
         * \param pl Planner pointer.
         * \param ext Executor pointer.
         */
        ArmServoModeDirect(bot_planner::PlannerPtr pl, bot_executor::ExecutorPtr ext);

        /**
         * \brief Constructor with planner and function pointers.
         * \param pl Planner pointer.
         * \param getFunc Function pointer to get current joint values.
         * \param sendFunc Function pointer to send servo commands (optional).
         */
        ArmServoModeDirect(bot_planner::PlannerPtr pl, GetCurrentJointValues_func_t getFunc,
                           SendServo_func_t sendFunc = nullptr);

        /// \brief Default destructor.
        ~ArmServoModeDirect() override = default;

    public:
        /**
         * \brief Static factory method to create an instance with planner and executor.
         * \param pl Planner pointer.
         * \param ext Executor pointer.
         * \return Unique pointer to ArmServoModeDirect instance.
         */
        static ArmServoModeUniquePtr create(bot_planner::PlannerPtr pl, bot_executor::ExecutorPtr ext);

        /**
         * \brief Static factory method to create an instance with planner and function pointers.
         * \param pl Planner pointer.
         * \param getFunc Function pointer to get current joint values.
         * \param sendFunc Function pointer to send servo commands (optional).
         * \return Unique pointer to ArmServoModeDirect instance.
         */
        static ArmServoModeUniquePtr create(bot_planner::PlannerPtr pl, GetCurrentJointValues_func_t getFunc,
                                            SendServo_func_t sendFunc = nullptr);

    protected:
        /**
         * \brief Implementation of the servo control.
         * \param ret Reference to ErrorInfo object to store error information.
         */
        void servoImpl(bot_common::ErrorInfo &ret) override;
    };

    /// \brief Macro to register the ArmServoModeDirect algorithm.
    inline bot_common::REGISTER_ALGORITHM(ArmServoModeBase, ArmServoModeDirectName, ArmServoModeDirect,
                                          bot_planner::PlannerPtr, GetCurrentJointValues_func_t,
                                          SendServo_func_t);
}

#endif //UTTG_ARMSERVOMODEDIRECT_H