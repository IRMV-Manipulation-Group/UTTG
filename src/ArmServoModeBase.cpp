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

#include "imc/bot_servo/ArmServoModeBase.h"

#include <pthread.h>
#include <yaml-cpp/yaml.h>

#include <thread>
#include <utility>

#include "irmv/bot_common/log/log.h"
#include "imc/bot_servo/filter/filter_raw.hpp"
#include "imc/bot_traj_planner/trajectory_spline_iterative.hpp"
#include "irmv/bot_utils/loop_timer/loop_timer.hpp"
#include "irmv/bot_utils/multithread/thread_safe_container/thread_safe_deque.hpp"

namespace bot_servo {
StampedJoint::StampedJoint(const Eigen::VectorXd &joint) {
  joint_data = joint;
  stamp = std::chrono::steady_clock::now();
}

ArmServoModeBase::ArmServoModeBase(bot_planner::PlannerPtr pl,
                                   bot_executor::ExecutorPtr ext)
    : ext_(std::move(ext)), pl_(std::move(pl)), m_param(new ServoParameters()) {
  m_filter.reset(new FilterRaw<Eigen::VectorXd>());
  if (!ext_->isConnected())
    throw std::invalid_argument(
        "The executor must be connected to call this algorithm class");
}

ArmServoModeBase::ArmServoModeBase(bot_planner::PlannerPtr pl,
                                   GetCurrentJointValues_func_t getFunc,
                                   SendServo_func_t sendFunc)
    : pl_(std::move(pl)),
      m_getCurrentJointValues(std::move(getFunc)),
      m_param(new ServoParameters()),
      m_sendServo(std::move(sendFunc)) {
  ext_ = nullptr;
  m_filter.reset(new FilterRaw<Eigen::VectorXd>());
}

ArmServoModeBase::~ArmServoModeBase() {
  endServoMode();
  pl_.reset();
}

bool ArmServoModeBase::isServoHalt() { return buffer.empty(); }

bot_common::ErrorInfo ArmServoModeBase::setServoParameters(
    ServoParametersPtr param) {
  m_param = std::move(param);
  return bot_common::ErrorInfo::OK();
}

bot_common::ErrorInfo ArmServoModeBase::servoToPoint(const Eigen::VectorXd &q) {
  if (pl_ == nullptr) {
    return {bot_common::ErrorCode::Error, "Not initialized"};
  }
  auto val = pl_->getValidatorPtr();
  auto ret = val->isValid(q, false);
  if (ret.IsOK()) {
    // boost watch dog thread, when no watch dog thread is launched
    // when watchdog is already running, nothing happened;
    boostWatchDog();

    // filter
    m_filter->observe(q);
    auto updated = m_filter->update();

    if (updated.has_value()) {
      buffer.push_back(updated.value());
      buffer_condition.notify_one();
    }

    // boost servo, when no servo is launched
    // when servo is already running, nothing happened;
    boostServo();
  }
  return execute_ret;
}

std::pair<bot_common::ErrorInfo, Eigen::VectorXd> ArmServoModeBase::servoToPose(
    const Eigen::Isometry3d &p) {
  if (pl_ == nullptr) {
    return {
        bot_common::ErrorInfo{bot_common::ErrorCode::Error, "Not initialized"},
        Eigen::VectorXd{}};
  }
  Eigen::VectorXd reference;

  if (m_getCurrentJointValues)
    reference =
        buffer.empty() ? m_getCurrentJointValues() : buffer.back().joint_data;
  else if (ext_)
    reference = buffer.empty() ? ext_->getCurrentJointValues()
                               : buffer.back().joint_data;

  auto ret = pl_->getKinematicsPtr()->getNearestApproxIK(p, reference);
  if (ret.first < std::numeric_limits<double>::max()) {
    return {servoToPoint(ret.second), ret.second};
  }
  PLOGD
      << "No ik solutions inside limits found or the solution is in collision";
  return std::make_pair(
      bot_common::ErrorInfo{bot_common::ErrorCode::IKFailed,
                            "No ik solutions inside limits found or the "
                            "solution is in collision"},
      ret.second);
}

bot_common::ErrorInfo ArmServoModeBase::endServoMode() {
  startServo = false;
  buffer_condition.notify_all();

  // first stop send worker
  if (maintain_worker.joinable()) maintain_worker.join();

  if (watch_dog.joinable()) watch_dog.join();

  // in case of hang up
  if (servo_worker.joinable()) servo_worker.join();

  buffer.clear();

  return bot_common::ErrorInfo::OK();
}

void ArmServoModeBase::servoFrame() {
  using namespace bot_common;

  bool first_time = true;
  bot_common::ErrorInfo ret;

  while (true) {
    // Only exit if startServo is false and the buffer is empty
    if (!startServo && buffer.empty()) {
      PLOGD << "Now buffer is empty, buffer size is: " << buffer.size();
      break;
    }

    // to avoid busy wait
    std::unique_lock<std::mutex> lock(buffer_mutex);
    buffer_condition.wait(lock,
                          [this]() { return !buffer.empty() || !startServo; });

    // special handle on first point; must start at the first point;
    if (first_time) {
      // warning: this buffer pop_front method is blocking
      auto front = buffer.pop_front();

      auto current_traj =
          pl_->solvePTP(ret, send_data.get(), front.joint_data, false);

      // fist point invalid to achieve
      if (!ret.IsOK()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      } else {
        ret =
            executeCurrentTrajectory(current_traj, current_traj->getDuration());
        if (ret.error_code() == ErrorCode::RobotConnectFailed) {
          execute_ret = ret;
          PLOGW << "Robot disconnected, now quit the servo mode";
          break;
        }
      }
      // only reach the first point, we take fist time initialize finished
      first_time = false;
    } else {
      servoImpl(ret);
    }
  }
  startServo = false;
  if (ext_) execute_ret = ext_->waitForFinish();
  buffer.clear();
}

bot_common::ErrorInfo ArmServoModeBase::executeCurrentTrajectory(
    const bot_traj_planner::TrajectoryPtr &current_traj_, double renew_time) {
  double resample_delta = 1. / m_param->desired_rate;
  auto start = std::chrono::steady_clock::now();
  using namespace bot_common;

  if (current_traj_ == nullptr || current_traj_->empty()) {
    return {ErrorCode::TrajectoryPlanningFailed, "empty trajectory provided"};
  }

  const double duration = std::min(current_traj_->getDuration(),
                                   std::max(renew_time, resample_delta));
  if (duration <= std::numeric_limits<double>::epsilon()) {
    return ErrorInfo::OK();
  }

  size_t sample_count = std::ceil(duration / resample_delta);
  std::vector<double> time_instants;
  for (size_t sample = 0; sample <= sample_count; ++sample) {
    double local_t =
        std::min(duration, static_cast<double>(sample) * resample_delta);
    time_instants.emplace_back(local_t);
  }
  // special check last two points, in case of undiscriminating points
  if (time_instants.back() - *(time_instants.rbegin() + 1) < resample_delta) {
    time_instants.erase((time_instants.end() - 2));
  }

  // we do not send the first point
  time_instants.erase(time_instants.cbegin());

  Eigen::VectorXd sent;

  LoopTimer rate(1. / resample_delta);
  for (int i = 0; i < time_instants.size(); ++i) {
    sent = current_traj_->computePositionAt(time_instants[i]);

    send_data.set(sent);
    // only sleep when we can
    if (i < time_instants.size() - 1) {
      rate.sleep();
    }
  }
  return ErrorInfo::OK();
}

void ArmServoModeBase::servoSendWorker() {
  LoopTimer rate(m_param->desired_rate);
  while (startServo.load()) {
    // keep the current positions;
    if (m_sendServo)
      m_sendServo(send_data.get());
    else if (ext_)
      ext_->sendServo(send_data.get());

    rate.sleep();
  }
}

void ArmServoModeBase::extractStampedJointPath(
    std::vector<Eigen::VectorXd> &path) {
  path.clear();

  Eigen::VectorXd data_stamped(send_data.get().size() + 1);
  data_stamped << 0., send_data.get();
  path.emplace_back(data_stamped);

  for (auto iter = buffer.cbegin(); iter != buffer.cend(); ++iter) {
    double d = static_cast<double>(std::distance(buffer.cbegin(), iter)) + 1;
    data_stamped << 1. / m_param->servo_rate * d, iter->joint_data;
    path.emplace_back(data_stamped);
    if (path.size() > 50) break;
  }
}

void ArmServoModeBase::boostWatchDog() {
  checked_counter = 0;
  // special handle, only when we encounter the first valid point, then we start
  // servo mode
  if (!watch_dog.joinable()) {
    startServo = true;

    watch_dog = std::thread([this]() {
      while (true) {
        checked_counter++;
        // if (checked_counter > freq) {
        if (checked_counter > 3) {
          PLOGI << "over 3 times not called servoToPoint in the expected "
                   "frequency, stop all servo thread";
          break;
        }

        if (!execute_ret.IsOK()) {
          PLOGW << "execution failed, stop all servo thread";
          buffer.clear();
          break;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(size_t(1e3 / m_param->servo_rate)));
      }
      // stop worker thread
      if (servo_worker.joinable()) {
        startServo = false;
        buffer_condition.notify_all();
        if (maintain_worker.joinable()) maintain_worker.join();
        servo_worker.join();
      }
    });
  }

  if (!startServo && watch_dog.joinable()) watch_dog.join();
}

void ArmServoModeBase::boostServo() {
  // only start servo worker, when watchdog is launched;
  if (watch_dog.joinable() && !servo_worker.joinable()) {
    if (m_getCurrentJointValues)
      send_data.set(m_getCurrentJointValues());
    else
      send_data.set(ext_->getCurrentJointValues());

    if (ext_ || m_sendServo)
      maintain_worker = std::thread([this] { return this->servoSendWorker(); });

    servo_worker = std::thread([this] { return this->servoFrame(); });

    // priority set
    pthread_t handle = servo_worker.native_handle();

    sched_param sch_params{};
    sch_params.sched_priority = 20;

    if (pthread_setschedparam(handle, SCHED_RR, &sch_params) != 0) {
      PLOGW << "Failed to set thread priority.";
    }
  }
}

const std::string &ArmServoModeBase::getMethodName() { return method_name; }

const Eigen::VectorXd &ArmServoModeBase::queryCurrentCommand() {
  return send_data.get();
}
}  // namespace bot_servo
