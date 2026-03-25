// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2025 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
#define NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_

#include <Eigen/Dense>

#include <cstdint>
#include <string>
#include <algorithm>
#include <cmath>

#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/constraints.hpp"

#include "nav2_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

// Forward declaration of utils method, since utils.hpp can't be included here due
// to recursive inclusion.
namespace utils
{
float clamp(const float lower_bound, const float upper_bound, const float input);
}

/**
 * @class mppi::MotionModel
 * @brief Abstract motion model for modeling a vehicle
 */
class MotionModel
{
public:
  /**
    * @brief Constructor for mppi::MotionModel
    */
  MotionModel() = default;

  /**
    * @brief Destructor for mppi::MotionModel
    */
  virtual ~MotionModel() = default;

  /**
    * @brief Initialize motion model on bringup and set required variables
    * @param control_constraints Constraints on control
    * @param model_dt duration of a time step
    */
  void initialize(const models::ControlConstraints & control_constraints, float model_dt)
  {
    control_constraints_ = control_constraints;
    model_dt_ = model_dt;
  }

  /**
   * @brief With input velocities, find the vehicle's output velocities
   * @param state Contains control velocities to use to populate vehicle velocities
   */
  virtual void predict(models::State & state)
  {
    const bool is_holo = isHolonomic();
    float max_delta_vx = model_dt_ * control_constraints_.ax_max;
    float min_delta_vx = model_dt_ * control_constraints_.ax_min;
    float max_delta_vy = model_dt_ * control_constraints_.ay_max;
    float min_delta_vy = model_dt_ * control_constraints_.ay_min;
    float max_delta_wz = model_dt_ * control_constraints_.az_max;

    unsigned int n_cols = state.vx.cols();

    for (unsigned int i = 1; i < n_cols; i++) {
      auto lower_bound_vx = (state.vx.col(i - 1) >
        0).select(
        state.vx.col(i - 1) + min_delta_vx,
        state.vx.col(i - 1) - max_delta_vx);
      auto upper_bound_vx = (state.vx.col(i - 1) >
        0).select(
        state.vx.col(i - 1) + max_delta_vx,
        state.vx.col(i - 1) - min_delta_vx);

      state.cvx.col(i - 1) = state.cvx.col(i - 1)
        .cwiseMax(lower_bound_vx)
        .cwiseMin(upper_bound_vx);
      state.vx.col(i) = state.cvx.col(i - 1);

      state.cwz.col(i - 1) = state.cwz.col(i - 1)
        .cwiseMax(state.wz.col(i - 1) - max_delta_wz)
        .cwiseMin(state.wz.col(i - 1) + max_delta_wz);
      state.wz.col(i) = state.cwz.col(i - 1);

      if (is_holo) {
        auto lower_bound_vy = (state.vy.col(i - 1) >
          0).select(
          state.vy.col(i - 1) + min_delta_vy,
          state.vy.col(i - 1) - max_delta_vy);
        auto upper_bound_vy = (state.vy.col(i - 1) >
          0).select(
          state.vy.col(i - 1) + max_delta_vy,
          state.vy.col(i - 1) - min_delta_vy);
        state.cvy.col(i - 1) = state.cvy.col(i - 1)
          .cwiseMax(lower_bound_vy)
          .cwiseMin(upper_bound_vy);
        state.vy.col(i) = state.cvy.col(i - 1);
      }
    }
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  virtual bool isHolonomic() = 0;

  /**
   * @brief Whether the motion model samples a steering state instead of angular velocity
   * @return Bool if steering controls are used
   */
  virtual bool usesSteeringControls() const
  {
    return false;
  }

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  virtual void applyConstraints(models::ControlSequence & /*control_sequence*/) {}

  /**
   * @brief Apply stateful steering constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   * @param initial_steering_angle Steering angle at the current control cycle start
   */
  virtual void applySteeringConstraints(
    models::ControlSequence & /*control_sequence*/,
    float /*initial_steering_angle*/) const {}

protected:
  float model_dt_{0.0};
  models::ControlConstraints control_constraints_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f};
};

/**
 * @class mppi::AckermannMotionModel
 * @brief Ackermann motion model
 */
class AckermannMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::AckermannMotionModel
    */
  explicit AckermannMotionModel(ParametersHandler * param_handler, const std::string & name)
  {
    auto getParam = param_handler->getParamGetter(name + ".AckermannConstraints");
    getParam(min_turning_r_, "min_turning_r", 0.2);
    getParam(wheelbase_, "wheelbase", 1.0);
    getParam(delta_max_, "delta_max", 0.0);
    getParam(delta_dot_max_, "delta_dot_max", 0.0);
    use_steering_model_ = wheelbase_ > 0.0f && delta_max_ > 0.0f && delta_dot_max_ > 0.0f;
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }

  bool usesSteeringControls() const override
  {
    return use_steering_model_;
  }

  void predict(models::State & state) override
  {
    if (!use_steering_model_) {
      MotionModel::predict(state);
      return;
    }

    float max_delta_vx = model_dt_ * control_constraints_.ax_max;
    float min_delta_vx = model_dt_ * control_constraints_.ax_min;
    float max_delta_delta = model_dt_ * delta_dot_max_;

    unsigned int n_cols = state.vx.cols();

    for (unsigned int i = 1; i < n_cols; i++) {
      auto lower_bound_vx = (state.vx.col(i - 1) > 0).select(
        state.vx.col(i - 1) + min_delta_vx,
        state.vx.col(i - 1) - max_delta_vx);
      auto upper_bound_vx = (state.vx.col(i - 1) > 0).select(
        state.vx.col(i - 1) + max_delta_vx,
        state.vx.col(i - 1) - min_delta_vx);

      state.cvx.col(i - 1) = state.cvx.col(i - 1)
        .cwiseMax(lower_bound_vx)
        .cwiseMin(upper_bound_vx);
      state.vx.col(i) = state.cvx.col(i - 1);

      state.cdelta.col(i - 1) = state.cdelta.col(i - 1)
        .cwiseMax(state.delta.col(i - 1) - max_delta_delta)
        .cwiseMin(state.delta.col(i - 1) + max_delta_delta)
        .cwiseMax(-delta_max_)
        .cwiseMin(delta_max_);
      state.delta.col(i) = state.cdelta.col(i - 1);
      state.wz.col(i) =
        steeringAngleToAngularVelocity(state.vx.col(i).eval(), state.delta.col(i).eval());
      state.cwz.col(i - 1) = state.wz.col(i);
    }
  }

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  void applyConstraints(models::ControlSequence & control_sequence) override
  {
    if (use_steering_model_) {
      control_sequence.wz =
        steeringAngleToAngularVelocity(control_sequence.vx, control_sequence.delta);
      return;
    }

    const auto wz_constrained = control_sequence.vx.abs() / min_turning_r_;
    control_sequence.wz = control_sequence.wz
      .max((-wz_constrained))
      .min(wz_constrained);
  }

  void applySteeringConstraints(
    models::ControlSequence & control_sequence,
    float initial_steering_angle) const override
  {
    if (!use_steering_model_) {
      return;
    }

    const float max_delta_delta = model_dt_ * delta_dot_max_;
    float delta_last = std::clamp(initial_steering_angle, -delta_max_, delta_max_);

    for (unsigned int i = 0; i != control_sequence.delta.size(); i++) {
      float & delta_curr = control_sequence.delta(i);
      delta_curr = std::clamp(delta_curr, -delta_max_, delta_max_);
      delta_curr = std::clamp(delta_curr, delta_last - max_delta_delta, delta_last + max_delta_delta);
      delta_last = delta_curr;
      control_sequence.wz(i) = steeringAngleToAngularVelocity(control_sequence.vx(i), delta_curr);
    }
  }

  /**
   * @brief Get minimum turning radius of ackermann drive
   * @return Minimum turning radius
   */
  float getMinTurningRadius() {return min_turning_r_;}
  float getSteeringAngleMax() const {return delta_max_;}
  float getSteeringRateMax() const {return delta_dot_max_;}
  float inferSteeringAngle(float vx, float wz, float fallback_delta) const
  {
    if (!use_steering_model_) {
      return fallback_delta;
    }

    constexpr float min_speed_for_inference = 1e-3f;
    if (std::fabs(vx) < min_speed_for_inference) {
      return std::clamp(fallback_delta, -delta_max_, delta_max_);
    }

    const float inferred_delta = std::atan((wheelbase_ * wz) / vx);
    return std::clamp(inferred_delta, -delta_max_, delta_max_);
  }

  float steeringAngleToAngularVelocity(float vx, float delta) const
  {
    return use_steering_model_ ? (vx / wheelbase_) * std::tan(delta) : 0.0f;
  }

  Eigen::ArrayXf steeringAngleToAngularVelocity(
    const Eigen::ArrayXf & vx, const Eigen::ArrayXf & delta) const
  {
    if (!use_steering_model_) {
      return Eigen::ArrayXf::Zero(vx.size());
    }

    return ((vx / wheelbase_) * delta.tan()).eval();
  }

private:
  float min_turning_r_{0};
  float wheelbase_{1.0f};
  float delta_max_{0.0f};
  float delta_dot_max_{0.0f};
  bool use_steering_model_{false};
};

/**
 * @class mppi::DiffDriveMotionModel
 * @brief Differential drive motion model
 */
class DiffDriveMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::DiffDriveMotionModel
    */
  DiffDriveMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }
};

/**
 * @class mppi::OmniMotionModel
 * @brief Omnidirectional motion model
 */
class OmniMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::OmniMotionModel
    */
  OmniMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return true;
  }
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
