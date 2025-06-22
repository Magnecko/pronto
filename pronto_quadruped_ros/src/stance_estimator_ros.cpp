/* Copyright (c) 2018-2019 University of Oxford
 * All rights reserved.
 *
 * Author: Marco Camurri (mcamurri@robots.ox.ac.uk)
 *
 * This file is part of pronto_quadruped,
 * a library for leg odometry on quadruped robots.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "pronto_quadruped_ros/stance_estimator_ros.hpp"

namespace pronto {
namespace quadruped {

StanceEstimatorROS::StanceEstimatorROS(const rclcpp::Node::SharedPtr& node,
                                       FeetContactForces &feet_forces) :
     StanceEstimator(feet_forces), node_(node)
{
    // get parameters for the leg odometry
    std::string legodo_prefix = "legodo.";

    // stance estimator parameters
    std::vector<double> beta;

    double hysteresis_low = 50;
    uint64_t stance_hysteresis_delay_low  = 0;
    uint64_t stance_hysteresis_delay_high = 0;
    int stance_hysteresis_delay_low_int = 0;
    int stance_hysteresis_delay_high_int = 0;
    double hysteresis_high = 50;
    double stance_threshold = 50;

    // get parameter if we run in simulation or on hardware
    useSimulation_ = node_->get_parameter("use_sim_time").as_bool();
    RCLCPP_INFO(node_->get_logger(), "useSimulation_: %d", useSimulation_);


    if (!node_->get_parameter(legodo_prefix + "stance_adjust_timing", this->stance_adjust_timing_)){
      RCLCPP_WARN(node_->get_logger(), "Could not read stance_adjust_timing from param server. Using default 0 i.e. no adjustment.");
    } else {
      RCLCPP_INFO(node_->get_logger(), "using stance_adjust_timing: %d", this->stance_adjust_timing_);
    }

    auto stanceCallback = [this](magnecko_msgs::msg::LegState::SharedPtr msg) {
      std::vector<size_t> state = {0,0,0,0};

      for (int i = 0; i < 4; ++i){
        //   LegId(i): urdf -> Pronto convention since i=0,1,2,3 is in urdf
        //   legIdMap(): Pronto -> urdf convention
        size_t state_i = msg->leg_states[this->legIdMap(LegID(i))];

        // TODO: use_sim_time to determin hardware or simulation
        // Important: only delayContactDetection() can be used when running on hardware
        switch (stance_adjust_timing_)
        {
        case 1:
          delayContactDetection(this->stance_delay_, this->stance_delay_counter_, i, state_i, this->getMagnetState(i));
          break;
        case 2:
          // earlyContactDetection(this->leg_swing_time_shortened_, this->stance_delay_counter_, i, state_i, this->getMagnetState(i), this->falling_edge_contact_);
          break;
        case 3:
          // earlyContactDetectionP3d(this->p3dContactDetectionThresholdZ_, state_i, i);
          break;
        
        default:
          break;
        }

        state[i] = state_i;
      }
      this->setMagnetState(state);
    };

    auto contactFirstFootCallback = [this](gazebo_msgs::msg::ContactsState msg) {
      uint8_t state = 0;
      if (!msg.states.empty()){
          state = 1;
      }
      this->setContactSensorState(state, 0);
    };

    auto contactSecondFootCallback = [this](gazebo_msgs::msg::ContactsState msg) {
      uint8_t state = 0;
      if (!msg.states.empty()){
          state = 1;
      }
      this->setContactSensorState(state, 2);
    };

    auto contactThirdFootCallback = [this](gazebo_msgs::msg::ContactsState msg) {
      uint8_t state = 0;
      if (!msg.states.empty()){
          state = 1;
      }
      this->setContactSensorState(state, 3);
    };

    auto contactFourthFootCallback = [this](gazebo_msgs::msg::ContactsState msg) {
      uint8_t state = 0;
      if (!msg.states.empty()){
          state = 1;
      }
      this->setContactSensorState(state, 1);
    };

    auto p3dFirstFootCallback = [this](nav_msgs::msg::Odometry msg) {
      // function needs to be adjusted for walking on other surfaces than ground e.g. wall, ceiling
      
      // rotate right 1 step
      rotate(this->p3dFirstFootZ_.begin(), this->p3dFirstFootZ_.begin()+this->p3dFirstFootZ_.size()-1, this->p3dFirstFootZ_.end());
      this->p3dFirstFootZ_[0] = msg.pose.pose.position.z; // distance from the ground
    };

    auto p3dSecondFootCallback = [this](nav_msgs::msg::Odometry msg) {
      // function needs to be adjusted for walking on other surfaces than ground e.g. wall, ceiling
      // rotate right 1 step
      rotate(this->p3dSecondFootZ_.begin(), this->p3dSecondFootZ_.begin()+this->p3dSecondFootZ_.size()-1, this->p3dSecondFootZ_.end());
      this->p3dSecondFootZ_[0] = msg.pose.pose.position.z; // distance from the ground
    };

    auto p3dThirdFootCallback = [this](nav_msgs::msg::Odometry msg) {
      // function needs to be adjusted for walking on other surfaces than ground e.g. wall, ceiling
      
      // rotate right 1 step
      rotate(this->p3dThirdFootZ_.begin(), this->p3dThirdFootZ_.begin()+this->p3dThirdFootZ_.size()-1, this->p3dThirdFootZ_.end());
      this->p3dThirdFootZ_[0] = msg.pose.pose.position.z; // distance from the ground
    };

    auto p3dFourthFootCallback = [this](nav_msgs::msg::Odometry msg) {
      // function needs to be adjusted for walking on other surfaces than ground e.g. wall, ceiling
      
      // rotate right 1 step
      rotate(this->p3dFourthFootZ_.begin(), this->p3dFourthFootZ_.begin()+this->p3dFourthFootZ_.size()-1, this->p3dFourthFootZ_.end());
      this->p3dFourthFootZ_[0] = msg.pose.pose.position.z; // distance from the ground
    };

    if (!node_->get_parameter("ins.timestep_dt", this->timestep_dt_)){
      RCLCPP_WARN(node_->get_logger(), "Could not read timestep_dt from param server. Using default timestep_dt with default 0.0025 sec.");
    } else {
      RCLCPP_INFO(node_->get_logger(), "using timestep_dt: %f", this->timestep_dt_);
    }

    int stance_mode;
    if(!node_->get_parameter(legodo_prefix + "stance_mode", stance_mode)){
        RCLCPP_WARN(node_->get_logger(), "Could not read the stance mode from param server. Using threshold with default 50 N.");
        setMode(Mode::THRESHOLD);
    } else if(stance_mode < 5){
      setMode(static_cast<StanceEstimator::Mode>(stance_mode));
    } else {
      RCLCPP_WARN(node_->get_logger(), "Invalid stance mode from param server. Using threshold with default 50 N.");
      setMode(Mode::THRESHOLD);
    }

    switch(mode_){
    case Mode::THRESHOLD:
      if(!node_->get_parameter(legodo_prefix + "stance_threshold", stance_threshold)){
        RCLCPP_WARN(node_->get_logger(), "Could not read the stance threshold from param server. Using default 50 N.");
      }
      break;
    case Mode::HYSTERESIS:
      if(!node_->get_parameter(legodo_prefix + "stance_hysteresis_low", hysteresis_low)){
        RCLCPP_WARN(node_->get_logger(), "Could not read the stance_hysteresis_low from param server. Using default 50 N.");
      }
      if(!node_->get_parameter(legodo_prefix + "stance_hysteresis_high", hysteresis_high)){
        RCLCPP_WARN(node_->get_logger(), "Could not read the stance_hysteresis_high from param server. Using default 50 N.");
      }
      if(!node_->get_parameter(legodo_prefix + "stance_hysteresis_delay_low", stance_hysteresis_delay_low_int)){
        RCLCPP_WARN(node_->get_logger(), "Could not read the stance_hysteresis_delay_low from param server. Using default 0 ns.");
      }
      if(!node_->get_parameter(legodo_prefix + "stance_hysteresis_delay_high", stance_hysteresis_delay_high_int)){
        RCLCPP_WARN(node_->get_logger(), "Could not read the stance_hysteresis_delay_high from param server. Using default 0 ns.");
      }
      stance_hysteresis_delay_low  = stance_hysteresis_delay_low_int;
      stance_hysteresis_delay_high = stance_hysteresis_delay_high_int;
      break;
    case Mode::REGRESSION:
      if(!node_->get_parameter(legodo_prefix + "stance_regression_beta", beta)){
        RCLCPP_WARN(node_->get_logger(), "Could not read the stance_regression_beta from param server. Setting mode to THRESHOLD with default value of 50 N.");
        setMode(Mode::THRESHOLD);
      }
      break;
    case Mode::MODE_SCHEDULE:
      legStateSubscription_ = node->create_subscription<magnecko_msgs::msg::LegState>("/leg_state_topic", 10, stanceCallback);
      break;
    }

    if(!node_->get_parameter(legodo_prefix + "stance_output_simulation_ground_truth", stance_output_simulation_ground_truth_) || !useSimulation_){
        RCLCPP_WARN(node_->get_logger(), "Could not read the stance_output_simulation_ground_truth from param server. Will display NO Ground truth contact stance.");
        stance_output_simulation_ground_truth_ = false;
    }
    if ((stance_output_simulation_ground_truth_ || mode_ == Mode::GAZEBO) && useSimulation_){
      contactSensorFirstFootSubscription_ = node->create_subscription<gazebo_msgs::msg::ContactsState>("/contact_sensors/first_foot", 10, contactFirstFootCallback);
      contactSensorSecondFootSubscription_ = node->create_subscription<gazebo_msgs::msg::ContactsState>("/contact_sensors/second_foot", 10, contactSecondFootCallback);
      contactSensorThirdFootSubscription_ = node->create_subscription<gazebo_msgs::msg::ContactsState>("/contact_sensors/third_foot", 10, contactThirdFootCallback);
      contactSensorFourthFootSubscription_ = node->create_subscription<gazebo_msgs::msg::ContactsState>("/contact_sensors/fourth_foot", 10, contactFourthFootCallback);

      p3dFirstFootSubscription_ = node->create_subscription<nav_msgs::msg::Odometry>("/p3d_first_foot/gazebo_ground_truth_odom", 10, p3dFirstFootCallback);
      p3dSecondFootSubscription_ = node->create_subscription<nav_msgs::msg::Odometry>("/p3d_second_foot/gazebo_ground_truth_odom", 10, p3dSecondFootCallback);
      p3dThirdFootSubscription_ = node->create_subscription<nav_msgs::msg::Odometry>("/p3d_third_foot/gazebo_ground_truth_odom", 10, p3dThirdFootCallback);
      p3dFourthFootSubscription_ = node->create_subscription<nav_msgs::msg::Odometry>("/p3d_fourth_foot/gazebo_ground_truth_odom", 10, p3dFourthFootCallback);

      RCLCPP_INFO(node_->get_logger(), "Publishing contact stance ground truth from simulation to topic /contact_sensors/stance");
    } else{
      RCLCPP_WARN(node_->get_logger(), "Topic /contact_sensors/stance contains the same data as /stance i.e. not the ground truth from simulation");
    }

    setParams(beta, stance_threshold, hysteresis_low, hysteresis_high, stance_hysteresis_delay_low, stance_hysteresis_delay_high); 
}

void StanceEstimatorROS::StanceEstimatorROS::delayContactDetection(const double &delay, std::vector<int> &counter, int &id, size_t &state, uint8_t last_state_bak) {
  /* change variable state only to introduce a delay */

  uint8_t last_state = this->getMagnetState(id);
  // check for rising edge (0 -> 1) a.k.a. new contact detected
  if (last_state == 0 && state > 0){
    ++this->stance_delay_counter_[id];

    // check if delay has passed
    if (this->stance_delay_counter_[id] >= this->stance_delay_sec_){
      state = 1;
      this->stance_delay_counter_[id] = 0;
      RCLCPP_INFO(node_->get_logger(), "delay foot %d by %f seconds", id, this->stance_delay_sec_);
    } else {
      state = 0;
    }
  }
};

void StanceEstimatorROS::StanceEstimatorROS::earlyContactDetection(const double &delay, std::vector<int>& counter, int &id, size_t& state, uint8_t last_state, std::vector<bool> &falling_edge){
  /* change variable state to contact detected after leg_swing_time_shortened_ samples after contact has been lost */

  double safety_factor = 1.0; // ensure detection of next falling edge after this step has been completed

  // check for falling edge (1 -> 0) a.k.a. contact lost
  if (last_state > 0 && state == 0){
    falling_edge[id] = true;   
  } 
  
  // check if contact should have been established
  if (counter[id] > (int)(this->leg_swing_time_sec_ / this->timestep_dt_ * safety_factor)) {
    counter[id] = 0;
    falling_edge[id] = false;
    RCLCPP_INFO_STREAM(node_->get_logger(), "reset leg " << id << "\n");
  }

  if (falling_edge[id]) { 
    ++counter[id];

    // check if delay has passed
    if (counter[id] >= delay){  
      state = 1;
    } else {
      state = 0;
    }    
  } 
}

void StanceEstimatorROS::StanceEstimatorROS::earlyContactDetectionP3d(double &threshold_z, size_t& state, int leg_id) {
  // trigger contact estimation early based on position of foot above the ground
  
  // This function must not be used when running on hardware
  if (!this->useSimulation_){
    RCLCPP_WARN(node_->get_logger(), "'earlyContactDetection()' must not be used on hardware (useSimulation_: %d). state is not changed.", useSimulation_);
    return;
  }

  std::vector<float> *vec;

  switch (leg_id)
  {
  case 0:
    vec = &(this->p3dFirstFootZ_);
    break;
  case 1:
    vec = &(this->p3dFourthFootZ_);
    break;
  case 2:
    vec = &(this->p3dSecondFootZ_);
    break;
  case 3:
    vec = &(this->p3dThirdFootZ_);
    break;
  }
  
  float change_in_z = (*vec)[0] - (*vec)[vec->size()-1];

  if ((*vec)[0] < threshold_z && change_in_z <= -1e-2){
    // overwrite state to assume contact
    state = 1;    
  }
  
}

}  // namespace quadruped
}  // namespace pronto
