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

    auto stanceCallback = [this](magnecko_msgs::msg::LegState::SharedPtr msg) {
      std::vector<size_t> state = {0,0,0,0};
      for (size_t i = 0; i < 4; ++i){
        state[i] = msg->leg_states[this->legIdMap(LegID(i))];
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

    if(!node_->get_parameter(legodo_prefix + "stance_output_simulation_ground_truth", stance_output_simulation_ground_truth_)){
        RCLCPP_WARN(node_->get_logger(), "Could not read the stance_output_simulation_ground_truth from param server. Will display NO Ground truth contact stance.");
        stance_output_simulation_ground_truth_ = false;
    }
    if (stance_output_simulation_ground_truth_ || mode_ == Mode::GAZEBO){
      contactSensorFirstFootSubscription_ = node->create_subscription<gazebo_msgs::msg::ContactsState>("/contact_sensors/first_foot", 10, contactFirstFootCallback);
      contactSensorSecondFootSubscription_ = node->create_subscription<gazebo_msgs::msg::ContactsState>("/contact_sensors/second_foot", 10, contactSecondFootCallback);
      contactSensorThirdFootSubscription_ = node->create_subscription<gazebo_msgs::msg::ContactsState>("/contact_sensors/third_foot", 10, contactThirdFootCallback);
      contactSensorFourthFootSubscription_ = node->create_subscription<gazebo_msgs::msg::ContactsState>("/contact_sensors/fourth_foot", 10, contactFourthFootCallback);
      RCLCPP_INFO(node_->get_logger(), "Publishing contact stance ground truth from simulation to topic /contact_sensors/stance");
    } else{
      RCLCPP_WARN(node_->get_logger(), "Topic /contact_sensors/stance contains the same data as /stance i.e. not the ground truth from simulation");
    }

    setParams(beta, stance_threshold, hysteresis_low, hysteresis_high, stance_hysteresis_delay_low, stance_hysteresis_delay_high); 
}

}  // namespace quadruped
}  // namespace pronto
