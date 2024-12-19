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

#pragma once

#include <pronto_quadruped/StanceEstimator.hpp>
#include <pronto_quadruped_commons/leg_bool_map.h>
#include "magnecko_msgs/msg/leg_state.hpp"
#include "gazebo_msgs/msg/contacts_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/rclcpp.hpp>

namespace pronto {
namespace quadruped {

class StanceEstimatorROS : public StanceEstimator {
public:
    StanceEstimatorROS(const rclcpp::Node::SharedPtr& node,
                       FeetContactForces& feet_forces);

    void getStanceGroundTruth(LegBoolMap &stance) {
      for(int leg_id  = 0; leg_id < _LEGS_COUNT; leg_id++) {
        // same as GAZEBO mode
        stance[leg_id] = this->getContactSensorState(leg_id);
      }
    };

    void delayContactDetection(const double &delay, std::vector<int>& counter, int &id, size_t& state, uint8_t last_state);
    void earlyContactDetection(const double &leg_swing_time_shortened, std::vector<int>& counter, int &id, size_t& state, uint8_t last_state, std::vector<bool> &falling_edge);
    void earlyContactDetectionP3d(double &threshold_z, size_t& state, int leg_id);

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<magnecko_msgs::msg::LegState>::SharedPtr legStateSubscription_;

    bool useSimulation_;
    uint8_t stance_adjust_timing_;

    bool stance_output_simulation_ground_truth_;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contactSensorFirstFootSubscription_;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contactSensorSecondFootSubscription_;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contactSensorThirdFootSubscription_;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contactSensorFourthFootSubscription_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr p3dFirstFootSubscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr p3dSecondFootSubscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr p3dThirdFootSubscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr p3dFourthFootSubscription_;

    double timestep_dt_ = 0.0025; // default value

    // variables to delay timing of CE
    const double stance_delay_sec_ = 0.2;  // seconds

    // TODO: get stace_topic_freq_ from param
    // const double stance_topic_freq_ = 333;   // Hz. frequency of topic
    const int stance_delay_ = stance_delay_sec_ / timestep_dt_;   // number of samples to wait
    std::vector<int> stance_delay_counter_ = {0, 0, 0, 0}; 

    // variables to advance timing of CE
    std::vector<float> p3dFirstFootZ_ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // newest measurement at index=0, lates measurement at index=9
    std::vector<float> p3dSecondFootZ_ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<float> p3dThirdFootZ_ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<float> p3dFourthFootZ_ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double p3dContactDetectionThresholdZ_ = 0.02;   // meters (mpc swing_heigt = 0.12m)

    const double leg_swing_time_sec_ = 0.525;  // seconds
    const double leg_swing_end_advance_sec_ = 0.2;  // seconds
    const int leg_swing_time_shortened_ = (leg_swing_time_sec_ - leg_swing_end_advance_sec_) / timestep_dt_;   // number of samples to wait
    std::vector<bool> falling_edge_contact_ = {false, false, false, false};


    
 
    size_t legIdMap(const LegID& leg){
        // convert from Pronto to urdf convention
        switch (leg)
        {
        case pronto::quadruped::LF:
            return 0;
        case pronto::quadruped::LH:
            return 3;
        case pronto::quadruped::RF:
            return 1;
        case pronto::quadruped::RH:
            return 2;
        }
    }
};
}  // namespace quadruped
}  // namespace pronto
