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

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<magnecko_msgs::msg::LegState>::SharedPtr legStateSubscription_;

    bool stance_output_simulation_ground_truth_;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contactSensorFirstFootSubscription_;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contactSensorSecondFootSubscription_;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contactSensorThirdFootSubscription_;
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contactSensorFourthFootSubscription_;
 
    size_t legIdMap(const LegID& leg){
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
