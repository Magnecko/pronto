/* Copyright (c) 2015-2021
 * Istituto Italiano di Tecnologia (IIT), University of Oxford
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

#include "pronto_quadruped/DataLogger.hpp"
#include "pronto_quadruped/StanceEstimatorBase.hpp"
#include <pronto_utils/SchmittTrigger.hpp>

// Eigen
#include <Eigen/Dense>

// iit-rbd
#include <iit/rbd/rbd.h>
#include <iit/rbd/utils.h>

// iit-commons
#include <pronto_quadruped_commons/declarations.h>
#include <pronto_quadruped_commons/feet_contact_forces.h>

// std
#include <memory>
#include <iostream>

namespace pronto {
namespace quadruped {

using pronto::quadruped::LF;
using pronto::quadruped::RF;
using pronto::quadruped::LH;
using pronto::quadruped::RH;

using iit::rbd::X;
using iit::rbd::Y;
using iit::rbd::Z;

struct GRFStat {
    uint64_t tic_stance;
    uint64_t tic_swing;
    Eigen::ArrayXd stance_cluster;
    Eigen::ArrayXd swing_cluster;
    uint32_t cluster_size;
};

struct GRFShortStat {
    double mu_swing;
    double sigma_swing;
    double pi_swing;
    double delta_swing;
    double M2_swing;
    uint64_t n_swing;

    double mu_stance;
    double sigma_stance;
    double pi_stance;
    double delta_stance;
    double M2_stance;
    uint64_t n_stance;
};


class StanceEstimator : public StanceEstimatorBase {
    // typedef to remove ugly and verbose traited templated code
public:
    enum class Mode {THRESHOLD = 0, HYSTERESIS, REGRESSION, MODE_SCHEDULE, GAZEBO};

public:
    StanceEstimator(FeetContactForces& feet_contact_forces,
                    double force_threshold = 50);

    StanceEstimator(FeetContactForces& feet_contact_forces,
                    const Mode& mode,
                    const std::vector<double>& beta = std::vector<double>(4, 0.25),
                    const double& force_threshold = 50,
                    const double& hysteresis_low = 50,
                    const double& hysteresis_high = 150);

    virtual ~StanceEstimator() = default;

    /**
     * @brief setJointStates
     * @param q
     * @param qd
     * @param tau
     * @param orient
     * @param qdd
     * @param xd
     * @param xdd
     * @param omega
     * @param omegad
     * @deprecated
     */
    void setJointStates(const JointState &q,
                        const JointState &qd,
                        const JointState &tau,
                        const Quaterniond &orient,
                        const JointState &qdd,
                        const Vector3d &xd,
                        const Vector3d &xdd,
                        const Vector3d &omega,
                        const Vector3d &omegad) override;

    void setJointStates(const uint64_t &nsec,
                        const JointState &q,
                        const JointState &qd,
                        const JointState &tau,
                        const Quaterniond &orient,
                        const JointState &qdd,
                        const Vector3d &xd,
                        const Vector3d &xdd,
                        const Vector3d &omega,
                        const Vector3d &omegad)  override;


    bool getStance(LegBoolMap &stance) override;

    bool getStance(LegBoolMap& stance,
                   LegScalarMap& stance_probability) override;


    LegVectorMap getGRF() override;

    bool getGRF(LegVectorMap& grf) override;

    void getGrf_W(LegVectorMap& leg_status);

    void getGrfDelta(LegDataMap<double>& grfDelta){
        grfDelta = grForceDelta;
    }

    virtual void getNormalizedGRF(Eigen::Vector4d& normgrf);

    bool isStance(LegID leg) const override;

    void updateStat(double sample,
                    bool is_stance,
                    int index);

    void setMode(const Mode& mode);

    StanceEstimator::Mode getMode();

    void setParams(const std::vector<double>& beta,
                   const double& force_threshold = 50,
                   const double& hysteresis_low = 50,
                   const double& hysteresis_high = 150,
                   const uint64_t &hysteresis_delay_low = 250,
                   const uint64_t &hysteresis_delay_high = 250);

    void setMagnetState(std::vector<size_t> state){
        magnetStates_ = state;
    };

    void setContactSensorState(uint8_t state, uint8_t index){
        contactSensorStates_[index] = state;
    };
    uint8_t getContactSensorState(uint8_t index){
        return contactSensorStates_[index];
    };
private:
    LegVectorMap grForce_W;
    LegVectorMap grForce_des;
    LegDataMap<double> grForceDelta;

    double force_threshold_;
    double falling_edge_threshold_;
    double rising_edge_threshold_;
    double falling_edge_delay_;
    double rising_edge_delay_;

    LegDataMap<SchmittTrigger> force_triggers_;

    std::vector<double> beta_;
    Eigen::Vector4d stance_weights_ = Eigen::Vector4d::Zero();

    GRFStat stat;
    GRFShortStat gss[4];

    std::vector<size_t> magnetStates_ = {1, 1, 1, 1};
    std::vector<size_t> contactSensorStates_ = {1, 1, 1, 1};

protected:
    FeetContactForces& feet_contact_forces_;
    Mode mode_;

    LegVectorMap grf_;
    LegBoolMap stance_;
    uint64_t nsec_; // time in nanoseconds

    // Joint States used to compute GRF and then the stance
    JointState q_;
    JointState qd_;
    JointState tau_;
    Quaterniond orient_;
    JointState qdd_;
    Vector3d xd_;
    Vector3d xdd_;
    Vector3d omega_;
    Vector3d omegad_;
};

} // quadruped
} // namespace pronto
