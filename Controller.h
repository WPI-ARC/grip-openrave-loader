/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#include <vector>
#include <Eigen/Core>

namespace dynamics { class SkeletonDynamics; }

namespace planning {

class Trajectory;

class Controller {

public:
    Controller(dynamics::SkeletonDynamics* _skel, const std::vector<int> &_actuatedDofs,
               const Eigen::VectorXd &_kP, const Eigen::VectorXd &_kD, const std::vector<int> &_ankleDofs, const Eigen::VectorXd &_anklePGains, const Eigen::VectorXd &_ankleDGains);
    virtual ~Controller();
    

    void setTrajectory(const Trajectory* _trajectory, double _startTime, const std::vector<int> &_dofs);

    // Returns zero torque for nonactuated DOFs
    Eigen::VectorXd getTorques(const Eigen::VectorXd& _dof, const Eigen::VectorXd& _dofVel, double _time);

protected: 
    Eigen::Vector3d evalAngMomentum(const Eigen::VectorXd& _dofVel);
    Eigen::VectorXd adjustAngMomentum(Eigen::VectorXd _deltaMomentum, Eigen::VectorXd _controlledAxis);

    dynamics::SkeletonDynamics* mSkel;
    std::vector<int> mTrajectoryDofs;
    Eigen::VectorXd mDesiredDofs;
    Eigen::MatrixXd mKp;
    Eigen::MatrixXd mKd;
    Eigen::MatrixXd mSelectionMatrix;
    const Trajectory* mTrajectory;
    double mStartTime;
    double mPreOffset;
    std::vector<int> mAnkleDofs;
    Eigen::VectorXd mAnklePGains;
    Eigen::VectorXd mAnkleDGains;
};

}
