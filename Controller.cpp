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
 *
 * @file Controller.cpp
 * @author T. Kunz
 * Modified by Jim Mainprice
 */

#include "Controller.h"
#include <dynamics/SkeletonDynamics.h>
#include <planning/Trajectory.h>
#include <kinematics/Dof.h>
#include <iostream>
#include <dynamics/BodyNodeDynamics.h>
//#include <utils/UtilsMath.h>

using namespace std;
using namespace Eigen;

namespace planning {

Controller::Controller(dynamics::SkeletonDynamics* _skel, const vector<int> &_actuatedDofs,
                       const VectorXd &_kP, const VectorXd &_kD, const VectorXd &_kI, const vector<int> &_ankleDofs, const VectorXd &_anklePGains, const VectorXd &_ankleDGains) :
    mSkel(_skel),
    mKp(_kP.asDiagonal()),
    mKd(_kD.asDiagonal()),
    mKi(_kI.asDiagonal()),
    mAnkleDofs(_ankleDofs),
    mAnklePGains(_anklePGains),
    mAnkleDGains(_ankleDGains),
    mTrajectory(NULL)
{
    const int nDof = mSkel->getNumDofs();

    mSelectionMatrix = MatrixXd::Zero(nDof, nDof);
    for (int i = 0; i < _actuatedDofs.size(); i++) {
        mSelectionMatrix(_actuatedDofs[i], _actuatedDofs[i]) = 1.0;
    }

    mDesiredDofs.resize(nDof);
    for (int i = 0; i < nDof; i++) {
        mDesiredDofs[i] = mSkel->getDof(i)->getValue();
    }

    Vector3d com = mSkel->getWorldCOM();
    double cop = 0.0;
    mPreOffset = com[0] - cop;

    m_use_balance = true;

    mError = VectorXd::Zero(mSkel->getNumDofs());
}


Controller::~Controller(){
    //clean up to avoid mem. leaks
    delete mTrajectory;
}

void Controller::setTrajectory(const Trajectory* _trajectory, double _startTime, const std::vector<int> &_dofs)
{
    mTrajectoryDofs = _dofs;
    mTrajectory = _trajectory;
    mStartTime = _startTime;
}


VectorXd Controller::getTorques(const VectorXd& _dof, const VectorXd& _dofVel, double _time)
{
    Eigen::VectorXd desiredDofVels = VectorXd::Zero(mSkel->getNumDofs());

    if(mTrajectory && _time - mStartTime >= 0.0 & _time - mStartTime <= mTrajectory->getDuration()) {
        for(unsigned int i = 0; i < mTrajectoryDofs.size(); i++) {
            mDesiredDofs[mTrajectoryDofs[i]] = mTrajectory->getPosition(_time - mStartTime)[i];
            desiredDofVels[mTrajectoryDofs[i]] = mTrajectory->getVelocity(_time - mStartTime)[i];
        }
    }

    VectorXd torques;
    const double mTimestep = 0.001;

    // SPD controller
    // J. Tan, K. Liu, G. Turk. Stable Proportional-Derivative Controllers. IEEE Computer Graphics and Applications, Vol. 31, No. 4, pp 34-44, 2011.
    MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
    VectorXd p = -mKp * (_dof - mDesiredDofs + _dofVel * mTimestep);
    VectorXd d = -mKd * (_dofVel - desiredDofVels);
    VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d);
    torques = p + d - mKd * qddot * mTimestep;

    // ankle strategy for sagital plane
    Vector3d com = mSkel->getWorldCOM();
    double cop = 0.0;
    double offset = com[0] - cop;

    //cout << "offset : " << offset << endl;

    if( true /*m_use_balance || offset > 0.03*/ )
    {
        for(unsigned int i = 0; i < mAnkleDofs.size(); i++)
        {
            //cout << "torques[mAnkleDofs[" << i << " ] : " << torques[mAnkleDofs[i]] << endl;
            torques[mAnkleDofs[i]] += mAnklePGains[i] * offset + mAnkleDGains[i] * (offset - mPreOffset) / mTimestep;
        }

        mPreOffset = offset;
    }

    return mSelectionMatrix * torques;
}

/**
VectorXd Controller::getTorques(const VectorXd& _dof, const VectorXd& _dofVel, double _time) {

    Eigen::VectorXd desiredDofVels = VectorXd::Zero(mSkel->getNumDofs());

//    if(mTrajectory && _time - mStartTime >= 0.0 & _time - mStartTime <= mTrajectory->getDuration()) {
//        for(unsigned int i = 0; i < mTrajectoryDofs.size(); i++) {
//            mDesiredDofs[mTrajectoryDofs[i]] = mTrajectory->getPosition(_time - mStartTime)[i];
//            desiredDofVels[mTrajectoryDofs[i]] = mTrajectory->getVelocity(_time - mStartTime)[i];
//        }
//    }

    VectorXd torques;
    const double mTimestep = 0.001;

    mError +=  mDesiredDofs -_dof;

    // SPD controller
    // J. Tan, K. Liu, G. Turk. Stable Proportional-Derivative Controllers. IEEE Computer Graphics and Applications, Vol. 31, No. 4, pp 34-44, 2011.
    MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
    VectorXd P = mKp * (mDesiredDofs - _dof + _dofVel *  mTimestep);
    VectorXd D = mKd * (desiredDofVels - _dofVel );
//    VectorXd I = mKi * ( mError );
//    cout << "P : " << P.transpose() << endl;
//    cout << "D : " << D.transpose() << endl;
//    cout << "I : " << I.transpose() << endl;
    VectorXd qddot = invM * (-mSkel->getCombinedVector() + P + D );
    torques = P + D - mKd * qddot * mTimestep;
//    torques = P + D + I;

    for(unsigned int i = 0; i < mAnkleDofs.size(); i++) {
        cout << "Error on : " << i << " = " << _dof[mAnkleDofs[i]] - mDesiredDofs[mAnkleDofs[i]] << endl;
    }

    if( use_balance )
    {
        // ankle strategy for sagital plane
        Vector3d com = mSkel->getWorldCOM();
        double cop = 0.0;
        double offset = com[0] - cop;

        for(unsigned int i = 0; i < mAnkleDofs.size(); i++) {
            torques[mAnkleDofs[i]] = mAnklePGains[i] * offset + mAnkleDGains[i] * (offset - mPreOffset) / mTimestep;

        }

        mPreOffset = offset;
    }
   

    return mSelectionMatrix * torques;
}
*/
}
