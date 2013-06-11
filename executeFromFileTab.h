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

// Modified by Jim Mainprice

#ifndef __SAMPLE_MANIPULATION_TAB__
#define __SAMPLE_MANIPULATION_TAB__

//#include <robotics/Robot.h>
 #include <dynamics/SkeletonDynamics.h>

#include <Eigen/Core>

#include <vector>
#include <list>
#include <tuple>

#include <Tabs/GRIPTab.h>

#include <dart/planning/PathFollowingTrajectory.h>

namespace planning { class Controller; }

class openraveTrajectory
{
public:
    std::vector<Eigen::VectorXd> positions;
    std::vector<Eigen::VectorXd> velocities;
    std::vector<double> deltatime;
};

class executeFromFileTab : public GRIPTab
{
public:
  executeFromFileTab() {};
  executeFromFileTab(wxWindow * parent, wxWindowID id = -1,
	      const wxPoint & pos = wxDefaultPosition,
	      const wxSize & size = wxDefaultSize,
	      long style = wxTAB_TRAVERSAL);
  virtual ~executeFromFileTab(){};
  
  virtual void GRIPEventSimulationBeforeTimestep(); /**< Implement to apply forces before simulating a dynamic step */
  virtual void GRIPEventRender();
  virtual void GRIPEventSceneLoaded();
  virtual void GRIPEventSimulationAfterTimestep(); /**< Implement to save world states in simulation*/
  virtual void GRIPEventSimulationStart(); 
  virtual void GRIPStateChange();
  
  void onCheckShowCollMesh(wxCommandEvent &evt);

  void initScene();

  void printDofIndexes();
  void closeHuboHands( Eigen::VectorXd& q );
  void setHuboConfiguration( Eigen::VectorXd& q, bool is_position );

  void onButtonLoadFile(wxCommandEvent &evt);
  void onButtonPlayTraj(wxCommandEvent &evt);

  void loadTrajectoryFromFiles();
  void loadTrajectoryFromFile(std::string filename, openraveTrajectory& traj);
  void setHuboJointIndicies();
  void setTrajectory();
  void setPath();
  std::vector<int> mActuatedDofs;
  std::vector<openraveTrajectory> mTrajs;
  std::list<Eigen::VectorXd> mPath;
  int mTrajId;

  void drawAxes(Eigen::VectorXd origin, double size, tuple<double,double,double> color);
  void drawAxesWithOrientation(const Eigen::Matrix4d& transformation, double size, tuple<double,double,double> color);
  void drawAxesRGB( const Eigen::Matrix4d& transformation, double size );
  
  planning::Controller* mController;
//  planning::Grasper* grasper;
  wxCheckBox* checkShowCollMesh;
  kinematics::BodyNode* selectedNode;
  dynamics::SkeletonDynamics* mRobot;
  dynamics::SkeletonDynamics* mWheel;

  bool m_is_traj_set;
  planning::Trajectory* mTrajectory;
  
  std::string eeName;  
  Eigen::Vector3d currentGraspPoint;
  
  DECLARE_DYNAMIC_CLASS(manipulationTab)
  DECLARE_EVENT_TABLE()
};

#endif
