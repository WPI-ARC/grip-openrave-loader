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

#include "executeFromFileTab.h"

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <iostream>

#include <collision/CollisionSkeleton.h>
#include <dynamics/SkeletonDynamics.h>
#include <dynamics/ContactDynamics.h>
#include <kinematics/ShapeBox.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <planning/PathPlanner.h>
#include <planning/PathShortener.h>
#include <planning/PathFollowingTrajectory.h>
#include "Controller.h"

#include <libxml/parser.h>
#include <tuple>
#include <functional>

template <class T>
bool convert_text_to_num(T& t,
                         const std::string& s,
                         std::ios_base& (*f)(std::ios_base&))
{
    std::istringstream iss(s);
    return !(iss >> f >> t).fail();
}

/// Define IDs for buttons
enum DynamicSimulationTabEvents {
    id_button_DoPlanning = 8345,
    id_button_RelocateObjects,
    id_button_SetStart,
    id_button_SetGoal,
    id_button_SetPredefStart,
    id_button_SetPredefGoal,
    id_button_ShowStart,
    id_button_ShowGoal,
    id_button_Grasping,
    id_button_OpenHand,
    id_button_CloseHand,
    id_button_LoadFile,
    id_button_PlayTraj,
    id_label_Inst,
    id_checkbox_showcollmesh,
};
using namespace std;

// Handler for events executeFromFileTab
BEGIN_EVENT_TABLE(executeFromFileTab, wxPanel)
EVT_COMMAND(id_button_LoadFile, wxEVT_COMMAND_BUTTON_CLICKED, executeFromFileTab::onButtonLoadFile)
EVT_COMMAND(id_button_PlayTraj, wxEVT_COMMAND_BUTTON_CLICKED, executeFromFileTab::onButtonPlayTraj)
EVT_CHECKBOX(id_checkbox_showcollmesh, executeFromFileTab::onCheckShowCollMesh)
END_EVENT_TABLE() 
IMPLEMENT_DYNAMIC_CLASS(executeFromFileTab, GRIPTab)


executeFromFileTab::executeFromFileTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, long style) :
    GRIPTab(parent, id, pos, size, style) {
    wxSizer* sizerFull = new wxBoxSizer(wxHORIZONTAL);

    // Create Static boxes (outline of your Tab)
    wxStaticBox* ss3Box = new wxStaticBox(this, -1, wxT("File"));
    
    // Create sizers for these static boxes
    wxStaticBoxSizer* ss3BoxS = new wxStaticBoxSizer(ss3Box, wxVERTICAL);

    // Execute from file
    ss3BoxS->Add(new wxButton(this, id_button_LoadFile, wxT("Load File")), 0, wxALL, 1);
    ss3BoxS->Add(new wxButton(this, id_button_PlayTraj, wxT("Play Trajectory")), 0, wxALL, 1);

    // Add the boxes to their respective sizers
    sizerFull->Add(ss3BoxS, 1, wxEXPAND | wxALL, 6);
    SetSizer(sizerFull);

    // Additional settings
    mController = NULL;

    mTrajId = 0;
}

/// Setup grasper when scene is loaded as well as populating arm's DoFs
void executeFromFileTab::GRIPEventSceneLoaded() {

    // Find robot and wheel
    for(int i = 0; i < mWorld->getNumSkeletons(); i++)
    {
        if(mWorld->getSkeleton(i)->getName() == "GolemHubo"){
            mRobot = (robotics::Robot*) mWorld->getSkeleton(i);
        }

        if(mWorld->getSkeleton(i)->getName() == "wheel"){
            mWheel = (robotics::Robot*) mWorld->getSkeleton(i);
        }
    }

    mWheel->setImmobileState(true);

    mActuatedDofs.resize( mRobot->getNumDofs() - 6 );
    for (unsigned int i = 0; i < mActuatedDofs.size(); i++) {
        mActuatedDofs[i] = i + 6;
    }

    // Define right arm nodes
//    const string armNodes[] = {"Body_RSP", "Body_RSR", "Body_RSY", "Body_REP", "Body_RWY", "Body_RWP"};
//    mArmDofs.resize(6);
//    for (int i = 0; i < mArmDofs.size(); i++) {
//        mArmDofs[i] = mRobot->getNode( armNodes[i].c_str())->getDof(0)->getSkelIndex();
//    }

    //Define palm effector name; Note: this is robot dependent!
    eeName = "Body_RWP";
}

void executeFromFileTab::printDofIndexes()
{
    if(mRobot == NULL){
        cout << "No robot in the scene" << endl;
        return;
    }

    for(int i=0;i<mRobot->getNumDofs();i++)
    {
        kinematics::Dof* dof = mRobot->getDof(i);
        cout << "Dof(" << i << ") : " << dof->getName();
        if(dof->getJoint()->getParentNode())
            cout << " , Parent : " << dof->getJoint()->getParentNode()->getName();
        if(dof->getJoint()->getChildNode())
            cout << " , Child : " << dof->getJoint()->getChildNode()->getName();
        cout << endl;
    }
}

/// Setup hubo configuration
void executeFromFileTab::setHuboConfiguration( Eigen::VectorXd& q, bool is_position ) {

    if(mRobot == NULL){
        cout << "No robot in the scene" << endl;
        return;
    }

    Eigen::VectorXd hubo_config(57);

    int or_indexes[] = {25,  0, 14, 13, 26,  2,  1, 16, 15,  4,
                         3, 18, 17,  6,  5, 20, 19,  8,  7, 22,
                        21, 10,  9, 24, 23, 12, 11, 44, 47, 53,
                        50, 56, 29, 32, 38, 35, 41, 42, 45, 36,
                        46, 54, 27, 30, 36, 48, 39, 28, 46, 52,
                        49, 55, 28, 31, 37, 49, 40 };

    for (int i = 0; i<57; i++) {
        hubo_config[i] = q[or_indexes[i]];
    }

    if( is_position )
    {
        hubo_config[7] += (21.19 * M_PI / 180.0); // lsr id : 7 = 13 - 6
        hubo_config[8] -= (21.19 * M_PI / 180.0); // rsr id : 8 = 14 - 6

        hubo_config[37-6] = -hubo_config[37-6]; // Thumbs
        hubo_config[42-6] = -hubo_config[42-6];
    }

    q = hubo_config;
}

struct robot_and_dof
{
    int nb_dofs;
    std::string robot_name;
    std::string type;
};

bool fct_sort( std::pair<int,robot_and_dof> a, std::pair<int,robot_and_dof> b)
{
    return a.first < b.first;
}

void executeFromFileTab::loadTrajecoryFromFile( std::string filename, openraveTrajectory& traj )
{
    cout << "-------------------------------------------" << endl;
    cout << " load file : " << filename << endl;

    xmlDocPtr doc;
    xmlNodePtr cur;
    xmlNodePtr root;
    xmlChar* tmp;

    doc = xmlParseFile(filename.c_str());
    if(doc==NULL)
    {
        cout << "Document not parsed successfully (doc==NULL)" << endl;
        return;
    }

    root = xmlDocGetRootElement(doc);
    if (root == NULL)
    {
        cout << "Document not parsed successfully" << endl;
        xmlFreeDoc(doc);
        return;
    }

    if (xmlStrcmp(root->name, xmlCharStrdup("trajectory")))
    {
        cout << "Document of the wrong type root node not trajectory" << endl;
        xmlFreeDoc(doc);
        return;
    }

    cur = root->xmlChildrenNode->next;

    //    while (cur != NULL)
    //    {
    //        cout << cur->name << endl;
    //        cur = cur->next->next;
    //    }

    if (xmlStrcmp(cur->name, xmlCharStrdup("configuration")))
    {
        cout << "Error : no node named configuration" << endl;
        xmlFreeDoc(doc);
        return;
    }

    std::vector< std::pair<int,robot_and_dof> > offsets;

    xmlNodePtr node =  cur->xmlChildrenNode->next;

    while( node != NULL )
    {
        //cout << xmlGetProp( node, xmlCharStrdup("name") ) << endl;
        robot_and_dof rd;

        offsets.push_back(std::make_pair(0,rd));

        tmp = xmlGetProp( node, xmlCharStrdup("offset") );
        if (tmp == NULL)
        {
            cout << "Error: no prop named offset" << endl;
            return;
        }
        convert_text_to_num<int>( offsets.back().first, (char*)tmp, std::dec );
        //cout << offsets.back().first << endl;

        tmp = xmlGetProp( node, xmlCharStrdup("dof") );
        if (tmp == NULL)
        {
            cout << "Error: no prop named offset" << endl;
            return;
        }
        convert_text_to_num<int>( offsets.back().second.nb_dofs, (char*)tmp, std::dec );
        //cout << offsets.back().second.nb_dofs << endl;

        std::stringstream ss( (char *)xmlGetProp( node, xmlCharStrdup("name") ) );
        std::string line;

        std::getline( ss, line, ' ' );
        offsets.back().second.type = line;
        //cout << offsets.back().second.type << endl;

        std::getline( ss, line, ' ' );
        offsets.back().second.robot_name = line;
        //cout << offsets.back().second.robot_name << endl;

        node = node->next->next;
    }

    std::sort( offsets.begin(), offsets.end(), fct_sort );

    // ------------------------------------------------

    cur = cur->next->next;

    if (xmlStrcmp(cur->name, xmlCharStrdup("data")))
    {
        cout << "Error : no node named data" << endl;
        xmlFreeDoc(doc);
        return;
    }

    tmp = xmlGetProp( cur, xmlCharStrdup("count") );
    if (tmp == NULL)
    {
        cout << "Error: no prop named count" << endl;
        return;
    }
    int count = 0;
    convert_text_to_num<int>( count, (char*)tmp, std::dec );
    //cout << count << endl;

    tmp = xmlNodeGetContent( cur );
    if (tmp == NULL)
    {
        cout << "Error: no prop named count" << endl;
        return;
    }

    std::string configuration( (char*)(tmp) );
    // cout << configuration << endl;
    std::stringstream ss( configuration );
    std::vector<double> values;
    std::string line;
    while( std::getline(ss,line,' ') )
    {
        double val;
        convert_text_to_num<double>( val, line, std::dec );
        values.push_back( val );
    }

    cout << "values.size() : " << values.size() << endl;

    xmlFreeDoc(doc);

    traj.positions.resize(count);
    traj.velocities.resize(count);
    traj.deltatime.resize(count);

    //cout << "count : " << count << endl;

    std::string robot_name = "Hubo";

    int ith_value=0;
    int configuration_offset=0;

    for(int i=0;i<count;i++)
    {
        for(int k=0;k<int(offsets.size());k++)
        {
            if( offsets[k].second.type != "deltatime" &&
                offsets[k].second.robot_name != robot_name )
            {
                ith_value += offsets[k].second.nb_dofs;
                continue;
            }

            int start = ith_value + offsets[k].first;
            int end = ith_value + offsets[k].first + offsets[k].second.nb_dofs;

            if( end > values.size() )
            {
                cout << " name : "  <<  offsets[k].second.robot_name << ", ith_value : " << ith_value << endl;
                cout << " type : " << offsets[k].second.type << endl;
                cout << " nb of dof : " << offsets[k].second.nb_dofs << endl;
                cout << " end : "  <<   end << endl;
                cout << "ERROR Reading trajectory" << endl;
                continue;
            }

            configuration_offset += offsets[k].second.nb_dofs;

            if( offsets[k].second.type == "joint_values" )
            {
                traj.positions[i].resize( offsets[k].second.nb_dofs );

                int l=0;
                for(int j=start;j<end;j++)
                {
                    traj.positions[i][l++] = values[j];
                }
                //cout << "position : start = " << start << " , end = " << end << endl;
                //cout << traj.positions[i].transpose() << endl;
            }

            if( offsets[k].second.type == "joint_velocities" )
            {
                traj.velocities[i].resize( offsets[k].second.nb_dofs );

                int l=0;
                for(int j=start;j<end;j++)
                {
                    traj.velocities[i][l++] = values[j];
                }
                //cout << "velocities : start = " << start << " , end = " << end << endl;
            }

            if( offsets[k].second.type == "deltatime" )
            {
                int l=0;
                for(int j=start;j<end;j++)
                {
                    traj.deltatime[i] = values[l++];
                }

                ith_value += configuration_offset;
                configuration_offset = 0;
                //cout << "deltatime : start = " << start << " , end = " << end << endl;
            }
        }
    }
}

void executeFromFileTab::setHuboJointIndicies()
{
    for(int i=0;i<int(mTrajs.size());i++)
    {
        for(int j=0;j<int(mTrajs[i].positions.size());j++)
        {
            setHuboConfiguration( mTrajs[i].positions[j], true );
        }

        for(int j=0;j<int(mTrajs[i].velocities.size());j++)
        {
            setHuboConfiguration( mTrajs[i].velocities[j], false );
        }
    }
}

void executeFromFileTab::setPath()
{
    mPath.clear();

    std::vector<int> traj_indexes(5);
    traj_indexes[0] = 0;
    traj_indexes[1] = 1;
    traj_indexes[2] = 2;
    traj_indexes[3] = 1;
    traj_indexes[4] = 3;

    for(int i=0;i<int(traj_indexes.size());i++)
    {
        for(int j=0;j<int(mTrajs[traj_indexes[i]].positions.size());j++)
        {
            mPath.push_back( mTrajs[traj_indexes[i]].positions[j] );
        }
    }
}

void executeFromFileTab::setTrajectory()
{
    if(mController != NULL){
        delete mController;
    }

    // Define PD controller gains
    Eigen::VectorXd kI = 100.0 * Eigen::VectorXd::Ones( mRobot->getNumDofs() );
    Eigen::VectorXd kP = 500.0 * Eigen::VectorXd::Ones( mRobot->getNumDofs() );
    Eigen::VectorXd kD = 100.0 * Eigen::VectorXd::Ones( mRobot->getNumDofs() );

    // Define gains for the ankle PD
    std::vector<int> ankleDofs(2);
    ankleDofs[0] = 27;
    ankleDofs[1] = 28;
    // Define gains for the ankle PD
    const Eigen::VectorXd anklePGains = -1000.0 * Eigen::VectorXd::Ones(2);
    const Eigen::VectorXd ankleDGains = -200.0 * Eigen::VectorXd::Ones(2);

    // Tweeked PID gains
    // const Eigen::VectorXd anklePGains = -100.0 * Eigen::VectorXd::Ones(2);
    // const Eigen::VectorXd ankleDGains = -50.0 * Eigen::VectorXd::Ones(2);

    // Update robot's pose
    mRobot->setConfig( mActuatedDofs, mTrajs[0].positions[0] );

    // Create controller
    mController = new planning::Controller( mRobot, mActuatedDofs, kP, kD, ankleDofs, anklePGains, ankleDGains );

    // CHECK
    //cout << "Offline Plan Size: " << path.size() << endl;
    mRobot->update();

    // Create trajectory; no need to shorten path here
    const Eigen::VectorXd maxVelocity = 0.6 * Eigen::VectorXd::Ones( mActuatedDofs.size() );
    const Eigen::VectorXd maxAcceleration = 0.6 * Eigen::VectorXd::Ones(mActuatedDofs.size() );
    planning::Trajectory* trajectory = new planning::PathFollowingTrajectory( mPath, maxVelocity, maxAcceleration );

    std::cout << "Trajectory duration: " << trajectory->getDuration() << endl;
    mController->setTrajectory( trajectory, 0, mActuatedDofs );

    printf("Controller time: %f \n", mWorld->mTime);
}

// Load from 4 files and show the start configuration
// of each trajectory when pushed again
void executeFromFileTab::onButtonLoadFile(wxCommandEvent &evt) {

    if( mRobot == NULL )
    {
        cout << "No robot in the scene" << endl;
        return;
    }

    Eigen::VectorXd q(57);

    if( mTrajs.empty() || mTrajId==0 )
    {
        //std::string dir = "/home/jmainpri/workspace/drc/wpi_openrave/hubo/matlab/";
        std::string dir = "../trajs/";

        mTrajs.clear();
        mTrajs.resize(4);

        loadTrajecoryFromFile( dir + "movetraj0.txt", mTrajs[0] );
        loadTrajecoryFromFile( dir + "movetraj1.txt", mTrajs[1] );
        loadTrajecoryFromFile( dir + "movetraj2.txt", mTrajs[2] );
        loadTrajecoryFromFile( dir + "movetraj3.txt", mTrajs[3] );

        setHuboJointIndicies();
        setPath();
        setTrajectory();

        cout << "Set start confiuration of traj 0 " << endl;
        q = mTrajs[0].positions[0];
        mTrajId++;
    }
    else if( mTrajId == 1 )
    {
        cout << "Set start confiuration of traj 1 " << endl;
        q = mTrajs[1].positions[0];
        mTrajId++;
    }
    else if( mTrajId == 2 )
    {
        cout << "Set start confiuration of traj 2 " << endl;
        q = mTrajs[2].positions[0];
        mTrajId++;
    }
    else if( mTrajId == 3 )
    {
        cout << "Set start confiuration of traj 3 " << endl;
        q = mTrajs[3].positions[0];
        mTrajId = 0;
    }

    mRobot->setConfig( mActuatedDofs, q );
    viewer->DrawGLScene();
}

// Plays the loaded path
void executeFromFileTab::onButtonPlayTraj(wxCommandEvent &evt)
{
    std::list<Eigen::VectorXd>::const_iterator it;

    for( it=mPath.begin(); it != mPath.end(); it++ )
    {
        mRobot->setConfig( mActuatedDofs, *it );
        viewer->DrawGLScene();
        usleep( 25000 );
    }
}

/// Handle event for drawing grasp markers
void executeFromFileTab::onCheckShowCollMesh(wxCommandEvent &evt) {
}


/// Before each simulation step we set the torques the controller applies to the joints and check for plan's accuracy
void executeFromFileTab::GRIPEventSimulationBeforeTimestep() {

    Eigen::VectorXd positionTorques = mController->getTorques( mRobot->getPose(), mRobot->getQDotVector(), mWorld->mTime );
    // section here to control the fingers for force-based grasping
    // instead of position-based grasping
    mRobot->setInternalForces( positionTorques );

    cout << "positionTorques : " << positionTorques.transpose() << endl;
    cout << "mRobot->getPose() : " << mRobot->getPose().transpose() << endl;
}

/// Handle simulation events after timestep
void executeFromFileTab::GRIPEventSimulationAfterTimestep() {
}

/// Handle simulation start events
void executeFromFileTab::GRIPEventSimulationStart() {
}

/// Store selected node in tree-view data as grasper's objective
void executeFromFileTab::GRIPStateChange() {

    if (!selectedTreeNode) {
        return;
    }
    switch (selectedTreeNode->dType) {
    case Return_Type_Object:
    case Return_Type_Robot:
        selectedNode = ((kinematics::Skeleton*)selectedTreeNode->data)->getRoot();
        break;
    case Return_Type_Node:
        selectedNode = (kinematics::BodyNode*)selectedTreeNode->data;
        break;
    default:
        fprintf(stderr, "someone else's problem.");
        assert(0);
        exit(1);
    }
}

/// Render grasp' markers such as grasping point
void executeFromFileTab::GRIPEventRender() {

    tuple<double,double,double> red = std::make_tuple (1, 0, 0 );

    if( mRobot )
    {
        Eigen::Matrix4d T = mRobot->getNode("Body_Hip")->getWorldTransform();
        // cout << "Body_Hip : " << endl << T << endl;
        drawAxesWithOrientation( T, 0.3, red );
    }

//    if( mWheel )
//    {
//        Eigen::Matrix4d T = mWheel->getNode("handle")->getWorldTransform();
//        //cout << "handle : " << endl << T << endl;
//        drawAxesWithOrientation( T, 0.3, red );
//    }

    drawAxes( Eigen::VectorXd::Zero(3), 0.3, red  );

    glFlush();
}

/// Method to draw XYZ axes
void executeFromFileTab::drawAxes(Eigen::VectorXd origin, double size, tuple<double,double,double> color){
    glBegin(GL_LINES);
    glColor3f(get<0>(color), get<1>(color), get<2>(color));
    glVertex3f(origin(0) - size, origin(1), origin(2));
    glVertex3f(origin(0) + size, origin(1), origin(2));

    //glColor3f(0, 0, 1);
    glVertex3f(origin(0), origin(1) - size, origin(2));
    glVertex3f(origin(0), origin(1) + size, origin(2));

    //glColor3f(0, 1, 0);
    glVertex3f(origin(0), origin(1), origin(2) - size);
    glVertex3f(origin(0), origin(1), origin(2) + size);
    glEnd();
}

/// Method to draw XYZ axes with proper orientation. Collaboration with Justin Smith
void executeFromFileTab::drawAxesWithOrientation(const Eigen::Matrix4d& transformation, double size, tuple<double,double,double> color) {
    Eigen::Matrix4d basis1up, basis1down, basis2up, basis2down;
    basis1up << size, 0.0, 0.0, 0,
            0.0, size, 0.0, 0,
            0.0, 0.0, size, 0,
            1.0, 1.0, 1.0, 1;

    basis1down << -size, 0.0, 0.0, 0,
            0.0, -size, 0.0, 0,
            0.0, 0.0, -size, 0,
            1.0, 1.0, 1.0, 1;

    basis2up = transformation * basis1up;
    basis2down = transformation * basis1down;


    glBegin(GL_LINES);
    glColor3f(get<0>(color), get<1>(color), get<2>(color));
    glVertex3f(basis2down(0, 0), basis2down(1, 0), basis2down(2, 0));
    glVertex3f(basis2up(0, 0), basis2up(1, 0), basis2up(2, 0));

    //glColor3f(0, 0, 1);
    glVertex3f(basis2down(0, 1), basis2down(1, 1), basis2down(2, 1));
    glVertex3f(basis2up(0, 1), basis2up(1, 1), basis2up(2, 1));

    //glColor3f(0, 1, 0);
    glVertex3f(basis2down(0, 2), basis2down(1, 2), basis2down(2, 2));
    glVertex3f(basis2up(0, 2), basis2up(1, 2), basis2up(2, 2));
    glEnd();
}

// Local Variables:
// c-basic-offset: 4
// End:
