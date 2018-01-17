#ifndef RANDOM_WAYPOINT_GENERATOR_H
#define RANDOM_WAYPOINT_GENERATOR_H

#include <rtt_ros_tools/rtt_ros_tools.h>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/Activity.hpp>
#include <rtt_rosclock/rtt_rosclock.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_kdl.h>

#include <random>
#include <memory>

class RandomWaypointGenerator : public RTT::TaskContext{

public:

       RandomWaypointGenerator(const string &name);

       bool configureHook();
       bool startHook();
       void updateHook();
       
       virtual ~RandomWaypointGenerator(){}

protected:

       RTT::OutputPort<geometry_msgs::PoseStamped> outPort_TrajectoryGoalPose;
      
       std::string robot_description, root_link, tip_link, frame_id;
       geometry_msgs::PoseStamped trajectory_goal_pose;
       KDL::Frame trajectory_goal_frame;
       KDL::Chain kdl_chain;
       std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver;
       std::uniform_real_distribution<double> angles;
       KDL::JntArray rand_joint_angles;
       std::vector<double> joint_lower_lims;
       std::vector<double> joint_upper_lims;
       std::random_device random;
};

ORO_LIST_COMPONENT_TYPE(RandomWaypointGenerator)

#endif 
