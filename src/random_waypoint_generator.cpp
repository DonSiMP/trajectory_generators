#include <trajectory_generators/random_waypoint_generator.h>

// Create a new instance of RandomWaypointGenerator. The PreOperational argument forces you to make a call to configureHook().
RandomWaypointGenerator::RandomWaypointGenerator(const string &name) : RTT::TaskContext(name, PreOperational)
{
  // Add RTT port.
  this->addPort("trajectory_goal_pose", outPort_TrajectoryGoalPose);
  
  // Add RTT properties.
  this->addProperty("robot_description", robot_description).doc("URDF or SDF");
  this->addProperty("root_link", root_link).doc("first link in KDL chain");
  this->addProperty("tip_link", tip_link).doc("last link in KDL chain");
  this->addProperty("frame_id", frame_id).doc("reference frame for coordinate system");
}

// Configure this component.
bool RandomWaypointGenerator::configureHook()
{
    // Abort if port is not connected.
    if(!outPort_TrajectoryGoalPose.connected()){
       RTT::log(RTT::Error) << "outPort_TrajectoryGoalPose is not connected!" << RTT::endlog();
       return false;
    }
    // Get KDL chain.
    rtt_ros_tools::getChainFromURDF(kdl_chain, this);
    
    // Resize.
    rand_joint_angles.resize(kdl_chain.getNrOfJoints());
    joint_lower_lims.resize(kdl_chain.getNrOfJoints());
    joint_upper_lims.resize(kdl_chain.getNrOfJoints());
    
    // Parse parameters.
    rtt_ros_tools::getJointPoseLimsFromURDF(joint_lower_lims, joint_upper_lims, kdl_chain, this);
    rtt_ros_tools::getFrameID(frame_id, this);
    
    // Create fk solver.
    fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));

    return true;
}

// Start this component.
bool RandomWaypointGenerator::startHook()
{
    return true;
}

// Run update loop.
void RandomWaypointGenerator::updateHook()
{
    // Generate random joint positions within joint limits.
    for(int i = 0; i<joint_lower_lims.size(); ++i)
    {
      std::uniform_real_distribution<double> angles(joint_lower_lims[i], joint_upper_lims[i]);
      rand_joint_angles(i) = angles(random);
    }
    // Solve forward kinematics to find the cartesian pose.
    fk_pos_solver->JntToCart(rand_joint_angles, trajectory_goal_frame);
    
    // Convert KDL frame to pose message.
    tf::poseKDLToMsg(trajectory_goal_frame, trajectory_goal_pose.pose);
    
    // Set frame id.
    trajectory_goal_pose.header.frame_id = frame_id;
    
    // Set time stamp.
    trajectory_goal_pose.header.stamp = rtt_rosclock::rtt_now();
    
    //send cartesian pose.
    outPort_TrajectoryGoalPose.write(trajectory_goal_pose);
}
