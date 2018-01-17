#include <trajectory_generators/random_waypoint_array_generator.h>

// Create a new instance of RandomWaypointArrayGenerator. The PreOperational argument forces you to call configureHook().
RandomWaypointArrayGenerator::RandomWaypointArrayGenerator(const string &name) : RTT::TaskContext(name, PreOperational)
{
  // Add RTT port.
  this->addPort("trajectory_waypoint_vector", outPort_WaypointVector);
  
  // Add RTT properties.
  this->addProperty("robot_description", robot_description).doc("URDF or SDF");
  this->addProperty("root_link", root_link).doc("first link in KDL chain");
  this->addProperty("tip_link", tip_link).doc("last link in KDL chain");
  this->addProperty("frame_id", frame_id).doc("reference frame for coordinate system");
}

// Configure this component.
bool RandomWaypointArrayGenerator::configureHook()
{
    // Abort if port is not connected.
    if(!outPort_WaypointVector.connected()){
        RTT::log(RTT::Error) << "outPort_WaypointVector is not connected!" << RTT::endlog();
        return false;
    }
    // Get KDL chain.
    rtt_ros_tools::getChainFromURDF(kdl_chain, this);
    
    // Resize.
    joint_lower_lims.resize(kdl_chain.getNrOfJoints());
    joint_lower_lims.resize(kdl_chain.getNrOfJoints());
    rand_joint_angles.resize(kdl_chain.getNrOfJoints());
    
    // Parse parameters.
    rtt_ros_tools::getJointPoseLimsFromURDF(joint_lower_lims, joint_upper_lims, kdl_chain, this);
    rtt_ros_tools::getFrameID(frame_id, this);    
    
    // Create forward kinematics solver.
    fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));

    return true;
}

// Start this component.
bool RandomWaypointArrayGenerator::startHook()
{
    return true;
}

// Run update loop.
void RandomWaypointArrayGenerator::updateHook()
{
    // Generate a random number of waypoints. Arbitrary limit set to 10.
    std::uniform_int_distribution<int> points(1, 10);
    
    num_of_points = points(random);

    waypoint_vector.poses.resize(num_of_points);

    // For each trajectory waypoint:
    for(int i = 0; i<num_of_points; ++i)
    {
        // For each joint, generate a random angle within limits.
        for(int i = 0; i<joint_lower_lims.size(); ++i)
        {
            std::uniform_real_distribution<double> angles(joint_lower_lims[i], joint_upper_lims[i]);
            rand_joint_angles(i) = angles(random);
        }
        // solve forward kinematics to find the cartesian pose.
        fk_pos_solver->JntToCart(rand_joint_angles, goal_pose_frame);

	// Convert KDL frame to pose message.
	tf::poseKDLToMsg(goal_pose_frame, waypoint_vector.poses[i]);
    }
    // Set frame id.
    waypoint_vector.header.frame_id = frame_id;
    
    // Set time stamp.
    waypoint_vector.header.stamp = rtt_rosclock::rtt_now();
    
    // Send through port.
    outPort_WaypointVector.write(waypoint_vector);
}
