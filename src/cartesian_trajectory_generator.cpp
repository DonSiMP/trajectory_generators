#include <trajectory_generators/cartesian_trajectory_generator.h>

// Create a new instance of CartesianTrajectory. The PreOperational argument forces you to call configureHook().
CartesianTrajectory::CartesianTrajectory(const string& name): RTT::TaskContext(name, PreOperational)
{
   // Add RTT ports.
   this->addPort("trajectory_waypoint_vector", inPort_WaypointVector);
   this->addPort("desired_trajectory_pos", outPort_DesiredPos);
   this->addPort("desired_trajectory_vel", outPort_DesiredVel);
   this->addPort("desired_trajectory_acc", outPort_DesiredAcc);

   // Add RTT properties.
   this->addProperty("frame_id", frame_id).doc("reference frame for trajectory waypoints");
   this->addProperty("tranform_waypoints_to_new_frame", transform_points).doc("transform waypoints to new reference frame");
   this->addProperty("task_space_vel_limit", task_space_vel_limit).doc("max allowable end effector velocity");
   this->addProperty("task_space_acc_limit", task_space_acc_limit).doc("max allowable end effector acceleration");
   this->addProperty("task_trajectory_corner_radius", task_trajectory_corner_radius).doc("radius for path roundness");
   this->addProperty("task_trajectory_equivalent_radius", task_trajectory_equivalent_radius).doc("equivalent radius for path roundness");
   this->addProperty("task_space_velocity_profile", task_space_velocity_profile).doc("name of the velocity profile");
}

// Configure this component.
bool CartesianTrajectory::configureHook()
{
   // Check if ports are connected.
   if(!inPort_WaypointVector.connected()){
      RTT::log(RTT::Error) << "inPort_WaypointVector is not connected!" << RTT::endlog();
      return false;
   }
   if(!outPort_DesiredPos.connected())
      RTT::log(RTT::Warning) << "outPort_DesiredPos is not connected." << RTT::endlog();
   
   if(!outPort_DesiredVel.connected())
      RTT::log(RTT::Warning) << "outPort_DesiredVel is not connected." << RTT::endlog();
   
   if(!outPort_DesiredAcc.connected())
      RTT::log(RTT::Warning) << "outPort_DesiredAcc is not connected." << RTT::endlog();
   
   // Parse parameters.
   rtt_ros_tools::getTaskSpaceVelLim(task_space_vel_limit, this);
   rtt_ros_tools::getTaskSpaceAccLim(task_space_acc_limit, this);
   rtt_ros_tools::getTaskTrajectoryCornerRadius(task_trajectory_corner_radius, this);
   rtt_ros_tools::getTaskTrajectoryEquivalentRadius(task_trajectory_equivalent_radius, this);
   rtt_ros_tools::getTaskSpaceVelProfile(vel_profile, this);
   rtt_ros_tools::transformWaypointsToNewFrame(transform_points, this);
   if(transform_points) rtt_ros_tools::getFrameID(frame_id, this);
   
   // Tell the update loop we need to compute a trajectory.
   trajectory_computed = false;

   return true;
}

// Start this component.
bool CartesianTrajectory::startHook(){
   
   return true;
}

// Run update loop.
void CartesianTrajectory::updateHook(){
   
   // If a waypoint array is recieved, than compute a trajectory.
   if(!trajectory_computed && inPort_WaypointVector.read(waypoint_array) == RTT::NewData) 
      
      computeTrajectory();
   
   // If trajectory is computed, run the trajectory.
   if(trajectory_computed){
      if(current_trajec_time < comp_trajec->Duration()){
	 
	 // Get desired states for the current time.
	 current_pos = comp_trajec->Pos(current_trajec_time);
	 current_vel = comp_trajec->Vel(current_trajec_time);
	 current_acc = comp_trajec->Acc(current_trajec_time);
	 
	 tf::poseKDLToMsg(current_pos, current_pos_msg);
	 tf::twistKDLToMsg(current_vel, current_vel_msg);
	 tf::twistKDLToMsg(current_acc, current_acc_msg);

	 // Increase timer
	 current_trajec_time += this->getPeriod();
	 
	 // Write desired cartesian position.
	 outPort_DesiredPos.write(current_pos_msg);
	 
	 // Write desired cartesian velocity.
	 outPort_DesiredVel.write(current_vel_msg);
	 
	 // Write desired cartesian acceleration.
	 outPort_DesiredAcc.write(current_acc_msg);
      }
      // Signal that the trajectory is finished.
      else trajectory_computed = false; 
   }
}
   
bool CartesianTrajectory::computeTrajectory(){

   if(transform_points){
      
      waypoint_array.header.frame_id = frame_id;
      pose_stamped.header = waypoint_array.header;
      
      try{
	 for(int i=0; i<waypoint_array.poses.size(); ++i){
	    
	    pose_stamped.pose = waypoint_array.poses[i];
	    tf_listener.transformPose(frame_id, pose_stamped, pose_stamped);
	    waypoint_array.poses[i] = pose_stamped.pose;
	 }
      }catch(tf::TransformException tf_exception){
	 std::cout << tf_exception.what() << std::endl;
	 return false;
      }
   }
   try{
      // Rotates a frame over the existing single rotation axis formed by start and end rotation. If more than one rotational axis exist, an arbitrary one will be choosen.
      interpolator = new KDL::RotationalInterpolation_SingleAxis();
      
      // Trajectory_Composite implements a trajectory that is composed of underlying trajectoria. Call Add to add a trajectory.
      comp_trajec = new KDL::Trajectory_Composite();
      
      // The specification of a path, composed of way-points with rounded corners.
      path = new KDL::Path_RoundedComposite(task_trajectory_corner_radius, task_trajectory_equivalent_radius, interpolator);
      
      // Trapezoidal vel profile constructed from max_vel and max_acc.
      vel_profile = new KDL::VelocityProfile_Trap(task_space_vel_limit, task_space_acc_limit);
      
      // Convert waypoint array to vector of frames.
      for(int i=0; i<waypoint_array.poses.size(); i++){
	 
	 tf::poseMsgToKDL(waypoint_array.poses[i], frame);
	 
	 frame_waypoint_vector.push_back(frame);
      }
      if(frame_waypoint_vector.size() > 1){
	
         // Add each frame to the path.
         std::for_each(frame_waypoint_vector.begin(), frame_waypoint_vector.end(), [&](KDL::Frame waypoint) {path->Add(waypoint);});
	 
	 // Finish creating the path.
         path->Finish();

         // Configure velocity profile based on trajectory start position and end position.
         vel_profile->SetProfile(0, path->PathLength());

         // Trajectory_Segment combines a VelocityProfile and a Path into a trajectory.
         traject = new KDL::Trajectory_Segment(path, vel_profile);

         // Add trajectory segment to the composite trajectory.
         comp_trajec->Add(traject);
      }
      else comp_trajec->Add(new KDL::Trajectory_Segment(new KDL::Path_Point(frame), vel_profile));

      // Wait 0.5s at the end of the trajectory.
      comp_trajec->Add(new KDL::Trajectory_Stationary(0.5, frame));
   }
   // Catch errors.
   catch(KDL::Error& error){
      std::cout << "Planning was attempted with waypoints: " << std::endl;
      for(auto const& point : frame_waypoint_vector){std::cout << point << std::endl;}
      std::cout <<  error.Description() << std::endl;
      std::cout <<  error.GetType() << std::endl;
      return false;
   }
   // Set trajectory time to 0 before starting new trajectory.
   current_trajec_time = 0.0;
   
   // Tell update loop that a trajectory is computed.
   trajectory_computed = true;
   
   return true;
}
