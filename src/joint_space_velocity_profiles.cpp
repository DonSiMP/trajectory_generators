#include <trajectory_generators/joint_space_velocity_profiles.h>

namespace joint_space_vel_prof{

cubicPolynomial::cubicPolynomial(const KDL::Chain& kdl_chain, RTT::TaskContext* this_taskcontext)
{
   rtt_ros_tools::useJointVelLims(use_vel_limits, this_taskcontext);
   rtt_ros_tools::useJointAccLims(use_acc_limits, this_taskcontext);
   
   if(use_vel_limits) rtt_ros_tools::getJointVelLimsFromURDF(kdl_chain, max_vel, this_taskcontext);
   if(use_acc_limits) rtt_ros_tools::getJointAccLims(kdl_chain, max_acc, this_taskcontext);
   if(!use_vel_limits && !use_acc_limits) RTT::log(RTT::Error) << "Can not create cubic polynomial velocity profile. \n"
      "Either \"use_joint_vel_lims\" or \"use_joint_acc_lims\" must be true." << RTT::endlog();
   
   qf_qi.resize(kdl_chain.getNrOfJoints());
   tf.resize(kdl_chain.getNrOfJoints());
   tfv.resize(kdl_chain.getNrOfJoints());
   tfa.resize(kdl_chain.getNrOfJoints());
   p0.resize(kdl_chain.getNrOfJoints());
   p2.resize(kdl_chain.getNrOfJoints());
   p3.resize(kdl_chain.getNrOfJoints());
   v1.resize(kdl_chain.getNrOfJoints());
   v2.resize(kdl_chain.getNrOfJoints());
   a0.resize(kdl_chain.getNrOfJoints());
   a1.resize(kdl_chain.getNrOfJoints());
}

//Find the trajectory duration for each joint.
void cubicPolynomial::solve(const KDL::JntArray& qi, const KDL::JntArray& qf, double& duration)
{ 
   KDL::Subtract(qf,qi,qf_qi);
   
   if(use_vel_limits && !use_acc_limits)
      for(int i=0; i<max_vel.size(); ++i)
	 tf[i] = 3 * std::abs(qf_qi(i)) / (2 * max_vel[i]);
      
   else if(!use_vel_limits && use_acc_limits)
      for(int i=0; i<max_acc.size(); ++i)
	 tf[i] = std::sqrt((6 * std::abs(qf_qi(i))) / max_acc[i]);
	 
   else if(use_vel_limits && use_acc_limits)
      for(int i=0; i<max_vel.size(); ++i){
	 tfv[i] = (3 * (qf_qi(i))) / (2 * max_vel[i]);
	 tfa[i] = std::sqrt((6 * (qf_qi(i))) / max_acc[i]);
	 tfv[i] > tfa[i] ? tf[i] = tfv[i] : tf[i] = tfa[i];
      }
   else RTT::log(RTT::Error) << "Can not create cubic polynomial velocity profile. Either \"use_joint_vel_lims\" \n"
				"or \"use_joint_acc_lims\" must be true." << RTT::endlog();
   
   for(int i=0; i<qi.rows(); ++i){
      p0[i] = qi(i);
   // p1[i] = 0;
      p2[i] = 3 * qf_qi(i) / (tf[i] * tf[i]);
      p3[i] = -2 * qf_qi(i) / (tf[i] * tf[i] * tf[i]);
      
   // v0[i] = 0;
      v1[i] = 2 * p2[i];
      v2[i] = 3 * p3[i];
      
      a0[i] = v1[i];
      a1[i] = 2 * v2[i];
   }
   
   duration = *std::max_element(tf.begin(), tf.end());

}

//Current desired joint positions.
void cubicPolynomial::get_desired_joint_pos(KDL::JntArray& desired_joint_pos, double& current_trajec_time) const
{
   for(int i=0; i<desired_joint_pos.rows(); ++i)
      desired_joint_pos(i) = (p3[i] * current_trajec_time + p2[i]) * current_trajec_time * current_trajec_time + p0[i];
}

//Current desired joint velocities.
void cubicPolynomial::get_desired_joint_vel(KDL::JntArray& desired_joint_vel, double& current_trajec_time) const
{
   for(int i=0; i<desired_joint_vel.rows(); ++i)
      desired_joint_vel(i) = current_trajec_time * (v2[i] * current_trajec_time + v1[i]);
}

//Current desired joint acceleration.
void cubicPolynomial::get_desired_joint_acc(KDL::JntArray& desired_joint_acc, double& current_trajec_time) const 
{
   for(int i=0; i<desired_joint_acc.rows(); ++i)
      desired_joint_acc(i) = a1[i] * current_trajec_time + a0[i];
}

//Quintic polynomial velocity profile.
quinticPolynomial::quinticPolynomial(const KDL::Chain& kdl_chain, RTT::TaskContext* this_taskcontext)
{
   rtt_ros_tools::useJointVelLims(use_vel_limits, this_taskcontext);
   rtt_ros_tools::useJointAccLims(use_acc_limits, this_taskcontext);
   
   if(use_vel_limits) rtt_ros_tools::getJointVelLimsFromURDF(kdl_chain, max_vel, this_taskcontext);
   if(use_acc_limits) rtt_ros_tools::getJointAccLims(kdl_chain, max_acc, this_taskcontext);
   if(!use_vel_limits && !use_acc_limits) RTT::log(RTT::Error) << "Can not create quintic polynomial velocity profile. \n"
      "Either \"use_joint_vel_lims\" or \"use_joint_acc_lims\" must be true." << RTT::endlog();
   
   qf_qi.resize(kdl_chain.getNrOfJoints());
   tf.resize(kdl_chain.getNrOfJoints());
   tfv.resize(kdl_chain.getNrOfJoints());
   tfa.resize(kdl_chain.getNrOfJoints());
   qf_qi.resize(kdl_chain.getNrOfJoints());
   p0.resize(kdl_chain.getNrOfJoints());
   p3.resize(kdl_chain.getNrOfJoints());
   p4.resize(kdl_chain.getNrOfJoints());
   p5.resize(kdl_chain.getNrOfJoints());
   v2.resize(kdl_chain.getNrOfJoints());
   v3.resize(kdl_chain.getNrOfJoints());
   v4.resize(kdl_chain.getNrOfJoints());
   a1.resize(kdl_chain.getNrOfJoints());
   a2.resize(kdl_chain.getNrOfJoints());
   a3.resize(kdl_chain.getNrOfJoints());
}

//Find the trajectory duration for each joint.
void quinticPolynomial::solve(const KDL::JntArray& qi, const KDL::JntArray& qf, double& duration)
{
   KDL::Subtract(qf,qi,qf_qi);

   if(use_vel_limits && !use_acc_limits)
      for(int i=0; i<max_vel.size(); ++i)
	 tf[i] = (15 * std::abs(qf_qi(i))) / (8 * max_vel[i]);
      
   else if(!use_vel_limits && use_acc_limits)
      for(int i=0; i<max_acc.size(); ++i)
	 tf[i] = std::sqrt(10 * std::abs(qf_qi(i)) / (std::sqrt(3) * max_acc[i]));
   
   else if(use_vel_limits && use_acc_limits)
      for(int i=0; i<max_vel.size(); ++i){
	 tfv[i] = (15 * (qf_qi(i))) / (8 * max_vel[i]);
	 tfa[i] = std::sqrt((10 * (qf_qi(i))) / (std::sqrt(3) * max_acc[i]));
	 tfv[i] > tfa[i] ? tf[i] = tfv[i] : tf[i] = tfa[i];
      }
      
   else RTT::log(RTT::Error) << "Can not create quintic polynomial velocity profile. \n"
				"Either \"use_joint_vel_lims\" or \"use_joint_acc_lims\" must be true." << RTT::endlog();
   
   for(int i=0; i<qi.rows(); ++i){
      p0[i] = qi(i);
   // p1[i] = 0;
   // p2[i] = 0;
      p3[i] = 10 * qf_qi(i) / std::pow(tf[i],3);
      p4[i] = -15 * qf_qi(i) / std::pow(tf[i],4);
      p5[i] = 6 * qf_qi(i) / std::pow(tf[i],5);
   // v0[i] = 0;
   // v1[i] = 0;
      v2[i] = 3 * p3[i];
      v3[i] = 4 * p4[i];
      v4[i] = 5 * p5[i];
   // a0[i] = 0;
      a1[i] = 2 * v2[i];
      a2[i] = 3 * v3[i];
      a3[i] = 4 * v4[i]; 
   }
   
   duration = *std::max_element(tf.begin(), tf.end());
}

//Current desired joint positions.
void quinticPolynomial::get_desired_joint_pos(KDL::JntArray& desired_joint_pos, double& current_trajec_time) const
{
   for(int i=0; i<desired_joint_pos.rows(); ++i)
      desired_joint_pos(i) = (current_trajec_time * (p5[i] * current_trajec_time + p4[i]) + p3[i]) * std::pow(current_trajec_time, 3) + p0[i];
}

//Current desired joint velocities.
void quinticPolynomial::get_desired_joint_vel(KDL::JntArray& desired_joint_vel, double& current_trajec_time) const
{
   for(int i=0; i<desired_joint_vel.rows(); ++i)
      desired_joint_vel(i) = current_trajec_time * current_trajec_time * (current_trajec_time * (p5[i] * current_trajec_time + p4[i]) + p3[i]);
}

//Current desired joint accelerations.
void quinticPolynomial::get_desired_joint_acc(KDL::JntArray& desired_joint_acc, double& current_trajec_time) const 
{
   for(int i=0; i<desired_joint_acc.rows(); ++i)
      desired_joint_acc(i) = current_trajec_time * (current_trajec_time * (a3[i] * current_trajec_time + a2[i]) + a1[i]); 
}

//Bang Bang velocity profile.
bangBang::bangBang(const KDL::Chain& kdl_chain, RTT::TaskContext* this_taskcontext)
{
   rtt_ros_tools::useJointVelLims(use_vel_limits, this_taskcontext);
   rtt_ros_tools::useJointAccLims(use_acc_limits, this_taskcontext);
   
   if(use_vel_limits) rtt_ros_tools::getJointVelLimsFromURDF(kdl_chain, max_vel, this_taskcontext);
   if(use_acc_limits) rtt_ros_tools::getJointAccLims(kdl_chain, max_acc, this_taskcontext);
   if(!use_vel_limits && !use_acc_limits) RTT::log(RTT::Error) << "Can not create bang bang velocity profile. \n"
      "Either \"use_joint_vel_lims\" or \"use_joint_acc_lims\" must be true." << RTT::endlog();
   
   qi_.resize(kdl_chain.getNrOfJoints());
   qf_qi.resize(kdl_chain.getNrOfJoints());
   tf.resize(kdl_chain.getNrOfJoints());
   t_half.resize(kdl_chain.getNrOfJoints());
   tfv.resize(kdl_chain.getNrOfJoints());
   tfa.resize(kdl_chain.getNrOfJoints());
}

//Find the trajectory duration for each joint.
void bangBang::solve(const KDL::JntArray& qi, const KDL::JntArray& qf, double& duration)
{
   
   KDL::Subtract(qf,qi,qf_qi);
   
   qi_.data = qi.data;
   
   if(use_vel_limits && !use_acc_limits)
      for(int i=0; i<max_vel.size(); ++i)
	 tf[i] = 2 * qf_qi(i) / max_vel[i];
      
   else if(!use_vel_limits && use_acc_limits)
      for(int i=0; i<qf.rows(); ++i)
	 tf[i] = 2 * std::sqrt(qf_qi(i) / max_acc[i]);
	 	 
   else if(use_vel_limits && use_acc_limits)
      for(int i=0; i<max_vel.size(); ++i){
	 tfv[i] = 2 * qf_qi(i) / max_vel[i];
	 tfa[i] = 2 * std::sqrt(qf_qi(i) / max_acc[i]);
	 tfv[i] > tfa[i] ? tf[i] = tfv[i] : tf[i] = tfa[i];
      }
   
   else RTT::log(RTT::Error) << "Can not create bang bang velocity profile. \n"
      "Either \"use_joint_vel_lims\" or \"use_joint_acc_lims\" must be true." << RTT::endlog();
   
   std::transform(tf.begin(), tf.end(), t_half.begin(), [](double time){return time / 2;});
   
   duration = *std::max_element(tf.begin(), tf.end());
}

//Current desired joint positions.
void bangBang::get_desired_joint_pos(KDL::JntArray& desired_joint_pos, double& current_trajec_time) const
{
   for(int i=0; i<desired_joint_pos.rows(); ++i)
   {
      if(current_trajec_time < t_half[i])
	 desired_joint_pos(i) = qi_(i) + 2 * qf_qi(i) * (current_trajec_time * current_trajec_time) / (tf[i] * tf[i]);
      
      else if(current_trajec_time < tf[i])
	 desired_joint_pos(i) = qi_(i) + (-1 + 4 * (current_trajec_time / tf[i]) - 2 * (current_trajec_time * current_trajec_time) / (tf[i] * tf[i])) * qf_qi(i);
      
      else return;
   }
}

//Current desired joint velocities.
void bangBang::get_desired_joint_vel(KDL::JntArray& desired_joint_vel, double& current_trajec_time) const
{
   for(int i=0; i<desired_joint_vel.rows(); ++i)
   {
      if(current_trajec_time < t_half[i])
	 desired_joint_vel(i) = 4 * qf_qi(i) * current_trajec_time / tf[i];
      
      else if(current_trajec_time < tf[i])
	 desired_joint_vel(i) = (4 / tf[i] - 4 * (current_trajec_time) / tf[i]) * qf_qi(i);
      
      else desired_joint_vel(i) = 0;
   }
}

//Current desired joint accelerations.
void bangBang::get_desired_joint_acc(KDL::JntArray& desired_joint_acc, double& current_trajec_time) const
{
   for(int i=0; i<desired_joint_acc.rows(); ++i)
   {
      if(current_trajec_time < t_half[i])
	 desired_joint_acc(i) = 4 * qf_qi(i) / tf[i];
      
      else if(current_trajec_time < tf[i])
	 desired_joint_acc(i) = (-4 / tf[i]) * qf_qi(i);
      
      else desired_joint_acc(i) = 0;
   }
}

//Trapezoidal velocity profile
trapezoidal::trapezoidal(const KDL::Chain& kdl_chain, RTT::TaskContext* this_taskcontext)
{
   rtt_ros_tools::getJointVelLimsFromURDF(kdl_chain, max_vel, this_taskcontext);
   rtt_ros_tools::getJointAccLims(kdl_chain, max_acc, this_taskcontext);
   
   qi_.resize(kdl_chain.getNrOfJoints());
   qf_.resize(kdl_chain.getNrOfJoints());
   qf_qi.resize(kdl_chain.getNrOfJoints());
   max_vel.resize(kdl_chain.getNrOfJoints());
   max_acc.resize(kdl_chain.getNrOfJoints());
   switch_time.resize(kdl_chain.getNrOfJoints());
   tf.resize(kdl_chain.getNrOfJoints());
}

//Find the trajectory duration for each joint.
void trapezoidal::solve(const KDL::JntArray& qi, const KDL::JntArray& qf, double& duration)
{
   for(int i=0; i<qf.rows(); ++i)
   {
      switch_time[i] = max_vel[i] / max_acc[i];
      
      qf_qi(i) = qf(i) - qi(i);
      
      tf[i] = switch_time[i] + qf_qi(i) / max_vel[i];
      
      qi_(i) = qi(i);
      
      qf_(i) = qf(i);
   }
   
   duration = *std::max_element(tf.begin(), tf.end());
}

//Current desired joint positions.
void trapezoidal::get_desired_joint_pos(KDL::JntArray& desired_joint_pos, double& current_trajec_time) const 
{
   for(int i=0; i < switch_time.size(); ++i)
   {
      if(current_trajec_time < switch_time[i])
	 for(int i=0; i< desired_joint_pos.rows(); ++i)
	    desired_joint_pos(i) = qi_(i) + 0.5 * (current_trajec_time * current_trajec_time * max_acc[i] * KDL::sign(qf_qi(i)));
	 
      else if(current_trajec_time < (tf[i] - switch_time[i]))
	 for(int i=0; i< desired_joint_pos.rows(); ++i)
	    desired_joint_pos(i) = qi_(i) + (current_trajec_time - (0.5 * switch_time[i])) * max_vel[i] * KDL::sign(qf_qi(i));
	        
      else if(current_trajec_time < tf[i])
	 for(int i=0; i< desired_joint_pos.rows(); ++i)
	    desired_joint_pos(i) = qf_(i) - 0.5 * (tf[i] - current_trajec_time) * (tf[i] - current_trajec_time) * max_acc[i] * KDL::sign(qf_qi(i));
		 
      else return;
   }
}

//Current desired joint velocities.
void trapezoidal::get_desired_joint_vel(KDL::JntArray& desired_joint_vel, double& current_trajec_time) const
{
   for(int i=0; i < switch_time.size(); ++i)
   {
      if(current_trajec_time < switch_time[i])
	 for(int i=0; i< desired_joint_vel.rows(); ++i)
	    desired_joint_vel(i) = current_trajec_time * max_acc[i] * KDL::sign(qf_qi(i));
	 
      else if(current_trajec_time < (tf[i] - switch_time[i]))
	  for(int i=0; i< desired_joint_vel.rows(); ++i)
	     desired_joint_vel(i) = 1;
	       
      else if(current_trajec_time < tf[i])
	  for(int i=0; i< desired_joint_vel.rows(); ++i)
	     desired_joint_vel(i) = (tf[i] - current_trajec_time) * max_acc[i] * KDL::sign(qf_qi(i));
	  
      else desired_joint_vel(i) = 0;       
   }
}

//Current desired joint accelerations.
void trapezoidal::get_desired_joint_acc(KDL::JntArray& desired_joint_acc, double& current_trajec_time) const 
{
   for(int i=0; i < switch_time.size(); ++i)
   {
      if(current_trajec_time < switch_time[i])
	 for(int i=0; i< desired_joint_acc.rows(); ++i)
	    desired_joint_acc(i) = KDL::sign(qf_qi(i)) * max_acc[i];
	 
      else if(current_trajec_time < (tf[i] - switch_time[i]))
	 for(int i=0; i< desired_joint_acc.rows(); ++i)
	    desired_joint_acc(i) = 0;
	       
      else if(current_trajec_time < tf[i])
	 for(int i=0; i< desired_joint_acc.rows(); ++i)
            desired_joint_acc(i) = max_acc[i] * KDL::sign(qf_qi(i));
	       
      else desired_joint_acc(i) = 0;
   }
}

bool getJointSpaceVelProfile(std::string vel_prof_name, const KDL::Chain& kdl_chain, std::unique_ptr<joint_space_vel_prof::velocityProfile>& vel_prof, RTT::TaskContext* this_taskcontext)
{
   if(vel_prof_name == "trapezoidal"){
      vel_prof.reset(new joint_space_vel_prof::trapezoidal(kdl_chain, this_taskcontext));
      RTT::log(RTT::Info) << "Creating trapezoidal joint space velocity profile." << RTT::endlog();
      return true;
   }
   else if(vel_prof_name == "bang_bang"){
      vel_prof.reset(new joint_space_vel_prof::bangBang(kdl_chain, this_taskcontext));
      RTT::log(RTT::Info) << "Creating bang_bang joint space velocity profile." << RTT::endlog();
      return true;
   }
   else if(vel_prof_name == "quintic_polynomial"){
      vel_prof.reset(new joint_space_vel_prof::quinticPolynomial(kdl_chain, this_taskcontext));
      RTT::log(RTT::Info) << "Creating quintic_polynomial joint space velocity profile." << RTT::endlog();
      return true;
   }
   else if(vel_prof_name == "cubic_polynomial"){
      vel_prof.reset(new joint_space_vel_prof::cubicPolynomial(kdl_chain, this_taskcontext));
      RTT::log(RTT::Info) << "Creating cubic_polynomial joint space velocity profile." << RTT::endlog();
      return true;
   }
   else{ RTT::log(RTT::Error) << "function argument " << vel_prof_name << " does not match a velocity profile."
      " Options are \"trapezoidal\", \"bang_bang\", \"quintic_polynomial\", \"cubic_polynomial\"" << RTT::endlog();
      return false;
   }
}

}
