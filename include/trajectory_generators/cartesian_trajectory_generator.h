#ifndef CARTESIAN_TRAJECTORY_GENERATOR_H
#define CARTESIAN_TRAJECTORY_GENERATOR_H

#include <rtt_ros_tools/rtt_ros_tools.h>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>

#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/path_point.hpp>
#include <kdl/framevel.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/utilities/error.h>

#include <tf_conversions/tf_kdl.h>
#include <tf/transform_listener.h>
#include <tf/exceptions.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <memory>

class CartesianTrajectory : public RTT::TaskContext
{
public:
  
    CartesianTrajectory(const std::string& name);

    bool configureHook();
    bool startHook();
    void updateHook();

    bool computeTrajectory();
    
    virtual ~CartesianTrajectory(){}

protected:
    
    RTT::InputPort<geometry_msgs::PoseArray> inPort_WaypointVector;
    RTT::OutputPort<geometry_msgs::Pose> outPort_DesiredPos;
    RTT::OutputPort<geometry_msgs::Twist> outPort_DesiredVel;
    RTT::OutputPort<geometry_msgs::Twist> outPort_DesiredAcc;

    bool trajectory_computed, transform_points;
    std::string frame_id, task_space_velocity_profile;
    double current_trajec_time, task_space_vel_limit, task_space_acc_limit;
    double task_trajectory_corner_radius, task_trajectory_equivalent_radius;
    geometry_msgs::PoseArray waypoint_array, transformed_waypoint_array;
    geometry_msgs::PoseStamped pose_stamped;
    KDL::Path_RoundedComposite* path;
    KDL::Trajectory* traject;
    KDL::Trajectory_Composite* comp_trajec;
    KDL::VelocityProfile* vel_profile;
    KDL::RotationalInterpolation_SingleAxis* interpolator;
    KDL::Frame frame, previous_frame, current_pos;
    geometry_msgs::Pose current_pos_msg;
    KDL::Twist current_vel, current_acc, frame_diff;
    geometry_msgs::Twist current_vel_msg, current_acc_msg;
    std::vector<KDL::Frame> frame_waypoint_vector;
    tf::TransformListener tf_listener;
};

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(CartesianTrajectory)

#endif

