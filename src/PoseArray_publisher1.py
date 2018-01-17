#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, Pose

def posePulisher():

	pub = rospy.Publisher('poseCommands', PoseArray)
	rospy.init_node('posePulisher')
	poseCommands = PoseArray()
        waypoint = Pose()
	i = 0;

    	poseCommands.header.frame_id = "base_link"
    	poseCommands.header.stamp = rospy.get_rostime()

	while not rospy.is_shutdown():
        pose.position.x = 0.01 + 0.5*i;
 	pose.position.y = 0.01;
  	pose.position.z = 0.04;
  	pose.orientation.x = 0.070711;
        pose.orientation.y = 0.070711;
 	pose.orientation.z = 0.0;
        pose.orientation.w = 0.0;
        poseCommands.poses.append(waypoint)

 	pose.position.y += 0.3;
  	waypoints.poses.push_back(pose);
  
  	pose.position.y -= 0.5;
  	pose.position.z -= 0.1;

	poseCommands.poses.append(waypoint)

	pub.publish(poseCommands)
	rospy.sleep(1)

if __name__ == '__main__':
	try:
		posePulisher()
	except rospy.ROSInterruptException: pass
