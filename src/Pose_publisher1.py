#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped


def posePulisher():
    pub = rospy.Publisher('posePoint', PoseStamped, queue_size=1000)
    rospy.init_node('posePublisher')
    posePoint = PoseStamped()
    i = 0

    while not rospy.is_shutdown():
        posePoint.position.x = 0.1 + 0.2 * i
        posePoint.position.y = 0.1
        posePoint.position.z = 0.1
        posePoint.orientation.x = 0.0
        posePoint.orientation.y = 0.0
        posePoint.orientation.z = 0.0
        posePoint.orientation.w = 0.0

        pub.publish(posePoint)
        i = i + 1
        rospy.spin()


if __name__ == '__main__':
    try:
        posePulisher()
    except rospy.ROSInterruptException:
        pass
