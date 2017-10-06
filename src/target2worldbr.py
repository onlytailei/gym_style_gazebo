#!/usr/bin/env python
# coding=utf-8
'''
Author:Tai Lei
Date:Thu 09 Feb 2017 04:08:17 PM CST
Info:
'''

#!/usr/bin/env python  
import roslib
import rospy
import tf
 
if __name__ == '__main__':
    target_x = -0.5
    target_y = -5
    rospy.init_node('target2worldbr')
    
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        target_x = rospy.get_param("TARGET_X")
        target_y = rospy.get_param("TARGET_Y")
        br.sendTransform((target_x, target_y, 0.0),
                (0.0, 0.0, 0.0, 1.0),
                rospy.Time.now(),
                "target",
                "default_world")
        rate.sleep()
