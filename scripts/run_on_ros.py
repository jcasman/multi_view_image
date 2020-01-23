#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pytheta

from std_msgs.msg import Int32MultiArray

rospy.init_node('bat')
pub = rospy.Publisher('bat_lv', Int32MultiArray, queue_size=1)
rate = rospy.Rate(0.05)

t_list = pytheta.connect_init()
print("実行結果{},type={}".format( t_list,type(t_list) ) )



while not rospy.is_shutdown():
    pub.publish(
		Int32MultiArray(data = pytheta.get_bat_lv(t_list) ) 
	)
    rate.sleep()

print("\nend")
