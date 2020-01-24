#!/usr/bin/env python
# -*- coding: utf-8 -*-

# pylint: disable=C0111
# ↑プログラムの説明ドキュメントがないよ！というエラーの防止

import rospy
from std_msgs.msg import Int32MultiArray

import pytheta

rospy.init_node('bat_lv_pub')
pub = rospy.Publisher('bat_lv', Int32MultiArray, queue_size=1)
rate = rospy.Rate(0.5)

t_list = pytheta.connect_init()
print("実行結果{},type={}".format( t_list,type(t_list) ) )

while not rospy.is_shutdown():
	pub.publish(
		Int32MultiArray(data = pytheta.get_bat_lv(t_list) ) 
	)
	rate.sleep()

print("\nend")
