#!/usr/bin/env python
# -*- coding: utf-8 -*-

# pylint: disable=C0111
# ↑プログラムの説明ドキュメントがないよ！というエラーの防止

import rospy
from std_msgs.msg import Int32MultiArray

import pytheta

def bat_lv_pub():
	rospy.init_node('bat_lv_pub')
	pub = rospy.Publisher('bat_lv', Int32MultiArray, queue_size=1)
	rate = rospy.Rate(0.5)

	t_list = pytheta.connect_init()
	while not rospy.is_shutdown():
		data = Int32MultiArray(data = pytheta.get_bat_lv(t_list) ) 
		rospy.loginfo(data)
		pub.publish(data)
		rate.sleep()

	rospy.loginfo("bat_lv_pub is end")

if __name__ == '__main__':
	try:
		bat_lv_pub()
	except rospy.ROSInterruptException: 
		pass
