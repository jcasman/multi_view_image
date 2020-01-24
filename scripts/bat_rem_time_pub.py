#!/usr/bin/env python
# -*- coding: utf-8 -*-

# pylint: disable=C0111
# ↑プログラムの説明ドキュメントがないよ！というエラーの防止



import time
import datetime as dtime
import numpy as np
import rospy
from std_msgs.msg import Int32MultiArray




def callback(message):
	rospy.loginfo("Get battery Lv! (%s)", message.data)

if(__name__ == '__main__'):
	rospy.init_node('bat_rem_time_pub')
	sub = rospy.Subscriber('bat_lv', Int32MultiArray, callback)
	rospy.loginfo("bat_rem_time_pub is start")
	rospy.spin()
	rospy.loginfo("bat_rem_time_pub is end")
