#!/usr/bin/env python
# -*- coding: utf-8 -*-

# pylint: disable=C0111
# ↑プログラムの説明ドキュメントがないよ！というエラーの防止

import rospy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String

def callback(message):
    rospy.loginfo("get message! [%s]", message.data) 

rospy.init_node('bat_rem_time_pub')
sub = rospy.Subscriber('bat_lv', Int32MultiArray, callback) 
rospy.spin()