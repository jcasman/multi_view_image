#!/usr/bin/env python
# -*- coding: utf-8 -*-

# pylint: disable=C0111
# ↑プログラムの説明ドキュメントがないよ！というエラーの防止



import time
import datetime as dtime
import numpy as np
import rospy
from std_msgs.msg import Int32MultiArray



flag = False
data_X = np.empty( (0,2),int )
data_Y = np.empty( (0,3),int )

def callback(message):
	global flag
	global data_X
	global data_Y
	data_Ans = []

	nowtime = time.time()

	data_X = np.vstack( [data_X, np.array([nowtime,1] ) ] )
	data_Y = np.vstack( [data_Y, np.array(message.data) ] )
	
	#rospy.loginfo("get message! (\n%s)", data_Y)
	
	for n in range(data_Y.shape[1] ):
		
		a,b = np.linalg.lstsq(data_X,data_Y[:,n])[0]
		
		data_Ans.append( b/a - nowtime )

	rospy.loginfo("get message! (%s)", data_Ans)

	

if(__name__ == '__main__'):
	rospy.init_node('bat_rem_time_pub')
	sub = rospy.Subscriber('bat_lv', Int32MultiArray, callback)
	rospy.loginfo("bat_rem_time_pub is start")
	rospy.spin()
	rospy.loginfo("bat_rem_time_pub is end")
