#!/usr/bin/env python
# -*- coding: utf-8 -*-

# pylint: disable=C0111
# ↑プログラムの説明ドキュメントがないよ！というエラーの防止


from __future__ import print_function

import logging
#import six
import sys
import subprocess as sp
import urllib as url #py3ならば、import urllib.parse as url である。
import os
# import glob #アンマウント先が存在しているかの判定に使用していた。
import re # バスデバイス文字列を分割するため使用
import threading
import gphoto2 as gp


def port_ptpcam(addr):
	#print('debug [{:s}]'.format(addr) )
	bus,dev = re.split('[:,]', addr)[1:]
	#print('debug [{}] [{}]'.format(bus,dev) )
	return "--bus={} --dev={}".format(bus,dev)

def unmount_theta(theta_list):
	for addr in theta_list:
		# if( glob.glob('/run/user/1000/gvfs/gphoto2:host=*') ):
		path = "/run/user/1000/gvfs/gphoto2:host=%5B"+url.quote(addr)+"%5D"
		if( os.path.exists(path) ):
			print("[{:s}]をアンマウントします".format(path) )
			sp.check_output(["gvfs-mount","-u",path])
			# cmd = "gvfs-mount -u /run/user/1000/gvfs/mtp:host=%5B"+url.quote(addr)+"%5D"
			# sp.check_output(cmd,shell=True) #こういう書き方もある。
		else:
			print("[{:s}]はアンマウント済みです".format(path))

def check_if_theta(camera_list):
	theta_list = []
	for index, (name, addr) in enumerate(camera_list):
		print('[debug] [{:d}]:[{:s}] [{:s}]'.format(index, addr, name))

		if (name == "USB PTP Class Camera"):	# このままではシータであるかの判定が甘い。今後改善
			# print("[debug]シータです")
			theta_list.append(addr)

		else:
			# print("[debug]シータではないです")
			pass
	return theta_list


def connect_init():
	logging.basicConfig(
		format='%(levelname)s: %(name)s: %(message)s', level=logging.WARNING
	)
	callback_obj = gp.check_result(gp.use_python_logging() )

	camera_list = []
	for name, addr in gp.check_result(gp.gp_camera_autodetect() ):
		camera_list.append((name, addr))
	if not camera_list:
		print('MTPデバイスが何もありません')
		return 1
	camera_list.sort(key=lambda x: x[0])

	theta_list = check_if_theta(camera_list)
	unmount_theta(theta_list)

	"""
	for addr in theta_list:
		print('[{:s}]'.format(addr) )
	"""
	return theta_list


def inner_start_capture(addr):
	# print('debug[{}]'.format(addr) )
	sp.check_output(
		"gphoto2 --set-config movie=1 --port={}".format(addr),
		shell=True
	)
def start_capture(theta_list):
	for addr in theta_list:
		thread = threading.Thread(name=addr, target=inner_start_capture, args=(addr,))
		thread.start()

def finish_capture(theta_list):
	for addr in theta_list:
		# print('debug[{}]'.format( port_ptpcam(addr) ) )
		sp.check_output(
			"ptpcam -R 0x1018,0xFFFFFFFF {}".format( port_ptpcam(addr) ),
			shell=True
		)

def get_serial(theta_list):
	for addr in theta_list:
		print('[{}]'.format(addr) )
		sp.check_call(
			"gphoto2 --get-config=serialnumber --port={}".format(addr),
			shell=True
		)
		print("")

def get_bat_lv(theta_list):
	for addr in theta_list:
		print('[{}]'.format(addr) )
		sp.check_call(
			"gphoto2 --get-config=batterylevel --port={}".format(addr),
			shell=True
		)
		print("")

def check_rem_time_v(theta_list):
	for addr in theta_list:
		print('[{}]'.format(addr) )
		sp.check_call(
			"gphoto2 --get-config=d80d --port={}".format(addr),
			shell=True
		)
		print("")




if __name__ == "__main__":
	sys.exit(connect_init())
