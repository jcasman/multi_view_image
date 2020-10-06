#!/usr/bin/env python
# -*- coding: utf-8 -*-

# pylint: disable=C0111
# ↑プログラムの説明ドキュメントがないよ!というエラーの防止

# pylint: disable=C0321
# 改行したほうがいいよ！という推奨を無視

# Trailing whitespace
# pylint: disable=C0303

from __future__ import print_function

import logging
import sys
import subprocess as sp
import os
# import glob #アンマウント先が存在しているかの判定に使用していた。
import re # バスデバイス文字列を分割するため使用
import threading
import six
import gphoto2 as gp

if six.PY2: import urllib as url
else:       import urllib.parse as url


def port_ptpcam(addr):
	#print('debug [{:s}]'.format(addr) )
	bus, dev = re.split('[:,]', addr)[1:]
	#print('debug [{}] [{}]'.format(bus,dev) )
	return "--bus={} --dev={}".format(bus, dev)

class no_xtp_dev(Exception):
	"""
	xTPデバイスが何もないことを示すエラー
	"""
	def __str__(self):
		return "xTPデバイスが何もありません"

def get_xtp_dev_list():
	"""
	接続されているxTPデバイスのリストを作成する。  
	xTPデバイスとは、PTP、MTPデバイスの総称である。(勝手に名付けた。)


	Returns
	-------
	xtp_dev_lis : list
		接続されているxTPデバイスのリスト
	"""	
	xtp_dev_list = []
	for name, addr in gp.check_result(gp.gp_camera_autodetect() ):
		xtp_dev_list.append((name, addr))
	if not xtp_dev_list:
		raise no_xtp_dev()
	return sorted(xtp_dev_list)

def check_if_theta(xtp_dev_list):
	"""
	lsusbを用いて獲得したMTP or PTPデバイスの一覧の中からThetaを抽出する。  
	gPhoto2側の認識機能はマウント状態では使用不能なためこのような実装となった。

	Parameters
	----------
	xtp_dev_lis : list
		接続されているxTPデバイスのリスト

	Returns
	-------
	theta_list : list
		接続されているThetaのリスト
	"""

	print("[debug] check_if_theta 処理開始")
	theta_list = []
	for index, (name, addr) in enumerate(xtp_dev_list):
		print('[debug] [{:d}]:[{:s}] [{:s}]'.format(index, addr, name))

		dev = addr.rsplit(',', 1)[1]
		cmd = "lsusb -vd 05ca: -s " + dev + "|grep iProduct"
		res = repr(sp.Popen(cmd, stdout=sp.PIPE, shell=True).communicate()[0]).decode('utf-8')

		print('[debug] [{}]'.format(res))
		if re.match(".*RICOH THETA.*",res):
			print("[debug]シータです")
			theta_list.append(addr)

		else:
			print("[debug]シータではないです")
			#pass

	print("[debug] check_if_thetaは正常終了")
	return theta_list

def unmount_theta(theta_list):
	"""	
	マウントされているThetaをアンマウントする。  
	そもそもマウントしないようにすれば良いかもしれないが、  
	他の機材の使用に師匠が出うる設定が必要なのでこの実装となった。

	Parameters
	----------
	theta_list : list
		接続されているThetaのリスト
	"""	
	for addr in theta_list:
		path = "/run/user/1000/gvfs/gphoto2:host=%5B"+url.quote(addr)+"%5D"
		if os.path.exists(path):
			print("[{:s}]をアンマウントします".format(path) )
			sp.check_output(["gvfs-mount", "-u", path])
			# cmd = "gvfs-mount -u /run/user/1000/gvfs/mtp:host=%5B"+url.quote(addr)+"%5D"
			# sp.check_output(cmd,shell=True) #こういう書き方もある。
		else:
			print("[{:s}]はアンマウント済みです".format(path))

def connect_init():
	logging.basicConfig(
		format='%(levelname)s: %(name)s: %(message)s', level=logging.WARNING
	)
	callback_obj = gp.check_result(gp.use_python_logging() )


	xtp_dev_list = get_xtp_dev_list()
	theta_list = check_if_theta(xtp_dev_list)
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
	threads = []
	for addr in theta_list:

		threads.append(
			threading.Thread(name=addr, target=inner_start_capture, args=(addr,) )
		)
	for i in threads:
		i.start()

def inner_finish_capture(addr):
	"""
		二回繰り返すのはThetaSにおける誤動作防止。
		根本的解決の見込みは今のところなし。
	"""
	# print('debug[{}]'.format(addr) )
	sp.check_output(
		"ptpcam -R 0x1018,0xFFFFFFFF {}".format( port_ptpcam(addr) ),
		shell=True
	)
	sp.check_output(
		"ptpcam -R 0x1018,0xFFFFFFFF {}".format( port_ptpcam(addr) ),
		shell=True
	)
def finish_capture(theta_list):
	threads = []
	for addr in theta_list:

		threads.append(
			threading.Thread(name=addr, target=inner_finish_capture, args=(addr,) )
		)
	for i in threads:
		i.start()




def get_serial(theta_list):
	for addr in theta_list:
		print('[{}]'.format(addr) )
		sp.check_call(
			"gphoto2 --get-config=serialnumber --port={}".format(addr),
			shell=True
		)
		print("")

def get_bat_lv(theta_list):
	result_list = []
	for addr in theta_list:
		result = sp.check_output(
			"gphoto2 --get-config=batterylevel --port={}".format(addr),
			shell=True
		)
		result = result.rsplit(" ",1)[1]
		result_list.append( int( result.rstrip("%\n") ) )
	return result_list

def check_rem_time_v(theta_list):
	for addr in theta_list:
		print('[{}]'.format(addr) )
		sp.check_call(
			"gphoto2 --get-config=d80d --port={}".format(addr),
			shell=True
		)
		print("")



def _unittest():
	theta_list = connect_init()



if __name__ == "__main__":
	sys.exit(_unittest())
