#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
pytheta is a Python library that interfaces with gphoto2 to control multiple RICOH THETA cameras 
using Ubuntu. It is distributed under the GPL3 license.  The RICOH THETA must be turned
on and connected to the computer with a USB cable. 


--------
Japanese
--------
複数台の全天球カメラ：ThetaをUbuntuPC上で制御するためのプログラム。
ライセンスはGPL3

"""
# pylint: disable=C0111
# JP: ↑プログラムの説明ドキュメントがないよ!というエラーの防止
# EN: ↑ Prevention of the error where there is no program description document!

# pylint: disable=C0321
# JP: 改行したほうがいいよ！という推奨を無視
# EN: Ingnoring the suggestion that you should start a new line!

# Trailing whitespace
# pylint: disable=C0303

from __future__ import print_function

import logging
import sys
import subprocess as sp
import os
import time
import re
import threading
import six
import gphoto2 as gp

from get_files import get_files as get_files_inner

if   six.PY2: import urllib as url
elif six.PY3: import urllib.parse as url


# Properties (milliseconds)
TIMEOUT = 10


class no_xtp_dev(Exception):
	"""
	Error indicating that there are no xTP devices

	--------
	Japanese
	--------
	xTPデバイスが何もないことを示すエラー
	"""
	def __str__(self):
		return "xTPデバイスが何もありません"


def get_xtp_dev_list():
	"""
		JP: 接続されているxTPデバイスのリストを作成する。  
		xTPデバイスとは、PTP、MTPデバイスの総称である。(勝手に名付けた。)
		EN: Make a list of connected xTP devices.
		xTP device is a general term for PTP and MTP devices. (I named it arbitrarily.)


		Returns
		-------
		xtp_dev_lis : list
			JP: 接続されているxTPデバイスのリスト
			EN: List of connected xTP devices
	"""
	xtp_dev_list = []
	for name, addr in gp.check_result(gp.gp_camera_autodetect() ):
		xtp_dev_list.append((name, addr))
	if not xtp_dev_list:
		raise no_xtp_dev()
	return sorted(xtp_dev_list)


def check_if_theta(xtp_dev_list):
	"""
	Using lsusb, extract RICOH THETA camera from the list of MTP or PTP devices acquired.
	We are using this technique with lsusb since gPhoto2 cannot be used to identify the
	RICOH THETA cameras when they are mounted.

	Parameters
	----------
	xtp_dev_lis : list
		List of connected xTP devices

	Returns
	-------
	theta_list : list
		List of connected THETAs

	--------
	Japanese
	--------
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
			print("[debug] Found THETA camera. [シータです]")
			theta_list.append(addr)

		else:
			print("[debug] Did not find THETA camera. [シータではないです]")
			#pass

	print("[debug] check_if_theta successfully completed. [は正常終了]")
	return theta_list


def unmount_theta(theta_list):
	"""
		JP: マウントされているThetaをアンマウントする。  
		そもそもマウントしないようにすれば良いかもしれないが、  
		他の機材の使用に師匠が出うる設定が必要なのでこの実装となった。
		EN: Unmount mounted THETAs.
		Ultimately, it may be better not to mount it, but this implementation was made because 
		it is necessary to make settings that allow the master to use other equipment.


		Parameters
		----------
		theta_list : list
			JP: 接続されているThetaのリスト
			EN: List of connected THETAs
	"""
	for addr in theta_list:
		path = "/run/user/1000/gvfs/gphoto2:host=%5B"+url.quote(addr)+"%5D"
		if os.path.exists(path):
			print("[{:s}]をアンマウントします".format(path) )
			sp.check_output(["gvfs-mount", "-u", path])
			# cmd = "gvfs-mount -u /run/user/1000/gvfs/mtp:host=%5B"+url.quote(addr)+"%5D"
			# sp.check_output(cmd,shell=True) #JP: こういう書き方もある。EN: Can be written this way, too.
		else:
			print("[{:s}] camera unmount completed [はアンマウント済みです]".format(path))


def connect_init():
	"""
	A function that summarizes the items to be performed all at once when connecting.

	Returns
	-------
	theta_list : list
		List of connected THETAs

	--------
	Japanese
	--------
	接続に際して一斉に行う項目をまとめた関数

	Returns
	-------
	theta_list : list
		接続されているThetaのリスト

	"""

	logging.basicConfig(
		format='%(levelname)s: %(name)s: %(message)s', level=logging.WARNING
	)
	callback_obj = gp.check_result(gp.use_python_logging() )


	xtp_dev_list = get_xtp_dev_list()
	theta_list = check_if_theta(xtp_dev_list)
	# I was getting an error with the unmount and commented it out.
	unmount_theta(theta_list)

	for addr in theta_list:
		print('[{:s}]'.format(addr) )

	return theta_list


def camera_control_util(addr):
	"""
	A utility that summarizes the basic parts of a function 
	that you need in order to select and send to a Theta sending a command in Python-gPhoto2.

	
	Parameters
	----------
	addr : char
		ID for connected THETA

	Returns
	-------
	camera : Camera object
	parent_widget : camera widget object

	--------
	Japanese
	--------
	Python-gPhoto2において
	コマンドを送信するThetaを選択して送信する必要がある関数の
	基本部分をまるっとまとめたユーティリティ。

	Parameters
	----------
	addr : char
		接続されているThetaのID

	Returns
	-------
	camera : Camera object
	parent_widget : camera widget object



	"""

	camera = gp.Camera()
	# Search for a port by camera port name (?)
	# JP: カメラのポート名でポートを検索(?)

	port_info_list = gp.PortInfoList()
	port_info_list.load()

	idx = port_info_list.lookup_path(addr)
	camera.set_port_info(port_info_list[idx])
	camera.init()

	try:
		parent_widget = camera.get_config()
	except gp.GPhoto2Error:
		# JP: 念の為例外を立てる処理はそのまま。意味があるかは不明
		# EN: Just in case, the process of making an exception remains the same. Not sure if it makes sense
		raise RuntimeError("Unable to connect to Camera")

	return camera, parent_widget


def select_config_util(parent_widget, child_name, grandchild_name):
	"""
	A utility that puts together all the overlapping 
	parts when getting various statuses with Python-gPhoto2.
	
	Parameters
	----------
	parent_widget : camera widget object
	child_name : char
		Name of child widget
	grandchild_name : char
		Name of grandchild widget

	Returns
	-------
	grandchild_widget : camera widget object？
		JP: 孫ウィジェット
		EN: Grandchild widget

	--------
	Japanese
	--------
	Python-gPhoto2によって諸ステータスを取得する際に 重複する部分をまるっとまとめたユーティリティ。
	
	Parameters
	----------
	parent_widget : camera widget object
	child_name : char
		子ウィジェットの名前
	grandchild_name : char
		孫ウィジェットの名前

	Returns
	-------
	grandchild_widget : camera widget object？
		孫ウィジェット
	"""
	# JP: 設定対象の子ウィジェットを選択
	# EN: Select the child widget to be set
	
	child_widget = parent_widget.get_child_by_name(child_name) 

	# JP: 同 孫ウィジェットを選択
	# EN: Choosing the same grandchild widget

	grandchild_widget = child_widget.get_child_by_name(grandchild_name) 
	return grandchild_widget


def inner_start_capture(addr):
	"""
		JP: 撮影開始処理を担う実処理部分
		EN: Actual processing part responsible for the start of shooting processing

		Parameters
		----------
		addr : char
			JP: 接続されているThetaのID
			EN: ID of the connected THETA
	"""

	# JP: 実行対象設定済みのカメラオブジェクトと親ウィジェットを出力
	# EN: Outputs the camera object and parent widget that have been set to be executed
	camera, camera_config = camera_control_util(addr)
	# JP: 孫ウィジェットの取得
	# EN: Selecting the grandchild widget
	movie = select_config_util(camera_config, 'actions', "movie")
	
	movie.set_value(1) # JP:値を指定 EN: Specify a value.  
	camera.set_config(camera_config) # EN: Change to appropriate value. JP:値を適応

	camera.exit()


def start_capture(theta_list):
	"""
		JP: 本体関数inner_start_captureをマルチスレッドで実行するための関数
		EN: Function for executing the body function inner_start_capture in multiple threads

		Parameters
		----------
		theta_list : list
			JP: 接続されているThetaのリスト
			EN: List of connected THETAs
	"""
	threads = []
	for addr in theta_list:
		threads.append(
			threading.Thread(name=addr, target=inner_start_capture, args=(addr,) )
		)
	for i in threads:
		i.start()

	for i in threads:
		i.join()


def inner_finish_capture(addr):
	"""
	JP: 撮影終了処理を担う実処理部分
	現状Python-gPhoto2による方法が不明なため、CLI-gPhoto2で代用
	EN: Actual processing part responsible for shooting end processing
	Currently, the method using Python-gPhoto2 is unknown, so CLI-gPhoto2 is used instead.
	 
	Parameters
	----------
	addr : char
		JP: 接続されているThetaのID
		EN: ID of connected THETA
	"""

	# JP: 実行対象設定済みのカメラオブジェクトと親ウィジェットを出力
	# EN: Outputs the camera object and parent widget that have been set to be executed
	camera, camera_config = camera_control_util(addr)
	# JP: 孫ウィジェットを取得して"."以下で値を指定
	# EN: Get the grandchild widget and specifues the value under "."
	select_config_util(camera_config, 'actions', "opcode").set_value("0x1018,0xFFFFFFFF")
	camera.set_config(camera_config) # JP: 値を適応  EN: Set value

	camera.exit()


def finish_capture(theta_list):
	"""
	Function for executing the body function inner_finish_capture in multiple threads

	Parameters
	----------
	addr : char
		EN: ID of connected THETAs

	--------
	Japanese
	--------
	本体関数inner_finish_captureをマルチスレッドで実行するための関数

	Parameters
	----------
	addr : char
		接続されているThetaのID
	"""
	
	threads = []
	for addr in theta_list:
		threads.append(
			threading.Thread(name=addr, target=inner_finish_capture, args=(addr,) )
		)
	for i in threads:
		i.start()

	for i in threads:
		i.join()


def get_serial(theta_list):
	"""
		JP: theta_listに含まれる各Thetaのシリアル番号を出力する。  
		ループを関数外部にするか、否かは今後の実装次第。  
		逐一gp.Cameraを行っているが、これを一回やるでだけにできないか考えている。
		EN: Output the serial number of each Theta included in theta_list.
		Whether or not the loop is outside the function depends on future implementation.
		I am doing gp.Camera over and over, but I am wondering if I can do this only once.

		Parameters
		----------
		theta_list : list
			JP: 接続されているThetaのリスト
			EN: List of connected THETAs
	"""

	for addr in theta_list:
		camera, camera_config = camera_control_util(addr)
		serial = select_config_util(camera_config, 'status', 'serialnumber').get_value()
		print(serial)
		camera.exit()
		print("")


def get_bat_lv(theta_list):
	"""
		JP: theta_listに含まれる各Thetaの現在のバッテリーレベルを整数で出力する。  
		ループを関数外部にするか、否かは今後の実装次第。  
		逐一gp.Cameraを行っているが、これを一回やるでだけにできないか考えている。
		EN: Outputs as an integer the current battery level of each THETA contained in theta_list
		Whether or not the loop is outside the function depends on future implementation.
		I am doing gp.Camera over and over, but I am wondering if I can do this only once.

		Parameters
		----------
		theta_list : list
			JP: 接続されているThetaのリスト
			EN: List of connected THETAs
	"""
	result_list = []
	for addr in theta_list:
		camera, camera_config = camera_control_util(addr)
		battery_level = select_config_util(camera_config, 'status', 'batterylevel').get_value()
		camera.exit()
		battery_level = int(''.join( [x for x in battery_level if x.isdigit() ] ) )
		result_list.append(battery_level)
	return result_list


def get_rem_time_v(theta_list):
	"""
		JP: ストレージ容量に起因する残時間の表示[second]
		EN: Display of remaining time due to storage capacity [in seconds]

		Parameters
		----------
		theta_list : list
			JP: 接続されているThetaのリスト
			EN: List of connected THETAs

		Returns
		-------
		result_list : list
			JP: 実行結果のリスト
			EN: List of execution results

	"""
	"""　
		# JP: デバイスプロパティコード：0xD80D RemainingVideos
		# JP:（ベンダー拡張プロパティ）
		# JP: 詳細：https://api.ricoh/docs/theta-usb-api/property/remaining_videos/
		# EN: Device Property Code: 0xD80D RemainingVideos
		# EN: Vendor Extension Properties
		# EN: Details: https://api.ricoh/docs/theta-usb-api/property/remaining_videos/
	"""

	result_list = []
	for addr in theta_list:
		camera, camera_config = camera_control_util(addr)
		result_list.append(select_config_util(camera_config, 'other', 'd80d').get_value())
		camera.exit()
	return result_list


def get_files(theta_list):
	"""
		JP: カメラからファイルをダウンロードして本体ストレージへ保存する。
		そして、カメラ側ストレージからこれを削除する。
		EN: Download the file from the camera and save it in the main unit storage.
		Then, delete it from  storage on the camera side.

		Parameters
		----------
		theta_list : list
			JP: 接続されているThetaのリスト
			EN: List of connected THETAs
	"""
	# JP: 保存先 親ディレクトリ
	# EN: Destination parent directory
	PHOTO_DIR = os.path.expanduser('~/Pictures/from_camera')
	# JP: 保存先 小ディレクトリ
	# EN: Save destination sub directory
	PHOTO_SUB_DIR = '%Y/%Y_%m_%d/'

	threads = []
	for addr in theta_list:
		camera, _ = camera_control_util(addr)
		threads.append(
			threading.Thread(
				name=addr,
				target=get_files_inner,
				args=(camera, PHOTO_DIR, PHOTO_SUB_DIR,)
			)
		)
	for i in threads:
		i.start()

	for i in threads:
		i.join()


def _unittest():
	"""
		JP: テスト
		EN: Test
	"""

	theta_list = connect_init()
	get_serial(theta_list)
	time.sleep(1)
	start_capture(theta_list)
	#time.sleep(2)
	for _ in range(5):
		print(get_bat_lv(theta_list) )
		#time.sleep(1)
		print(get_rem_time_v(theta_list) )
		print("")
		time.sleep(1)

	finish_capture(theta_list)

	time.sleep(2)

	get_files(theta_list)


if __name__ == "__main__":
	sys.exit(_unittest())
