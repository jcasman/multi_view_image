#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
複数台の全天球カメラ：ThetaをUbuntuPC上で制御するためのプログラム。
ライセンスはGPL3
"""
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
	"""
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
	unmount_theta(theta_list)

	#for addr in theta_list:
	#	print('[{:s}]'.format(addr) )

	return theta_list


def camera_control_util(addr):
	"""
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

	# カメラのポート名でポートを検索(?)
	port_info_list = gp.PortInfoList()
	port_info_list.load()

	idx = port_info_list.lookup_path(addr)
	camera.set_port_info(port_info_list[idx])
	camera.init()

	try:
		parent_widget = camera.get_config()
	except gp.GPhoto2Error:
		# 念の為例外を立てる処理はそのまま。意味があるかは不明
		raise RuntimeError("Unable to connect to Camera")

	return camera, parent_widget


def select_config_util(parent_widget, child_name, grandchild_name):
	"""
		Python-gPhoto2によって諸ステータスを取得する際に
		重複する部分をまるっとまとめたユーティリティ。

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
	# 設定対象の子ウィジェットを選択
	child_widget = parent_widget.get_child_by_name(child_name) 

	# 同 孫ウィジェットを選択
	grandchild_widget = child_widget.get_child_by_name(grandchild_name) 
	return grandchild_widget


def inner_start_capture(addr):
	"""
		撮影開始処理を担う実処理部分

		Parameters
		----------
		addr : char
			接続されているThetaのID
	"""

	# 実行対象設定済みのカメラオブジェクトと親ウィジェットを出力
	camera, camera_config = camera_control_util(addr)
	# 孫ウィジェットの取得
	movie = select_config_util(camera_config, 'actions', "movie")
	
	movie.set_value(1) # 値を指定
	camera.set_config(camera_config) # 値を適応

	camera.exit()


def start_capture(theta_list):
	"""
		本体関数inner_start_captureをマルチスレッドで実行するための関数

		Parameters
		----------
		theta_list : list
			接続されているThetaのリスト
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
		撮影終了処理を担う実処理部分
		現状Python-gPhoto2による方法が不明なため、CLI-gPhoto2で代用

		Parameters
		----------
		addr : char
			接続されているThetaのID
	"""

	# 実行対象設定済みのカメラオブジェクトと親ウィジェットを出力
	camera, camera_config = camera_control_util(addr)
	# 孫ウィジェットを取得して"."以下で値を指定
	select_config_util(camera_config, 'actions', "opcode").set_value("0x1018,0xFFFFFFFF")
	camera.set_config(camera_config) # 値を適応

	camera.exit()


def finish_capture(theta_list):
	"""
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
		theta_listに含まれる各Thetaのシリアル番号を出力する。  
		ループを関数外部にするか、否かは今後の実装次第。  
		逐一gp.Cameraを行っているが、これを一回やるでだけにできないか考えている。

		Parameters
		----------
		theta_list : list
			接続されているThetaのリスト
	"""

	for addr in theta_list:
		camera, camera_config = camera_control_util(addr)
		serial = select_config_util(camera_config, 'status', 'serialnumber').get_value()
		print(serial)
		camera.exit()
		print("")


def get_bat_lv(theta_list):
	"""
		theta_listに含まれる各Thetaの現在のバッテリーレベルを整数で出力する。  
		ループを関数外部にするか、否かは今後の実装次第。  
		逐一gp.Cameraを行っているが、これを一回やるでだけにできないか考えている。

		Parameters
		----------
		theta_list : list
			接続されているThetaのリスト
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
		ストレージ容量に起因する残時間の表示[second]

		Parameters
		----------
		theta_list : list
			接続されているThetaのリスト

		Returns
		-------
		result_list : list
			実行結果のリスト

	"""
	"""　
		# デバイスプロパティコード：0xD80D RemainingVideos
		# （ベンダー拡張プロパティ）
		# 詳細：https://api.ricoh/docs/theta-usb-api/property/remaining_videos/
	"""

	result_list = []
	for addr in theta_list:
		camera, camera_config = camera_control_util(addr)
		result_list.append(select_config_util(camera_config, 'other', 'd80d').get_value())
		camera.exit()
	return result_list


def get_files(theta_list):
	"""
		カメラからファイルをダウンロードして本体ストレージへ保存する。
		そして、カメラ側ストレージからこれを削除する。

		Parameters
		----------
		theta_list : list
			接続されているThetaのリスト
	"""
	# 保存先 親ディレクトリ
	PHOTO_DIR = os.path.expanduser('~/Pictures/from_camera')
	# 保存先 小ディレクトリ
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
		テスト
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
