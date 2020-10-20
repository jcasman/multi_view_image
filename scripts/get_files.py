#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
カメラからファイルをダウンロードして本体ストレージへ保存する。
そして、カメラ側ストレージからこれを削除する。
"""



from __future__ import print_function

from datetime import datetime
import logging
import os
import sys

import gphoto2 as gp


# 備忘録のコメントアウト類が多いことと、単機能のわりにそもそもコードが複雑で長いので、
# 本体と分離する形で実装とする。
# Python-gphoto2 のサンプルコードcopy-filesを元に開発。
# 一部clear-spaceを流用(削除部分)


def get_target_dir(timestamp,PHOTO_DIR, PHOTO_SUB_DIR): # -> dest_dir
	"""
		保存先の小ディレクトリのパスを
		datatime型のフォーマットに則って出力する関数

		Parameters
		----------
		timestamp : datatime
			ファイルの情報のタイムスタンプから変換したdatatime

		PHOTO_DIR : char
			保存先 親ディレクトリ

		PHOTO_SUB_DIR
			保存先 小ディレクトリ
			datatime型のフォーマットに沿った変数を使用可能

		Returns
		-------
		dest_dir : char
			保存先の小ディレクトリのパス
	"""
	return os.path.join(PHOTO_DIR, timestamp.strftime(PHOTO_SUB_DIR) )


def list_computer_files(PHOTO_DIR): # -> computer_files
	"""
		コンピュータ:PHOTO_DIR内のフォルダを走査しファイル一覧を取得する関数。
		ついでにサムネイルフォルダが消される。

		Parameters
		----------
		PHOTO_DIR : char
			保存先 親ディレクトリ

		Returns
		-------
		computer_files : list
			コンピュータ:PHOTO_DIR内のファイル一覧
	"""
	computer_files = []

	# os.walk とは、ディレクトリの走査を行う関数である。
	# 参考：https://www.sejuku.net/blog/63816
	for root, dirs, files in os.walk(PHOTO_DIR):
		for name in files:
			if '.thumbs' in dirs:
				dirs.remove('.thumbs')
				# サムネイルフォルダがあれば、削除する。
			if name in ('.directory',):
				continue
			ext = os.path.splitext(name)[1].lower()
			if ext in ('.db',):
				continue
			computer_files.append(os.path.join(root, name))
	return computer_files


def list_camera_files(camera, path='/'): # -> camera_files
	"""
		カメラ内のフォルダを走査しファイル一覧を取得する関数。
		再帰的実行を駆使して走査する。

		Parameters
		----------
		camera : gp.camera object
		path : char

		Returns
		-------
		camera_files : list
			カメラ内のファイル一覧
	"""
	camera_files = []
	# ファイルリスト獲得
	gp_list = gp.check_result( gp.gp_camera_folder_list_files(camera, path) )
	for name, _ in gp_list:
		camera_files.append(os.path.join(path, name))

	# フォルダリスト獲得
	folders = []
	gp_list = gp.check_result( gp.gp_camera_folder_list_folders(camera, path) )
	for name, _ in gp_list:
		folders.append(name)

	# サブフォルダを巡回(再起的)
	for name in folders:
		camera_files.extend( list_camera_files( camera, os.path.join(path, name) ) )
	return camera_files


def get_camera_file_info(camera, path): # => file info object
	"""
		pathのファイルの情報を収めたオブジェクトを取得する関数。

		Parameters
		----------
		camera : gp.camera object
		path : char

		Returns
		-------
		file info object
	"""
	folder, name = os.path.split(path)
	return gp.check_result( gp.gp_camera_file_get_info(camera, folder, name) )


def delete_file(camera, path):
	"""
		pathのファイルの情報を収めたオブジェクトを取得する関数。

		Parameters
		----------
		camera : gp.camera object
		path : char
	"""
	folder, name = os.path.split(path)
	print("カメラ側の[{}]の削除開始".format(name) )
	camera.file_delete(folder, name)
	print("カメラ側の[{}]の削除終了".format(name) )


def get_files(camera, PHOTO_DIR, PHOTO_SUB_DIR):
	"""
		カメラからファイルをダウンロードして本体ストレージへ保存する。
		そして、カメラ側ストレージからこれを削除する。
		Parameters
		----------
		camera : gp.camera object

		Returns
		-------
		return code

	"""
	logging.basicConfig(
		format='%(levelname)s: %(name)s: %(message)s', level=logging.WARNING)
	callback_obj = gp.check_result(gp.use_python_logging())

	# カメラ内のフォルダを走査しファイル一覧を取得
	computer_files = list_computer_files(PHOTO_DIR)


	print('カメラからのファイルリストの取得中...')
	camera_files = list_camera_files(camera)
	if not camera_files:
		print('ファイルが見つかりませんでした。')
		return 1
	print('ファイルのダウンロード開始...')
	for path in camera_files:
		#path に保存対象のファイルのフルパスが与えられる
		#info に保存対象のファイルの情報を格納したオブジェクトが与えられる
		info = get_camera_file_info(camera, path)

		#timestamp に保存対象のファイルの日付情報がdatatime形式で与えられる
		timestamp = datetime.fromtimestamp(info.file.mtime)

		#path をディレクトリ名:folderとファイル名:nameに分ける
		folder, name = os.path.split(path)

		#保存先ディレクトリを取得
		dest_dir = get_target_dir(timestamp, PHOTO_DIR, PHOTO_SUB_DIR)

		#保存先ディレクトリとファイル名:nameを合わせ、dest_dirを生成
		dest = os.path.join(dest_dir, name)

		if dest in computer_files:
			# すでにダウンロード済みだったものの場合これ以下が実行される。
			# 下の工程で本体へ保存したものは削除しているので、
			# このループに入ることは何らかの原因で削除失敗した場合に限る。
			# つまりこの部分は念押し用。
			print("[{}]はすでに存在します".format(name) )

			#本体へ保存したのでカメラ側は削除
			delete_file(camera, path)

			# 以後の工程をパスします
			continue

		# 保存工程開始
		print('[{}] を [{}] へ保存'.format(path, dest_dir) )

		# 保存先フォルダがないなら作る
		if not os.path.isdir(dest_dir):
			os.makedirs(dest_dir)
			print("[{}] は存在しないため作成".format(dest_dir) )

		print("[{}]のダウンロード開始".format(name) )
		# 保存対象を決定？
		camera_file = gp.check_result(gp.gp_camera_file_get(camera, folder, name, gp.GP_FILE_TYPE_NORMAL))

		# 保存の実体　そして保存できたかの確認？ここで成否分岐しなくていいのか？
		gp.check_result(gp.gp_file_save(camera_file, dest))

		print("[{}]のダウンロード完了".format(name) )

		#本体へ保存したのでカメラ側は削除
		delete_file(camera, path)

	print("全保存工程完了")
	camera.exit()
	return 0


def _unittest():
	"""
	実行テスト
	"""

	# 保存先 親ディレクトリ
	PHOTO_DIR = os.path.expanduser('~/Pictures/from_camera')
	# 保存先 小ディレクトリ
	PHOTO_SUB_DIR = '%Y/%Y_%m_%d/'

	camera = gp.Camera()
	get_files(camera, PHOTO_DIR, PHOTO_SUB_DIR)


if __name__ == "__main__":
	sys.exit(_unittest())
