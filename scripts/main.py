#!/usr/bin/env python
# -*- coding: utf-8 -*-

# pylint: disable=C0111
# ↑プログラムの説明ドキュメントがないよ！というエラーの防止
import time
import pytheta



theta_list = pytheta.connect_init()

time.sleep(2)

pytheta.start_capture(theta_list)
time.sleep(3)
for _ in range(5):
	print(pytheta.get_bat_lv(theta_list) )
	time.sleep(1)
	print(pytheta.get_rem_time_v(theta_list) )
	print("")
	time.sleep(1)

time.sleep(2)

pytheta.finish_capture(theta_list)

