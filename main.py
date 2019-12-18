#!/usr/bin/env python
# -*- coding: utf-8 -*-

# pylint: disable=C0111
# ↑プログラムの説明ドキュメントがないよ！というエラーの防止
import time
import pytheta

t_list = pytheta.connect_init()
print("実行結果{},type={}".format( t_list,type(t_list) ) )

pytheta.start_capture(t_list)

time.sleep(10)

pytheta.finish_capture(t_list)
