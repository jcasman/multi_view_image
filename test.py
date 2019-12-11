#!/usr/bin/env python
# -*- coding: utf-8 -*-

# pylint: disable=C0111
# ↑プログラムの説明ドキュメントがないよ！というエラーの防止
import PyTheta

t_list = PyTheta.connect_init()
print("実行結果{},type={}".format( t_list,type(t_list) ) )

PyTheta.start_capture(t_list)
PyTheta.finish_capture(t_list)
