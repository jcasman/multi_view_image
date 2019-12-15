#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 1秒おきと2秒おきにprintする処理をthreadで実行
import threading
import time

def worker(interval):
	for n in range(10,1):
		time.sleep(interval)
		print("%s --> %d" % (threading.current_thread().name, n))

th1 = threading.Thread(name="a", target=worker, args=(1,))
th2 = threading.Thread(name="b", target=worker, args=(5,))

th1.start()
th2.start()
