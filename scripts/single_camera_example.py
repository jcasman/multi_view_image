import pytheta
import time

theta_list = pytheta.connect_init()
print(theta_list)
print("THETA Battery Level:" + str(pytheta.get_bat_lv(theta_list)) )
time.sleep(1)
print("THETA Serial Number below")
pytheta.get_serial(theta_list)
print("")
time.sleep(1)
print("Camera recording remaining time in seconds: " + str(pytheta.get_rem_time_v(theta_list)) )
print("")
time.sleep(1)
camera, camera_config = pytheta.camera_control_util(theta_list[0])

movie = pytheta.select_config_util(camera_config, 'actions', "movie")
# set to record video
movie.set_value(1)
camera.set_config(camera_config)
camera.exit()
print("camera video recording started")
print("")

# take 4 seconds of video
time.sleep(4)

# stop video recording
camera, camera_config = pytheta.camera_control_util(theta_list[0])
pytheta.select_config_util(camera_config, 'actions', "opcode").set_value("0x1018,0xFFFFFFFF")
camera.set_config(camera_config) 
camera.exit()
print("camera video recording stopped")
