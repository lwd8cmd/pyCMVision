import pyCMVision as cam_module
import cv2

cam = cam_module.Camera()

strformat = "{0[name]: <40}|{0[value]: >5}|{0[default]: >7}|{0[min]: >6}|{0[max]: >6}|{0[step]: >5}|"
print(strformat.format({"name":"name", "value":"value", "default":"default", "min":"min", "max":"max", "step":"step"}))
print("-"*75)
for p in cam.settings():
	print(strformat.format(p));

while cv2.waitKey(1) & 0xff is not ord('q'):
	cv2.imshow('tava', cam.image("bgr"))
