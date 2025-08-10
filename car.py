import paho.mqtt.client as mqtt
import cv2
import re
import threading
from gpiozero import RGBLED,Robot,Motor
import time
import numpy as np
BROKER = "test.mosquitto.org"
PORT = 1883
TOPIC = "karist"

led = RGBLED(red=11,green=9,blue=10)
motor =Robot(left=Motor(4,14),right=Motor(17,18))

flag=False

def on_message(client, userdata,msg):
	global flag
	cmd = msg.payload.decode()
	print(cmd)
	
	if not flag and cmd != "6":
		
		if cmd == "1":
			motor.forward(0.5)
			led.color = (0,1,0)
			print("motor forward")
		elif cmd == '2':
			motor.stop()   
			led.color = (1,0,0)
			print("motor stop")
		elif cmd == '3':
			motor.left(0.5)
			led.color=(1,0,1)    
			print("motor left")
		elif cmd == '4':
			motor.right(0.5)
			led.color=(0,1,1)    
			print("motor right")
		elif cmd == '5':
			motor.backward(0.5) 
			led.color=(1,1,1)   
			print("motor backward")
	elif cmd=="6":
		motor.stop()
		flag = not flag
		print("auto")
def camera_thread():
	global motor
	cap = cv2.VideoCapture(0)
	if not cap.isOpened():
		print("Failed to open camera")
		return
	while True:
		ret, frame = cap.read()
		if not ret:
			break
		hsv_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
		frame_width = hsv_frame.shape[1]
		frame_center=frame_width//2
		lower_blue = np.array([100, 130, 50])
		upper_blue = np.array([140, 255, 255])
		
		blue_mask = cv2.inRange(hsv_frame,lower_blue,upper_blue)
		blue_objects = cv2.bitwise_and(frame,frame,mask=blue_mask)
		contours,_ = cv2.findContours(blue_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		if flag:
			for contour in contours:
				(x, y), radius = cv2.minEnclosingCircle(contour)
			
				if radius>=50:
					frame_width = frame.shape[1]             
					scaled_x = int(x / frame_width *180) 
					print(scaled_x)
					if scaled_x <=45:
						motor.right(0.5)
						led.color=(0,1,1)    
					elif scaled_x >= 145:
						motor.left(0.5)
						led.color=(1,0,1)
					else:
						motor.forward(0.5)
						led.color=(0,1,0)

			
		cv2.imshow('Webcam', frame)
		cv2.imshow('mask',blue_mask)
		if cv2.waitKey(1) == ord('q'):
			break
	cap.release()
	cv2.destroyAllWindows()
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_message = on_message
client.connect(BROKER, PORT, 60)
client.subscribe(TOPIC+"/Cmd")
client.loop_start()

threading.Thread(target=camera_thread,daemon=True).start()

try:
    while True:
        time.sleep(0.1) 
except KeyboardInterrupt:
    print("exit")
    client.loop_stop()
    client.disconnect()
finally:
	motor.stop()