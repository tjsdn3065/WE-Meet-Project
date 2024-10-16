import time

import serial
import re
import threading
import math

class Sensor:

    def __init__(self):
        self.angle = 0                                                      # 각도
        self.length = 0                                                     # 길이
        self.x = 0                                                          # 센서 기준 리더 x 좌표
        self.y = 0                                                          # 센서 기준 리더 y 좌표
        #self.v=None
        self.ser = serial.Serial('COM3', 19200, timeout=1)
        self.thread = threading.Thread(target=self.read)
        self.thread.daemon = True
        self.thread.start()


    def read(self):
        while True:
            output = self.ser.read(self.ser.in_waiting)
            if len(output) == 0: continue
            tfs_str = output.decode('utf-8')
            last_numbers = re.findall(r'-?\d+', tfs_str)
            #self.v=last_numbers
            self.length = int(last_numbers[1])
            self.angle = (int(last_numbers[0]) / 10.0) * (6.28 / 360)
            self.x = self.length*math.cos(self.angle)
            self.y = self.length*math.sin(self.angle)

#sensor = Sensor()
#while True:
    #print(str(sensor.length) + " " + str(sensor.angle))
    #print(sensor.v,type(sensor.v))
#     print(str(sensor.x) + " " + str(sensor.y))