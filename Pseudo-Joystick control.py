import threading
import time
import math
from Sensor import Sensor
from Driver import Driver

l_ref = 500

def driving():
    while True:
        v = -0.6*(sensor.length - l_ref) # 양수는 후진 음수는 전진
        w = 2 * v / 255 * math.sin(sensor.angle)
        if(sensor.length < 5):
            driver.drive_body(0,0)
        else:
            driver.drive_body(v, w)
        time.sleep(0.02)

sensor = Sensor()
driver = Driver()
drive_thread = threading.Thread(target=driving).start()
