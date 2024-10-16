import threading
import time
import math
from Sensor import Sensor
from Leader_tracking import Driver
import matplotlib.pyplot as plt

l_ref = 500
a_ref = 0.5

def driving():
    while True:
        v = -0.5*(sensor.length - l_ref) # 양수는 후진 음수는 전진
        w = 2 * v / 255 * math.sin(sensor.angle)
        driver.drive_body(v, w,sensor.length,sensor.angle)
        time.sleep(0.1)

# 시각화를 위한 새 스레드
def visualize():
    while True:
        driver.setup_visualization()
        time.sleep(0.2)  # 매 5초마다 시각화 업데이트

sensor = Sensor()
driver = Driver()
drive_thread = threading.Thread(target=driving).start()

visualize_thread = threading.Thread(target=visualize)
visualize_thread.start()
