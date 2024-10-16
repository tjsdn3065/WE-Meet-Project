import time

import serial
import threading
import serial.rs485
import numpy as np

class Driver:
    def __init__(self):
        self.rw = 65
        self.Lw = 255
        self.v_max = 500
        self.w_max = 5

        self.v_ref = 0
        self.w_ref = 0
        self.speedL_ref = 0
        self.speedR_ref = 0

        self.speedL = 0
        self.speedR = 0
        self.v = 0                                  # 선속도 (mm/s)
        self.w = 0                                  # 각속도 (rad/s)

        # self.A=np.array([[       self.rw/2,       self.rw/2],
        #                  [-self.rw/self.Lw, self.rw/self.Lw]])
        # self.q=None
        # self.left_wheel_w=0
        # self.right_wheel_w = 0
        # self.wheel_w=None

        self.writing = False

        self.ser = serial.Serial('COM7', 19200, 8, 'N', 1)
        # self.ser = serial.rs485.RS485('COM4', 19200, 8, 'N', 1, write_timeout=None)
        # self.ser.rs485_mode = serial.rs485.RS485Settings(rts_level_for_tx=True, rts_level_for_rx=True, loopback=False, delay_before_tx=None, delay_before_rx=None)
        self.thread = threading.Thread(target=self.read)
        self.thread.daemon = True
        self.thread.start()

    def get_speed(self, data):
        if len(data) >= 24:
            sum_data = sum(data[0:23])
            chk_num = (~sum_data) + 1 & 0xFF
            if chk_num == data[23]:
                # print("sucess")
                combinedR = (data[6] << 8) | data[5]
                combinedL = (data[15] << 8) | data[14]
                if combinedR >= 0x8000:
                    combinedR -= 0x10000
                if combinedL >= 0x8000:
                    combinedL -= 0x10000
                # print(combinedR)
                return combinedR, combinedL, data[24:]
            else:
                print("failed")
                return self.speedR, self.speedL, data[5:]
        else:
            return self.speedR, self.speedL, data

    def find_pattern(self, lst):
        pattern = [128, 183, 1, 210, 18]
        pattern_len = 5
        for i in range(len(lst) - pattern_len + 1):
            if lst[i:i + pattern_len] == pattern:
                return i
        return -1

    def read(self):
        data = [183, 128, 1, 10, 1, 61]
        self.send_data(data)                                # 초기화
        data = []
        while True:
            while self.writing: continue                    # 초기화가 잘 진행 되었다면
            output = self.ser.read(self.ser.in_waiting)
            integer_list = list(output)
            if len(integer_list) == 0: continue
            data += integer_list
            index = self.find_pattern(data)
            if(index == -1): continue
            data = data[index:]
            self.speedR, self.speedL, data = self.get_speed(data)
            self.v = (self.rw * 0.1047)*(self.speedR + self.speedL)/2
            self.w = (self.rw * 0.1047)*(self.speedR - self.speedL)/self.Lw
            # self.q = np.array([[self.v],
            #                    [self.w]])
            # self.wheel_w=np.linalg.inv(self.A).dot(self.q)
            # self.left_wheel_w=self.wheel_w[0][0]
            # self.right_wheel_w = self.wheel_w[1][0]
            # self.drive_body(0, 0)
            # time.sleep(0.02)

    def drive_body(self, v_ref, w_ref):
        v_ref = max(-self.v_max, min(self.v_max, v_ref))
        w_ref = max(-self.w_max, min(self.w_max, w_ref))
        self.v_ref = v_ref
        self.w_ref = w_ref
        speedL_ref = (self.v_ref - self.w_ref * self.Lw / 2.0) / (self.rw*0.1047)
        speedR_ref = (self.v_ref + self.w_ref * self.Lw / 2.0) / (self.rw*0.1047)
        self.drive_motors(speedR_ref, speedL_ref)

    def drive_motors(self,speedR_ref, speedL_ref):
        self.speedR_ref = speedR_ref
        self.speedL_ref = speedL_ref
        sr1, sr2 = self.encode_motor_speed(self.speedR_ref)
        sl1, sl2 = self.encode_motor_speed(self.speedL_ref)
        data = [183, 128, 1, 207, 7, 1, sr1, sr2, 1, sl1, sl2, 0]
        self.send_data(data)

    def encode_motor_speed(self, speed):
        s1 = int(speed % 256)
        s2 = int(speed / 256)
        if speed < 0:
            s2 = s2 + 255
        return s1, s2

    def send_data(self, data):
        sum_data = sum(data)
        chk_num = (~sum_data) + 1 & 0xFF
        data.append(chk_num)
        bytes_data = bytes(data)
        self.ser.reset_output_buffer()
        self.writing = True
        self.ser.write(bytes_data)
        self.writing = False

# driver=Driver()
#driver.drive_motors(0,0)
#driver.drive_body(0,0)
#while True:
#    print(driver.wheel_w)
#    print(driver.right_wheel_w)
#    time.sleep(1)