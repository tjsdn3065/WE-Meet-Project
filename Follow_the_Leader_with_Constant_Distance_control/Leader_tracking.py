import math
import time

import serial.rs485
import threading
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.constants import golden, golden_ratio


class Driver:
    def __init__(self):
        self.rw = 65
        self.Lw = 255
        self.v_max = 500
        self.w_max = 3

        self.v_ref = 0
        self.w_ref = 0
        self.speedL_ref = 0
        self.speedR_ref = 0

        self.speedL = 0
        self.speedR = 0
        self.v = 0                                  # 선속도 (mm/s)
        self.w = 0                                  # 각속도 (rad/s)

        self.A=np.array([[       self.rw/2,       self.rw/2],
                         [-self.rw/self.Lw, self.rw/self.Lw]])
        self.q=None
        self.left_wheel_w=0
        self.right_wheel_w = 0
        self.wheel_w=None
        self.predict_direction=0
        self.dt=0.01
        self.s_r=None
        self.robot_position_x = 0
        self.robot_position_y = 0
        self.k=1
        self.new_leader_position_x = 0
        self.new_leader_position_y = 0
        self.leader_position_x = 0
        self.leader_position_y = 0
        self.s_l = None
        self.leader_positions = []  # 리더의 위치를 저장할 리스트 초기화
        self.k_a=0.3
        self.k_b=0.7

        self.robot_trace = []  # 로봇 위치 기록
        self.leader_trace = []  # 리더 위치 기록

        self.writing = False

        self.ser = serial.Serial('COM4', 19200, 8, 'N', 1, timeout=1, write_timeout=1)
        #self.ser = serial.Serial('COM7', 19200, 8, 'N', 1)
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

    def drive_body(self, v_ref, w_ref,length_ref,angle_ref):
        if length_ref < 5:
            self.drive_motors(0, 0)
        else:
            # 단계 1: dead reckoning을 사용하여 로봇의 자세를 추정
            # 로봇 방향 계산
            self.predict_direction=self.predict_direction + self.w * self.dt
            # 로봇이 이동한 거리
            self.s_r = self.v * self.dt
            # 로봇 위치 업데이트
            self.robot_position_x = self.robot_position_x + self.s_r * math.cos(self.k*self.predict_direction + angle_ref)
            self.robot_position_y = self.robot_position_y + self.s_r * math.sin(self.k*self.predict_direction + angle_ref)

            # 단계 2: 리더의 위치 계산
            self.new_leader_position_x = self.robot_position_x + length_ref * math.cos(self.k*self.predict_direction + angle_ref)
            self.new_leader_position_y = self.robot_position_y + length_ref * math.sin(self.k*self.predict_direction + angle_ref)
            # 리더의 새 위치를 리스트에 추가
            self.leader_positions.append((self.new_leader_position_x, self.new_leader_position_y))

            # 단계 3: 리더가 이동한 거리 계산
            #self.s_l=math.sqrt((self.new_leader_position_x-self.leader_position_x)**2 +(self.new_leader_position_y-self.leader_position_y)**2)
            self.leader_position_x = self.new_leader_position_x
            self.leader_position_y = self.new_leader_position_y

            #print(self.robot_position_x,self.robot_position_y)
            #print(self.leader_position_x,self.leader_position_y)
            #print(self.s_l)

            self.update_positions((self.robot_position_x/1000, self.robot_position_y/1000), (self.new_leader_position_x/1000, self.new_leader_position_y/1000))

            # 단계 4: 목표 각도 결정
            position = self.leader_find_first_long_distance_position(500)
            self.remove_long_distance_position(1000)
            if position is None:
                self.v_ref = max(-self.v_max, min(self.v_max, v_ref))
                self.w_ref = max(-self.w_max, min(self.w_max, w_ref))
            else:
                goal_angle=math.atan2(position[1]-self.robot_position_y,position[0]-self.robot_position_x)-self.predict_direction
                #print(goal_angle)
                v = -0.5*(length_ref-500)*(1-self.k_a*abs(goal_angle))
                w = -self.k_b*goal_angle

                self.v_ref = max(-self.v_max, min(self.v_max, v_ref))
                self.w_ref = max(-self.w_max, min(self.w_max, w_ref))

            speedL_ref = (self.v_ref - self.w_ref * self.Lw / 2.0) / (self.rw * 0.1047)
            speedR_ref = (self.v_ref + self.w_ref * self.Lw / 2.0) / (self.rw * 0.1047)
            # self.drive_motors(0, 0)
            self.drive_motors(speedR_ref, speedL_ref)


    def leader_find_first_long_distance_position(self, threshold=500):
        """리더 위치가 주어진 임계값 이상인 첫 번째 위치를 뒤에서부터 찾고, 찾은 위치 이전의 모든 데이터를 제거합니다."""
        for i in range(len(self.leader_positions) - 1, -1, -1):
            position = self.leader_positions[i]
            distance = math.sqrt((self.leader_position_x - position[0]) ** 2 + (self.leader_position_y - position[1]) ** 2)
            if distance > threshold:
                #self.leader_positions = self.leader_positions[i:]  # 찾은 위치에서 리스트 끝까지 데이터를 유지
                return position
        return None  # 조건을 만족하는 위치가 없을 경우 None 반환

    def robot_find_first_long_distance_position(self, threshold=500):
        """로봇 위치를 기준으로 하여 리더 위치가 주어진 임계값 이상인 첫 번째 위치를 찾습니다."""
        for position in self.leader_positions:
            distance = math.sqrt(
                (self.robot_position_x - position[0]) ** 2 + (self.robot_position_y - position[1]) ** 2)
            if distance > threshold:
                return position
        return None  # 조건을 만족하는 위치가 없을 경우 None 반환

    def remove_long_distance_position(self, threshold=1000):
        """리더 위치가 주어진 임계값 이상인 첫 번째 위치를 뒤에서부터 찾고, 찾은 위치 이전의 모든 데이터를 제거합니다."""
        for i in range(len(self.leader_positions) - 1, -1, -1):
            position = self.leader_positions[i]
            distance = math.sqrt((self.leader_position_x - position[0]) ** 2 + (self.leader_position_y - position[1]) ** 2)
            if distance > threshold:
                self.leader_positions = self.leader_positions[i:]  # 찾은 위치에서 리스트 끝까지 데이터를 유지
                break

    def update_positions(self, new_robot_pos, new_leader_pos):
        self.robot_trace.append(new_robot_pos)
        self.leader_trace.append(new_leader_pos)

    def animate_positions(self, frame):
        if self.robot_trace and self.leader_trace:  # 로봇과 리더의 위치 데이터가 있는지 확인
            robot_x, robot_y = zip(*self.robot_trace)
            leader_x, leader_y = zip(*self.leader_trace)

            self.ax.clear()  # 이전 그래프 데이터 삭제
            self.ax.plot(robot_x, robot_y, 'bo-', label='Robot Path')
            self.ax.plot(leader_x, leader_y, 'ro-', label='Leader Path')
            self.ax.scatter(robot_x[-1], robot_y[-1], color='blue', s=100, label='Current Robot Position')
            self.ax.scatter(leader_x[-1], leader_y[-1], color='red', s=100, label='Current Leader Position')
            self.ax.set_title('Robot and Leader Trajectory')
            self.ax.set_xlabel('X position')
            self.ax.set_ylabel('Y position')
            self.ax.legend()
            self.ax.axis('equal')
        else:
            print("No data available for visualization.")

    def setup_visualization(self):
        self.fig, self.ax = plt.subplots()
        # FuncAnimation을 설정합니다. 이 함수는 정의된 animate_positions를 주기적으로 호출합니다.
        ani = FuncAnimation(self.fig, self.animate_positions, interval=200)
        plt.show()

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