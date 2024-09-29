import numpy as np
from math import sin,cos,atan
from datetime import datetime

class Tether_Control():

    def __init__(self,l_m=None,l_d=None,theta=None):
        self.cur_time = datetime.now()                              # 현재 t
        self.prev_time=None                                         # 이전 t
        self.l_m=l_m                                                # 측정된 테더 길이
        self.l_d=l_d                                                # 원하는 테더 길이
        self.theta=theta                                            # 측정된 테더 각도
        self.b=0.3                                                  # 팔로워 폭
        self.r=0.05                                                 # 바퀴 반지름
        self.k_p=None                                               # 비례 속도 이득
        self.k_a=None                                               # 평행 이동 속도 제어 이득
        self.k_b=None                                               # 각속도 제어 이득
        self.goal_angle = self.theta                                # 목표 각도
        self.V_r = -self.k_p*(self.l_m-self.l_d)                    # 평행 이동 속도
        self.W_r = -2*self.V_r/self.b*sin(self.goal_angle)          # 각속도
        self.w_L=None                                               # 왼쪽 바퀴 각속도
        self.w_R=None                                               # 오른쪽 바퀴 각속도
        self.A=np.array([ [self.r/2,           self.r/2],           # 바퀴의 각속도를 몸체의 속도로 변환
                          [-self.r/self.b,self.r/self.b] ])
        self.q=np.array([ [self.V_r],
                          [self.W_r] ])
        self.A_inverse=np.linalg.inv(self.A)                        # 역행렬 A
        self.V=self.A_inverse.dot(self.q)
        self.P=np.array([ [0],                                      # 팔로워의 위치
                          [0] ])
        self.T_list=[]                                              # 테더 위치 기록
        self.T=np.array([ [self.l_m*cos(self.theta)],               # 테더의 위치
                          [self.l_m*sin(self.theta)] ])
        self.prev_T=None                                            # 이전 테더 위치
        self.P_direction=None                                       # 팔로워의 방향
        self.s_r=None                                               # 테더가 이동한 거리
        self.tether_angle=None                                      # 관성 기준 프레임에서의 테더 각도
        self.E=np.array([ [cos(self.tether_angle),-sin(self.tether_angle)], # 테더 이동량 변환 행렬
                          [sin(self.tether_angle), cos(self.tether_angle)] ])
        self.s_t=None                                               # 테더 끝이 이동한 거리

    def calculate_the_orientation_of_the_follower_in_inertial_reference_frame(self):  # 관성 기준 프레임에서 팔로워 방향 계산
        prev_P_direction=self.P_direction
        self.P_direction=prev_P_direction+self.W_r*(self.cur_time-self.prev_time)

    def  calculate_the_distance_traveled_by_the_tether(self):      # 테더가 이동한 거리 계산
        self.s_r=self.V_r*(self.cur_time-self.prev_time)

    def update_the_position_of_the_follower(self):                  # 팔로워의 위치 업데이트
        prev_P=self.P
        self.P=prev_P+self.E.dot(np.array([ [self.s_r],
                                                   [0] ]))

    def compute_the_position_of_the_tether_tip(self):               # 테더 끝 위치 계산
        self.prev_T=self.T
        self.T=self.P+self.E.dot(np.array([ [self.l_m],
                                                   [0] ]))
        self.T_list.append(self.T)

    def calculate_the_distance_travelled_by_the_tether_tip(self):   # 테더 끝이 이동한 거리 계산
        self.s_t=np.linalg.norm((self.T-self.prev_T))

    def determine_the_target_angle(self):
        for t in self.T_list:
            if self.l_d >= np.linalg.norm((self.T - t)):
                T_s_t_const=t
            else:
                break
        self.goal_angle=atan(((T_s_t_const[1][0]-self.P[1][0])/(T_s_t_const[0][0]-self.P[0][0]))-self.P_direction)





def main():
    tether_control=Tether_Control()
