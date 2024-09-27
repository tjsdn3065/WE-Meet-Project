import numpy as np
from math import sin,cos
from datetime import datetime

class Tether_Control():

    def __init__(self,l_m=None,l_d=None,theta=None):
        self.cur_time = datetime.now()                              # 현재 t
        self.prev_time=None                                         # 이전 t
        self.l_m=l_m                                                # 측정된 테더 길이
        self.l_d=l_d                                                # 원하는 테더 길이
        self.theta=theta                                            # 측정된 테더 각도
        self.b=0.3                                                  # 로봇 폭
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
        self.P=np.array([ [0],                                      # 로봇의 위치
                          [0] ])
        self.T=np.array([ [self.l_m*cos(self.theta)],               # 테더의 위치
                          [self.l_m*sin(self.theta)] ])
        self.P_direction=None                                       # 로봇의 방향





def main():
    tether_control=Tether_Control()

