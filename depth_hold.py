# -*- coding: UTF-8 -*-
import time
import sys
from pymavlink import mavutil
import numpy as np
import math
f1 = 'depth.txt'
f2 = 'pitch.txt'
f3 = 'roll.txt'
f4 = 'time.txt'
f5 = 'pwm.txt'


class PID():
    # 初始化
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.output = 0.0

    # 设置目标值
    def set_target(self, target):
        self.SetPoint = target

    """
    计算PID值:
    u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
    """
    def calcu(self, feedback_value):
        error = self.SetPoint - feedback_value
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
        if delta_time >= self.sample_time:
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time
            if delta_time > 0:
                self.DTerm = delta_error / delta_time
            # 保存上一次误差和时间为下一次运算
            self.last_time = self.current_time
            self.last_error = error
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    # 设置Kp值
    def set_kp(self, p_gain):
        self.Kp = p_gain

    # 设置Ki值
    def set_ki(self, i_gain):
        self.Ki = i_gain

    # 设置Kd值
    def set_kd(self, d_gain):
        self.Kd = d_gain

    # 设间隔时间
    def set_time(self, sample_time):
        self.sample_time = sample_time


master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()


# Arm
def arm():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)


# 单个推进器控制
def set_motor_pwm(channel, pwm):
    master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                    0, channel, 1, pwm, 100, 1, 0, 0)


# 推力分配计算
def force_distri(out1, out2, out3):
    arr1 = np.array([[1, 1, 1, 1],
                    [-0.15, -0.15, 0.15, 0.15],
                    [0.25, -0.25, 0.25, -0.25]])
    arr2 = np.array([[out1], [out2], [out3]])
    a = np.mat(arr1)
    a1 = a.I
    a2 = np.mat(arr2)
    global out
    out = a1*a2


# 保存数据
def save_data(data1, data2, data3, data4, data5):
    with open(f1, 'a') as a:
        a.writelines(str(data1)+'\n')
    with open(f2, 'a') as b:
        b.writelines(str(data2)+'\n')
    with open(f3, 'a') as c:
        c.writelines(str(data3)+'\n')
    with open(f4, 'a') as d:
        d.writelines(str(data4)+'\n')
    with open(f5, 'a') as e:
        e.writelines(str(data5)+'\n')


# 创建三个实例
depth_hold = PID(10, 2, 0.01)
pitch_hold = PID(1, 0.8, 0)
roll_hold = PID(1, 0.8, 0)
depth = 0.0
pitch = 0.0
roll = 0.0
arm()
while True:
    try:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'VFR_HUD':
            alt = msg.alt
            depth = -alt
        if msg.get_type() == 'ATTITUDE':
            pitch = msg.pitch
            roll = msg.roll
        print("深度为 "+str(depth)+" 米")
        print("俯仰角为 "+str(pitch)+" pi")
        print("横倾角为 "+str(roll)+" pi")
        depth_hold.set_time(0)
        depth_hold.set_target(0.5)
        depth_hold.calcu(depth)
        pitch_hold.set_time(0)
        pitch_hold.set_target(0.000)
        pitch_hold.calcu(pitch)
        roll_hold.set_time(0)
        roll_hold.set_target(0.000)
        roll_hold.calcu(roll)
        print("定深PID输出为 "+str(depth_hold.output))
        print("俯仰PID输出为 "+str(pitch_hold.output))
        print("横滚PID输出为 "+str(roll_hold.output))
        force_distri(depth_hold.output, pitch_hold.output, roll_hold.output)
        forces = [out[0, 0], out[1, 0], out[2, 0], out[3, 0]]
        f = []
        for force in forces:
            if force > 5:
                force = 5
            elif force < -5:
                force = -5
            f.append(force)
        pwm1 = f[0]/5*400 + 1500
        pwm2 = f[1]/5*400 + 1500
        pwm3 = f[2]/5*400 + 1500
        pwm4 = f[3]/5*400 + 1500
        t = time.time()
        # save_data(depth, pitch, roll, t, [pwm1, pwm2, pwm3, pwm4])
        print("推进器输出PWM值分别为 "+str(pwm1)+' '+str(pwm2)+' '+str(pwm3)+' '+str(pwm4))
        set_motor_pwm(4, pwm1)
        set_motor_pwm(5, pwm2)
        set_motor_pwm(6, pwm3)
        set_motor_pwm(7, pwm4)
        print('\n')
    except KeyboardInterrupt:
        pass

