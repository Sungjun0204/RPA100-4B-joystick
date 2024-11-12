#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

# 상수
DEG2RAD = 0.0174533
RAD2DEG = 57.2958
MM2M = 0.001

# 로봇 파라미터 설정
L1 = 0.549846  # 링크 1의 길이 [m]
L2 = 0.449478  # 링크 2의 길이 [m]
m1 = 10.  # 링크 1의 질량 [kg]
m2 = 10.  # 링크 2의 질량 [kg]
I1 = (1 / 12) * m1 * L1 ** 2  # 링크 1의 관성 모멘트 [kg*m^2]
I2 = (1 / 12) * m2 * L2 ** 2  # 링크 2의 관성 모멘트 [kg*m^2]
m_device = 18  # 장치의 질량 [kg]
I_device_cm = (1 / 12) * m_device * (0.2 ** 2 + 0.3 ** 2)  # [kg*m^2] 물체의 COM기준 관성 모멘트

# SCARA 로봇의 원점 설정
# scara_origin = np.array([0.920, 0.090])  # SCARA 로봇의 베이스 위치 [m]
scara_origin = np.array([0., 0.])  # SCARA 로봇의 베이스 위치 [m]

dt = 0.005  # 시간 간격 [s]
Kp = np.diag([100, 100])  # PD 제어기의 비례 게인
Kd = np.diag([20, 20])  # PD 제어기의 미분 게인
M_d = np.diag([1, 1])  # 가상의 질량 행렬
D_d = np.diag([50, 50])  # 댐핑 행렬
K_d = np.diag([150, 150])  # 강성 행렬
distance_threshold = 0.05  # 로봇 팔과 점 사이의 최대 거리 [m]

# 초기 설정
q = np.array([-3.975, 25.543]) * DEG2RAD  # 초기 관절 각도 [rad]
qd = np.array([0.0, 0.0])  # 초기 관절 각속도 [rad/s]
end_point = np.array([0.1, 0.1])  # 목표 지점 [m]

# 퍼블리셔와 서브스크라이버 콜백 함수 설정
pub = None
w_axis_angle = 0.0  # 초기 W축 각도 설정
initial_direction = None  # 초기 방향 설정
calculation_done = False  # 계산이 완료되었는지 여부를 저장하는 플래그






# /scara_coordi 메세지 구독 콜백 함수
def scara_coordi_callback(data):
    global scara_end_effector_pos, q, w_axis_angle, MM2M, DEG2RAD, initial_direction
    # /scara_coordi로부터 받은 데이터에서 엔드 이펙터 위치와 각 관절 각도 갱신
    scara_end_effector_pos = np.array(data.data[:2]) * MM2M  # 엔드 이펙터의 위치 [m] (XY 좌표만 사용)
    q = np.array(data.data[3:5]) * DEG2RAD  # 1번, 2번 관절 각도 [rad]
    w_axis_angle = data.data[6] * DEG2RAD  # W축의 각도 [rad]

    # 초기 방향 설정 (처음 받은 데이터로 설정)
    if initial_direction is None:
        initial_direction = np.array([np.cos(w_axis_angle), np.sin(w_axis_angle)])


def magnet_callback(data):
    global q, qd, scara_end_effector_pos, w_axis_angle, initial_direction, calculation_done
    
    # 구독된 자석 위치
    p = np.array(data.data[:2])  # 자석의 위치 [m] (XY 좌표만 사용)

    # 현재 엔드 이펙터 위치 사용 (구독된 데이터 사용)
    x = scara_end_effector_pos

    # 자코비안 계산
    J = jacobian(q, L1, L2)
    
    # 엔드 이펙터 속도 계산
    x_dot = np.dot(J, qd)

    # 점과 로봇 팔의 거리 계산
    distance = np.linalg.norm(x - p)

    # 임피던스 제어 적용
    e_p = p - x  # 위치 오차
    e_v = -x_dot  # 속도 오차
    F_impedance = np.dot(M_d, np.zeros(2)) + np.dot(D_d, e_v) + np.dot(K_d, e_p)

    # 목표 위치 계산
    if distance > distance_threshold:
        x_target = p
    else:
        x_target = end_point #+ scara_origin[:2]

    # 목표 위치 방향 및 계산
    direction_to_target = x_target - x
    if np.linalg.norm(direction_to_target) > 1e-6:
        direction_to_target /= np.linalg.norm(direction_to_target)
    else:
        direction_to_target = np.array([0.0, 0.0])

    step_size = 0.1
    x_desired = x + direction_to_target * step_size

    # 목표 위치로의 역기구학 계산
    q_desired = inverse_kinematics(x_desired - scara_origin[:2], L1, L2)

    # W축의 각도 유지 계산 (글로벌 좌표계 기준으로 초기 방향 유지)
    if initial_direction is not None:
        current_direction = np.array([np.cos(w_axis_angle), np.sin(w_axis_angle)])
        angle_difference = np.arctan2(initial_direction[1], initial_direction[0]) - np.arctan2(current_direction[1], current_direction[0])
        w_desired = w_axis_angle + angle_difference
    else:
        w_desired = w_axis_angle

    # 관절 공간에서의 오차
    e_q = q_desired - q
    e_qd = -qd

    # 관절 토크 계산
    tau = np.dot(Kp, e_q) + np.dot(Kd, e_qd) + np.dot(J.T, F_impedance)

    # 동역학 행렬 계산
    M, C, G = dynamics_matrices(q, qd, m1, m2, L1, L2, I1, I2, m_device, I_device_cm)

    # 관절 가속도 계산
    epsilon = 1e-6
    M_reg = M + epsilon * np.eye(len(M))
    qdd = np.linalg.solve(M_reg, tau - np.dot(C, qd) - G)

    # 상태 업데이트
    qd += qdd * dt
    q += qd * dt

    # 계산 완료 후 플래그 설정
    calculation_done = True


    # 관절 각도를 발행 (q1, q2, w 축 각도 포함)
    publish_joint_angles(q, w_desired)

    # 다음 이동 좌표를 발행
    # msg = Float32MultiArray()
    # msg.data = (forward_kinematics(q, L1, L2)*1000).tolist() + [1]
    # pub.publish(msg)

    # # 관절 각도를 발행 (q1, q2, w 축 각도 포함)
    # msg = Float32MultiArray()
    # q_deg = q * RAD2DEG
    # w_desired_deg = w_desired * RAD2DEG
    # msg.data = q_deg.tolist() + [w_desired_deg]
    # pub.publish(msg)


def publish_joint_angles(q, w_desired):
    global pub, calculation_done

    # 계산 완료된 경우에만 발행
    if calculation_done:
        msg = Float32MultiArray()
        q = q * RAD2DEG
        w_desired = w_desired * RAD2DEG
        msg.data = q.tolist() + [w_desired]
        pub.publish(msg)
        calculation_done = False  # 발행 후 플래그 초기화




# 순방향 운동학 함수
def forward_kinematics(q, L1, L2):
    theta1, theta2 = q
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)
    return np.array([x2, y2])

# 역기구학 함수
def inverse_kinematics(x, L1, L2):
    x_end, y_end = x
    cos_theta2 = (x_end**2 + y_end**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1, 1)
    sin_theta2 = np.sqrt(1 - cos_theta2**2)
    theta2 = np.arctan2(sin_theta2, cos_theta2)
    k1 = L1 + L2 * cos_theta2
    k2 = L2 * sin_theta2
    theta1 = np.arctan2(y_end, x_end) - np.arctan2(k2, k1)
    return np.array([theta1, theta2])

# 자코비안 계산 함수
def jacobian(q, L1, L2):
    theta1, theta2 = q
    J11 = -L1 * np.sin(theta1) - L2 * np.sin(theta1 + theta2)
    J12 = -L2 * np.sin(theta1 + theta2)
    J21 = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    J22 = L2 * np.cos(theta1 + theta2)
    return np.array([[J11, J12], [J21, J22]])

# 동역학 행렬 계산 함수
def dynamics_matrices(q, qd, m1, m2, L1, L2, I1, I2, m_device, I_device_cm):
    theta1, theta2 = q
    theta1_dot, theta2_dot = qd
    
    # 기존 질량 행렬 계산
    M11 = I1 + I2 + (m1 * (L1 ** 2)) / 4 + m2 * (L1 ** 2 + (L2 ** 2) / 4 + L1 * L2 * np.cos(theta2))
    M12 = I2 + m2 * ((L2 ** 2) / 4 + (L1 * L2 * np.cos(theta2)) / 2)
    M21 = M12
    M22 = I2 + m2 * (L2 ** 2) / 4
    M = np.array([[M11, M12], [M21, M22]])

    # 코리올리 및 원심력 행렬 C 계산
    C11 = -m2 * L1 * L2 * np.sin(theta2) * theta2_dot / 2
    C12 = -m2 * L1 * L2 * np.sin(theta2) * (theta1_dot + theta2_dot) / 2
    C21 = m2 * L1 * L2 * np.sin(theta2) * theta1_dot / 2
    C22 = 0
    C = np.array([[C11, C12], [C21, C22]])

    # 중력 벡터 G (중력 제거)
    G = np.array([0.0, 0.0])

    return M, C, G


##############
### 메인함수 ###
##############

def impedance_control():
    global pub, scara_end_effector_pos

    #### node setting ####
    rospy.init_node('impedance_control', anonymous=True)

    #### PUblish section ####
    pub = rospy.Publisher('scara_joint_commands', Float32MultiArray, queue_size=10)

    #### Subscribe section ####
    rospy.Subscriber('magnet_pos', Float32MultiArray, magnet_callback)
    rospy.Subscriber('scara_coordi', Float32MultiArray, scara_coordi_callback)
    
    rate = rospy.Rate(1) # 100hz
    scara_end_effector_pos = np.zeros(2)  # 초기값 설정 (XY 좌표만 사용)

    #### 알고리즘 파트 ####
    while not rospy.is_shutdown():
        # 계산이 완료된 경우에만 발행이 이루어지도록 함
        publish_joint_angles(q, w_axis_angle)

        rate.sleep()
             
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()




if __name__ == '__main__':
    impedance_control()