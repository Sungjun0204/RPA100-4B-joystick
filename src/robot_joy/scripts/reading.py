#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)

import binascii
import rospy
import packets    # SCARA 패킷 명령어 저장되어 있는 사용자 헤더파일
from std_msgs.msg import String
import struct


'''

SCARA의 현재 위치에 대한 패킷은 다음과 같이 echo된다

02  // STX
FF  // Dummy
30  // flag
20 20 37 30 37 2E 30 32 31 20  // x축 xxxxx.xxx 
20 20 20 35 33 2E 33 33 31 20  // y축 xxxxx.xxx 
20 20 20 2D 33 2E 36 31 39 20  // z축 xxxxx.xxx 
20 2D 31 30 36 2E 36 30 31 20  // w축 xxxxx.xxx 
31  // ARM (로봇의 자세 여부)
03  // ETX
E4  // LRC

로봇 자세를 의미하는 ARM의 경우는 다음의 경우가 있다.
0: Left form (왼쪽으로 굽어져 있는 형태)
1: right form (오른쪽으로 굽어져 있는 형태)
2: no form (굽어져있지 않은 형태)

'''




## 기기로부터 입력받은 현재위치 패킷 값, 동시에 파싱 진행하는 함수##
# def parse_axis_data(hex_data):
#     # 주어진 헥사 데이터를 ASCII로 변환
#     ascii_data = binascii.unhexlify(hex_data).decode('ascii')
#     # 변환된 데이터를 float로 변환
#     return float(ascii_data.strip())

# def xyz_callback(data):
#     packet = data.data
#     packet_list = packet.split()

#     try:
#         # 각 축 데이터의 헥사 문자열 추출
#         x_hex = ''.join(packet_list[3:13])
#         y_hex = ''.join(packet_list[13:23])
#         z_hex = ''.join(packet_list[23:33])
#         w_hex = ''.join(packet_list[33:43])
        
#         # 헥사 데이터를 ASCII 문자열로 변환 후 float으로 파싱
#         x_val = parse_axis_data(x_hex)
#         y_val = parse_axis_data(y_hex)
#         z_val = parse_axis_data(z_hex)
#         w_val = parse_axis_data(w_hex)
        
#         # ARM 상태 추출 및 설명
#         arm = int(packet_list[43], 16)
#         arm_states = {
#             0: "Left form",
#             1: "Right form",
#             2: "No form"
#         }
#         arm_status = arm_states.get(arm, "Unknown form")

#         rospy.loginfo("X axis: %f", x_val)
#         rospy.loginfo("Y axis: %f", y_val)
#         rospy.loginfo("Z axis: %f", z_val)
#         rospy.loginfo("W axis: %f", w_val)
#         rospy.loginfo("ARM status: %s", arm_status)

#     except Exception as e:
#         rospy.logwarn("Error processing packet: %s", str(e))


def convert_to_float(str_value):
    try:
        return float(str_value)
    except ValueError:
        return None

def xyz_callback(data):
    if(len(data.data) == 46):
        # 수신된 메시지를 문자열로 추출
        x_hex = data.data[3:12]
        y_hex = data.data[13:22]
        z_hex = data.data[23:32]
        w_hex = data.data[33:42]
        arm_hex = data.data[43]
        
        # 문자열의 각 문자를 ASCII 값으로 변환 후 16진수 문자열로 변환
        # hex_string = ''.join(format(ord(char), '02x') for char in message_string)
        
        # 16진수 문자열 출력
        # rospy.loginfo("Received string: %s", message_string)
        # rospy.loginfo("Converted to hex: %s", hex_string)

        x_val = convert_to_float(x_hex)
        y_val = convert_to_float(y_hex)
        z_val = convert_to_float(z_hex)
        w_val = convert_to_float(w_hex)

        print(x_val, y_val, z_val, w_val)


        arm = convert_to_float(arm_hex)

        ## 현재 SCARA의 자세 정보 확인
        arm_states = {
                0: "Left form",
                1: "Right form",
                2: "No form"
            }
        arm_status = arm_states.get(arm, "Unknown form")

        print("ARM status: %s", arm_status)

    else:
        print("...adjusting....")




    


##############
### 메인함수 ###
##############

def reading():
    #### node setting ####
    rospy.init_node('reading', anonymous=True)

    #### Subscribe section ####
    rospy.Subscriber('/read', String, xyz_callback)       # SCARA 패킷 값 구독
    
    
    rate = rospy.Rate(100) # 100hz


    #### 알고리즘 파트 ####
    while not rospy.is_shutdown():
        

        rate.sleep()
             
             

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    reading()