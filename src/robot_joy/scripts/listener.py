#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)


import rospy
import threading
import packets    # SCARA 패킷 명령어 저장되어 있는 사용자 헤더파일
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import sys
import signal

'''
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
'''

#### global variables ####
servo_on = 0
servo_off = 0
whereareyou = 0
joystic_xyzw = [0.0, 0.0, 0.0, 0.0]
speed_btn = [0, 0]
speed_num = 0



## node 비정상 작동 시 강제 종료를 위한 함수 ##
def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
    

## 조이스틱 값을 받는 함수 ##
def joycallback(data):
    global servo_on, servo_off, whereareyou, joystic_xyzw
    servo_off = data.buttons[8]
    servo_on = data.buttons[9]
    whereareyou = data.buttons[0]  # Y버튼 (현재 좌표 확인)
    joystic_xyzw[0] = data.axes[0]  # x
    joystic_xyzw[1] = data.axes[1]  # y
    joystic_xyzw[2] = data.axes[4]  # z
    joystic_xyzw[3] = data.axes[3]  # w
    speed_btn[0] = data.buttons[4]  # speed +
    speed_btn[1] = data.buttons[5]  # speed -

    
## 속도 인자 값을 조절하는 함수 ##
def speed_func(dn):
    global speed_num

    speed_num += dn
    if speed_num >= 50000:
        speed_num -= dn
    elif speed_num <= 0:
        speed_num -= dn

    arr = []
    for i in str(speed_num):
        arr.append(i)

    for i in range(5):
         packets.SPEED[i+4] = '0'   # speed value reset
    
    for i in range(len(arr)):       # new speed value input
         packets.SPEED[8-i] = arr[len(arr)-1-i]         


## 패킷의 LRC 값을 계산해주는 함수 ##
def lrc_calc(arr1):
    # DUMMY 부터 EXT 이전까지 XOR 연산
    LRC = 0x00  # 초기 LRC 값
    for byte in arr1[1:-3]:  # DUMMY 부터 EXT 이전까지
        LRC ^= byte

    # 계산된 LRC 값을 패킷에 넣기
    arr1[-2] = LRC


# ## 기기로부터 입력받은 현재위치 패킷 값 ##
# def xyz_callback(data):
#     print(data)




##############
### 메인함수 ###
##############

def listener():

    rospy.init_node('listener', anonymous=True)

    #### Subscribe section ####
    # rospy.Subscriber('chatter', String, callback)
    rospy.Subscriber('/joy', Joy, joycallback)              # 조이스틱 값 구독
    # rospy.Subscriber('/read', String, xyz_callback)       # SCARA 패킷 값 구독


    #### Publish section ####
    pub = rospy.Publisher('packet', String, queue_size=10)
    rate = rospy.Rate(100) # 10hz


    #### 알고리즘 파트 ####
    while not rospy.is_shutdown():

        if servo_on == 1:
            pub.publish(packets.SVON); #rospy.loginfo(packets.SVON)
        if servo_off == 1:
            pub.publish(packets.SVOFF); #rospy.loginfo(packets.SVOFF)
        
        # 현재 End Effect 위치 값 받기 (Cartecian 좌표계 기준)
        if whereareyou == 1:
            pub.publish(''.join(packets.COORDI))
        
        # X moving
        if (joystic_xyzw[0] <= -0.8 and joystic_xyzw[0] >= -1.0):
            pub.publish(packets.MUVX[1]); #rospy.loginfo(packets.MUVX[0])
        if (joystic_xyzw[0] >= 0.8 and joystic_xyzw[0] <= 1.0):
            pub.publish(packets.MUVX[0]); #rospy.loginfo(packets.MUVX[1])

        # Y moving
        if (joystic_xyzw[1] <= -0.8 and joystic_xyzw[1] >= -1.0):
            pub.publish(packets.MUVY[0]); #rospy.loginfo(packets.MUVY[1])
        if (joystic_xyzw[1] >= 0.8 and joystic_xyzw[1] <= 1.0):
            pub.publish(packets.MUVY[1]); #rospy.loginfo(packets.MUVY[0])

        # Z moving
        if (joystic_xyzw[2] <= -0.8 and joystic_xyzw[2] >= -1.0):
            pub.publish(packets.MUVZ[0]); #rospy.loginfo(packets.MUVZ[0])
        if (joystic_xyzw[2] >= 0.8 and joystic_xyzw[2] <= 1.0):
            pub.publish(packets.MUVZ[1]); #rospy.loginfo(packets.MUVZ[1])

        # W moving
        if (joystic_xyzw[3] <= -0.8 and joystic_xyzw[3] >= -1.0):
            pub.publish(packets.MUVW[0]); #rospy.loginfo(packets.MUVW[0])
        if (joystic_xyzw[3] >= 0.8 and joystic_xyzw[3] <= 1.0):
            pub.publish(packets.MUVW[1]); #rospy.loginfo(packets.MUVW[1])   
        
        # Speed value Change
        if speed_btn[0] == 1: 
            speed_func(100); pub.publish(''.join(packets.SPEED)); rospy.loginfo(''.join(packets.SPEED))
        if speed_btn[1] == 1: 
            speed_func(-100); pub.publish(''.join(packets.SPEED)); rospy.loginfo(''.join(packets.SPEED))


        rate.sleep()
                     

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()




if __name__ == '__main__':
    listener()

