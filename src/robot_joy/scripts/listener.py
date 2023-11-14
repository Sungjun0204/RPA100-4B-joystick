#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import packets
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
joystic_xyzw = [0.0, 0.0, 0.0, 0.0]
speed_btn = [0, 0]
speed_num = 0




def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
    

def joycallback(data):
    global servo_on, servo_off, joystic_xyzw
    servo_off = data.buttons[6]
    servo_on = data.buttons[7]
    joystic_xyzw[0] = data.axes[0]  # x
    joystic_xyzw[1] = data.axes[1]  # y
    joystic_xyzw[2] = data.axes[4]  # z
    joystic_xyzw[3] = data.axes[3]  # w
    speed_btn[0] = data.buttons[4]  # speed +
    speed_btn[1] = data.buttons[5]  # speed -

    
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

    
def listener():

    rospy.init_node('listener', anonymous=True)

    #### Subscribe section ####
    # rospy.Subscriber('chatter', String, callback)
    rospy.Subscriber('/joy', Joy, joycallback)


    #### Publish section ####
    pub = rospy.Publisher('packet', String, queue_size=10)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        if servo_on == 1:
            pub.publish(packets.SVON); #rospy.loginfo(packets.SVON)
        if servo_off == 1:
            pub.publish(packets.SVOFF); #rospy.loginfo(packets.SVOFF)
        
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
        rate.sleep()

        # Speed value Change
        if speed_btn[0] == 1: 
            speed_func(100); pub.publish(''.join(packets.SPEED)); rospy.loginfo(''.join(packets.SPEED))
        if speed_btn[1] == 1: 
            speed_func(-100); pub.publish(''.join(packets.SPEED)); rospy.loginfo(''.join(packets.SPEED))
             
             



    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
