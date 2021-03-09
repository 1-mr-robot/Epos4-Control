#!/usr/bin/env python

import rospy
from epos_arm_control.msg import epos
from std_msgs.msg import Float64

VELOCITY = 0
POSITION = 1
CURRENT=2
MODE = POSITION

def position_publish(x):
    pub = rospy.Publisher('exoskel_control', epos, queue_size=10)
    rospy.init_node('epos_control', anonymous=True)
    rate = rospy.Rate(200) # 10hz
    pos_vel=epos()
    pos_vel.mode = MODE
    if pos_vel.mode==0:
        pos_vel.velocity=x    
    elif pos_vel.mode==1:
        pos_vel.angle=x
        pos_vel.position=x*16384*81/360
    elif pos_vel.mode==2:
        pos_vel.torque=x
        pos_vel.current=(x/30)*1000
    else:
        print('please input mode=0 or 1 or 2')

    while not rospy.is_shutdown():
        rospy.loginfo(pos_vel)
        pub.publish(pos_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        x=float(input('The reference number:'))
        position_publish(x)
    except rospy.ROSInterruptException:
        pass
