#!/usr/bin/env python

import rospy
import geometry_msgs.msg
from std_msgs.msg import Float64

class Kinematics:
    left_wheel_vel = 0.0
    right_wheel_vel = 0.0
    l = 0.20/2 # m half of body width
    r = 0.08 # m wheel radius

    def __init__(self):
        self.receiver = rospy.Subscriber("/course_agv/velocity", geometry_msgs.msg.Twist,self.callback) # sbscribe for line and angle velocity,and then process
        self.vel_left_pub = rospy.Publisher('/course_agv/left_wheel_velocity_controller/command',Float64, queue_size=10)
        self.vel_right_pub = rospy.Publisher('/course_agv/right_wheel_velocity_controller/command',Float64, queue_size=10)

    def callback(self, data):
        vx = data.linear.x
        vw = data.angular.z
        self.left_wheel_vel,self.right_wheel_vel = self.kinematics(vx,vw)

    def kinematics(self,vx,vw):
        # too simple method , TODO
        # counterclockwise is positive
        # left_wheel_vel  = (vx + self.l*vw)/self.r
        # right_wheel_vel = (vx - self.l*vw)/self.r
        left_wheel_vel  = (vx - self.l*vw)/self.r
        right_wheel_vel = (vx + self.l*vw)/self.r
        return left_wheel_vel,right_wheel_vel
        # rx = 1.0/0.08
        # rw = (0.1+0.08/6)/0.08
        # return rx*vx-rw*vw,rx*vx+rw*vw

    def publish(self):
        self.vel_left_pub.publish(self.left_wheel_vel)
        self.vel_right_pub.publish(self.right_wheel_vel)

def main():
    node_name = "course_agv_kinematics"
    print("node : ",node_name)
    try:
        
        rospy.init_node(node_name)
        k = Kinematics()
        rate = rospy.Rate(rospy.get_param('~publish_rate',200))
        while not rospy.is_shutdown():
            k.publish()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
