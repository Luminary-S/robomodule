#!/usr/bin/python
# -*- coding: utf-8 -*-
# import sys
# sys.path.append("../")


import rospy,time,math
from frame_node import FrameNode
from control_msgs.msg import SingleJointPositionActionGoal as SJPA
from control_msgs.msg import SingleJointPositionFeedback as SJPF

"""
control_msgs::SingleJointPositionActionGoal
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
actionlib_msgs/GoalID goal_id
  time stamp
  string id
control_msgs/SingleJointPositionGoal goal
  float64 position
  duration min_duration
  float64 max_velocity

control_msgs::SingleJointPositionFeedback
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float64 position
float64 velocity
float64 error
"""
class Motor():

    def __init__(self,name):
        # super().__init__()
        self.name = name
        self.real_speed = 0.0
        self.real_position = 0.0
        self.real_current = 0.0
        self.d_speed = 0.0
        self.d_position = 0.0
        self.d_current = 0.0
        self.d_min_duration = 0.0
        self.error_pos = self.d_position - self.real_position
        self.error_vel = self.d_speed - self.real_speed
        self.init_pos = 0.0

    def set_real_data(self,pos, vel, error):
        self.real_speed = vel
        self.real_position = pos
        self.error_pos = error

    def set_target(self, p, v):
        self.d_speed = v
        self.d_position = p

class testNode(FrameNode):

    def __init__(self):
        super(testNode,self).__init__()
        self.cleanMotor = Motor("CleanMotor")
        self.rate = 30

    def define_node_publisher(self):
        self.pub_clean = rospy.Publisher("CleanMotor_cmd", SJPA, queue_size = 1)
    
    def define_node_subscriber(self):
        self.clean_sub = rospy.Subscriber("canbus_motor_node/CleanMotor_data", SJPF, self.callback_clean, queue_size=1)

    def read_fd_msg(self,msg):
        return msg.position, msg.velocity, msg.error

    def callback_clean(self,msg):
        p,v,e = self.read_fd_msg(msg)
        self.cleanMotor.set_real_data(p,v,e)

    def publish( self, pub, name, data):
        msg =  SJPA()
        msg.header.frame_id = name
        msg.header.stamp = rospy.Time.now()
        # data = [position, velicity]  or [position, velicity, duration]
        msg.goal.position = data[0]
        msg.goal.max_velocity = data[1]
        msg.goal.min_duration = rospy.Duration(0) if len(data) == 2.0 else rospy.Duration( data[2] ) #data[2]
        pub.publish(msg)

    def clean_test(self, cmd):
        pub = self.pub_clean 
        name = self.cleanMotor.name
        direction_list = [ "set", "forward", "backward", "init", "stop"]
        direction = direction_list[cmd]
        print("direction: ", direction)
        try:
            if direction  == "forward":
                vd = 200
                pd = 0
                p = self.cleanMotor.real_position + pd
                v = vd
                self.cleanMotor.set_target(p, v)
            elif direction == "backward":                 
                vd = -200
                pd = 0
                p = self.cleanMotor.real_position + pd
                v = vd
                self.cleanMotor.set_target(p, v)
            elif direction == "stop":
                vd = 0
                pd = 0
                self.cleanMotor.set_target(pd, vd)
            elif direction == "init" :
                vd = 200
                pd = self.cleanMotor.init_pos
                self.cleanMotor.set_target(pd, vd)
            # elif direction == "set":

            data = [self.cleanMotor.d_position, self.cleanMotor.d_speed ]
            self.publish(pub,name,data)

        except KeyError as e:
            print("no step cmd getta!")
    
    def spin(self):
        rate = self.Rate(self.rate)
        i = 0
        while not self.is_shutdown():
            # self.get_rosparams()
            if i == 0 :
                self.cleanMotor.init_pos = self.cleanMotor.real_position   
            # print() 
            else: 
                try:
                    print("===in python 3 it is input, in python2 should use raw_input ====")
                    inp = raw_input("direction? [0,1,2,3,4,5], [ set, forward, backward, init, stop, print]: ")  #in python3， no function raw_input,input gets string
                    # print("input: %s" %(inp))
                    if inp == "0":
                        cmd = int(inp)
                        # print(cmd)
                        inp2 = raw_input("relative pos(mm) and vel(mm/s), - and + including, e.g. 500: ")
                        # print(inp2)
                        pd = self.cleanMotor.real_position + float(inp2.split(",")[0]) 
                        vd = float(inp2.split(",")[1])
                        self.cleanMotor.set_target(pd, vd)
                    if inp == "1" or inp =="2" or inp =="3":
                        cmd = int(inp)
                        # print(cmd)
                        print("normal cmd!")
                    # print("input: %s" %(inp))
                        # print("now pos: %s, now velocity: %f" %(self.cleanMotor.real_position, self.cleanMotor.real_speed) )
                        # self.clean_test(int(inp)) 
                    if inp == "q" or inp =="4":
                        # print("enter here")
                        cmd = 4
                        # print(cmd) 
                        print("stop motor!")                     
                    if inp == "5":
                        print("now pos: %s, now velocity: %f" %(self.cleanMotor.real_position, self.cleanMotor.real_speed) )
                        # print("now pos: ", )
                        continue

                        # self.clean_test(4)
                        # continue
                    # print("-----------1")
                    self.clean_test(cmd)
                    # print("-----------2")
                except KeyboardInterrupt:
                    rospy.signal_shutdown("KeyboardInterrupt")
                    raise
            i +=1
            rate.sleep()
    
def test():
    tNode = testNode()
    tNode.init_node("test_clean_node")
    tNode.spin()

if __name__ == "__main__":
    test()