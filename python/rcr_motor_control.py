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

    def set_real_data( self, pos, vel, error ):
        self.real_speed = vel
        self.real_position = pos
        self.error_pos = error

    def set_target(self, p, v):
        self.d_speed = v
        self.d_position = p

class testNode(FrameNode):

    def __init__(self):
        super(testNode,self).__init__()
        self.rcrMotor = Motor("RCRMotor")
        self.rate = 30

    def define_node_publisher(self):
        self.pub_rcr = rospy.Publisher("RCRMotor_cmd", SJPA, queue_size = 1)
    
    def define_node_subscriber(self):
        self.rcr_sub = rospy.Subscriber("canbus_motor_node/RCRMotor_data", SJPF, self.callback_rcr)

    def read_fd_msg(self, msg):
        return msg.position, msg.velocity, msg.error

    def callback_rcr(self, msg):
        p,v,e = self.read_fd_msg(msg)
        self.rcrMotor.set_real_data(p,v,e)

    def publish( self, pub, name, data):
        msg =  SJPA()
        msg.header.frame_id = name
        msg.header.stamp = rospy.Time.now()
        # data = [position, velicity]  or [position, velicity, duration]
        msg.goal.position = data[0]
        msg.goal.max_velocity = data[1]
        msg.goal.min_duration = rospy.Duration(0) if len(data) == 2.0 else rospy.Duration( data[2] ) #data[2]
        # print(msg)
        pub.publish(msg)

    def rcr_test(self, cmd):
        pub = self.pub_rcr 
        name = self.rcrMotor.name
        direction_list = [ "set", "step up", "step down",  "init", "stop" ]
        # mode_list = ["",]
        direction = direction_list[cmd]

        print("direction: ", direction)
        try:
            if direction  == "step down":
                pd = -20 # mm
                vd = 500 # mm/s
                p = self.rcrMotor.real_position + pd
                v = vd
                self.rcrMotor.set_target(p, v)
            elif direction == "step up":                 
                pd = 20 # mm
                vd = 500 # mm/s
                p = self.rcrMotor.real_position + pd
                v = vd
                self.rcrMotor.set_target(p, v)
            elif direction == "stop":
                print("stop!")
                vd = 0
                pd = 0
                self.rcrMotor.set_target(self.rcrMotor.real_position, vd)
                # return 
            elif direction == "init" :
                # pd = 500 # 0.5m absolute height
                vd = 500 # mm/s
                self.rcrMotor.set_target(self.rcrMotor.init_pos, vd)
                # qd = q_init
            data = [ self.rcrMotor.d_position, self.rcrMotor.d_speed ]
            print("data: ",  data)
            self.publish(pub,name,data)
        except KeyError as e:
            print("no step cmd getta!")

    def spin(self):
        rate = self.Rate(self.rate)
        i = 0
        while not self.is_shutdown():
            # self.get_rosparams()
            if i == 0 :
                self.rcrMotor.init_pos = self.rcrMotor.real_position    
            else:
                try:
                    print("===in python 3 it is input, in python2 should use raw_input ====")
                    inp = raw_input("direction? [0,1,2,3,4,5], [ set, step up, step down, init, stop, print]: ")  #in python3， no function raw_input,input gets string
                    if inp == "0":
                        inp2 = raw_input("relative pos(mm) and vel(mm/s), - and + including, e.g. 500, 100: ")
                        pd = self.rcrMotor.real_position + float(inp2.split(",")[0])
                        vd = float(inp2.split(",")[1])
                        self.rcrMotor.set_target(pd, vd)
                    if inp == "1" or inp =="2" or inp =="3":
                        cmd = int(inp)
                        # print(cmd)
                        print("normal cmd!")
                    if inp == "5":
                        print("now pos: %s, now velocity: %f" %(self.rcrMotor.real_position, self.rcrMotor.real_speed) )
                        # print("now pos: ", )
                        continue
                    if inp == "q"or inp =="4":
                        # print("enter here")
                        cmd = 4
                        # print(cmd) 
                        print("stop motor!") 
                    self.rcr_test(int(inp))
                except KeyboardInterrupt:
                    rospy.signal_shutdown("KeyboardInterrupt")
                    raise
            i +=1
            rate.sleep()
    
def test():
    tNode = testNode()
    tNode.init_node("test_rcr_node")
    tNode.spin()

if __name__ == "__main__":
    test()