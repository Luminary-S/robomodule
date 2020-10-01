#!/usr/bin/python
# -*- coding: utf-8 -*-
# import sys
# sys.path.append("../")

import yaml,os
import rospy,time,math
from ur_node import URNode

class testNode(URNode):

    def __init__(self):
        super(testNode,self).__init__()

    def ik_test(self, cmd):
        pub = self.joint_pub
        # p1 = []
        # self.ur_move_to_point(pub, p1)
        direction_list = [ "up", "down", "back", "forward", "left", "right", "start", "init", "stop" ]
        direction = direction_list[cmd]
        len = 0.01
        self.ur.set_step_move_STEP(len)
        q = self.now_ur_pos
        
        # q = [-0.6705663839923304, -0.6085761229144495, 2.005333423614502, 5.002155303955078, 1.0891844034194946, 4.865204811096191]
        print("direction: ", direction)
        try:
            if direction  == "down":
                qd = self.ur.ur_step_move_down(q)
            elif direction == "up":                 
                # q = self.ur.get_q()
                qd = self.ur.ur_step_move_up(q)
                # qd = self.ur.ur_step_move_up(q)
            elif direction == "back": 
                qd = self.ur.ur_step_move_backward(q)
            elif direction == "forward": 
                qd = self.ur.ur_step_move_forward(q)
            elif direction == "right": 
                qd = self.ur.ur_step_move_right(q)
            elif direction == "left": 
                qd = self.ur.ur_step_move_left(q)
            elif direction == "stop":
                print("stop!")
                self.ur_stop()
                return 
            elif direction == "init" :
                q_init = [-27.196, -34.005, 117.804, 277.021, 56.843, 269.964]
                qd = [i/180*math.pi for i in q_init]
            elif direction == "start" :
                qd = [i/180*math.pi for i in [-38.45, -37.84, 114.24, 280.19, 66.54, 268.49]]
            # print("direction: ",direction)
            print("qd: ",  qd)
            # self.ur_movej_to_point(pub, qd)
            self.set_moveType("sj")
            self.ur_move_to_point(pub,qd)
        except KeyError as e:
            print("no step cmd getta!")
    
    def spin_ik_test(self):
        rate = self.Rate(self.rate)
        while not self.is_shutdown():
            # self.get_rosparams()    
            try:
                print("===in python 3 it is input, in python2 should use raw_input ====")
                inp = raw_input("direction? [0,1,2,3,4,5,6,7,8], [forward, back, down, up, left, right, start, init, stop]: ")  #in python3， no function raw_input,input gets string
                if inp == "q":
                    break
                self.ik_test(int(inp))
            except KeyboardInterrupt:
                rospy.signal_shutdown("KeyboardInterrupt")
                raise
            rate.sleep()
    
def test():
    tNode = testNode()
    tNode.init("test_ur_node")
    tNode.spin_ik_test()

if __name__ == "__main__":
    test()