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

    def __init__(self):
        super().__init__()
        self.real_speed = 0.0
        self.real_position = 0.0
        self.real_current = 0.0
        self.d_speed = 0.0
        self.d_position = 0.0
        self.d_current = 0.0
        self.d_min_duration = 0.0
        self.error_pos = self.d_position - self.real_position
        self.error_vel = self.d_speed - self.real_speed

    def set_real_data(pos, vel, error):
        self.real_speed = vel
        self.real_position = pos
        self.error_pos = error


class testNode(FrameNode):

    def __init__(self):
        super(testNode,self).__init__()
        self.cleanMotor = Motor()
        self.rcrMotor = Motor()

    def define_node_publisher(self):
        self.pub_clean = rospy.Publisher("CleanMotor_cmd", SJPA, queue_size = 1)
        self.pub_rcr = rospy.Publisher("RCRMotor_cmd", SJPA, queue_size = 1)
    
    def define_node_subscriber(self):
        # img_sub = rospy.Subscriber('/baseCamImage', Image, self.callback_basecam, queue_size=1)
        rcr_sub = rospy.Subscriber("canbus_motor_node/RCRMotor_data", SJPF, self.callback_rcr)
        # force_sub = rospy.Subscriber("/robotiq_ft_sensor", ft_sensor, self.callback_ft_sensor, queue_size=1)
        clean_sub = rospy.Subscriber("canbus_motor_node/CleanMotor_data", SJPF, self.callback_clean, queue_size=1)

    def read_fd_msg(msg):
        return msg.position, msg.velocity, msg.error

    def callback_rcr(msg):
        self.rcrMotor.set_real_data(pos, vel, error)
        # pass

    def callback_clean(msg):
        self.cleanMotor.set_real_data(pos, vel, error)


    def publish( pub, name, data):
        msg =  SingleJointPositionActionGoal()
        msg.header.frame_id = name
        msg.header.stamp = rospy.Time.now()
        # data = [position, velicity]  or [position, velicity, duration]
        msg.goal.position = data[0]
        msg.goal.max_velocity = data[1]
        msg.goal.min_duration = 0 if len(data) == 2 else data[2]#data[2]
        pub.publish(msg)

    def ik_test(self, cmd):
        pub = self.joint_pub
        # p1 = []
        # self.ur_move_to_point(pub, p1)
        direction_list = [ "up", "down",  "init", "stop" ]
        mode_list = ["",]
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
            elif direction == "stop":
                print("stop!")
                self.ur_stop()
                return 
            elif direction == "init" :
                q_init = 1000 # 1m height
                qd = q_init

            print("qd: ",  qd)

        except KeyError as e:
            print("no step cmd getta!")
    
    def spin_ik_test(self):
        rate = self.Rate(self.rate)
        while not self.is_shutdown():
            # self.get_rosparams()    
            try:
                print("===in python 3 it is input, in python2 should use raw_input ====")
                inp = raw_input("direction? [0,1,2,3], [ up, down, init, stop]: ")  #in python3， no function raw_input,input gets string
                if inp == "q":
                    break
                self.ik_test(int(inp))
            except KeyboardInterrupt:
                rospy.signal_shutdown("KeyboardInterrupt")
                raise
            rate.sleep()
    
def test():
    tNode = testNode()
    tNode.init("test_robomodule_node")
    tNode.spin_ik_test()

if __name__ == "__main__":
    test()