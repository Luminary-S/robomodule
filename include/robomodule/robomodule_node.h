#ifndef ROBOMODULE_NODE_H
#define ROBOMODULE_NODE_H
///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, CUHK CURI.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/**
 * This is a ros node for RoboModule driver of DC motor with incremental
 * encoder.
 */

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <iostream>
#include <list>
#include <map>
#include <string>

#include "control_msgs/SingleJointPositionActionGoal.h"
#include "control_msgs/SingleJointPositionFeedback.h"
#include "robomodule/robomoduleCan.h"
#include "robomodule/controlcan.h"
#include "ros/ros.h"
// #define _CONFIG_FILE
// "/home/sgl/catkin_new/src/robomodule/config/can_config.yaml"
#define ALL_MOTOR_CONFIG_FILE \
  "/home/sgl/catkin_new/src/robomodule/config/all_motor_config.yaml"
#define TEST_CONFIG_FILE \
  "/home/sgl/catkin_new/src/robomodule/config/clean_motor_config.yaml"

namespace rmnode {

class RobomoduleNode {
 public:
  RobomoduleNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  RobomoduleNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                 std::string name);
  virtual ~RobomoduleNode() {}

  void ReadConfigFileByName();
  void ReadAllID_ConfigFile(const std::string& file_address);
  // void ReadConfigFile(const std::string& file_address);
  // communication related

  // main
  int CheckAlive(ros::NodeHandle& nh);
  void update();

  // ros related
  // void init_node(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  void define_pub_sub_server(ros::NodeHandle& nh);
  void ParamInit(ros::NodeHandle& nh_private);
  // callback
  void cmdCallback(
      const control_msgs::SingleJointPositionActionGoalConstPtr& msg);
  void Publish();

  // get set
  // float get_real_pos(){return pd;}
  // float get_real_pos(){return vd;}

 private:
  rmCan::RobomoduleCAN rbCan;
  ros::Publisher motor_pub_;
  ros::Subscriber motor_sub_;
  std::string topic_name_;
  double pub_rate_ = 30.0;
  std::string NAME_;
  // float P_;
  // float I_;
  // float D_;
  double pd;
  double vd;
  int ALIVE_;

  std::map<std::string, std::list<int>> motor_map;
  // std::string _CONFIG_FILE=;
};

}  // namespace  rmnode

#endif