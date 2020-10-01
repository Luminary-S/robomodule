#ifndef CANBUS_MOTOR_NODE_H
#define CANBUS_MOTOR_NODE_H

#include <list>
#include <mutex>
// #include <pthread.h>
#include <thread>
#include <vector>

#include "control_msgs/SingleJointPositionActionGoal.h"
#include "control_msgs/SingleJointPositionFeedback.h"
#include "robomodule/robomoduleCan.h"
#include "ros/ros.h"

#define ALL_MOTOR_CONFIG_FILE \
  "/home/sgl/catkin_new/src/robomodule/config/all_motor_config.yaml"

namespace dcmotor {

struct CMD {
    float vd;
    float pd;
    int cmd_id;
    float error;
};

struct MotorInfo {
    float v;
    float p;
    float error;
};

struct Motor {
  // ros::Subscriber sub;
  // ros::Publisher pub;
  float rate;
  int alive;
//   std::vector<CMD> 
  rmCan::RobomoduleCAN canMotor;
};

typedef std::list<rmCan::RobomoduleCAN> LISTMOTOR;
typedef std::map<std::string, Motor> MAPMOTOR;

class canBusNode {
 public:
  canBusNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  virtual ~canBusNode() {}

  // config
  void ParamInit(ros::NodeHandle& nh_private);
  void readConfigFile(const std::string& file_address);
//   std::thread memThread(ros::NodeHandle& nh, std::string name) {
//     return std::thread(&canBusNode::proc, this,  name);
//     // return std::thread{std::bind(&canBusNode::proc, motor)};
//   }
  void proc(std::string name);
  void update();

  // ros related
  void cmdCallback(
      const control_msgs::SingleJointPositionActionGoalConstPtr& msg,
      const std::string& motor_name);
  void Publish(ros::Publisher& motor_pub, const std::string& frame_id, const MotorInfo& data);
  MotorInfo getMotorInfo(Motor& m);

  float get_rate() const { return pub_rate_; }

//   std::vector<std::thread> t_vec;

 private:
 ros::NodeHandle nh_;
  std::map<std::string, Motor> motorunits_map;
  std::map<std::string, std::thread> thread_map;
  // motor list
  //   vector<rmCan::RobomoduleCAN> motorUnits;  // can unit list

  //   vector<ros::Subscriber> cmdSubs;
  //   vector<ros::Publisher> dataPubs;

  // ros
  double pub_rate_;

};  // end class
}  // namespace dcmotor
#endif