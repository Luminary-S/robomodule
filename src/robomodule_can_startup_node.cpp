// #include "robomodule/robomodule_node.h"

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

// using rmnode::RobomoduleNode;

class rbCANStartup(){

}

rbCANStartup::rbCANStartup(ros::NodeHandle& nh,
                               ros::NodeHandle& nh_private) {}

void RobomoduleNode::ReadConfigFile(const std::string& file_address) {
  std::cout << file_address << std::endl;
  YAML::Node can_config = YAML::LoadFile(file_address);
    rbCan.ParamInitWithFile(file_address);
//   device_type_ = can_config["PowerLimit"].as<double>();
//   device_index_ = can_config["GroupNum"].as<int>();
//   can_index_ = can_config["DeviceNum"].as<int>();
//   wait_time_ = can_config["Mode"].as<int>();
//   Diameter_ = can_config["Diameter"].as<double>();
//   device_type_ = can_config["BasicEncoderLines"].as<int>();
//   device_type_ = can_config["ReductionRatio"].as<int>();
  P_ = can_config["P"].as<double>();
  I_ = can_config["I"].as<double>();
  D_ = can_config["D"].as<double>();
}


void RobomoduleNode::update() {
  // init_node();
}

void RobomoduleNode::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
  // rcr_pub = nh.advertise<sensor_msgs::JointState>("rcr_now", 10);
  rcr_sub = nh.subscribe("rcr_target", 10, callback);

  ParamInit(nh);

  ReadConfigFile(_CONFIG_FILE);
}

void RobomoduleNode::Publish() {
  sensor_msgs::JointState js;
  js.header.frame_id = "rcrmotor";
  js.header.stamp = ros::Time::now();
  js.velocity.push_back((double)real_velocity);
  js.position.push_back((double)real_position);
  js.effort.push_back((double)real_current);
  pub.publish(js);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, argv[1]);
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  RobomoduleNode rbNode(nh, nh_private);
  ros::spin();
  return 0;
}