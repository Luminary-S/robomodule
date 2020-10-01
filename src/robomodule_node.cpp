#include "robomodule/robomodule_node.h"

#include <ctime>

#include "ros/ros.h"

using rmnode::RobomoduleNode;

// for: can_startup node
RobomoduleNode::RobomoduleNode(ros::NodeHandle& nh,
                               ros::NodeHandle& nh_private) {
  // 1. init paramters
  ParamInit(nh);
  rbCan.can_activate();
  // ReadConfigFile(TEST_CONFIG_FILE);
  // rbCan.init();
  // std::cout << "abfdfdfdf" << std::endl;
  // ReadAllID_ConfigFile(ALL_MOTOR_CONFIG_FILE);
  // // std::cout << "ab" << std::endl;
  ros::Rate r(pub_rate_);
  // // nh.setParam("CleanMotor/alive", 1);
  while (ros::ok()) {
    //   // for (auto it = motor_map.begin(); it != motor_map.end(); it++) {
    //   //   std::string motorname = it->first;
    //   //   std::string paramname = motorname + "_node/alive";
    //   //   rbCan.set_id(*(it->second.begin()), *(it->second.end()));
    //   //   int temp_alive;
    //   //   nh.getParam(paramname, temp_alive);
    //   //   if (temp_alive == 1 && rbCan.CHECK_DRIVER_ALIVE()) {
    //   //     nh.setParam(paramname, 1);
    //   //     std::cout << motorname + "is alive!" << std::endl;
    //   //     // ROS_INFO(motorname + "is alive!");

    //   //   } else {
    //   //     nh.setParam(paramname, 0);
    //   //     std::cout << motorname + "is DEAD!" << std::endl;
    //   //     // ROS_INFO(motorname + "is DEAD!");
    //   //   }
    //   // }
    //   // ros::spinOnce();
    //   // std::cout << "ab" << std::endl;
    r.sleep();
  }
  rbCan.can_close();
}

// for: motor node
RobomoduleNode::RobomoduleNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                               std::string str) {
  // 1. init paramters
  NAME_ = str;
  sleep(2);
  ParamInit(nh);
  ReadConfigFileByName();
  // rbCan.init_robomodule_setting();
  std::cout << "PARAM INIT FINISHED" << std::endl;
  define_pub_sub_server(nh);

  if (CheckAlive(nh) == 1) {
    ROS_INFO("%s is alive!", NAME_.c_str());
    // time_t now1 = time(0);
    // rbCan.can_activate();
    // sleep(1);
    // time_t now2 = time(0);
    // std::cout << "can activate need time: "<< now2-now1<< std::endl;

    // ROS_INFO("%s is alive!", NAME_.c_st;
    // std::cout << "can activate"<< std::endl;
    // while (true) {
      try {
        rbCan.init_robomodule_setting();
        // break;
      } catch (const char* msg) {
        std::cout << msg << std::endl;
        // continue;
      }
    // }
  } else {
    ROS_INFO("I have dead: %s is DEAD!", NAME_.c_str());
    return;
  }
  return;
  ros::Rate r(pub_rate_);
  while (ros::ok()) {
    if (CheckAlive(nh)) {
      update();
    } else {
      ROS_INFO("I have dead: %s is DEAD!", NAME_.c_str());
      continue;
    }
    r.sleep();
  }
}

int RobomoduleNode::CheckAlive(ros::NodeHandle& nh) {
  std::string paramname = NAME_ + "_node/alive";
  nh.getParam(paramname, ALIVE_);
  // std::cout << NAME_ << "\n"<<ALIVE_<< std::endl;
  return ALIVE_;
}

void RobomoduleNode::update() {
  // check node mode
  rbCan.getData();
  // float vd = speed_controller( pd, rbCan.get_real_pos(), 1.0/pub_rate_);
  // rbCan.set_vd(vd);
  rbCan.update();
  // rbCan.sendCMD();
  Publish();
}

void RobomoduleNode::ReadConfigFileByName() {
  // std::cout << file_address << std::endl;
  // YAML::Node can_config = YAML::LoadFile(ALL_MOTOR_CONFIG_FILE);
  rbCan.ParamInitWithFileByName(ALL_MOTOR_CONFIG_FILE, NAME_);
}

// void RobomoduleNode::ReadConfigFile(const std::string& file_address) {
//   std::cout << file_address << std::endl;
//   YAML::Node can_config = YAML::LoadFile(file_address);
//   rbCan.ParamInitWithFile(file_address);
//   P_ = can_config["P"].as<double>();
//   I_ = can_config["I"].as<double>();
//   D_ = can_config["D"].as<double>();
// }

void RobomoduleNode::ReadAllID_ConfigFile(const std::string& file_address) {
  std::cout << file_address << std::endl;
  YAML::Node can_config = YAML::LoadFile(file_address);
  // rbCan.ParamInitWithFile(file_address);
  // device_type_ = can_config[""]["PowerLimit"].as<double>();
  // device_type_ = can_config["PowerLimit"].as<double>();

  for (auto it = can_config.begin(); it != can_config.end(); it++) {
    std::list<int> id;
    id.push_back(it->second["GroupNum"].as<int>());
    id.push_back(it->second["DeviceNum"].as<int>());
    motor_map[it->first.as<std::string>()] = id;
  }
}

void RobomoduleNode::ParamInit(ros::NodeHandle& nh_private) {
  nh_private.param("pub_rate", pub_rate_,
                   30.0);  // last value is the default if no setting
  nh_private.param("topic_name", topic_name_, std::string("can_startup"));
  vd = 0.0;
  pd = 0.0;
  ALIVE_ = 0;
}

void RobomoduleNode::define_pub_sub_server(ros::NodeHandle& nh) {
  std::string sub_name = NAME_ + "_cmd";
  motor_sub_ = nh.subscribe<control_msgs::SingleJointPositionActionGoal>(
      sub_name, 10, &RobomoduleNode::cmdCallback, this);
  motor_pub_ =
      nh.advertise<control_msgs::SingleJointPositionFeedback>(NAME_, 10);
}

void RobomoduleNode::cmdCallback(
    const control_msgs::SingleJointPositionActionGoalConstPtr& msg) {
  pd = msg->goal.position;
  vd = msg->goal.max_velocity;
  rbCan.set_pd(pd);
  // rbCan.set_pwm();
  rbCan.set_vd(vd);
}

void RobomoduleNode::Publish() {
  // sensor_msgs::JointState js;
  control_msgs::SingleJointPositionFeedback mp;
  mp.header.frame_id = NAME_;  //"rcrmotor";
  mp.header.stamp = ros::Time::now();
  // mp.header.frame_id = ;
  mp.velocity = rbCan.get_real_vel();
  mp.position = rbCan.get_real_pos();
  mp.error = pd - rbCan.get_real_cur();
  motor_pub_.publish(mp);
}

int main(int argc, char** argv) {
  if (argc < 4) {
    ros::init(argc, argv, "CANStartup");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    RobomoduleNode rbNode(nh, nh_private);
  } else if (argc == 4) {
    ros::init(argc, argv, argv[1]);
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    RobomoduleNode rbNode(nh, nh_private, argv[1]);
  }

  ros::spin();
  return 0;
}