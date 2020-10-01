#include "robomodule/canbus_motor_node.h"

#include <ctime>

#include "ros/ros.h"

using dcmotor::canBusNode;

canBusNode::canBusNode() {

}

void canBusNode::ParamInit(ros::NodeHandle& nh_private) {
  nh_private.param("pub_rate", pub_rate_,
                   30.0);  // last value is the default if no setting
}

void canBusNode::readConfigFile(
                                const std::string& file_address) {
  // read config file and init rbCan list
  std::cout << file_address << std::endl;
  YAML::Node can_config = YAML::LoadFile(file_address);

  for (auto it = can_config.begin(); it != can_config.end(); it++) {
    // rmCan::RobomoduleCAN motor;
    std::string name = it->first.as<std::string>();
    std::cout << "new motor found : " << name << std::endl;
    Motor m;
    m.rate = it->second["pub_rate"].as<double>();
    m.alive = it->second["Alive"].as<int>();
    m.canMotor.set_name(name);
    m.canMotor.ParamInitWithFileByName(ALL_MOTOR_CONFIG_FILE, name);

    motorunits_map[name] = m;
  }
}

void canBusNode::proc(std::string name) {
  //   getCMD();
  //   motor.update();
  // 2. define its pub and sub
//   motor = (Motor)motor;
//   std::string name = motorunits_map[name].canMotor.get_name();
  ros::Subscriber sub;
  ros::Publisher pub;
  std::string topic_name = name + "_cmd";     // e.g.: CleanMotor_cmd
  std::string fd_data_name = name + "_data";  // e.g.: CleanMotor_data
  sub = nh.subscribe<control_msgs::SingleJointPositionActionGoal>(
      topic_name, 10, boost::bind(&canBusNode::cmdCallback, this, _1, name));
  pub =
      nh.advertise<control_msgs::SingleJointPositionFeedback>(fd_data_name, 10);
  ros::Rate r(motorunits_map[name].rate);
  while (ros::ok()) {
    std::cout << name << "is printing!" << std::endl;
    //   Publish(pub, name);
    r.sleep();
    ros::spinOnce();
  }
  motorunits_map[name].alive = 0;
}

void canBusNode::cmdCallback(
    const control_msgs::SingleJointPositionActionGoalConstPtr& msg,
    const std::string& motor_name) {
  motorunits_map[motor_name].canMotor.set_pd(msg->goal.position);
  //   rbCan.set_pwm(pwm);
  motorunits_map[motor_name].canMotor.set_vd(msg->goal.max_velocity);
}

void canBusNode::update() {
  //   std::thread thread();
  //   thread.join();
}

void canBusNode::Publish(ros::Publisher motor_pub,
                         const std::string& motor_name) {
  // sensor_msgs::JointState js;
  control_msgs::SingleJointPositionFeedback mp;
  mp.header.frame_id =
      motorunits_map[motor_name].canMotor.get_name();  //"rcrmotor";
  mp.header.stamp = ros::Time::now();
  // mp.header.frame_id = ;
  mp.velocity = motorunits_map[motor_name].canMotor.get_real_vel();
  mp.position = motorunits_map[motor_name].canMotor.get_real_pos();
  mp.error = motorunits_map[motor_name].canMotor.get_pd() -
             motorunits_map[motor_name].canMotor.get_real_cur();
  motor_pub.publish(mp);
}

int main(int argc, char** argv) {
  // init ros canbus_motor_node
  if (argc < 4) {
    ros::init(argc, argv, "canbus_motor_node");
    // ros::NodeHandle nh;
    // ros::NodeHandle nh_private("~");
    // RobomoduleNode rbNode(nh, nh_private);
  } else if (argc == 4) {
    ros::init(argc, argv, argv[1]);

    // RobomoduleNode rbNode(nh, nh_private, argv[1]);
  }
  
  ros::NodeHandle nh_private("~");
  canBusNode cb;

    // 1. read launch params
  cb.ParamInit(nh_private);  // params  from launch file
  // 2. read config files and define variables and pub, sub
  cb.readConfigFile( ALL_MOTOR_CONFIG_FILE);  // params from config yaml file
  int nSize = cb.motorunits_map.size();
//   pthread_t threads[nSize];

  rmCan::RobomoduleCAN canStartUP;
  canStartUP.can_activate();

  // 4. check alive motor and init the alive motor, hardware related
  std::map<std::string, dcmotor::Motor>::iterator ite;
  int num = 0;
  for (ite = cb.motorunits_map.begin(); ite != cb.motorunits_map.end(); ++ite) {
    std::string name = ite->first;
    if (ite->second.alive == 1) {
      std::cout << name << " is alive, start init..." << std::endl;
      ite->second.canMotor.init_robomodule_setting();
      std::cout << name << " init finished! " << std::endl;

        std::thread t( &canBusNode::proc, &cb, name );
        t.detach();
      //   );
    //   pthread_attr_t attr;
    //    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    //   pthread_create(&threads[num], attr,  );
    // std::thread t = memThread(ite->second);
    } else {
      std::cout << name
                << "is dead, if it should be started up, set the motor's Alive "
                   "param in config file."
                << std::endl;
    }
    num++;
  }
  //   return;

  // main thread
  float rate = cb.get_rate();
  ros::Rate r(rate);
  while (ros::ok()) {
    std::cout << "main check thread is running!" << std::endl;
    bool running = false;
    for (ite = cb.motorunits_map.begin(); ite != cb.motorunits_map.end(); ++ite) {
      if (ite->second.alive == 1) {
        running = true;
        std::cout << "sub thread " << ite->first << " is still running!"
                  << std::endl;
      }
    }
    if (!running) {
      break;
    }
    r.sleep();
    ros::spinOnce();
  }

  // thread exit
  //   pthread_exit(NULL);
  std::cout << "all process is done!" << std::endl;
  canStartUP.can_close();
  return 0;
}