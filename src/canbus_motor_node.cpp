#include "robomodule/canbus_motor_node.h"

#include <ctime>

#include "ros/ros.h"

namespace dcmotor {

canBusNode::canBusNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
  // 1. read launch params
  nh_ = nh;
  ParamInit(nh_private);  // params  from launch file
  // 2. read config files and define variables and pub, sub
  readConfigFile(ALL_MOTOR_CONFIG_FILE);  // params from config yaml file
}

void canBusNode::ParamInit(ros::NodeHandle& nh_private) {
  nh_private.param("pub_rate", pub_rate_,
                   30.0);  // last value is the default if no setting
}

void canBusNode::readConfigFile(const std::string& file_address) {
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

// cmd callback
void canBusNode::cmdCallback(
    const control_msgs::SingleJointPositionActionGoalConstPtr& msg,
    const std::string& motor_name) {
  motorunits_map[motor_name].canMotor.set_pd(msg->goal.position);
  //   rbCan.set_pwm(pwm);
  motorunits_map[motor_name].canMotor.set_vd(msg->goal.max_velocity);
}

void canBusNode::Publish(ros::Publisher& pub, const std::string& frame_id,
                         const MotorInfo& data) {
  // sensor_msgs::JointState js;
  control_msgs::SingleJointPositionFeedback mp;
  mp.header.frame_id = frame_id;  // NAME_;  //"rcrmotor";
  mp.header.stamp = ros::Time::now();
  // mp.header.frame_id = ;
  mp.velocity = data.v;
  mp.position = data.p;
  mp.error = data.error;
  pub.publish(mp);
}

MotorInfo canBusNode::getMotorInfo(Motor& m) {
  MotorInfo data;
  data.v = m.canMotor.get_real_vel();
  data.p = m.canMotor.get_real_pos();
  data.error = m.canMotor.get_pd() - m.canMotor.get_real_cur();
  return data;
}

// motor thread processing detail
void canBusNode::proc(std::string name) {
  ros::Subscriber sub;
  ros::Publisher pub;
  std::string topic_name = name + "_cmd";     // e.g.: CleanMotor_cmd
  std::string fd_data_name = name + "_data";  // e.g.: CleanMotor_data
  sub = nh_.subscribe<control_msgs::SingleJointPositionActionGoal>(
      topic_name, 10, boost::bind(&canBusNode::cmdCallback, this, _1, name));
  pub = nh_.advertise<control_msgs::SingleJointPositionFeedback>(fd_data_name,
                                                                 10);
  ros::Rate r(motorunits_map[name].rate);
  while (ros::ok()) {
    std::cout << name << " is printing!" << std::endl;

    // 1. get cmd
    motorunits_map[name].canMotor.update();
    MotorInfo data = getMotorInfo(motorunits_map[name]);
    Publish(pub, name, data);

    r.sleep();
    ros::spinOnce();
  }
  motorunits_map[name].alive = 0;
}

// main function, canbus operation
void canBusNode::update() {
  //   int nSize = motorunits_map.size();
  // //   pthread_t threads[nSize];

  rmCan::RobomoduleCAN canStartUP;
  canStartUP.can_activate();

  // create sub thread
  std::map<std::string, Motor>::iterator ite;
  //   int num = 0;
  for (ite = motorunits_map.begin(); ite != motorunits_map.end(); ++ite) {
    std::string name = ite->first;
    if (ite->second.alive == 1) {
      std::cout << name << " is alive, start init..." << std::endl;
      ite->second.canMotor.init_robomodule_setting();
      if (ite->second.canMotor.transtmit_status()) {
        std::cout << name << " init finished! " << std::endl;
      } else {
        std::cout << name << " sth error in the transmitting process !"
                  << std::endl;
        return;
      }

      std::thread t(&canBusNode::proc, this, name);
      // std::thread t = memThread(nh,name);
      //   t.detach();
      thread_map[name] = t;

    } else {
      std::cout << name
                << "is dead, if it should be started up, set the motor's Alive "
                   "param in config file."
                << std::endl;
    }
    // num++;
  }

// https://www.bookstack.cn/read/Cpp_Concurrency_In_Action/content-chapter2-2.5-chinese.md
//https://wizardforcel.gitbooks.io/cpp-11-faq/content/77.html
  for (std::map<std::string, std::thread>::iterator it = thread_map.begin();
       it != thread_map.end(); it++){
    std::cout << it->first << " add into thread list !" << std::endl;
  it->second.join();
       }
  //   return;

  // main thread
  //   float rate = get_rate();
  //   ros::Rate r(rate);
  //   while (ros::ok()) {
  //     std::cout << "main check thread is running!" << std::endl;
  //     bool running = false;
  //     for (ite = motorunits_map.begin(); ite != motorunits_map.end(); ++ite)
  //     {
  //       if (ite->second.alive == 1) {
  //         running = true;
  //         std::cout << "sub thread " << ite->first << " is still running!"
  //                   << std::endl;
  //       }
  //     }
  //     if (!running) {
  //       break;
  //     }
  //     r.sleep();
  //     ros::spinOnce();
  //   }

  // thread exit
  //   pthread_exit(NULL);
  std::cout << "all process is done!" << std::endl;
  canStartUP.can_close();
}

}  // namespace dcmotor

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
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  dcmotor::canBusNode cb(nh, nh_private);

  cb.update();
  return 0;
}
