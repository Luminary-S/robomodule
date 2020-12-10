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
    m.mode = it->second["Mode"].as<int>();
    m.canMotor.set_name(name);
    m.canMotor.ParamInitWithFileByName(ALL_MOTOR_CONFIG_FILE, name);
    m.multiplier = get_rate() / m.rate;
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
  control_msgs::SingleJointPositionFeedback msg;
  msg.header.frame_id = frame_id;  // NAME_;  //"rcrmotor";
  msg.header.stamp = ros::Time::now();
  // mp.header.frame_id = ;
  msg.velocity = data.v;
  msg.position = data.p;
  msg.error = data.error;
  pub.publish(msg);
}

MotorInfo canBusNode::getMotorInfo(Motor& m) {
  MotorInfo data;
  data.v = m.canMotor.get_real_vel();
  data.p = m.canMotor.get_real_pos();
  if (m.mode == VELOCITY_MODE) {
    data.error = m.canMotor.get_vd() - m.canMotor.get_real_vel();
  } else if (m.mode == POS_VEL_MODE) {
    data.error = m.canMotor.get_pd() - m.canMotor.get_real_pos();
  }

  return data;
}

void canBusNode::motor_ros_init(Motor& m, std::string name) {
  // ros::Subscriber sub;
  // ros::Publisher pub;
  std::string topic_name = name + "_cmd";     // e.g.: CleanMotor_cmd
  std::string fd_data_name = name + "_data";  // e.g.: CleanMotor_data
  m.sub = nh_.subscribe<control_msgs::SingleJointPositionActionGoal>(
      topic_name, 10, boost::bind(&canBusNode::cmdCallback, this, _1, name));
  m.pub = nh_.advertise<control_msgs::SingleJointPositionFeedback>(fd_data_name,
                                                                   10);

  // motorunits_map[name].alive = 0;
}

void canBusNode::update_motor_param(std::string n) {
  int md;
  std::string name = "test_" + n + "_node/" + n + "mode";
  ros::param::get(name, md);

  motorunits_map[name].mode = md;
  motorunits_map[name].canMotor.set_mode(md);
  motorunits_map[name].canMotor.init_robomodule_setting();
}

void canBusNode::startUP() {
  // 1. start can analysis
  // rmCan::RobomoduleCAN canStartUP;

  std::cout << " begin start up..." << std::endl;
  // 2. init each motor
  std::map<std::string, Motor>::iterator ite;

  for (ite = motorunits_map.begin(); ite != motorunits_map.end(); ++ite) {
    std::cout << " ----  read motor data file ----" << std::endl;
    std::string name = ite->first;
    if (ite->second.alive == 1) {
      std::cout << name << " is alive, start init..." << std::endl;
      while (true) {
        if (!ite->second.canMotor.transmit_status()) {
          std::cout << name << " motor no transimitting ! " << std::endl;
          ite->second.canMotor.init_robomodule_setting();
        } else {
          std::cout << name << " motor init success ! " << std::endl;
          std::cout << name << " transimitting state: "
                    << ite->second.canMotor.transmit_status() << std::endl;
          motor_ros_init(ite->second, name);
          // ite->second.canMotor.update();
          //           sleep(0.5);
          // ite->second.canMotor.update();
          // sleep(0.5);
          // ite->second.canMotor.update();
          // ite->second.canMotor.update();
          break;
        }
      }

    } else {
      std::cout << name
                << "is dead, if it should be started up, set the motor's Alive "
                   "param in config file."
                << std::endl;
      return;
    }
    sleep(1);
    std::cout << " ---- one motor init finished ----" << std::endl;
  }
}

void canBusNode::timerProc(unsigned int timer_cnt) {
  std::map<std::string, Motor>::iterator ite;
  // motorunits_map["CleanMotor"].canMotor.update();
  // std::cout << motorunits_map["CleanMotor"].canMotor << std::endl;
  // std::cout << motorunits_map["RCRMotor"] << std::endl;
  // return;
  for (ite = motorunits_map.begin(); ite != motorunits_map.end(); ++ite) {
    std::string name = ite->first;
    if (ite->second.alive == 1) {
      std::cout << name << " is alive, start working..." << std::endl;
      if (ite->second.canMotor.transmit_status()) {
        //   std::cout << name << " motor no transimitting ! " << std::endl;
        //   ite->second.canMotor.init_robomodule_setting();
        // } else {
        std::cout << name << " motor works WELL ! " << std::endl;
        // multiplier is 2, means if main rate is 100, motor control rate is 50,
        // s.t. cnt counts to 2 times, motor gets and sends one cmd
        if (timer_cnt % ite->second.multiplier == 0) {
          std::cout << name << " send cmds ! " << std::endl;
          ite->second.canMotor.update();

          // MotorInfo data = getMotorInfo(ite->second);
          Publish(ite->second.pub, name, getMotorInfo(ite->second));
        }
      } else {
        std::cout << name << " is not alive!" << std::endl;
      }
    }
  }
}

void canBusNode::update() {
  int64_t timer_cnt = 0;
  canStartUP.can_activate();
  startUP();
  std::cout << " ----- finish startup ! ------ " << std::endl;

  float rate = get_rate();
  ros::Rate r(rate);
  while (ros::ok()) {
    timer_cnt = timer_cnt + 1;
    std::cout << " ----- loop " << timer_cnt << " ! ------ " << std::endl;
    timerProc(timer_cnt);
    r.sleep();
    ros::spinOnce();
    if (_DEBUG_) {
      if (timer_cnt == 20) {
        return;
      }
    }
  }
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
