#include "robomodule/robomoduleCan.h"

// #include <serial/serial.h>
#include <unistd.h>

#include <iostream>
#include <string>

#include "robomodule/controlcan.h"
#include "robomodule/kinco/can_application.h"

using rmCan::RobomoduleCAN;
// using mobile_base::CanApplication;

// RobomoduleCAN::RobomoduleCAN(unsigned int GID, unsigned int PID) {
//   _GID = GID;
//   _PID = PID;
// }

RobomoduleCAN::RobomoduleCAN() {
  // _GID = GID;
  // _PID = PID;
}

unsigned int RobomoduleCAN::SET_CAN_ID(unsigned int index) {
  // can id = goup + num + control index ;
  // e.g.: 0x010; group 0, num 1 , control index of init  0.
  // _CAN_ID = (unsigned char)( 0* 16 * 16 * 16 + _GID * 16 * 16 + _PID * 16 +
  // index  );
  return (unsigned int)(((_GID << 8) & 0xFFFF) + ((_PID << 4) & 0xFFFF) +
                        index);
  // std::cout << _CAN_ID << std::hex << " 0x" << _CAN_ID << std::endl;
}

void RobomoduleCAN::INIT_MOTOR_DRIVER() {
  // SET_CAN_ID(0);
  // _CAN_ID = 0x0110;
  _CAN_ID = SET_CAN_ID(0);
  std::cout << _CAN_ID << std::hex << " 0x" << _CAN_ID << std::endl;
  PVCI_CAN_OBJ obj = can_app.GetVciObject(1, _CAN_ID);  // 0x0010
  // PVCI_CAN_OBJ obj = can_app.GetVciObject(1, 0x1008);
  obj->DataLen = 8;
  for (size_t i = 0; i < 8; i++) {
    obj->Data[i] = 0x55;
  }
  TRANSMIT_FLAG = can_app.SendCommand(obj, 1);
  // VCI_Transmit(3,0,0,obj,1);
  delete obj;
  // ros::Duration(0.5).sleep();
  sleep(0.5);  // 500ms
}

void RobomoduleCAN::SET_DRIVER_MODA(unsigned int mode) {
  // _CAN_ID = 0x0111;
  _CAN_ID = SET_CAN_ID(1);
  PVCI_CAN_OBJ obj = can_app.GetVciObject(
      1, _CAN_ID);  // can id = goup + num + control index ; here is  0x010;
                    // group 0, num 1 , control index of init  0.
  // PVCI_CAN_OBJ obj = can_app.GetVciObject(1, 0x1008);
  obj->Data[0] = (u_char)(
      mode);  // 0x03;// 1: open loop; 2 current; 3: velocity; 4: position;
              // 5:vel-pos; 6:curr-vel;7:cur-pos;8:cur-vel-pos
  obj->DataLen = 8;
  for (size_t i = 1; i < 8; i++) {
    obj->Data[i] = 0x55;
  }
  TRANSMIT_FLAG = can_app.SendCommand(obj, 1);
  delete obj;
  // ros::Duration(0.5).sleep();
  sleep(0.5);
}

void RobomoduleCAN::RESET_DRIVER_MODA(unsigned int mode) {
  INIT_MOTOR_DRIVER();
  SET_DRIVER_MODA(mode);
}

void RobomoduleCAN::SET_DRIVER_AUTO_BACK_INFO(int period) {
  // _CAN_ID = 0x0111;
  _CAN_ID = SET_CAN_ID(10);
  PVCI_CAN_OBJ obj = can_app.GetVciObject(
      1, _CAN_ID);  // can id = goup + num + control index ; here is  0x010;
                    // group 0, num 1 , control index of init  0.
  // PVCI_CAN_OBJ obj = can_app.GetVciObject(1, 0x1008);
  obj->Data[0] = (u_char)(period);  // period of transmitting vel, pos, cur;
  obj->Data[1] = 0x00;  //(u_char)(period); period of transmitting CTL1/CTL2
                        // high low, and PWM value
  obj->DataLen = 8;
  for (size_t i = 2; i < 8; i++) {
    obj->Data[i] = 0x55;
  }
  TRANSMIT_FLAG = can_app.SendCommand(obj, 1);
  delete obj;
  // ros::Duration(0.5).sleep();
  sleep(0.5);
}

bool RobomoduleCAN::CHECK_DRIVER_ALIVE() {
  _CAN_ID = SET_CAN_ID(15);
  PVCI_CAN_OBJ obj = can_app.GetVciObject(1, _CAN_ID);
  obj->DataLen = 8;
  for (size_t i = 0; i < 8; i++) {
    obj->Data[i] = 0x55;
  }
  can_app.SendCommand(obj, 1);
  delete obj;
  int data_len = 100;
  PVCI_CAN_OBJ data = new VCI_CAN_OBJ[data_len];
  can_app.GetData(data, data_len);
  bool flag = 0;
  if (data[0].ID == SET_CAN_ID(11)) {
    for (size_t i = 0; i < 8; i++) {
      if (data[0].Data[i] == 0x55) {
        flag = 1;
        continue;
      } else {
        flag = 0;
        break;
      }
    }
  }

  if (flag == 1) {
    return true;
  } else {
    return false;
  }
}
// void ERROR_CHECK()

void RobomoduleCAN::can_activate() {
  can_app.CanActivate(_CONFIG_FILE);
  sleep(1);
}

void RobomoduleCAN::INIT_CAN_PARAM() { can_app.ReadCanFile(_CONFIG_FILE); }

void RobomoduleCAN::can_close() { can_app.CanClose(); }

// open-loop moda, suggest send rate: 10ms
void RobomoduleCAN::open_loop_cmd(int pwm) {
  _CAN_ID = SET_CAN_ID(2);
  int temp_pwm = 0;
  PVCI_CAN_OBJ cmd_obj = can_app.GetVciObject(1, _CAN_ID);
  //  cmd_obj->Data[i] = ....
  if (pwm > MAX_PWM) {
    temp_pwm = MAX_PWM;
  } else {
    temp_pwm = pwm;
  }
  cmd_obj->Data[0] = (unsigned char)((temp_pwm >> 8) & 0xff);
  cmd_obj->Data[1] = (unsigned char)((temp_pwm)&0xff);
  cmd_obj->Data[2] = 0x55;
  cmd_obj->Data[3] = 0x55;
  cmd_obj->Data[4] = 0x55;
  cmd_obj->Data[5] = 0x55;
  cmd_obj->Data[6] = 0x55;
  cmd_obj->Data[7] = 0x55;
  cmd_obj->DataLen = 8;
  TRANSMIT_FLAG = can_app.SendCommand(cmd_obj, 1);

  delete cmd_obj;
}

// velocity moda, index: 4, suggest send rate: 10ms
void RobomoduleCAN::vel_cmd(float vel) {
  _CAN_ID = SET_CAN_ID(4);
  int temp_pwm = MAX_PWM;
  PVCI_CAN_OBJ cmd_obj = can_app.GetVciObject(1, _CAN_ID);

  int16_t temp_velocity = vel / Vel2RPM;
  cmd_obj->Data[0] = (unsigned char)((temp_pwm >> 8) & 0xff);
  cmd_obj->Data[1] = (unsigned char)((temp_pwm)&0xff);
  cmd_obj->Data[2] = (unsigned char)((temp_velocity >> 8) & 0xff);
  cmd_obj->Data[3] = (unsigned char)(temp_velocity & 0xff);
  cmd_obj->Data[4] = 0x55;
  cmd_obj->Data[5] = 0x55;
  cmd_obj->Data[6] = 0x55;
  cmd_obj->Data[7] = 0x55;
  cmd_obj->DataLen = 8;
  TRANSMIT_FLAG = can_app.SendCommand(cmd_obj, 1);

  delete cmd_obj;
}

// pos moda, index: 5, suggest send rate: 10ms
void RobomoduleCAN::pos_cmd(float pos) {
  _CAN_ID = SET_CAN_ID(5);
  int temp_pwm = MAX_PWM;
  PVCI_CAN_OBJ cmd_obj = can_app.GetVciObject(1, _CAN_ID);

  int32_t temp_position = pos * Pos2Encoder;  //* 150000.0 / 125.0;
  cmd_obj->Data[0] = (unsigned char)((temp_pwm >> 8) & 0xff);
  cmd_obj->Data[1] = (unsigned char)((temp_pwm)&0xff);
  cmd_obj->Data[2] = 0x55;
  cmd_obj->Data[3] = 0x55;
  cmd_obj->Data[4] = (unsigned char)((temp_position >> 8) & 0xff);
  cmd_obj->Data[5] = (unsigned char)(temp_position >> 16 & 0xff);
  cmd_obj->Data[6] = (unsigned char)(temp_position >> 8 & 0xff);
  cmd_obj->Data[7] = (unsigned char)(temp_position & 0xff);
  cmd_obj->DataLen = 8;
  TRANSMIT_FLAG = can_app.SendCommand(cmd_obj, 1);

  delete cmd_obj;
}

// vel-pos moda, index: 6, suggest send rate: 10ms
void RobomoduleCAN::vel_pos_cmd(double vel, double pos) {
  // cmd index: 6
  _CAN_ID = SET_CAN_ID(6);
  PVCI_CAN_OBJ cmd_obj = can_app.GetVciObject(1, _CAN_ID);
  //  cmd_obj->Data[i] = ....
  int temp_pwm = MAX_PWM;
  int16_t temp_velocity = vel / Vel2RPM;
  int32_t temp_position = pos * Pos2Encoder;  //* 150000.0 / 125.0;
  cmd_obj->Data[0] = (unsigned char)((temp_pwm >> 8) & 0xff);
  cmd_obj->Data[1] = (unsigned char)((temp_pwm)&0xff);
  cmd_obj->Data[2] = (unsigned char)((temp_velocity >> 8) & 0xff);
  cmd_obj->Data[3] = (unsigned char)(temp_velocity & 0xff);
  cmd_obj->Data[4] = (unsigned char)((temp_position >> 24) & 0xff);
  cmd_obj->Data[5] = (unsigned char)((temp_position >> 16) & 0xff);
  cmd_obj->Data[6] = (unsigned char)((temp_position >> 8) & 0xff);
  cmd_obj->Data[7] = (unsigned char)(temp_position & 0xff);
  cmd_obj->DataLen = 8;
  TRANSMIT_FLAG = can_app.SendCommand(cmd_obj, 1);

  delete cmd_obj;
}

void RobomoduleCAN::ParamInitWithFile(const std::string& file_address) {
  std::cout << file_address << std::endl;
  YAML::Node can_config = YAML::LoadFile(file_address);
  _DIAMETER = can_config["Diameter"].as<double>();
  _REDUCTION_RATIO = can_config["ReductionRatio"].as<int>();
  _BASIC_ENCODER_LINES = can_config["BasicEncoderLines"].as<int>();
  _GID = can_config["GroupNum"].as<int>();
  _PID = can_config["DeviceNum"].as<int>();
  power_limit = can_config["PowerLimit"].as<double>();
  moda = can_config["Mode"].as<int>();
}

void RobomoduleCAN::ParamInitWithFileByName(const std::string& file_address,
                                            const std::string& name) {
  std::cout << file_address << std::endl;
  YAML::Node can_config = YAML::LoadFile(file_address);
  _DIAMETER = can_config[name]["Diameter"].as<double>();
  _REDUCTION_RATIO = can_config[name]["ReductionRatio"].as<int>();
  _BASIC_ENCODER_LINES = can_config[name]["BasicEncoderLines"].as<int>();
  _GID = can_config[name]["GroupNum"].as<int>();
  _PID = can_config[name]["DeviceNum"].as<int>();
  // _ALIVE = can_config[name]["Alive"].as<int>();
  power_limit = can_config[name]["PowerLimit"].as<double>();
  moda = can_config[name]["Mode"].as<int>();
  P_ = can_config[name]["P"].as<double>();
  I_ = can_config[name]["I"].as<double>();
  D_ = can_config[name]["D"].as<double>();
  can_app.ReadCanFile(_CONFIG_FILE);
  std::cout<< "GID: " << _GID <<", pid: "<<_PID<< std::endl;
}

void RobomoduleCAN::init_robomodule_setting() {
  MAX_PWM = power_limit * MAX_PWM;
  Vel2RPM = M_PI * _DIAMETER / 60.0;  // mm/s to RPM
  Pos2Encoder =
      1.0 / M_PI / _DIAMETER * _BASIC_ENCODER_LINES * _REDUCTION_RATIO;
  // std::cout << _GID << std::endl;
  
  INIT_MOTOR_DRIVER();
  SET_DRIVER_MODA(moda);
  SET_DRIVER_AUTO_BACK_INFO(10);
}

void RobomoduleCAN::SEND_CMD() {
  int index = moda + 1;
  // SET_CAN_ID(index);
  switch (index) {
    case 2 /* constant-expression */:
      /* code */
      open_loop_cmd(pwm);
      break;
    case 4 /* constant-expression */:
      /* code */
      vel_cmd(vd);
      break;
    case 5 /* constant-expression */:
      /* code */
      pos_cmd(pd);
      break;
    case 6 /* constant-expression */:
      /* code */
      vel_pos_cmd(vd, pd);
      break;

    default:
      break;
  }
}

void RobomoduleCAN::GET_DATA() {
  int data_len = 100;
  PVCI_CAN_OBJ data = new VCI_CAN_OBJ[data_len];
  can_app.GetData(data, data_len);
  if (data[0].ID == SET_CAN_ID(11)) {
    int16_t real_cur = (data[0].Data[0] << 8) | data[0].Data[1];
    int16_t real_vel = (data[0].Data[2] << 8) | data[0].Data[3];
    int32_t real_pos = (data[0].Data[4] << 24) | (data[0].Data[5] << 16) |
                       (data[0].Data[6] << 8) | data[0].Data[7];
    real_vel = real_vel * Vel2RPM;
    real_pos = real_pos / Pos2Encoder;
    std::cout << "real_current: " << real_cur << "  \n";
    std::cout << "real_velocity: " << real_vel << "  \n";
    std::cout << "real_position: " << real_pos << "  \n" << std::endl;
  } else if (data[0].ID == SET_CAN_ID(13) && data[0].Data[0] == 0x01) {
    EMERGENCY = 1;
    std::cout << "motor is stuck!!!" << std::endl;
    return;
  } else if (data[0].ID == SET_CAN_ID(12)) {
    real_PWM = (data[0].Data[6] << 8) | data[0].Data[7];
  }

  delete[] data;
}

void RobomoduleCAN::update(){
    // 1. get cmd, for test, if ros, it should be subscribe from msg
    set_vd(2.0);
    set_pd(1.0);
    // 2. send cmd
    SEND_CMD();
    // 3. publish data
    GET_DATA();
}

float RobomoduleCAN::speed_controller(float target, float position,
                                   float interval) {
  // ROS_INFO(" speed controller part ");
  float speed;
  float speed_des = (target - position) / interval;
  // int kp = 10;
  speed = speed_des - P_ / 100.0 *  (target - position) ;
  // packDataCmd(target, (int)(speed*100), 0, direction, 1);
  return speed;
}

// int main(int argc, char** argv) {
//   RobomoduleCAN rbCan;
//   // rbCan.set_id(1, 1);
//   rbCan.ParamInitWithFileByName(ALL_MOTOR_CONFIG_FILE, "CleanMotor");
//   rbCan.can_activate();
//   // sleep(1);
//   // while(true){
//   // PVCI_CAN_OBJ obj = rbCan.set_can_obj(1, 0x0110);
//   // // PVCI_CAN_OBJ obj_ptr;
//   // PVCI_CAN_OBJ obj = new VCI_CAN_OBJ[1];
//   // for (size_t i = 0; i < 1; i++) {
//   //   obj->ID = 0x0110;
//   //   obj->RemoteFlag = 0;
//   //   obj->SendType = 0;
//   //   obj->RemoteFlag = 0;
//   //   obj->ExternFlag = 0;
//   //   // }

//   // }
//   //     obj->DataLen = 8;
//   //   for (size_t i = 0; i < 8; i++) {
//   //     obj->Data[i] = 0x55;
//   //   }
//   // rbCan.
//   // VCI_Transmit(3, 0, 0, obj, 1);
//   // rbCan.init_robomodule_setting();
//   rbCan.INIT_MOTOR_DRIVER();
//   // delete[] obj;
//   // sleep(1);
//   // }
//   // // rbCan.SET_CAN_ID(1);
//   // rbCan.INIT_MOTOR_DRIVER();
//   // rbCan.SET_DRIVER_MODA(3);  // velocity moda
//   // ros::init(argc, argv, "test_cpp");
//   // ros::NodeHandle nh;

//   // ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("rcr_now", 10);
//   // ros::Subscriber sub = nh.subscribe("rcr_target", 10, callback);

//   // rbCan.can_app.CanActivate("/home/sgl/catkin_new/src/sensor_startup/config/kinco/can_config.yaml");
//   // ros::Duration(1.0).sleep();

//   // PVCI_CAN_OBJ obj = can_app.GetVciObject(1, 0x0010); // can id = goup + num
//   // + control index ; here is  0x010; group 0, num 1 , control index of init 0.
//   // //PVCI_CAN_OBJ obj = can_app.GetVciObject(1, 0x1008);
//   // obj->DataLen = 8;
//   // for (size_t i = 0; i < 8; i++)
//   // {
//   // obj->Data[i] = 0x55;
//   // }

//   // can_app.SendCommand(obj, 1);
//   // ros::Duration(0.5).sleep();

//   // obj->ID = 0x0011;
//   // obj->Data[0] = 0x05; // velocity-position mode, index 5
//   // can_app.SendCommand(obj, 1);

//   // ros::Duration(0.8).sleep();

//   // obj->ID = 0x001a; // set automatically return data, index a
//   // obj->Data[0] = 0x0a;
//   // obj->Data[1] = 0x0; // if setting as 0x0a, other info can be gotta, refer
//   // to guide. can_app.SendCommand(obj, 1);

//   // delete obj;

//   // ros::Duration(0.8).sleep();

//   // int data_len = 100;
//   // int count = 0;
//   // while (ros::ok())
//   // {
//   // PVCI_CAN_OBJ data = new VCI_CAN_OBJ[data_len];

//   // //std::cout << std::hex << "0x" << std::to_string(data[0].ID) <<
//   // std::endl; can_app.GetData(data, data_len); std::cout << std::hex << "0x"
//   // << (int)data[0].ID << std::endl; int16_t real_current = 0; int16_t
//   // real_velocity = 0; int32_t real_position = 0; if (data[0].ID == 0x01B)
//   // {
//   //   real_current = (data[0].Data[0] << 8) | data[0].Data[1];
//   //   real_velocity = (data[0].Data[2] << 8) | data[0].Data[3];
//   //   real_position = (data[0].Data[4] << 24) | (data[0].Data[5] << 16) |
//   //   (data[0].Data[6] << 8) | data[0].Data[7];
//   // }
//   // std::cout << "real_current: " << real_current << "  \n";
//   // std::cout << "real_velocity: " << real_velocity << "  \n";
//   // std::cout << "real_position: " << real_position << "  \n";
//   //   // for (size_t i = 0; i < 8; i++)
//   //   // {
//   //   //   std::cout << std::hex << "0x" << (int)data[0].Data[i] << "  ";
//   //   // }
//   //   // std::cout << std::endl;
//   //   delete[] data;

//   //   ros::Duration(1.0).sleep();

//   //   sensor_msgs::JointState js;
//   //   js.header.frame_id = "rcrmotor";
//   //   js.header.stamp = ros::Time::now();

//   //   js.velocity.push_back( (double)real_velocity/ 40.0 * 12.7 ); // mm/s
//   //   js.position.push_back( (double)real_position / 150000.0 * 125  ); // mm
//   //   js.effort.push_back( (double)real_current );
//   //   pub.publish(js);

//   //   ros::spinOnce();
//   // }

//   // can_app.CanClose();
//   rbCan.can_close();

//   return 0;
// }
