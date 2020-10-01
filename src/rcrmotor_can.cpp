#include <iostream>
// #include <serial/serial.h>
#include <string>
#include <unistd.h>

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/JointState.h"

#include "robomodule/controlcan.h"

#include "robomodule/kinco/can_application.h"

mobile_base::CanApplication can_app;
void vel_pos_cmd(double vel, double pos);

void callback(const sensor_msgs::JointState &msg)
{
  // translation of data
  double vel = msg.velocity[0];
  double pos = msg.position[0];
  double cur = msg.effort[0];

  vel_pos_cmd(vel, pos);
}

void vel_pos_cmd(double vel, double pos)
{
  // cmd index: 9
  PVCI_CAN_OBJ cmd_obj = can_app.GetVciObject(1, 0x019);
  //  cmd_obj->Data[i] = ....
  int temp_pwm = 5000;
  int16_t temp_velocity = vel * 40 / 12.7;
  int32_t temp_position = pos * 150000.0 / 125.0;
  cmd_obj->Data[0] = (unsigned char)((temp_pwm >> 8) & 0xff);
  cmd_obj->Data[1] = (unsigned char)((temp_pwm)&0xff);
  cmd_obj->Data[2] = (unsigned char)((temp_velocity >> 8) & 0xff);
  cmd_obj->Data[3] = (unsigned char)(temp_velocity & 0xff);
  cmd_obj->Data[4] = (unsigned char)((temp_position >> 24) & 0xff);
  cmd_obj->Data[5] = (unsigned char)((temp_position >> 16) & 0xff);
  cmd_obj->Data[6] = (unsigned char)((temp_position >> 8) & 0xff);
  cmd_obj->Data[7] = (unsigned char)(temp_position & 0xff);
  cmd_obj->DataLen = 8;
  can_app.SendCommand(cmd_obj, 1);

  delete cmd_obj;
}

void INIT_MOTOR_DRIVER()
{
  PVCI_CAN_OBJ obj = can_app.GetVciObject(1, 0x0010); // can id = goup + num + control index ; here is  0x010; group 0, num 1 , control index of init  0.
  //PVCI_CAN_OBJ obj = can_app.GetVciObject(1, 0x1008);
  obj->DataLen = 8;
  for (size_t i = 0; i < 8; i++)
  {
    obj->Data[i] = 0x55;
  }
  can_app.SendCommand(obj, 1);
  delete obj;
  // ros::Duration(0.5).sleep();
}

void SET_DRIVER_MODA(int index)
{
  PVCI_CAN_OBJ obj = can_app.GetVciObject(1, 0x0011); // can id = goup + num + control index ; here is  0x010; group 0, num 1 , control index of init  0.
  //PVCI_CAN_OBJ obj = can_app.GetVciObject(1, 0x1008);
  obj->Data[0] = index; //1: open loop; 2 current; 3: velocity; 4: position; 5:vel-pos; 6:curr-vel;7:cur-pos;8:cur-vel-pos
  obj->DataLen = 8;
  for (size_t i = 0; i < 8; i++)
  {
    obj->Data[i] = 0x55;
  }
  can_app.SendCommand(obj, 1);
  delete obj;
  // ros::Duration(0.5).sleep();
}

struct rcr
{
  /* velocity,height,current(effort) */
  double velocity;
  double height;
  double current;
};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_cpp");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("rcr_now", 10);
  ros::Subscriber sub = nh.subscribe("rcr_target", 10, callback);

  can_app.CanActivate("/home/sgl/catkin_new/src/sensor_startup/config/kinco/can_config.yaml");
  ros::Duration(1.0).sleep();

  PVCI_CAN_OBJ obj = can_app.GetVciObject(1, 0x0010); // can id = goup + num + control index ; here is  0x010; group 0, num 1 , control index of init  0.
  //PVCI_CAN_OBJ obj = can_app.GetVciObject(1, 0x1008);
  obj->DataLen = 8;
  for (size_t i = 0; i < 8; i++)
  {
    obj->Data[i] = 0x55;
  }

  can_app.SendCommand(obj, 1);
  ros::Duration(0.5).sleep();
  
  obj->ID = 0x0011; 
  obj->Data[0] = 0x05; // velocity-position mode, index 5
  can_app.SendCommand(obj, 1);

  ros::Duration(0.8).sleep();

  obj->ID = 0x001a; // set automatically return data, index a
  obj->Data[0] = 0x0a;
  obj->Data[1] = 0x0; // if setting as 0x0a, other info can be gotta, refer to guide.
  can_app.SendCommand(obj, 1);

  delete obj;

  ros::Duration(0.8).sleep();

  int data_len = 100;
  int count = 0;
  while (ros::ok())
  {
    PVCI_CAN_OBJ data = new VCI_CAN_OBJ[data_len];

    //std::cout << std::hex << "0x" << std::to_string(data[0].ID) << std::endl;
    can_app.GetData(data, data_len);
    std::cout << std::hex << "0x" << (int)data[0].ID << std::endl;
    int16_t real_current = 0;
    int16_t real_velocity = 0;
    int32_t real_position = 0;
    if (data[0].ID == 0x01B)
    {
      real_current = (data[0].Data[0] << 8) | data[0].Data[1];
      real_velocity = (data[0].Data[2] << 8) | data[0].Data[3];
      real_position = (data[0].Data[4] << 24) | (data[0].Data[5] << 16) | (data[0].Data[6] << 8) | data[0].Data[7];
    }
    std::cout << "real_current: " << real_current << "  \n";
    std::cout << "real_velocity: " << real_velocity << "  \n";
    std::cout << "real_position: " << real_position << "  \n";
    // for (size_t i = 0; i < 8; i++)
    // {
    //   std::cout << std::hex << "0x" << (int)data[0].Data[i] << "  ";
    // }
    // std::cout << std::endl;
    delete[] data;

    ros::Duration(1.0).sleep();

    sensor_msgs::JointState js;
    js.header.frame_id = "rcrmotor";
    js.header.stamp = ros::Time::now();

    js.velocity.push_back( (double)real_velocity/ 40.0 * 12.7 ); // mm/s 
    js.position.push_back( (double)real_position / 150000.0 * 125  ); // mm
    js.effort.push_back( (double)real_current );
    pub.publish(js);

    ros::spinOnce();
  }

  can_app.CanClose();

  return 0;
}
