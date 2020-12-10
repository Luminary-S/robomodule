#include "robomodule/robomoduleCan.h"
#include "robomodule/controlcan.h"
#include "robomodule/kinco/can_application.h"
// #include "ros.h"
using mobile_base::CanApplication;
using rmCan::RobomoduleCAN;

int main(int argc, const char** argv) {
    
//     ros::init(argc, argv, "test_cpp");
//   ros::NodeHandle nh;

//   ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("rcr_now", 10);
//   ros::Subscriber sub = nh.subscribe("rcr_target", 10, callback);
  RobomoduleCAN rbCan;
  // rbCan.set_id(1, 1);
  rbCan.ParamInitWithFileByName(ALL_MOTOR_CONFIG_FILE, "CleanMotor");
  rbCan.can_activate();
  // rbCan.init_robomodule_setting();
  rbCan.INIT_MOTOR_DRIVER(); // long beep

  rbCan.SET_DRIVER_MODA(3); // short beep
  
  rbCan.SET_DRIVER_AUTO_BACK_INFO(10); // no beep
  //  return 0; 
  /*
  CanApplication can_app;
  can_app.CanActivate("/home/sgl/catkin_new/src/robomodule/config/can_config.yaml");
    //"/home/sgl/catkin_new/src/sensor_startup/config/kinco/can_config.yaml");
//   ros::Duration(1.0).sleep();

  PVCI_CAN_OBJ obj = can_app.GetVciObject(1, 0x0110); // can id = goup + num + control index ; here is  0x010; group 0, num 1 , control index of init  0.
  //PVCI_CAN_OBJ obj = can_app.GetVciObject(1, 0x1008);
  obj[0].DataLen = 8;
  for (size_t i = 0; i < 8; i++)
  {
    obj[0].Data[i] = 0x55;
  }

  can_app.SendCommand(obj, 1);
  sleep(0.5);
  
  obj->ID = 0x0011; 
  obj->Data[0] = 0x05; // velocity-position mode, index 5
  can_app.SendCommand(obj, 1);

  sleep(0.8);

  obj->ID = 0x001a; // set automatically return data, index a
  obj->Data[0] = 0x0a;
  obj->Data[1] = 0x0; // if setting as 0x0a, other info can be gotta, refer to guide.
  can_app.SendCommand(obj, 1);

  delete obj;

  sleep(0.8);
  */
std::cout << "success" << std::endl;
  int data_len = 100;
  int count = 0;
  while (true)
  {
    PVCI_CAN_OBJ data = new VCI_CAN_OBJ[data_len];

    // std::cout << std::hex << "0x" << std::to_string(data[0].ID) <<
    // std::endl; 
    uint data_num;
    int device_type_=3;
    int device_index_=0; 
    int can_index_=0;
    int wait_time_ =-1;
    data_num = VCI_GetReceiveNum(device_type_, device_index_, can_index_);
    int receive_num = VCI_Receive(device_type_, device_index_, can_index_, data,
                                  data_len, wait_time_);
    std::cout << " data_num : " << data_num << std::endl;
    std::cout << " receive_num : " << receive_num << std::endl;
    std::cout << std::hex << "0x"  << (int)data[0].ID << std::endl; 
    std::cout << std::dec << std::endl;
    // int16_t real_current = 0;
    // int16_t real_velocity = 0;
    // int32_t real_position = 0;
    if (data[0].ID == 0x11B)
    {
    int16_t real_cur = ((data[0].Data[0] << 8) | (data[0].Data[1]));
    int16_t real_vel = ((data[0].Data[2] << 8) | (data[0].Data[3]));
    int32_t real_pos = ((data[0].Data[4] << 24) | (data[0].Data[5] << 16) |
                        (data[0].Data[6] << 8) | (data[0].Data[7]));
    for (size_t i = 0; i < 8; i++)
    {
      std::cout << std::hex << "0x" << (int)data[0].Data[i] << "  ";
    }
    // std::cout << std::endl;
    std::cout << std::dec << std::endl;
    std::cout << "real_current: " << real_cur << "  \n";
    std::cout << "real_velocity: " << real_vel << "  \n";
    std::cout << "real_position: " << real_pos << "  \n" << std::endl;
    } else
    {
      std::cout << "no this motor data!" << std::endl;
    }
    
    delete[] data;
    sleep(0.001);

  }
//   int data_len = 100;
//   int count = 0;
//   while (ros::ok())
//   {
//     PVCI_CAN_OBJ data = new VCI_CAN_OBJ[data_len];

//     //std::cout << std::hex << "0x" << std::to_string(data[0].ID) << std::endl;
//     can_app.GetData(data, data_len);
//     std::cout << std::hex << "0x" << (int)data[0].ID << std::endl;
//     int16_t real_current = 0;
//     int16_t real_velocity = 0;
//     int32_t real_position = 0;
//     if (data[0].ID == 0x01B)
//     {
//       real_current = (data[0].Data[0] << 8) | data[0].Data[1];
//       real_velocity = (data[0].Data[2] << 8) | data[0].Data[3];
//       real_position = (data[0].Data[4] << 24) | (data[0].Data[5] << 16) | (data[0].Data[6] << 8) | data[0].Data[7];
//     }
//     std::cout << "real_current: " << real_current << "  \n";
//     std::cout << "real_velocity: " << real_velocity << "  \n";
//     std::cout << "real_position: " << real_position << "  \n";
//     // for (size_t i = 0; i < 8; i++)
//     // {
//     //   std::cout << std::hex << "0x" << (int)data[0].Data[i] << "  ";
//     // }
//     // std::cout << std::endl;
//     delete[] data;

//     ros::Duration(1.0).sleep();

//     sensor_msgs::JointState js;
//     js.header.frame_id = "rcrmotor";
//     js.header.stamp = ros::Time::now();

//     js.velocity.push_back( (double)real_velocity/ 40.0 * 12.7 ); // mm/s 
//     js.position.push_back( (double)real_position / 150000.0 * 125  ); // mm
//     js.effort.push_back( (double)real_current );
//     pub.publish(js);

//     ros::spinOnce();
//   }

  rbCan.can_close();

  return 0;

}