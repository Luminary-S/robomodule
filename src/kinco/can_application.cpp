#include "robomodule/kinco/can_application.h"

using mobile_base::CanApplication;

CanApplication::CanApplication() {}

CanApplication::~CanApplication() {}

void CanApplication::ReadCanFile(const std::string& file_address) {
  std::cout << file_address << std::endl;
  YAML::Node can_config = YAML::LoadFile(file_address);

  device_type_ = can_config["device_type"].as<int>();
  device_index_ = can_config["device_index"].as<int>();
  can_index_ = can_config["can_index"].as<int>();
  wait_time_ = can_config["wait_time"].as<int>();
  frame_len_ = can_config["frame_len"].as<int>();
}

void CanApplication::CanActivate(const std::string& file_address) {
  ReadCanFile(file_address);

  int open_state = VCI_OpenDevice(device_type_, device_index_, 0);
  CanDiagnostic("open can device", open_state);

  VCI_INIT_CONFIG can_init_config;
  can_init_config.AccCode = 0x00000000;
  can_init_config.AccMask = 0xFFFFFFFF;
  can_init_config.Filter = 0;
  can_init_config.Mode = 0;
  can_init_config.Timing0 = 0x00;  // 0x1400 & 0xff;//0x00;
  can_init_config.Timing1 = 0x1C;  // 0x14;  // 0x1400>>8 ;//0x1c;

  int init_state =
      VCI_InitCAN(device_type_, device_index_, can_index_, &can_init_config);
  CanDiagnostic("init can", init_state);

  int start_state = VCI_StartCAN(device_type_, device_index_, can_index_);
  CanDiagnostic("start can", start_state);
}

void CanApplication::CanClose() {
  VCI_CloseDevice(device_type_, device_index_);
}
void CanApplication::CanReset(){
  DWORD dwRel = VCI_ResetCAN(device_type_, device_index_, can_index_);
}

void CanApplication::CanDiagnostic(const std::string& description,
                                   const int& state) {
  switch (state) {
    case 1: {
      std::cout << "Successful! " << description << std::endl;
      break;
    }
    case 0: {
      std::cout << "Failure! " << description << std::endl;
      break;
    }
    case -1: {
      std::cout << "Lose USB Error! " << description << std::endl;
      break;
    }
    default: {
      std::cout << "Incorrect state feedback of CAN!" << std::endl;
      break;
    }
  }
}

PVCI_CAN_OBJ CanApplication::GetVciObject(const int& obj_num,
                                          const uint& initial_id) {
  PVCI_CAN_OBJ obj_ptr;
  obj_ptr = new VCI_CAN_OBJ[obj_num];
  for (size_t i = 0; i < obj_num; i++) {
    obj_ptr[i].ID = initial_id;
    obj_ptr[i].RemoteFlag = 0;
    obj_ptr[i].SendType = 0;
    obj_ptr[i].RemoteFlag = 0;
    obj_ptr[i].ExternFlag = 0;
  }

  return obj_ptr;
}

uint CanApplication::SendCommand(PVCI_CAN_OBJ obj, const uint& obj_len) {
  // std::cout << device_type_ << ", " << device_index_ << ", " << can_index_
  //           << ", " << obj << ", " << frame_len_ << std::endl;
  uint ret = 0;
  for (size_t i = 0; i < obj_len; i++) {
    // try {
    ret =
        VCI_Transmit(device_type_, device_index_, can_index_, obj, frame_len_);
    // if (ret == 0) {
    //   throw "send data error";
    // }
    // } catch (const std::exception&) {
    // }

    // unsigned int temp = VCI_Transmit(3,0,0,obj,1);
    std::cout << " send ret : " << ret << std::endl;
    if (ret == 0)
    {
      return ret;
    }   
  }
  return ret;
}

void CanApplication::GetData(PVCI_CAN_OBJ obj, const int& obj_len) {
  // std::cout << " device_type_ : " << device_type_ << std::endl;
  // std::cout << " device_index_ : " << device_index_ << std::endl;
  // std::cout << " can_index_ : " << can_index_ << std::endl;
  // std::cout << " wait_time_ : " << wait_time_ << std::endl;
  uint data_num;
  data_num = VCI_GetReceiveNum(device_type_, device_index_, can_index_);
  std::cout << " data_num : " << data_num << std::endl;
  // if (-1 == data_num) {
  //   std::cout << "Get data number failure!" << std::endl;
  // } else if (0 == data_num) {
  //   std::cout << "No data in the buffer" << std::endl;
  // }
  int receive_num = VCI_Receive(device_type_, device_index_, can_index_, obj,
                                obj_len, wait_time_);
  std::cout << " receive_num : " << receive_num << std::endl;


}
