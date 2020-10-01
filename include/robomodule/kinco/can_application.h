#ifndef CAN_APPLICATION_H
#define CAN_APPLICATION_H

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "robomodule/controlcan.h"

namespace mobile_base {

class CanApplication {
 public:
  CanApplication();
  virtual ~CanApplication();
  // read parameters in configuration files which is used to initialize can
  // device
  void ReadCanFile(const std::string& file_address);
  void CanClose();
  // open, initialize and start the can device
  void CanActivate(const std::string& file_address);
  PVCI_CAN_OBJ GetVciObject(const int& obj_num, const uint& initial_id);
  uint SendCommand(PVCI_CAN_OBJ obj, const uint& len);
  void GetData(PVCI_CAN_OBJ obj, const int& obj_len);

 private:
  void CanDiagnostic(const std::string& description, const int& state);

  int device_type_;
  int device_index_;
  int can_index_;
  int wait_time_;

  int frame_len_;
};  // class CanApplication

}  // namespace mobile_base

#endif
