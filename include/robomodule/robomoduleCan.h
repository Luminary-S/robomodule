#ifndef ROBOMODULECAN_H
#define ROBOMODULECAN_H
///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, CUHK CURI.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/**
 * This is a ros node for RoboModule driver of DC motor with incremental
 * encoder.
 */

#include <math.h>

#include <string>
#include <iomanip>

#include "robomodule/controlcan.h"
#include "robomodule/kinco/can_application.h"

#define _CONFIG_FILE \
  "/home/sgl/catkin_new/src/robomodule/config/can_config.yaml"
#define ALL_MOTOR_CONFIG_FILE \
  "/home/sgl/catkin_new/src/robomodule/config/all_motor_config.yaml"

namespace rmCan {

struct rcr {
  /* velocity,height,current(effort) */
  double velocity;
  double height;
  double current;
  double vd;
  double pd;
  double cd;
};

class RobomoduleCAN {
 public:
  // RobomoduleCAN(unsigned int GID, unsigned int PID);
  RobomoduleCAN();
  virtual ~RobomoduleCAN() { can_close(); };
  // void send_cmd(cmd);
  void SET_DRIVER_MODA(unsigned int mode);
  void RESET_DRIVER_MODA(unsigned int mode);
  void SET_DRIVER_INIT();
  void INIT_MOTOR_DRIVER();

  void open_loop_cmd(int pwm);
  void vel_cmd(float vel);
  void pos_cmd(float pos);
  void vel_pos_cmd(double vel, double pos);

  void SEND_CMD();
  void init_robomodule_setting();
  void GET_DATA();
  float speed_controller(float target, float position, float interval);
  void update();

  // CAN related
  void can_activate();
  void INIT_CAN_PARAM();
  void can_close();

  unsigned int SET_CAN_ID(unsigned int index);
  void SET_DRIVER_AUTO_BACK_INFO(int period);
  bool CHECK_DRIVER_ALIVE();

  // param init
  void ParamInitWithFile(const std::string& file_address);
  void ParamInitWithFileByName(const std::string& file_address,
                               const std::string& name);
  void set_pwm(int PWM) { pwm = PWM; }
  void set_vd(float V) { vd = V; }
  void set_pd(float P) { pd = P; }
  int get_pwm() const { return pwm; }
  float get_vd() const { return vd; }
  float get_pd() const { return pd; }
  double get_real_pos() const { return real_pos; }
  double get_real_vel() const { return real_vel; }
  double get_real_cur() const { return real_cur; }

  void set_name(std::string str) { name = str; }
  std::string get_name() const { return name; }
  // void set_alive(uint alive) { _ALIVE = alive; }
  // uint get_alive(){return _ALIVE;}
  bool transmit_status() {
    if (TRANSMIT_FLAG == 0) {
      return false;
    } else {
      return true;
    }
  }

  PVCI_CAN_OBJ set_can_obj(const int& obj_num, const uint& initial_id) {
    return can_app.GetVciObject(obj_num, initial_id);
  }

  void set_id(unsigned int GID, unsigned int PID) {
    _GID = GID;
    _PID = PID;
  }

  void set_mode(int mode){moda = mode;}

 private:
  std::string name;
  mobile_base::CanApplication can_app;
  unsigned int _GID;  // group number
  unsigned int _PID;  // dirver number
  unsigned int _CAN_ID;
  // unsigned int _ALIVE;
  float Rate_;
  int MAX_PWM = 5000;
  float power_limit = 0.5;
  int moda;
  float Vel2RPM;
  float Pos2Encoder;
  float _DIAMETER = 40;
  float _REDUCTION_RATIO = 90;
  float _BASIC_ENCODER_LINES = 11;

  uint TRANSMIT_FLAG = 0;

  uint status = 0;
  float P_;
  float I_;
  float D_;
  int pwm = 0;
  float vd = 0.0;
  float pd = 0.0;

  double vr = 0.0;
  double pr = 0.0;
  double pwm_r = 0.0;
  
  double real_pos = 0.0;
  double real_vel = 0.0;
  double real_cur = 0.0;
  int16_t real_PWM;
  bool EMERGENCY;
};  // end class
}  // namespace rmCan
#endif