Robomodule Driver in Can analysis
--------------------

1. Can analysis: Zhou Ligong USBCan-mini
2. robomodule: 403 and 109
3. data sheets for hardwares can be found in `./sheet` folder
4. `./lib` is the usbcan driver given by the product.
5. usb can port can be automatically found by the usbcan driver, no port should be configured.

# for new hardware
1. new motor: revise `./config/all_motor_config.yaml` motor configuration 
2. new can : refer to `./sheet/CAN_library_um.pdf` page 7 and revise
3. new robomodule hardware: change configuration in windows, refer to `./sheet/000-RoboModule-RMDS系列驱动器-调试前的准备.pdf` and `./sheet/002-RoboModule-RMDS系列驱动器-快速入门.pdf`
4. new robomodule control mode: refer to `./sheet/011-RoboModule-RMDS-CAN-protocal.pdf` and `./sheet/004-RoboModule-RMDS系列驱动器-驱动器运动模式讲解.pdf`


# test Can
test can communication with robomodule can 
1. launch file 
```ros
roslaunch robomodule test_can.launch
```
2. src file `./src/test_can.cpp`

# demo 
two motors working connected with USBCan  
1. launch file 
```ros
roslaunch robomodule test_can.launch
```
2. src file:
   1.  `./src/kinco/can_application.cpp`: communication methods for USBCan defined with config file `./config/can_config.yaml` 
   2.  `./src/robomoduleCan.cpp` : robomodule control protocol
   3.  `./src/canbus_motor_node.cpp`: can bus control motors registered in `./config/all_motor_config.yaml` 
3. control panel file:
   1. `./python/clean_motor_control.py`: control clean motor in terminal
   2. `./python/rcr_motor_control.py`: control RCR motor in terminal