/*
 * @Author: your name
 * @Date: 2020-07-15 17:10:40
 * @LastEditTime: 2020-07-20 20:31:05
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /cvte_lidar_slam/src/state_machine/test_state_machine.cpp
 */
#include "state_machine/slam_state_machine.hpp"
#include <iostream>

bool localization_callback() {
  std::cout
      << "***********************trans to localization*********************"
      << std::endl;
  return true;
}

bool mapping_callback() {
  std::cout << "******************trans to mapping***********************"
            << std::endl;
  return true;
}

bool return_mapping_callback() {
  std::cout
      << "******************trans to return mapping***********************"
      << std::endl;
  return true;
}

bool save_map_callback() {
  std::cout << "******************************save map***********************"
            << std::endl;
  return true;
}

bool enter_map_callback() {
  std::cout << "***************************enter map status "
               "callback******************************"
            << std::endl;
  return true;
}

int main(int argc, char **argv) {
  std::shared_ptr<cvte_lidar_slam::SlamStateMachine> ptr_slam_state_machine =
      cvte_lidar_slam::SlamStateMachine::getInstance();
  std::string init_state = ptr_slam_state_machine->getCurrentStateName();

  std::cout << "init state: " << init_state << std::endl;

  ptr_slam_state_machine->registerTransActionCallback(
      cvte_lidar_slam::START_LOCALIZATION, localization_callback);
  ptr_slam_state_machine->registerTransActionCallback(
      cvte_lidar_slam::START_MAPPING, mapping_callback);
  ptr_slam_state_machine->registerTransActionCallback(cvte_lidar_slam::SAVE_MAP,
                                                      save_map_callback);
  ptr_slam_state_machine->registerEntryActionCallback(
      cvte_lidar_slam::AD_STATU::MAPPING, enter_map_callback);
  ptr_slam_state_machine->registerTransActionCallback(
      cvte_lidar_slam::RETURN_MAPPING, return_mapping_callback);

  ptr_slam_state_machine->sendEvent(cvte_lidar_slam::START_MAPPING);
  std::string current_state = ptr_slam_state_machine->getCurrentStateName();
  std::cout << "current state: " << current_state << std::endl;

  ptr_slam_state_machine->sendEvent(cvte_lidar_slam::SAVE_MAP);
  current_state = ptr_slam_state_machine->getCurrentStateName();
  std::cout << "current state: " << current_state << std::endl;

  std::cout << "send start localization" << std::endl;
  ptr_slam_state_machine->sendEvent(cvte_lidar_slam::START_LOCALIZATION);
  current_state = ptr_slam_state_machine->getCurrentStateName();
  std::cout << "current state: " << current_state << std::endl;

  ptr_slam_state_machine->sendEvent(cvte_lidar_slam::CANCLE_MAPPING);
  current_state = ptr_slam_state_machine->getCurrentStateName();
  std::cout << "current state: " << current_state << std::endl;

  ptr_slam_state_machine->sendEvent(cvte_lidar_slam::START_LOCALIZATION);
  current_state = ptr_slam_state_machine->getCurrentStateName();
  std::cout << "current state: " << current_state << std::endl;

  // ptr_slam_state_machine->sendEvent(cvte_lidar_slam::STOP_LOCALIZAION);
  // current_state = ptr_slam_state_machine->getCurrentStateName();
  // std::cout << "current state: " << current_state << std::endl;

  // ptr_slam_state_machine->sendEvent(cvte_lidar_slam::START_LOCALIZATION);
  // current_state = ptr_slam_state_machine->getCurrentStateName();
  // std::cout << "current state: " << current_state << std::endl;

  // ptr_slam_state_machine->sendEvent(cvte_lidar_slam::STOP_LOCALIZAION);
  // current_state = ptr_slam_state_machine->getCurrentStateName();
  // std::cout << "current state: " << current_state << std::endl;

  // ptr_slam_state_machine->sendEvent(cvte_lidar_slam::START_LOCALIZATION);
  // current_state = ptr_slam_state_machine->getCurrentStateName();
  // std::cout << "current state: " << current_state << std::endl;

  ptr_slam_state_machine->sendEvent(cvte_lidar_slam::INIT_LOCALIZATION_SUCCESS);
  current_state = ptr_slam_state_machine->getCurrentStateName();
  std::cout << "current state: " << current_state << std::endl;

  ptr_slam_state_machine->sendEvent(cvte_lidar_slam::RETURN_MAPPING);
  current_state = ptr_slam_state_machine->getCurrentStateName();
  std::cout << "current state: " << current_state << std::endl;

  // ptr_slam_state_machine->sendEvent(cvte_lidar_slam::STOP);
  // current_state = ptr_slam_state_machine->getCurrentStateName();
  // std::cout << "current state: " << current_state << std::endl;

  return 0;
}