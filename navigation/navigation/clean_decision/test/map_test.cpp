/*
 * @Author: your name
 * @Date: 2021-10-21 15:58:02
 * @LastEditTime: 2021-10-21 18:52:40
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /navigation/navigation/clean_decision/test/map_test.cpp
 */
#include "data_base/obstacle_map.hpp"
#include <iostream>

void testReadFile() {
  CVTE_BABOT::ObstacleMap test_map;
  if (test_map.readMap("/home/cvte/Development/clean/map_2d.yaml")) {
    std::cout << "read file succeed" << std::endl;
    std::cout << "resolution: " << test_map.getResolution() << std::endl;
    std::cout << "height: " << test_map.getSizeInCellsY() << std::endl;
    std::cout << "width: " << test_map.getSizeInCellsX() << std::endl;
    std::cout << "origin_x: " << test_map.getOriginX() << std::endl;
    std::cout << "origin_y: " << test_map.getOriginY() << std::endl;
  } else {
    std::cout << "read file failed" << std::endl;
  }
}

void testSaveFile() {
  //   CVTE_BABOT::ObstacleMap test_map(200, 200, 0.1, 2, 2, 200);

  //   test_map.setOccupyThresh(100);
  //   test_map.setFreeThresh(50);
  CVTE_BABOT::ObstacleMap test_map;
  test_map.readMap("/home/cvte/Development/clean/map_2d.yaml");
  if (test_map.saveMap("/home/cvte/Development/clean/test_map_2d")) {
    std::cout << "save map succeed" << std::endl;
  } else {
    std::cout << "save map failed" << std::endl;
  }
}

void testSetValue() {
  CVTE_BABOT::ObstacleMap test_map;
  test_map.readMap("/home/cvte/Development/clean/map_2d.yaml");
  if (test_map.saveMap("/home/cvte/Development/clean/test_map_2d")) {
    std::cout << "save map succeed" << std::endl;
  } else {
    std::cout << "save map failed" << std::endl;
  }
  for (size_t j = 20; j < 30 && j < test_map.getSizeInCellsY(); j++) {
    for (size_t i = 0; i < test_map.getSizeInCellsX(); i++) {
      test_map.setMapPointValue(CVTE_BABOT::GridmapPoint(i, j), 200);
    }
  }
  test_map.saveMap("/home/cvte/Development/clean/test_map_2d");
}

int main(int argv, char **argc) {
  testReadFile();
  testSaveFile();
  testSetValue();
  return 0;
}
