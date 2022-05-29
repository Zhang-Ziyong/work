#include <iostream>
#include "marker_map.hpp"

int main (int argc, char** argv) 
{
  // 地图参数
  const size_t size_x = 50;
  const size_t size_y = 100;
  const double resolution = 0.1;
  const double origin_x = 0.0;
  const double origin_y = 0.0;
  
  // 申明marker对象，并初始化地图
  CVTE_BABOT::MarkerMap marker_map;
  marker_map.resetMap(size_x, size_y, resolution, origin_x, origin_y);

  // 设置marker区域
  Json::Value regions;
  Json::Value region1;
  // 第一个区域
  Json::Value p11, p12, p13;
  p11["x"] = 2.5;
  p11["y"] = 0.5;
  p12["x"] = 1.5;
  p12["y"] = 2.5;
  p13["x"] = 3.5;
  p13["y"] = 2.5;
  Json::Value points1;
  points1.append(p11);
  points1.append(p12);
  points1.append(p13);
  region1["points"] = points1;
  // 第二个区域
  Json::Value region2;
  Json::Value p21, p22, p23, p24;
  p21["x"] = 1.5;
  p21["y"] = 5.0;
  p22["x"] = 3.5;
  p22["y"] = 5.0;
  p23["x"] = 3.5;
  p23["y"] = 8.0;
  p24["x"] = 1.5;
  p24["y"] = 8.0;
  Json::Value points2;
  points2.append(p21);
  points2.append(p22);
  points2.append(p23);
  points2.append(p24);
  region2["points"] = points2;

  regions.append(region1);
  regions.append(region2);

  // 设置减速带区域
  marker_map.setStripsMarker(regions);

  // 打印标记信息
  for (size_t y = 0; y < size_y; y++){

    if (y < 10) std::cout << "0" << y << " ";
    else std::cout << y << " ";
    
    for (size_t x = 0; x < size_x; x++) {
      if (marker_map.getMarkerType(x, y) == CVTE_BABOT::MarkerType::STRIP) {
        std::cout << "x ";
      } else {
        std::cout << "_ ";
      }
    }
    std::cout << std::endl;
  }

  return 0;
}