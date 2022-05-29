/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file footprint_utils.cpp
 *
 *@brief 一些公共函数
 *
 *@author chenmingjian (chenmingjian@cvte.com)
 *@version current_algo.dev.1.0
 *@data 2019-04-25
 ************************************************************************/
#include "footprint_utils.hpp"
#include <glog/logging.h>
#include <iostream>
#include <sstream>
#include <vector>

namespace CVTE_BABOT {

double sign0(const double &d_x) {
  return d_x < 0.0 ? -1.0 : (d_x > 0.0 ? 1.0 : 0.0);
}

void padFootprint(const double &d_padding,
                  std::vector<WorldmapPoint> &v_footprint) {
  for (unsigned int i = 0; i < v_footprint.size(); i++) {
    WorldmapPoint &pt = v_footprint[i];
    pt.d_x += sign0(pt.d_x) * d_padding;
    pt.d_y += sign0(pt.d_y) * d_padding;
    LOG(INFO) << "v_footprint_padding " << i <<  "," << pt.d_x << "," << pt.d_y;
  }
}

std::vector<std::vector<double>> parseVVF(const std::string &input,
                                          std::string &error_return) {
  std::vector<std::vector<double>> result;
  std::stringstream input_ss(input);
  int depth = 0;
  std::vector<double> current_vector;
  current_vector.reserve(2);  // x and y
  result.reserve(10);  // polygon vertex points num less than 10 generally.

  while (!!input_ss && !input_ss.eof()) {
    switch (input_ss.peek()) {
      case EOF:
        break;
      case '[':
        depth++;
        if (depth > 2) {
          error_return = "Array depth greater than 2";
          return result;
        }
        input_ss.get();
        current_vector.clear();
        break;
      case ']':
        depth--;
        if (depth < 0) {
          error_return = "More close ] than open [";
          return result;
        }
        input_ss.get();
        if (depth == 1) {
          result.push_back(current_vector);
        }
        break;
      case ',':
      case ' ':
      case '\t':
        input_ss.get();
        break;
      default:  // All other characters should be part of the numbers.
        if (depth != 2) {
          std::stringstream err_ss;
          err_ss << "Numbers at depth other than 2. Char was '"
                 << char(input_ss.peek()) << "'.";
          error_return = err_ss.str();
          return result;
        }
        double value;
        input_ss >> value;
        if (!!input_ss) {
          current_vector.push_back(value);
        }
        break;
    }
  }

  if (depth != 0) {
    error_return = "Unterminated vector string.";
  } else {
    error_return = "";
  }

  return result;
}

bool makeFootprintFromString(const std::string &str_footprint,
                             std::vector<WorldmapPoint> &v_footprint) {
  std::string str_error;
  std::vector<std::vector<double>> vvf;
  vvf.reserve(10);
  vvf = CVTE_BABOT::parseVVF(str_footprint, str_error);

  if (str_error != "") {
    LOG(ERROR) << "Error parsing footprint parameter: " << str_error.c_str()
               << std::endl;
    LOG(ERROR) << "  Footprint string was:" << str_footprint.c_str()
               << std::endl;
    return false;
  }

  // convert vvf into points.
  if (vvf.size() < 3) {
    LOG(ERROR) << "You must specify at least three points for the robot "
                  "footprint, reverting to previous footprint."
               << std::endl;
    return false;
  }

  v_footprint.reserve(vvf.size());
  WorldmapPoint wm_point;
  for (unsigned int i = 0; i < vvf.size(); i++) {
    if (vvf[i].size() == 2) {
      wm_point.d_x = vvf[i][0];
      wm_point.d_y = vvf[i][1];
      v_footprint.push_back(wm_point);
    } else {
      LOG(ERROR) << "Points in the footprint specification must be pairs of "
                    "numbers.  Found a point with"
                 << int(vvf[i].size()) << "numbers." << std::endl;
      return false;
    }
  }
  return true;
}

boost::shared_array<bool> makeFiled() {
  return boost::shared_array<bool>(new bool[FIELD_NUM]);
}

void resetFiled(const boost::shared_array<bool> &ptr_field) {
  if (nullptr != ptr_field.get()) {
    memset(ptr_field.get(), false, FIELD_NUM * sizeof(bool));
  }
}

}  // namespace CVTE_BABOT