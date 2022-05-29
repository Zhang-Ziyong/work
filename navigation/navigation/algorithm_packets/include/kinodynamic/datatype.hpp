/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file datatype.hpp
 *
 *@brief kinodynamicA星规划算法需要依赖的一些数据类
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Copyright 2019 Boyu Zhou, Aerial Robotics Group
 * information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
 *@version Navigation-v2.0
 *@data 2020-12-02
 ************************************************************************/

#ifndef __DATATYPE_HPP
#define __DATATYPE_HPP

#include <eigen3/Eigen/Core>
#include <iostream>
#include <string>
#include <unordered_map>
#include <boost/functional/hash.hpp>

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

namespace CVTE_BABOT {

// voxel hashing
template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const &matrix) const {
    size_t seed = 0;
    size_t size = matrix.size();
    for (size_t i = 0; i < size; ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};  

class PathNode {
 public:
  PathNode() = default;
  ~PathNode() = default;

  Eigen::Vector3i index;
  Eigen::Vector2d input;
  Eigen::Matrix<double, 6, 1> state;
  PathNode *parent = NULL;
  double g_score = 0.0;
  double f_score = 0.0;
  double duration = 0.0;
  double time = 0.0;  // dyn
  int time_idx = 0;
  char node_state = NOT_EXPAND;
};

typedef PathNode *PathNodePtr;

class NodeComparator {
 public:
  bool operator()(PathNodePtr node1, PathNodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

class NodeHashTable {
 public:
  NodeHashTable() = default;
  ~NodeHashTable() = default;
  void insert(Eigen::Vector3i idx, PathNodePtr node) {
    data_3d_.insert(std::make_pair(idx, node));
  }
  void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node) {
    data_4d_.insert(
        std::make_pair(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  PathNodePtr find(Eigen::Vector3i idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  PathNodePtr find(Eigen::Vector3i idx, int time_idx) {
    auto iter =
        data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
    data_4d_.clear();
  }

 private:
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
      data_3d_;
  std::unordered_map<Eigen::Vector4i, PathNodePtr, matrix_hash<Eigen::Vector4i>>
      data_4d_;
};

}  // namespace CVTE_BABOT

#endif  // endof __DATATYPE_HPP
