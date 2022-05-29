/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file stop_shape.cpp
 *
 *@brief 停障框描述
 *
 *@author chenweijian
 *@modified
 *@version
 *@data
 ************************************************************************/
#include "stop_shape.hpp"
#include "log.hpp"
#include <map>
namespace CVTE_BABOT {

StopPolygon::StopPolygon(const std::vector<Eigen::Vector2d> &v_endpoint,
                         double start_theta, double end_theta) {
  if (v_endpoint.size() < 3) {
    LOG(ERROR) << "endpoing no enough";
  }

  v_endpoint_ = v_endpoint;
  start_theta_ = start_theta;
  end_theta_ = end_theta;
  getNormalVector();

  LOG(INFO) << "creat Polygon start " << start_theta_ << " end " << end_theta_
            << std::endl;
  for (int i = 0; i < v_endpoint_.size(); ++i) {
    LOG(INFO) << std::endl
              << "endpoint x " << v_endpoint_[i](0) << " y "
              << v_endpoint_[i](1) << std::endl
              << "product x " << v_product_[i].product_(0) << " y "
              << v_product_[i].product_(1) << std::endl
              << "mid_point x " << v_product_[i].origin_(0) << " y "
              << v_product_[i].origin_(1);
  }
}

bool StopPolygon::checkPonitInside(const Eigen::Vector2d &point) {
  // 先检查角度
  if (start_theta_ != end_theta_) {
    double product_start = point.dot(start_product_);
    double product_end = point.dot(end_product_);
    if (thetaLimit(end_theta_ - start_theta_) > 0) {
      // 方向相同
      if ((product_end <= 0 && product_start <= 0) == false) {
        return false;
      }
    } else {
      if (!(product_end <= 0 && product_start <= 0) == false) {
        return false;
      }
    }
  }

  for (int i = 0; i < v_product_.size(); ++i) {
    // 偏移
    Eigen::Vector2d check_point = point - v_product_[i].origin_offset_;

    // 点乘
    if (check_point.dot(v_product_[i].product_) > 0.0001) {
      return false;
    }
  }
  return true;
}

double StopPolygon::thetaLimit(double theta) {
  while (theta < -M_PI) { theta += 2 * M_PI; }
  while (theta > M_PI) { theta -= 2 * M_PI; }
  return theta;
}

void StopPolygon::setLimitInc(const double &x_max_inc, const double &x_min_inc,
                              const double &y_max_inc,
                              const double &y_min_inc) {
  x_max_inc_ = x_max_inc;
  x_min_inc_ = -x_min_inc;
  y_max_inc_ = y_max_inc;
  y_min_inc_ = -y_min_inc;
  inc_change_flag_ = true;

  // 偏移向量端点
  for (int i = 0; i < v_product_.size(); ++i) {
    // x
    if (v_product_[i].origin_(0) >= 0) {
      v_product_[i].origin_offset_(0) = x_max_inc_ + v_product_[i].origin_(0);
    } else if (v_product_[i].origin_(0) < 0) {
      v_product_[i].origin_offset_(0) = x_min_inc_ + v_product_[i].origin_(0);
    }
    // y
    if (v_product_[i].origin_(1) >= 0) {
      v_product_[i].origin_offset_(1) = y_max_inc_ + v_product_[i].origin_(1);
    } else if (v_product_[i].origin_(1) < 0) {
      v_product_[i].origin_offset_(1) = y_min_inc_ + v_product_[i].origin_(1);
    }
  }
}
void StopPolygon::setLimitInc(const SpeedDir dir, const double &temp) {
  if (dir == SHAPE_FRONT) {
    x_max_inc_ = temp;
  }
  if (dir == SHAPE_BACK) {
    x_min_inc_ = -temp;
  }
  if (dir == SHAPE_LEFT) {
    y_max_inc_ = temp;
  }
  if (dir == SHAPE_RIGHT) {
    y_min_inc_ = -temp;
  }
  inc_change_flag_ = true;

  // 偏移向量端点
  for (int i = 0; i < v_product_.size(); ++i) {
    // x
    if (v_product_[i].origin_(0) > 0) {
      v_product_[i].origin_offset_(0) = x_max_inc_ + v_product_[i].origin_(0);
    } else if (v_product_[i].origin_(0) < 0) {
      v_product_[i].origin_offset_(0) = x_min_inc_ + v_product_[i].origin_(0);
    }
    // y
    if (v_product_[i].origin_(1) > 0) {
      v_product_[i].origin_offset_(1) = y_max_inc_ + v_product_[i].origin_(1);
    } else if (v_product_[i].origin_(1) < 0) {
      v_product_[i].origin_offset_(1) = y_min_inc_ + v_product_[i].origin_(1);
    }
  }
}

void StopPolygon::clearLimitInc() {
  x_max_inc_ = 0;
  x_min_inc_ = 0;
  y_max_inc_ = 0;
  y_min_inc_ = 0;
  inc_change_flag_ = true;

  for (int i = 0; i < v_product_.size(); ++i) {
    v_product_[i].origin_offset_(0) = v_product_[i].origin_(0);
    v_product_[i].origin_offset_(1) = v_product_[i].origin_(1);
  }
}

void StopPolygon::getNormalVector() {
  Eigen::Matrix2d R;
  R << 0.0, 1.0, -1.0, 0.0;  //{cos,-sin,sin,cos} -90°旋转矩阵

  for (int i = 0; i < v_endpoint_.size() - 1; ++i) {
    // 法向量
    Eigen::Vector2d vecotr = R * (v_endpoint_[i + 1] - v_endpoint_[i]);
    Eigen::Vector2d origin = (v_endpoint_[i + 1] + v_endpoint_[i]) / 2;
    v_product_.emplace_back(vecotr, origin);
  }

  // 构建最后一个法向量
  Eigen::Vector2d vecotr = R * (v_endpoint_[0] - v_endpoint_.back());
  Eigen::Vector2d origin = (v_endpoint_.back() + v_endpoint_[0]) / 2;
  v_product_.emplace_back(vecotr, origin);

  start_product_(0) = 1 * cos(start_theta_ - (M_PI / 2));
  start_product_(1) = 1 * sin(start_theta_ - (M_PI / 2));

  // 结束角法向量 往逆时针旋转
  end_product_(0) = 1 * cos(end_theta_ + (M_PI / 2));
  end_product_(1) = 1 * sin(end_theta_ + (M_PI / 2));
}

void StopPolygon::getBoundingbox(double &x_max, double &x_min, double &y_max,
                                 double &y_min) {
  if (inc_change_flag_ == false) {
    x_max = x_max > x_max_ ? x_max : x_max_;
    x_min = x_min < x_min_ ? x_min : x_min_;
    y_max = y_max > y_max_ ? y_max : y_max_;
    y_min = y_min < y_min_ ? y_min : y_min_;
  } else {
    for (int i = 0; i < v_endpoint_.size(); ++i) {
      double x_max_inc = v_endpoint_[i](0) > 0 ? x_max_inc_ : 0;
      double x_min_inc = v_endpoint_[i](0) < 0 ? x_min_inc_ : 0;
      double y_max_inc = v_endpoint_[i](1) > 0 ? y_max_inc_ : 0;
      double y_min_inc = v_endpoint_[i](1) < 0 ? y_min_inc_ : 0;

      x_max = x_max < v_endpoint_[i](0) + x_max_inc_
                  ? v_endpoint_[i](0) + x_max_inc_
                  : x_max;

      x_min = x_min > v_endpoint_[i](0) + x_min_inc_
                  ? v_endpoint_[i](0) + x_min_inc_
                  : x_min;

      y_max = y_max < v_endpoint_[i](1) + y_max_inc_
                  ? v_endpoint_[i](1) + y_max_inc_
                  : y_max;

      y_min = y_min > v_endpoint_[i](1) + y_min_inc_
                  ? v_endpoint_[i](1) + y_min_inc_
                  : y_min;
    }

    inc_change_flag_ = false;
    x_max_ = x_max;
    x_min_ = x_min;
    y_max_ = y_max;
    y_min_ = y_min;
  }
}

StopShape::StopShape(const double &resolution) {
  setResolution(resolution);
}

void StopShape::push(const std::string &shape_key, std::vector<double> params,
                     double start_theta, double end_theta) {
  if (params.size() % 2 == 1 && params.size() < 6) {
    LOG(ERROR) << "creat shape error params size " << params.size();
  }
  std::vector<Eigen::Vector2d> v_endpoint;
  for (int i = 0; i < params.size(); i += 2) {
    v_endpoint.emplace_back(params[i], params[i + 1]);
  }

  m_polygon_.insert(decltype(m_polygon_)::value_type(
      shape_key, {v_endpoint, start_theta, end_theta}));

  // 创建缓存
  setInc(0, 0, 0, 0, shape_key);
  m_dir_shape_point_[shape_key] = CalculateStopShape(shape_key);
  if (shape_key != "all") {
    setInc(0, 0, 0, 0);
    m_dir_shape_point_["all"] = CalculateStopShape("all");
  }
  // TODO: front 比 back 小很多
  LOG(WARNING) << "creat: " << shape_key << " finish, "
               << " size: " << m_dir_shape_point_["all"].size();
}

void StopShape::push(const StopShapeConfig &config) {
  push(config.key, config.params, config.start_theta, config.end_theta);
}

bool StopShape::checkPonitInside(const Eigen::Vector2d &point,
                                 std::string key) {
  if (key == "all") {
    for (auto iter = m_polygon_.begin(); iter != m_polygon_.end(); ++iter) {
      if (iter->second.checkPonitInside(point)) {
        return true;
      }
    }
  } else {
    auto it_polygon = m_polygon_.equal_range(key);
    if (it_polygon.first != it_polygon.second) {
      for (auto iter = it_polygon.first; iter != it_polygon.second; ++iter) {
        if (iter->second.checkPonitInside(point)) {
          return true;
        }
      }
    }
  }
  return false;
}

bool StopShape::checkPonitInside(const double &x, const double &y,
                                 std::string key) {
  Eigen::Vector2d point = {x, y};
  return checkPonitInside(point, key);
}

void StopShape::setInc(double x_max_inc, double x_min_inc, double y_max_inc,
                       double y_min_inc, const std::string key) {
  m_inc_[key] = {x_max_inc, x_min_inc, y_min_inc, y_max_inc};

  if (key == "all") {
    for (auto &inc : m_inc_) {
      inc.second = {x_max_inc, x_min_inc, y_min_inc, y_max_inc};
    }
    for (auto iter = m_polygon_.begin(); iter != m_polygon_.end(); ++iter) {
      iter->second.setLimitInc(x_max_inc, x_min_inc, y_max_inc, y_min_inc);
    }
  } else {
    auto it_polygon = m_polygon_.equal_range(key);
    if (it_polygon.first != it_polygon.second) {
      for (auto iter = it_polygon.first; iter != it_polygon.second; ++iter) {
        iter->second.setLimitInc(x_max_inc, x_min_inc, y_max_inc, y_min_inc);
      }
    }
  }
}

void StopShape::setInc(const SpeedDir dir, const double &temp,
                       const std::string key) {
  if (key == "all") {
    for (auto &inc : m_inc_) {
      for (auto &inc_value : inc.second) { inc_value = temp; }
    }
    for (auto iter = m_polygon_.begin(); iter != m_polygon_.end(); ++iter) {
      iter->second.setLimitInc(dir, temp);
    }

  } else {
    auto it_polygon = m_polygon_.equal_range(key);
    if (it_polygon.first != it_polygon.second) {
      m_inc_[key][dir] = temp;
      for (auto iter = it_polygon.first; iter != it_polygon.second; ++iter) {
        iter->second.setLimitInc(dir, temp);
      }
    }
  }
}

void StopShape::clearInc() {
  for (auto iter = m_polygon_.begin(); iter != m_polygon_.end(); ++iter) {
    iter->second.clearLimitInc();
  }
  for (auto iter = m_inc_.begin(); iter != m_inc_.end(); ++iter) {
    iter->second = {0, 0, 0, 0};
  }
}

bool StopShape::checkPonitInside(const std::vector<Eigen::Vector2d> &v_point,
                                 std::string key,
                                 std::vector<Eigen::Vector2d> *v_crash_point,
                                 size_t limit_size, bool judge_at_once) {
  double x_max = INT_MIN;
  double x_min = INT_MAX;
  double y_max = INT_MIN;
  double y_min = INT_MAX;
  getBoundingbox(x_max, x_min, y_max, y_min);

  bool crash_flag = false;
  size_t crash_cnt = 0;

  for (int i = 0; i < v_point.size(); i++) {
    // 加速运算
    if (v_point[i](0) < x_min || v_point[i](0) > x_max ||
        v_point[i](1) < y_min || v_point[i](1) > y_max) {
      continue;
    }

    if (checkPonitInside(v_point[i], key)) {
      if (v_crash_point != nullptr) {
        v_crash_point->emplace_back(v_point[i]);
      }
      if (++crash_cnt > limit_size) {
        if (!judge_at_once) {
          return true;
        }
        crash_flag = true;
      }
    }
  }
  return crash_flag;
}

void StopShape::setResolution(const double &resolution) {
  resolution_ = resolution;
}

void StopShape::getBoundingbox(double &x_max, double &x_min, double &y_max,
                               double &y_min, const std::string key) {
  if (key == "all") {
    for (auto iter = m_polygon_.begin(); iter != m_polygon_.end(); ++iter) {
      iter->second.getBoundingbox(x_max, x_min, y_max, y_min);
    }
  } else {
    auto it_polygon = m_polygon_.equal_range(key);
    if (it_polygon.first != it_polygon.second)
      for (auto iter = it_polygon.first; iter != it_polygon.second; ++iter)
        iter->second.getBoundingbox(x_max, x_min, y_max, y_min);
  }
}

std::vector<Eigen::Vector2d> StopShape::getStopShape(const std::string key) {
  if (m_inc_.find(key) == m_inc_.end()) {
    LOG(ERROR) << "key error : " << key;
    return {};
  }

  if (m_inc_[key].size() < 4) {
    LOG(ERROR) << "point size error size: " << m_inc_[key].size();
    return {};
  }
  if (m_inc_[key][0] == 0 && m_inc_[key][1] == 0 && m_inc_[key][2] == 0 &&
      m_inc_[key][3] == 0) {
    // 取原始值
    // LOG(INFO) << "get normal shape ";
    return m_dir_shape_point_[key];
  } else {
    // 取增量就的值
    return CalculateStopShape(key);
  }
}

void StopShape::getStopShapeOriginal(
    std::vector<Eigen::Vector2d> **ptr_v_points, const std::string key) {
  if (m_dir_shape_point_.find(key) == m_dir_shape_point_.end()) {
    LOG(ERROR) << "key error : " << key;
    *ptr_v_points = nullptr;
    return;
  }
  *ptr_v_points = &m_dir_shape_point_[key];
}

std::vector<Eigen::Vector2d> StopShape::CalculateStopShape(
    const std::string key) {
  double x_max = INT_MIN;
  double x_min = INT_MAX;
  double y_max = INT_MIN;
  double y_min = INT_MAX;
  getBoundingbox(x_max, x_min, y_max, y_min, key);

  std::vector<Eigen::Vector2d> v_result_point;

  for (double x = x_min; x <= x_max; x += resolution_) {
    for (double y = y_min; y <= y_max; y += resolution_) {
      if (checkPonitInside(x, y, key)) {
        v_result_point.emplace_back(x, y);
      }
    }
  }

  return v_result_point;
}

}  // namespace CVTE_BABOT
