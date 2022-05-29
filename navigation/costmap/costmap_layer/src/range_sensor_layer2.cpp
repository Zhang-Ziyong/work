#include "range_sensor_layer2.hpp"

#include <glog/logging.h>

#include <sstream>
#include <unordered_set>
#include "costmap_mediator.hpp"
#include "costmap_range_data.hpp"
#include "layer.hpp"

namespace CVTE_BABOT {

bool RangeSensorLayer2::onInitialize() {
  if (!CostmapMediator::getPtrInstance()->isParameterReady()) {
    LOG(ERROR) << "Is mediator's Parameter initialized? ";
    return false;
  }

  if (!CostmapMediator::getPtrInstance()->isLayeredCostmapReady()) {
    LOG(ERROR) << "Is mediator's LayeredCostmap initialized? ";
    return false;
  }
  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "enabled",
                                              b_enabled_, true);
  if (!b_enabled_) {
    LOG(ERROR) << s_name_ << " disabled";
    return false;
  }
  getParams();

  if (range_data_buffer_names_.empty()) {
    LOG(ERROR) << "range_data_buffer_names_.IsEmpty" << std::endl;
    return false;
  }
  auto ptr_data = std::make_shared<CostmapRangeData>();
  for (const auto &topic_name : range_data_buffer_names_) {
    CostmapMediator::getPtrInstance()
        ->registerUpdateFunc<std::shared_ptr<CostmapRangeData>>(topic_name,
                                                                ptr_data);
  }

  return true;
}

void RangeSensorLayer2::matchSize() {}

void RangeSensorLayer2::getParams() {
  d_resolution_ =
      CostmapMediator::getPtrInstance()->getCostmap()->getResolution();
  ui_size_x_ =
      CostmapMediator::getPtrInstance()->getCostmap()->getSizeInCellsX();
  ui_size_y_ =
      CostmapMediator::getPtrInstance()->getCostmap()->getSizeInCellsY();

  CostmapMediator::getPtrInstance()->getParam(s_name_ + "." + "clear_time",
                                              clear_time_, 10.0);

  std::string topics_string;
  std::string source;
  CostmapMediator::getPtrInstance()->getParam(s_name_ + ".topics",
                                              topics_string, std::string(""));
  {
    std::stringstream ss(topics_string);
    while (ss >> source) { range_data_buffer_names_.push_back(source); }
  }

  // printf params
  LOG(WARNING) << "creat " << s_name_ << " layer";
  LOG(INFO) << s_name_ << " clear_time: " << clear_time_;
  LOG(INFO) << s_name_ << " topics_string " << topics_string;
}

bool RangeSensorLayer2::updateBounds(const WorldmapPose &wp_robot_pose,
                                     CostmapBound &cb_costmap_bound) {
  if (!b_enabled_) {
    LOG(INFO) << "layer unabled.";
    return false;
  }
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  if (CostmapMediator::getPtrInstance()->isRolling()) {
    updateOrigin(wp_robot_pose.d_x - getSizeInMetersX() / 2,
                 wp_robot_pose.d_y - getSizeInMetersY() / 2);
  }

  // 根据时间阈值滤除旧的障碍物信息
  time_t time_now = time(NULL);
  for (auto it = range_data_list_.begin(); it != range_data_list_.end();) {
    if (time_now - it->update_time > clear_time_) {
      range_data_list_.erase(it++);
    } else {
      //数据按时间顺序保存
      break;
    }
  }

  // get sensors data
  WorldmapPose wp_sensor_pose;

  unsigned int ix = 0;
  unsigned int iy = 0;
  RangeData update_data;
  std::shared_ptr<CostmapRangeData> ptr_data = nullptr;
  for (auto it : range_data_buffer_names_) {
    std::string ids_;
    CostmapMediator::getPtrInstance()->getParam(std::string(it + ".frame_id"),
                                                ids_, std::string(""));
    std::stringstream id_ss_(ids_);
    std::string id_temp_;
    while (id_ss_ >> id_temp_) {
      // 申请内存
      double max_range_ir = 0.0;
      double min_range_ir = 1.0;
      double clear_range = 0.0;

      // get params
      CostmapMediator::getPtrInstance()->getParam(
          std::string(it + "." + id_temp_ + ".max_d"), max_range_ir, 1.0);
      CostmapMediator::getPtrInstance()->getParam(
          std::string(it + "." + id_temp_ + ".min_d"), min_range_ir, 0.0);
      CostmapMediator::getPtrInstance()->getParam(
          std::string(it + "." + id_temp_ + ".clear_range"), clear_range, 0.0);
      // 遍历所有传感器
      if (CostmapMediator::getPtrInstance()->getData(it + "." + id_temp_,
                                                     ptr_data)) {
        // LOG(INFO) << "ptr_data->range_ " << ptr_data->range_;
        // 遍历所有的已知点
        if (!range_data_list_.empty() && clear_range > 0.01) {
          // 大于清除距离根据超声数据以及设定的fov清除点
          if (ptr_data->range_ > clear_range) {
            // 计算到世界坐标的变换矩阵
            WorldmapPose pose = ptr_data->world_pose_;
            Eigen::Matrix3d T;
            T << cos(pose.d_yaw), -sin(pose.d_yaw), pose.d_x,  //
                sin(pose.d_yaw), cos(pose.d_yaw), pose.d_y,    //
                0, 0, 1;

            // 将clear_fov 法向量投影到当前世界坐标下
            std::vector<Eigen::Vector2d> v_product;
            std::vector<Eigen::Vector2d> v_origin;
            for (int i = 0; i < ptr_data->v_clear_product.size(); i++) {
              Eigen::Vector3d product(ptr_data->v_clear_product[i](0),
                                      ptr_data->v_clear_product[i](1), 1);
              product = T * product;
              v_product.emplace_back(product(0), product(1));

              Eigen::Vector3d origin(ptr_data->v_clear_origin[i](0),
                                     ptr_data->v_clear_origin[i](1), 1);
              origin = T * origin;
              v_origin.emplace_back(origin(0), origin(1));
            }

            // 遍历超声所有点
            for (auto obs_it = range_data_list_.begin();
                 obs_it != range_data_list_.end();) {
              // 判断半空间
              int inside_size = 0;
              for (int i = 0; i < v_product.size(); ++i) {
                Eigen::Vector2d sensor_pose(obs_it->x, obs_it->y);
                sensor_pose = sensor_pose - v_origin[i];
                if (sensor_pose.dot(v_product[i] - v_origin[i]) < 0) {
                  ++inside_size;
                }
              }

              // 缓存当前指针并自增
              std::list<RangeData>::iterator obs_it_cache = obs_it++;

              // 在凸包内，清除该障碍物点
              if (inside_size >= v_product.size()) {
                LOG(INFO) << ptr_data->s_topic_name_ << "." << id_temp_
                          << " clear range " << obs_it_cache->x << " "
                          << obs_it_cache->y;
                range_data_list_.erase(obs_it_cache);
              }
            }
          }
        }

        if (ptr_data->range_ < ptr_data->min_range_ ||
            ptr_data->range_ > ptr_data->max_range_ ||
            ptr_data->range_ < min_range_ir ||
            ptr_data->range_ > max_range_ir) {
          continue;
        }
        // T = {cos, -sin
        //      sin,  cos}
        wp_sensor_pose.d_x =
            (+cos(ptr_data->world_pose_.d_yaw)) * ptr_data->sensor_pose_.d_x +
            (-sin(ptr_data->world_pose_.d_yaw)) * ptr_data->sensor_pose_.d_y +
            ptr_data->world_pose_.d_x;
        wp_sensor_pose.d_y =
            (+sin(ptr_data->world_pose_.d_yaw)) * ptr_data->sensor_pose_.d_x +
            (+cos(ptr_data->world_pose_.d_yaw)) * ptr_data->sensor_pose_.d_y +
            ptr_data->world_pose_.d_y;
        wp_sensor_pose.d_yaw = normalizeAngle(ptr_data->world_pose_.d_yaw +
                                              ptr_data->sensor_pose_.d_yaw);
        update_data.update_time = time_now;
        update_data.x =
            wp_sensor_pose.d_x + ptr_data->range_ * cos(wp_sensor_pose.d_yaw);
        update_data.y =
            wp_sensor_pose.d_y + ptr_data->range_ * sin(wp_sensor_pose.d_yaw);

        range_data_list_.push_back(update_data);
        LOG(INFO) << "rangeLayer2 find obs topic "               //
                  << ptr_data->s_topic_name_ << "." << id_temp_  //
                  << " range " << ptr_data->range_  // 传感器距离
                  << " x " << update_data.x         // 传感器tf
                  << " y " << update_data.y;        //
      }
    }
  }
  return true;
}

//把数据加入到master层？ 不采用单独层更新，而是直接数据融合到master层，减少计算
bool RangeSensorLayer2::updateCosts(
    const std::shared_ptr<Costmap2d> master_grid, const int &i_min_i,
    const int &i_min_j, const int &i_max_i, const int &i_max_j) {
  if (!b_enabled_) {
    LOG(INFO) << "layer unabled.";
    return false;
  }
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  auto master_array = master_grid->getCharMap();

  unsigned int ix, iy;
  // LOG(INFO) << "size " << range_data_list_.size();
  for (auto it = range_data_list_.begin(); it != range_data_list_.end(); it++) {
    ix = static_cast<int>((it->x - d_origin_x_) / d_resolution_);
    iy = static_cast<int>((it->y - d_origin_y_) / d_resolution_);
    if (ix < ui_size_x_ && iy < ui_size_y_) {
      // LOG(INFO) << "range2 mark: " << ix << " " << iy << std::endl;
      master_array[iy * ui_size_x_ + ix] = CVTE_BABOT::LETHAL_OBSTACLE;
    }
  }
  return true;
}

void RangeSensorLayer2::updateOrigin(const double &d_new_origin_x,
                                     const double &d_new_origin_y) {
  int cell_ox =
      static_cast<int>((d_new_origin_x - d_origin_x_) / d_resolution_);
  int cell_oy =
      static_cast<int>((d_new_origin_y - d_origin_y_) / d_resolution_);

  d_origin_x_ = d_origin_x_ + cell_ox * d_resolution_;
  d_origin_y_ = d_origin_y_ + cell_oy * d_resolution_;
}

void RangeSensorLayer2::activate() {
  b_enabled_ = true;
}
void RangeSensorLayer2::deactivate() {
  b_enabled_ = false;
  reset();
}
void RangeSensorLayer2::reset() {
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  range_data_list_.clear();
}

}  // namespace CVTE_BABOT