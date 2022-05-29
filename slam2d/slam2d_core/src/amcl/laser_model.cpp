#include "amcl/laser_model.hpp"
namespace slam2d_core {
namespace amcl {

double LaserModel::updateSensor(std::vector<Sample> &samples,
                                const unsigned int sample_count,
                                const LaserScanData &sensor_data) {
  return likelihoodFieldModel(samples, sample_count, sensor_data);
}

double LaserModel::likelihoodFieldModel(std::vector<Sample> &samples,
                                        const unsigned int sample_count,
                                        const LaserScanData &sensor_data) {
  double p = 1.0;
  double z = 0.0, pz = 0.0;
  double total_weight = 0.0;
  double obs_range = 0.0, obs_bearing = 0.0;
  double z_hit_denom = 2 * sigma_hit_ * sigma_hit_;
  double z_rand_mult = 1.0 / sensor_data.range_max;  //激光雷达最大距离
  double random_measurements = z_rand_ * z_rand_mult;
  double hit_x = 0.0, hit_y = 0.0;

  unsigned int step = (sensor_data.range_count - 1) / (max_beams_ - 1);
  // Step size must be at least 1
  if (step < 1) {
    step = 1;
  }

  unsigned int map_size_x = ptr_occupancy_map_->getSizeX();
  unsigned int map_size_y = ptr_occupancy_map_->getSizeY();
  double map_scale = ptr_occupancy_map_->getResolution();
  double map_ox = ptr_occupancy_map_->getOriginX();
  double map_oy = ptr_occupancy_map_->getOriginY();
  double map_max_occ_dist = ptr_occupancy_map_->getMaxOccDist();

  unsigned int index_x = 0;
  unsigned int index_y = 0;
  const std::vector<std::vector<Cell>> &map_cell =
      ptr_occupancy_map_->getCells();

  unsigned int pose_index_x = 0;
  unsigned int pose_index_y = 0;

  common::Rigid3 lase_pose;
  common::PointCloud point_cloud;
  for (size_t j = 0; j < sample_count; j++) {
    p = 1.0;
    lase_pose = samples[j].pose * laser_tf_;
    double yaw = common::quaternionToYaw(lase_pose.rotation());
    pose_index_x = (lase_pose.translation().x() - map_ox) / map_scale;
    pose_index_y = (lase_pose.translation().y() - map_oy) / map_scale;

    // if (pose_index_x < map_size_x && pose_index_y < map_size_y &&
    //     map_cell[pose_index_x][pose_index_y].occ_state <= 0)
    if (pose_index_x >= map_size_x || pose_index_y >= map_size_y ||
        map_cell[pose_index_x][pose_index_y].occ_state != 1) {
      for (size_t i = 0; i < sensor_data.range_count; i += step) {
        obs_range = sensor_data.ranges[i];
        obs_bearing = sensor_data.ranges_angle[i];

        if (obs_range >= sensor_data.range_max ||
            obs_range <= sensor_data.range_min) {
          continue;
        }

        if (obs_range != obs_range) {
          LOG(ERROR) << "range date was NAN ..";
          continue;
        }
        pz = 0.0;

        hit_x = lase_pose.translation().x() +
                obs_range * std::cos(yaw + obs_bearing);  // 0.2
        hit_y = lase_pose.translation().y() +
                obs_range * std::sin(yaw + obs_bearing);

        if (hit_x < map_ox || hit_y < map_oy) {
          z = map_max_occ_dist;
        } else {
          index_x = (hit_x - map_ox) / map_scale;
          index_y = (hit_y - map_oy) / map_scale;

          if (index_x < map_size_x && index_y < map_size_y) {
            z = map_cell[index_x][index_y].occ_dist;
          } else {
            z = map_max_occ_dist;
          }
        }
        pz += z_hit_ * std::exp(-(z * z) / z_hit_denom);
        pz += random_measurements;
        p += pz * pz * pz;
      }
      samples[j].weight *= p;
    } else {
      samples[j].weight = 0;
    }
    total_weight += samples[j].weight;
  }
  return total_weight;
}

slam2d_core::common::PointCloud LaserScanDataToPointCloud(
    const LaserScanData &ld) {
  slam2d_core::common::PointCloud point_cloud;
  for (size_t i = 0; i < ld.ranges.size(); ++i) {
    if (ld.range_min <= ld.ranges[i] && ld.ranges[i] <= ld.range_max &&
        i < ld.ranges_angle.size()) {
      const Eigen::AngleAxisd rotation(ld.ranges_angle[i],
                                       Eigen::Vector3d::UnitZ());
      slam2d_core::common::RangePoint point{
          rotation * (ld.ranges[i] * Eigen::Vector3d::UnitX())};
      point_cloud.push_back(point);
    }
  }
  return point_cloud;
}

}  // namespace amcl
}  // namespace slam2d_core