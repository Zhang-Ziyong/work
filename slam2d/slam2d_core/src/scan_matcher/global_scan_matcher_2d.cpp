#include "scan_matcher/global_scan_matcher_2d.hpp"

#include <cmath>
#include <map>

namespace slam2d_core {
namespace scan_matcher {

//最大值滑动窗口的定义和实现
class SlidingWindowMaximum {
 public:
  void addValue(const float value) {
    while (!non_ascending_maxima_.empty() &&
           value > non_ascending_maxima_.back()) {
      non_ascending_maxima_.pop_back();
    }
    non_ascending_maxima_.push_back(value);
  }

  void removeValue(const float value) {
    DCHECK(!non_ascending_maxima_.empty());
    DCHECK_LE(value, non_ascending_maxima_.front());
    if (value == non_ascending_maxima_.front()) {
      non_ascending_maxima_.pop_front();
    }
  }

  float getMaximum() const {
    DCHECK_GT(non_ascending_maxima_.size(), 0);
    return non_ascending_maxima_.front();
  }

  void checkIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

 private:
  std::deque<float> non_ascending_maxima_;
};

//多分辨类地图生成的核心类实现
PrecomputeOccupancyGrid2D::PrecomputeOccupancyGrid2D(
    const std::shared_ptr<slam2d_core::amcl::OccupancyGrid> occ_grid_ptr,
    const common::CellLimits &limits, int width,
    std::vector<float> *reusable_intermediate_grid)
    : offset_(-width + 1, -width + 1),
      wide_limits_(limits.num_x_cells + width - 1,
                   limits.num_y_cells + width - 1),
      min_score_(0.f),
      max_score_(1.f),
      cells_(wide_limits_.num_x_cells * wide_limits_.num_y_cells) {
  //   CHECK_GE(width, 1);
  //   CHECK_GE(limits.num_x_cells, 1);
  //   CHECK_GE(limits.num_y_cells, 1);
  const int stride = wide_limits_.num_x_cells;

  std::vector<float> &intermediate = *reusable_intermediate_grid;
  intermediate.resize(wide_limits_.num_x_cells * limits.num_y_cells);

  const std::vector<std::vector<slam2d_core::amcl::Cell>> &map_cell =
      occ_grid_ptr->getCells();

  for (int y = 0; y != limits.num_y_cells; ++y) {
    SlidingWindowMaximum current_values;
    current_values.addValue(cellStateToProbability(map_cell[0][y]));

    for (int x = -width + 1; x != 0; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.getMaximum();
      if (x + width < limits.num_x_cells) {
        current_values.addValue(cellStateToProbability(map_cell[x + width][y]));
      }
    }
    for (int x = 0; x < limits.num_x_cells - width; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.getMaximum();
      current_values.removeValue(cellStateToProbability(map_cell[x][y]));
      current_values.addValue(cellStateToProbability(map_cell[x + width][y]));
    }
    for (int x = std::max(limits.num_x_cells - width, 0);
         x != limits.num_x_cells; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.getMaximum();
      current_values.removeValue(cellStateToProbability(map_cell[x][y]));
    }
    current_values.checkIsEmpty();
  }

  for (int x = 0; x != wide_limits_.num_x_cells; ++x) {
    SlidingWindowMaximum current_values;
    current_values.addValue(intermediate[x]);
    for (int y = -width + 1; y != 0; ++y) {
      cells_[x + (y + width - 1) * stride] =
          computeCellValue(current_values.getMaximum());
      if (y + width < limits.num_y_cells) {
        current_values.addValue(intermediate[x + (y + width) * stride]);
      }
    }
    for (int y = 0; y < limits.num_y_cells - width; ++y) {
      cells_[x + (y + width - 1) * stride] =
          computeCellValue(current_values.getMaximum());
      current_values.removeValue(intermediate[x + y * stride]);
      current_values.addValue(intermediate[x + (y + width) * stride]);
    }
    for (int y = std::max(limits.num_y_cells - width, 0);
         y != limits.num_y_cells; ++y) {
      cells_[x + (y + width - 1) * stride] =
          computeCellValue(current_values.getMaximum());
      current_values.removeValue(intermediate[x + y * stride]);
    }
    current_values.checkIsEmpty();
  }
}
//栅格点概率映射到[0~256]
uint8_t PrecomputeOccupancyGrid2D::computeCellValue(
    const float probability) const {
  const int cell_value = std::lround((probability - min_score_) *
                                     (255.f / (max_score_ - min_score_)));
  CHECK_GE(cell_value, 0);
  CHECK_LE(cell_value, 255);
  return cell_value;
}
//占用栅格状态转化为概率
float PrecomputeOccupancyGrid2D::cellStateToProbability(
    slam2d_core::amcl::Cell cell) {
  //   static std::map<int, float> occ_state_to_probility_map;
  //   occ_state_to_probility_map[-1] = 0.f;
  //   occ_state_to_probility_map[0] = 0.5f;
  //   occ_state_to_probility_map[1] = 1.f;
  //   return occ_state_to_probility_map[cell.occ_state];
  static float occ_mac_dist = 0.5;
  return 1.f - cell.occ_dist / occ_mac_dist;
}

GlobalScanMatcher2D::GlobalScanMatcher2D(
    const GlobalScanMatcher2DOptions &options)
    : options_(options) {}

GlobalScanMatcher2D::~GlobalScanMatcher2D() {}

void GlobalScanMatcher2D::setMap(
    const std::shared_ptr<slam2d_core::amcl::OccupancyGrid> ptr_occupancy_map) {
  ptr_occupancy_map_ = ptr_occupancy_map;
  //生成多层的分辨率地图
  precomputation_Occupancy_grids_.reserve(options_.num_map_depth);
  const int max_width = 1 << (options_.num_map_depth - 1);
  const common::CellLimits limits(ptr_occupancy_map->getSizeX(),
                                  ptr_occupancy_map->getSizeY());

  std::vector<float> reusable_intermediate_grid;
  reusable_intermediate_grid.reserve((limits.num_x_cells + max_width - 1) *
                                     limits.num_y_cells);
  for (int i = 0; i != options_.num_map_depth; i++) {
    const int width = 1 << i;
    precomputation_Occupancy_grids_.emplace_back(
        ptr_occupancy_map_, limits, width, &reusable_intermediate_grid);
  }

  std::cout << "Multi resolution map has already generated. Map depth is "
            << options_.num_map_depth << ". Max width is "
            << (1 << options_.num_map_depth) << std::endl;
}

bool GlobalScanMatcher2D::scanMatchInFullMap(
    const slam2d_core::common::PointCloud &pointcloud,
    common::Rigid2 *pose_estimate) {
  LOG(INFO) << "Scan match in FullMap Start!" << std::endl;
  //计算搜索参数
  GlobalSearchParameters search_parameters;
  search_parameters.resolution = ptr_occupancy_map_->getResolution();
  search_parameters.angular_perturbation_step_size =
      options_.search_angle_resolution;
  search_parameters.num_angular_perturbations =
      M_PI / options_.search_angle_resolution;

  LOG(INFO) << "angle resolution is "
            << options_.search_angle_resolution * 180.0 / M_PI << " degrees";

  search_parameters.num_scans =
      2 * search_parameters.num_angular_perturbations + 1;

  const int num_linear_perturbations = 1e6;
  search_parameters.linear_bounds.reserve(search_parameters.num_scans);
  for (int i = 0; i != search_parameters.num_scans; ++i) {
    search_parameters.linear_bounds.push_back(
        GlobalSearchParameters::LinearBounds{
            -num_linear_perturbations, num_linear_perturbations,
            -num_linear_perturbations, num_linear_perturbations});
  }
  //计算地图中心点的相对于地图左下角远点的坐标
  const common::Rigid2 center = common::Rigid2::Translation(
      0.5 * ptr_occupancy_map_->getResolution() *
      Eigen::Vector2d(ptr_occupancy_map_->getSizeX(),
                      ptr_occupancy_map_->getSizeY()));

  if (matchWithSearchParameters(search_parameters, center, pointcloud,
                                pose_estimate)) {
    //将位姿转移到地图坐标系下
    common::Rigid2 map_orgin_pose =
        common::Rigid2(Eigen::Vector2d(ptr_occupancy_map_->getOriginX(),
                                       ptr_occupancy_map_->getOriginY()),
                       ptr_occupancy_map_->getOriginYaw());
    //将搜索位置转换到世界坐标系下
    *pose_estimate = map_orgin_pose * (*pose_estimate);
    LOG(INFO) << "Scan match in full Map Successed!,Scan pose("
              << pose_estimate->translation().x() << ","
              << pose_estimate->translation().y() << ","
              << pose_estimate->rotation().angle() << ")";
    return true;
  } else {
    LOG(WARNING) << "Scan match in full Map failed!";
    return false;
  }
  return false;
}

bool GlobalScanMatcher2D::scanMatchNearPose(
    const slam2d_core::common::PointCloud &pointcloud,
    common::Rigid2 *pose_estimate, const common::Rigid2 &init_pose) {
  LOG(INFO) << "Scan match in Windows Start!" << std::endl;
  LOG(INFO) << "Scan init_pose:(" << init_pose.translation().x() << ","
            << init_pose.translation().y() << ","
            << init_pose.rotation().angle() << ")";
  LOG(INFO) << "Windows width:(" << options_.search_windows_x_width << ","
            << options_.search_windows_y_width << ")";
  LOG(INFO) << "angle resolution is "
            << options_.search_angle_resolution * 180.0 / M_PI << " degrees";
  //计算搜索参数
  GlobalSearchParameters search_parameters;
  search_parameters.resolution = ptr_occupancy_map_->getResolution();
  search_parameters.angular_perturbation_step_size =
      options_.search_angle_resolution;
  search_parameters.num_angular_perturbations =
      std::round(M_PI / options_.search_angle_resolution /
                 (180.0 / options_.search_windows_angle));

  search_parameters.num_scans =
      2 * search_parameters.num_angular_perturbations + 1;

  const int num_x_linear_perturbations =
      options_.search_windows_x_width / search_parameters.resolution;
  const int num_y_linear_perturbations =
      options_.search_windows_y_width / search_parameters.resolution;

  search_parameters.linear_bounds.reserve(search_parameters.num_scans);
  for (int i = 0; i != search_parameters.num_scans; ++i) {
    search_parameters.linear_bounds.push_back(
        GlobalSearchParameters::LinearBounds{
            -num_x_linear_perturbations, num_x_linear_perturbations,
            -num_y_linear_perturbations, num_y_linear_perturbations});
  }
  //计算地图左下角在世界坐标系下的坐标
  common::Rigid2 map_orgin_pose =
      common::Rigid2(Eigen::Vector2d(ptr_occupancy_map_->getOriginX(),
                                     ptr_occupancy_map_->getOriginY()),
                     ptr_occupancy_map_->getOriginYaw());
  //计算初始点相对于地图左下点坐标
  const common::Rigid2 tran_init_pose = map_orgin_pose.inverse() * init_pose;

  if (matchWithSearchParameters(search_parameters, tran_init_pose, pointcloud,
                                pose_estimate)) {
    //将搜索位置转换到世界坐标系下
    *pose_estimate = map_orgin_pose * (*pose_estimate);
    LOG(INFO) << "Scan match in Windows Success!" << std::endl;
    return true;
  } else {
    LOG(WARNING) << "Scan match in Windows failed!" << std::endl;
    return false;
  }
}
//该函数所有计算均在相对于地图左下角的坐标系下进行的
bool GlobalScanMatcher2D::matchWithSearchParameters(
    GlobalSearchParameters search_parameters,
    const common::Rigid2 &initial_pose_estimate,
    const common::PointCloud &point_cloud, common::Rigid2 *pose_estimate) {
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const common::PointCloud rotated_point_cloud = common::transformPointCloud(
      point_cloud,
      common::Rigid3::Rotation(Eigen::AngleAxisd(
          initial_rotation.cast<double>().angle(), Eigen::Vector3d::UnitZ())));
  //在产生所有的旋转点云
  std::vector<common::PointCloud> rotated_scans;
  rotated_scans.reserve(search_parameters.num_scans);
  double delta_theta = -search_parameters.num_angular_perturbations *
                       search_parameters.angular_perturbation_step_size;
  // std::cout << "delta_theta:" << delta_theta << std::endl;
  // for (size_t i = 0; i < point_cloud.size(); i++) {
  //   std::cout << "point :" << i << "," << point_cloud[i].position.x() << ","
  //             << point_cloud[i].position.y() << std::endl;
  // }
  for (int scan_index = 0; scan_index < search_parameters.num_scans;
       ++scan_index,
           delta_theta += search_parameters.angular_perturbation_step_size) {
    rotated_scans.push_back(common::transformPointCloud(
        rotated_point_cloud, common::Rigid3::Rotation(Eigen::AngleAxisd(
                                 delta_theta, Eigen::Vector3d::UnitZ()))));
  }
  //将点云转成栅格化点云
  std::vector<std::vector<Eigen::Array2i>> discrete_scans;
  int cnt = 1;
  discrete_scans.reserve(rotated_scans.size());
  for (const common::PointCloud &scan : rotated_scans) {
    discrete_scans.emplace_back();
    discrete_scans.back().reserve(scan.size());

    for (const common::RangePoint &point : scan) {
      cnt++;
      const Eigen::Vector2d translated_point =
          Eigen::Affine2d(
              Eigen::Translation2d(initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y())) *
          point.position.head<2>();
      // if (cnt < 20) {
      //   std::cout << "estimatepose:(" << point.position.x() << ","
      //             << point.position.y() << "," << point.position.z() << ")"
      //             << std::endl;
      //   std::cout << "translated_point:(" << translated_point.x() << ","
      //             << translated_point.y() << ")" << std::endl;
      // }
      discrete_scans.back().push_back(Eigen::Array2i(
          common::roundToInt(
              translated_point.x() / search_parameters.resolution - 0.5),
          common::roundToInt(
              translated_point.y() / search_parameters.resolution - 0.5)));
    }
  }

  //自适应调整搜索框大小
  search_parameters.shrinkToFit(discrete_scans, ptr_occupancy_map_->getSizeX(),
                                ptr_occupancy_map_->getSizeY());
  //生成所有的最低分辨率下的candicated
  std::vector<GlobalCandidate2D> lowest_resolution_candidates;
  const int linear_step_size = 1 << (options_.num_map_depth - 1);
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_lowest_resolution_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + linear_step_size) /
        linear_step_size;
    const int num_lowest_resolution_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + linear_step_size) /
        linear_step_size;
    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }
  lowest_resolution_candidates.reserve(num_candidates);

  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           y_index_offset += linear_step_size) {
        lowest_resolution_candidates.emplace_back(
            scan_index, x_index_offset, y_index_offset, search_parameters);
      }
    }
  }
  //评价低分辨率地图下所有候选者得分
  scoreCandidates(precomputation_Occupancy_grids_[options_.num_map_depth - 1],
                  discrete_scans, &lowest_resolution_candidates);

  //进行分枝定界搜索
  const GlobalCandidate2D best_candidate = branchAndBound(
      discrete_scans, search_parameters, lowest_resolution_candidates,
      options_.num_map_depth - 1, options_.min_score);
  LOG(INFO) << "Best_candidata score:" << best_candidate.score
            << " score threshold is" << options_.min_score;
  if (best_candidate.score > options_.min_score) {
    //将位姿态转换到地图坐标系下（以地图左下角为原点）
    *pose_estimate = common::Rigid2(
        {initial_pose_estimate.translation().x() + best_candidate.x,
         initial_pose_estimate.translation().y() + best_candidate.y},
        initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
    LOG(INFO) << "Best_candidate: scanIndex" << best_candidate.scan_index << "("
              << best_candidate.x_index_offset << ","
              << best_candidate.y_index_offset << ","
              << best_candidate.orientation << ")";
    LOG(INFO) << "estimatepose_inmap:(" << pose_estimate->translation().x()
              << "," << pose_estimate->translation().y() << ","
              << pose_estimate->rotation().angle() << ")";
    return true;
  }
  return false;
}

void GlobalScanMatcher2D::scoreCandidates(
    const PrecomputeOccupancyGrid2D &precomputation_grid,
    const std::vector<DiscreteScan2D> &discrete_scans,
    std::vector<GlobalCandidate2D> *const candidates) const {
  for (GlobalCandidate2D &candidate : *candidates) {
    int sum = 0;
    for (const Eigen::Array2i &xy_index :
         discrete_scans[candidate.scan_index]) {
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
      sum += precomputation_grid.getValue(proposed_xy_index);
    }
    candidate.score = precomputation_grid.toScore(
        sum / static_cast<float>(discrete_scans[candidate.scan_index].size()));
  }
  std::sort(candidates->begin(), candidates->end(),
            std::greater<GlobalCandidate2D>());
}

GlobalCandidate2D GlobalScanMatcher2D::branchAndBound(
    const std::vector<DiscreteScan2D> &discrete_scans,
    const GlobalSearchParameters &search_parameters,
    const std::vector<GlobalCandidate2D> &candidates, const int candidate_depth,
    float min_score) const {
  if (candidate_depth == 0) {
    // Return the best candidate.
    return *candidates.begin();
  }

  GlobalCandidate2D best_high_resolution_candidate(0, 0, 0, search_parameters);

  best_high_resolution_candidate.score = min_score;
  for (const GlobalCandidate2D &candidate : candidates) {
    if (candidate.score <= best_high_resolution_candidate.score) {
      break;
    }
    std::vector<GlobalCandidate2D> higher_resolution_candidates;
    const int half_width = 1 << (candidate_depth - 1);
    for (int x_offset : {0, half_width}) {
      if (candidate.x_index_offset + x_offset >
          search_parameters.linear_bounds[candidate.scan_index].max_x) {
        break;
      }
      for (int y_offset : {0, half_width}) {
        if (candidate.y_index_offset + y_offset >
            search_parameters.linear_bounds[candidate.scan_index].max_y) {
          break;
        }
        higher_resolution_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }
    scoreCandidates(precomputation_Occupancy_grids_[candidate_depth - 1],
                    discrete_scans, &higher_resolution_candidates);
    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        branchAndBound(discrete_scans, search_parameters,
                       higher_resolution_candidates, candidate_depth - 1,
                       best_high_resolution_candidate.score));
  }
  return best_high_resolution_candidate;
}

}  // namespace scan_matcher
}  // namespace slam2d_core
