#include "scan_matcher/fast_correlative_scan_matcher_2d.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>

#include "Eigen/Geometry"

#include "glog/logging.h"

namespace slam2d_core {
namespace scan_matcher {
namespace {

// 管理最大值
class SlidingWindowMaximum {
 public:
  // 将最大值放在最前面，并排序
  void addValue(const float value) {
    while (!non_ascending_maxima_.empty() &&
           value > non_ascending_maxima_.back()) {
      non_ascending_maxima_.pop_back();
    }
    non_ascending_maxima_.push_back(value);
  }

  // 判断是否清空最大值，如果该区域内的数据遍历完了，此时最大值会被清掉
  void removeValue(const float value) {
    DCHECK(!non_ascending_maxima_.empty());
    DCHECK_LE(value, non_ascending_maxima_.front());
    if (value == non_ascending_maxima_.front()) {
      non_ascending_maxima_.pop_front();
    }
  }

  // 获取最大值
  float getMaximum() const {
    DCHECK_GT(non_ascending_maxima_.size(), 0);
    return non_ascending_maxima_.front();
  }

  void checkIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

 private:
  std::deque<float> non_ascending_maxima_;
};

}  // namespace

PrecomputationGrid2D::PrecomputationGrid2D(
    const common::ProbabilityGrid &grid, const common::CellLimits &limits,
    const int width, std::vector<float> *reusable_intermediate_grid)
    // 从左下角xy方向分别扩展-width+1大小，所以offset的xy要分别减去（width - 1）
    : offset_(-width + 1, -width + 1),
      // 该分辨率地图的网格数量是比原始地图大，因为要采样一个width*width内的最大值，所以每个(x，y)处，
      // 都需要计算一个width*width内的最大值
      wide_limits_(limits.num_x_cells + width - 1,
                   limits.num_y_cells + width - 1),
      min_score_(1.f - grid.getMaxCorrespondenceCost()),
      max_score_(1.f - grid.getMinCorrespondenceCost()),
      cells_(wide_limits_.num_x_cells * wide_limits_.num_y_cells) {
  CHECK_GE(width, 1);
  CHECK_GE(limits.num_x_cells, 1);
  CHECK_GE(limits.num_y_cells, 1);
  const int stride = wide_limits_.num_x_cells;

  std::vector<float> &intermediate = *reusable_intermediate_grid;
  intermediate.resize(wide_limits_.num_x_cells * limits.num_y_cells);
  // 每一行x0处，都获取该列y的内的最大值
  for (int y = 0; y != limits.num_y_cells; ++y) {
    SlidingWindowMaximum current_values;
    // 在[-width+1, 0]区间内，current_values的最大值的初值要取x = 0处的值
    current_values.addValue(
        1.f - std::abs(grid.getCorrespondenceCost(Eigen::Array2i(0, y))));
    // 获取[-width+1, 0]内width宽度内的最大值一直放入current_values里
    for (int x = -width + 1; x != 0; ++x) {
      // 每个(x + width - 1 + y * stride)处的值是从[-width + 1]开始的最大值
      intermediate[x + width - 1 + y * stride] = current_values.getMaximum();
      if (x + width < limits.num_x_cells) {
        current_values.addValue(1.f - std::abs(grid.getCorrespondenceCost(
                                          Eigen::Array2i(x + width, y))));
      }
    }
    //  求x=(x,x+width)区间的最大值
    for (int x = 0; x < limits.num_x_cells - width; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.getMaximum();
      current_values.removeValue(
          1.f - std::abs(grid.getCorrespondenceCost(Eigen::Array2i(x, y))));
      current_values.addValue(1.f - std::abs(grid.getCorrespondenceCost(
                                        Eigen::Array2i(x + width, y))));
    }
    // 求x=(width,limits.num_x_cells)区间的最大值
    for (int x = std::max(limits.num_x_cells - width, 0);
         x != limits.num_x_cells; ++x) {
      intermediate[x + width - 1 + y * stride] = current_values.getMaximum();
      current_values.removeValue(
          1.f - std::abs(grid.getCorrespondenceCost(Eigen::Array2i(x, y))));
    }
    current_values.checkIsEmpty();
  }

  // 再遍历x行，获取y方向上，width范围内的最大值，因此，在cell_里在(x+y*width)处存放的都是width*width范围内的最大值
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

uint8_t PrecomputationGrid2D::computeCellValue(const float probability) const {
  const int cell_value = std::lround((probability - min_score_) *
                                     (255.f / (max_score_ - min_score_)));
  CHECK_GE(cell_value, 0);
  CHECK_LE(cell_value, 255);
  return cell_value;
}

PrecomputationGridStack2D::PrecomputationGridStack2D(
    const common::ProbabilityGrid &grid,
    const FastCorrelativeScanMatcherOptions2D &options) {
  CHECK_GE(options.branch_and_bound_depth, 1);
  const int max_width = 1 << (options.branch_and_bound_depth - 1);
  precomputation_grids_.reserve(options.branch_and_bound_depth);
  std::vector<float> reusable_intermediate_grid;
  const common::CellLimits limits = grid.limits().cell_limits();
  reusable_intermediate_grid.reserve((limits.num_x_cells + max_width - 1) *
                                     limits.num_y_cells);
  for (int i = 0; i != options.branch_and_bound_depth; ++i) {
    const int width = 1 << i;
    precomputation_grids_.emplace_back(grid, limits, width,
                                       &reusable_intermediate_grid);
  }
}

FastCorrelativeScanMatcher2D::FastCorrelativeScanMatcher2D(
    const common::ProbabilityGrid &grid,
    const FastCorrelativeScanMatcherOptions2D &options)
    : options_(options),
      limits_(grid.limits()),
      precomputation_grid_stack_(
          std::make_unique<PrecomputationGridStack2D>(grid, options)) {}

FastCorrelativeScanMatcher2D::~FastCorrelativeScanMatcher2D() {}

bool FastCorrelativeScanMatcher2D::match(
    const common::Rigid2 &initial_pose_estimate,
    const common::PointCloud &point_cloud, const float min_score, float *score,
    common::Rigid2 *pose_estimate) const {
  const SearchParameters search_parameters(options_.linear_search_window,
                                           options_.angular_search_window,
                                           point_cloud, limits_.resolution());

  return matchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   point_cloud, min_score, score,
                                   pose_estimate);
}

bool FastCorrelativeScanMatcher2D::match(
    const common::Rigid2 &initial_pose_estimate,
    const common::PointCloud &point_cloud, const double &linear_search_window,
    const float min_score, float *score, common::Rigid2 *pose_estimate) const {
  const SearchParameters search_parameters(linear_search_window,
                                           options_.angular_search_window,
                                           point_cloud, limits_.resolution());

  return matchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   point_cloud, min_score, score,
                                   pose_estimate);
}

bool FastCorrelativeScanMatcher2D::matchFullSubmap(
    const common::PointCloud &point_cloud, float min_score, float *score,
    common::Rigid2 *pose_estimate) const {
  // 和整个submap匹配，为了不放过所有的可能，所以，线搜索和角度搜索都需要设计的很大，且搜索的中心位置设定在整个submap的中心
  const SearchParameters search_parameters(
      1e6 * limits_.resolution(),  // Linear search window, 1e6 cells/direction.
      M_PI,  // Angular search window, 180 degrees in both directions.
      point_cloud, limits_.resolution());
  const common::Rigid2 center = common::Rigid2::Translation(
      limits_.max() - 0.5 * limits_.resolution() *
                          Eigen::Vector2d(limits_.cell_limits().num_y_cells,
                                          limits_.cell_limits().num_x_cells));
  return matchWithSearchParameters(search_parameters, center, point_cloud,
                                   min_score, score, pose_estimate);
}

bool FastCorrelativeScanMatcher2D::matchWithSearchParameters(
    SearchParameters search_parameters,
    const common::Rigid2 &initial_pose_estimate,
    const common::PointCloud &point_cloud, float min_score, float *score,
    common::Rigid2 *pose_estimate) const {
  CHECK(score != nullptr);
  CHECK(pose_estimate != nullptr);

  // 将初始点云调整角度，调整到submap坐标系下
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const common::PointCloud rotated_point_cloud = common::transformPointCloud(
      point_cloud,
      common::Rigid3::Rotation(Eigen::AngleAxisd(
          initial_rotation.cast<double>().angle(), Eigen::Vector3d::UnitZ())));
  // 根据角度旋转生成一系列的PointCloud，虽然都在世界坐标系下，但是原点和世界坐标系不重合，该处的处理方式和前端的暴力匹配一样，不再阐述
  const std::vector<common::PointCloud> rotated_scans =
      generateRotatedScans(rotated_point_cloud, search_parameters);
  // 旋转后的所有PointCloud加上平移，同一转换到世界坐标系下，并计算在submap坐标系下的xy坐标，
  // 存放在DiscreteScan2D里（一个可能位姿，对应着一个点云数据），该处的处理方式和前端的暴力匹配一样，不再阐述
  const std::vector<DiscreteScan2D> discrete_scans = discretizeScans(
      limits_, rotated_scans,
      Eigen::Translation2d(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  // 将超过边界的点的坐标计算出来，并更新search_parameters里的边界
  search_parameters.shrinkToFit(discrete_scans, limits_.cell_limits());

  // 生成低分辨率地图的候选解，并且和最低分辨率的地图进行匹配，并且得到所有激光帧和候选解的得分
  const std::vector<Candidate2D> lowest_resolution_candidates =
      computeLowestResolutionCandidates(discrete_scans, search_parameters);
  // 分支定界搜索最好的可能解（多个分辨率搜索）
  const Candidate2D best_candidate = branchAndBound(
      discrete_scans, search_parameters, lowest_resolution_candidates,
      precomputation_grid_stack_->max_depth(), min_score);

  *score = best_candidate.score;
  // 最好的解的得分满足阈值才认为分支定界成功，产生回环
  if (best_candidate.score > min_score) {
    *pose_estimate = common::Rigid2(
        {initial_pose_estimate.translation().x() + best_candidate.x,
         initial_pose_estimate.translation().y() + best_candidate.y},
        initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
    return true;
  }
  return false;
}

std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::computeLowestResolutionCandidates(
    const std::vector<DiscreteScan2D> &discrete_scans,
    const SearchParameters &search_parameters) const {
  // 生成低分辨率的可能解
  std::vector<Candidate2D> lowest_resolution_candidates =
      generateLowestResolutionCandidates(search_parameters);
  // 对低分辨率的候选解打分
  scoreCandidates(
      precomputation_grid_stack_->get(precomputation_grid_stack_->max_depth()),
      discrete_scans, search_parameters, &lowest_resolution_candidates);
  return lowest_resolution_candidates;
}

// 生成低分辨率的候选解，候选解中包括scan的id，offset，和初始位姿的相对位姿，得分情况
std::vector<Candidate2D>
FastCorrelativeScanMatcher2D::generateLowestResolutionCandidates(
    const SearchParameters &search_parameters) const {
  const int linear_step_size = 1 << precomputation_grid_stack_->max_depth();
  int num_candidates = 0;
  // 遍历所有的帧，计算所有可行解的数量
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
  // 把所有的可行解放入candidates中
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           y_index_offset += linear_step_size) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

void FastCorrelativeScanMatcher2D::scoreCandidates(
    const PrecomputationGrid2D &precomputation_grid,
    const std::vector<DiscreteScan2D> &discrete_scans,
    const SearchParameters &search_parameters,
    std::vector<Candidate2D> *const candidates) const {
  (void) search_parameters;
  // 遍历所有可能解，并得到可能解的得分
  for (Candidate2D &candidate : *candidates) {
    int sum = 0;
    // 针对当前解，遍历所有的点云，如果点云和submap匹配的越好，得分肯定越高
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
  // 分数从高到底排序
  std::sort(candidates->begin(), candidates->end(),
            std::greater<Candidate2D>());
}

// 分支定界：递归的方法
Candidate2D FastCorrelativeScanMatcher2D::branchAndBound(
    const std::vector<DiscreteScan2D> &discrete_scans,
    const SearchParameters &search_parameters,
    const std::vector<Candidate2D> &candidates, const int candidate_depth,
    float min_score) const {
  // 结束条件：叶节点，不能再向下搜索，此时返回排序后candidates的第一个得分最高的解
  if (candidate_depth == 0) {
    // Return the best candidate.
    return *candidates.begin();
  }

  // 构造Candidate2D，
  // 参数分别为：scan_id（该id包含角度信息，参考前端的解释）；x方向offset（相对于初始位姿的x方向平移）;y方向Offset（相对于初始位姿的y方向平移）；
  // 搜索参数（参考Candidate2D,主要是用里面的resolution以计算真实xy做得分、num_angular_perturbations和angular_perturbation_step_size以计算相对于初始位姿的角度偏移)
  Candidate2D best_high_resolution_candidate(0, 0, 0, search_parameters);
  // 得分赋初值，初值必须赋值为设定的阈值
  best_high_resolution_candidate.score = min_score;
  for (const Candidate2D &candidate : candidates) {
    // 如果当前根节点的得分低，则没必要再往下搜索
    if (candidate.score <= min_score) {
      break;
    }
    // 存放该根节点的子节点：子节点共四个
    std::vector<Candidate2D> higher_resolution_candidates;
    // 步长减半，进行分支操作（因为网格采样的时候分辨率是除以2的），所有从粗糙层到下一层，其子节点应该有4个：
    // 当前的offset,(0,0),(offset/2,0),(0,offset/2),(offset/2,offset/2)，若超出边界，则不需要添加该子节点
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
        // 分支操作，将四个可能解放到新的vector<Candidate2D>中，作为下次递归的参数
        higher_resolution_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }

    // 对当前的四个子节点打分，并从得分高到得分低排序
    scoreCandidates(precomputation_grid_stack_->get(candidate_depth - 1),
                    discrete_scans, search_parameters,
                    &higher_resolution_candidates);
    // 递归，对得分最高的节点继续分支操作（因为该处的值赋值为得分最高的可能解的分），直到最底层，返回一个得分最高的叶节点，
    // 此时，得分最高的节点的得分肯定高于设定的阈值，如果没有可能解满足>min_score的要求，则返回值的分数为min_score，不满足分支定界的要求
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
