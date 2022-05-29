#ifndef PROBABILITY_VOXEL_LAYER_HPP
#define PROBABILITY_VOXEL_LAYER_HPP
#include <memory>
#include <mutex>

#include "obstacle_layer.hpp"
#include "probability_voxel_grid.hpp"
namespace CVTE_BABOT {
class ProbabilityVoxelLayer : public ObstacleLayer {
 public:
  ProbabilityVoxelLayer() = default;
  ~ProbabilityVoxelLayer() = default;

  ProbabilityVoxelLayer(const ProbabilityVoxelLayer &obj) = delete;
  ProbabilityVoxelLayer &operator=(const ProbabilityVoxelLayer &obj) = delete;

  void activate() override;
  void deactivate() override;
  void reset() override;

  bool onInitialize() override;
  void getParams() override;
  bool updateBounds(const WorldmapPose &wp_robot_pose,
                    CostmapBound &cb_costmap_bound) override;
  void updateOrigin(const double &d_new_origin_x,
                    const double &d_new_origin_y) override;
  void matchSize() override;

 protected:
  /*
   *@brief 利用体素尺寸对点云进行降采样
   */
  bool downSampleByGrid(const std::shared_ptr<const CostmapCloud> &raw_cloud,
                        std::shared_ptr<CostmapCloud> &tar_cloud) const;

  /*
   * @brief 获取"标记点云": 先进行降采样，再进行转换，减少获取点云的资源消耗
   */
  virtual bool getMarkingClouds(std::vector<std::shared_ptr<const CostmapCloud>>
                                    &marking_clouds) const override;

  /*
   * @brief 获取"清除点云": 先进行降采样，再进行转换，减少获取点云的资源消耗
   */
  virtual bool getClearingClouds(
      std::vector<std::shared_ptr<const CostmapCloud>> &clearing_clouds)
      const override;

 private:
  /*
   * todo: 添加注释
   */
  void clearFrustums(
      const std::vector<std::shared_ptr<const CostmapCloud>> &clearing_cloud);
  /*
   * todo: 添加注释
   */
  void markVoxelMap(
      const std::vector<std::shared_ptr<const CostmapCloud>> &mark_cloud,
      const time_t &mark_time);
  void clearByTime(const time_t &now_time,
                   const double &clearing_time_threshold);
  void updateCostsMap();
  void resetMaps() override;

  std::shared_ptr<ProbabilityVoxelGrid> ptr_grid_ = nullptr;

  double resolution_z_;
  double origin_z_;
  int unknown_threshold_;
  int mark_threshold_;
  int size_z_;
  double clear_time_threshold_;

  std::recursive_mutex mutex_;
};
}  // namespace CVTE_BABOT
#endif