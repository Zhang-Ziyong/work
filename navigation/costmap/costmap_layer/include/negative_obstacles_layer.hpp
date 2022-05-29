#ifndef NEGATIVE_OBSTACLE_LAYER_HPP
#define NEGATIVE_OBSTACLE_LAYER_HPP
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <memory>
#include <unordered_set>

#include "obstacle_layer.hpp"

namespace CVTE_BABOT {
class NegativeObstaclesLayer : public ObstacleLayer {
 public:
  NegativeObstaclesLayer() = default;
  ~NegativeObstaclesLayer() = default;

  NegativeObstaclesLayer(const NegativeObstaclesLayer& obj) = delete;
  NegativeObstaclesLayer& operator=(const NegativeObstaclesLayer& obj) = delete;
  void activate() override;
  void deactivate() override;
  void reset() override;

  bool onInitialize() override;
  void getParams() override;
  bool updateBounds(const WorldmapPose& wp_robot_pose,
                    CostmapBound& cb_costmap_bound) override;

  void matchSize() override;

 protected:
  /*
   * @brief 获取"标记点云": 先进行降采样，再进行转换，减少获取点云的资源消耗
   */
  virtual bool getMarkingClouds(
      std::vector<std::shared_ptr<const CostmapCloud>>& marking_clouds)
      const override;

  /*
   * @brief 获取"清除点云": 先进行降采样，再进行转换，减少获取点云的资源消耗
   */
  virtual bool getClearingClouds(
      std::vector<std::shared_ptr<const CostmapCloud>>& clearing_clouds)
      const override;
  void markNegativeObstacles(
      const std::vector<std::shared_ptr<const CostmapCloud>>& marking_clouds);

  bool clearNegativeObstacles(
      const std::vector<std::shared_ptr<const CostmapCloud>>& clear_clouds);

 private:
  void resetMaps() override;

  bool isPointInPolygon(const int x, const int y,
                        const std::vector<std::vector<float>>& points);

  bool getClearAreaInBase(const std::shared_ptr<const CostmapCloud>& cloud,
                          std::vector<Eigen::Vector4f>& clear_area);

  bool getLineIntersectWhitGround(const Eigen::Vector4f& p1,
                                  const Eigen::Vector4f& p2,
                                  Eigen::Vector4f& intersect_point);

  std::map<std::string, std::vector<Eigen::Vector4f>> chear_areas_;
  std::unordered_set<size_t> has_update_hash_;
};
}  // namespace CVTE_BABOT

#endif