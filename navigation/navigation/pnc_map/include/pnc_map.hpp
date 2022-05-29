/**
 * @file pnc_map.hpp
 * @author linyanlong(linyanlong@cvte.com)
 * @brief
 * @version 0.1
 * @date 2022-01-05
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef PNC_MAP_HPP_
#define PNC_MAP_HPP_
#include "hdmap.hpp"
#include "path.hpp"
namespace CVTE_BABOT {
class PncMap {
 public:
  PncMap() {
    ptr_subpath_ = nullptr;
    init_subpath_ = false;
    ptr_cvte_hdmap_ = cvte::hdmap::CvteHdMap::getInstance();
  }
  /**
   * @brief
   *
   */
  void clear() {
    ptr_subpath_ = nullptr;
    init_subpath_ = false;
  }

  bool InitSubPath() { return init_subpath_; }

  void SetPath(std::shared_ptr<SubPath> ptr_subpath) {
    clear();
    if (ptr_subpath == nullptr) {
      return;
    }
    init_subpath_ = true;
    ptr_subpath_ = ptr_subpath;
  }
  /**
   * @brief
   *
   * @param point
   * @return true
   * @return false
   */
  bool InPitArea(const math_utils::Vec2d &point) const;
  /**
   * @brief
   *
   * @param point
   * @return true
   * @return false
   */
  bool InSlopeArea(const math_utils::Vec2d &point) const;
  /**
   * @brief
   *
   * @param point
   * @return true
   * @return false
   */
  bool InElevatorArea(const math_utils::Vec2d &point) const;
  /**
   * @brief
   *
   * @param point
   * @return true
   * @return false
   */
  bool InNarrowArea(const math_utils::Vec2d &point) const;
  /**
   * @brief
   *
   * @param point
   * @return true
   * @return false
   */
  // bool InCarpetArea(const math_utils::Vec2d &point) const;
  /**
   * @brief
   *
   * @param point
   * @return true
   * @return false
   */
  bool InCleanArea(const math_utils::Vec2d &point) const;
  /**
   * @brief
   *
   * @param point
   * @return true
   * @return false
   */
  bool InProhibitedArea(const math_utils::Vec2d &point) const;
  /**
   * @brief
   *
   * @param point
   * @return true
   * @return false
   */
  bool InBlackArea(const math_utils::Vec2d &point) const;
  /**
   * @brief
   *
   * @param path_index
   * @param distance
   * @return true
   * @return false
   */
  bool ForwardInSlopeArea(int path_index, double distance) const;
  /**
   * @brief
   *
   * @param path_index
   * @param distance
   * @return true
   * @return false
   */
  bool ForwardInPitArea(int path_index, double distance) const;
  /**
   * @brief
   *
   * @param path_index
   * @param distance
   * @return true
   * @return false
   */
  bool BackwardInPitArea(int path_index, double distance) const;
  /**
   * @brief
   *
   * @param path_index
   * @param distance
   * @return true
   * @return false
   */
  bool ForwardInElevatorArea(int path_index, double distance) const;
  /**
   * @brief
   *
   * @param path_index
   * @param distance
   * @return true
   * @return false
   */
  bool ForwardInNarrowArea(int path_index, double distance) const;
  /**
   * @brief
   *
   * @param path_index
   * @param distance
   * @return true
   * @return false
   */
  // bool ForwardInCarpetArea(int path_index, double distance) const;
  /**
   * @brief
   *
   * @param path_index
   * @param distance
   * @return true
   * @return false
   */
  bool ForwardInCleanArea(int path_index, double distance) const;
  /**
   * @brief
   *
   * @param path_index
   * @param distance
   * @return true
   * @return false
   */
  bool ForwardInProhibitedArea(int path_index, double distance) const;
  /**
   * @brief
   *
   * @param path_index
   * @param distance
   * @return true
   * @return false
   */
  bool ForwardInBlackArea(int path_index, double distance) const;

  std::vector<cvte::hdmap::Polygon2d> GetClearArea() const;

  std::vector<cvte::hdmap::Polygon2d> GetCleanArea() const;

  // std::vector<cvte::hdmap::Polygon2d> GetCarpetArea() const;

  std::vector<cvte::hdmap::Polygon2d> GetPitArea() const;

  std::vector<cvte::hdmap::Polygon2d> GetSlopeArea() const;

  std::vector<cvte::hdmap::Polygon2d> GetElevatorArea() const;

  std::vector<cvte::hdmap::Polygon2d> GetProhibitedArea() const;

  std::vector<cvte::hdmap::Polygon2d> GetNarrowArea() const;

  bool GetPitBox(const math_utils::Vec2d &point, cvte::hdmap::Box2d &box) const;

 private:
  int getForwardPathIndex(int path_index, double distance) const;

 private:
  std::shared_ptr<cvte::hdmap::CvteHdMap> ptr_cvte_hdmap_;
  std::shared_ptr<SubPath> ptr_subpath_;
  bool init_subpath_;
};
}  // namespace CVTE_BABOT
#endif