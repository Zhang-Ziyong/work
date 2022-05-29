/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file min_box_object_builder.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-05-25
 ************************************************************************/
#ifndef __MIN_BOX_OBJECT_BUILDER_HPP
#define __MIN_BOX_OBJECT_BUILDER_HPP
#include "common/tracking_utils.hpp"

namespace CVTE_BABOT {
class MinBoxObjectBuilder {
 public:
  MinBoxObjectBuilder() = default;
  ~MinBoxObjectBuilder() = default;

  MinBoxObjectBuilder(const MinBoxObjectBuilder &) = delete;
  MinBoxObjectBuilder &operator=(const MinBoxObjectBuilder &) = delete;

  static void buildObjects(
      const std::vector<pcl::PointCloud<VPoint>::Ptr> &v_ptr_point_cloud,
      std::vector<ObjectPtr> &objects_obsved);

  static bool buildObject(const pcl::PointCloud<VPoint>::Ptr &ptr_point_cloud,
                          Object &object);
};
}  // namespace CVTE_BABOT
#endif
