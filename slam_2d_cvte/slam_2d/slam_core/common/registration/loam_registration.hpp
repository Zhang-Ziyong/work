/*
 * @Author: your name
 * @Date: 2020-08-14 15:32:58
 * @LastEditTime: 2020-08-14 18:10:26
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /new_cvte_lidar_slam/slam_core/common/registration/loam_registration.hpp
 */
#ifndef LOAM_REGISTRATION_HPP_
#define LOAM_REGISTRATION_HPP_
#include "common/data_struct/pc_base.hpp"
#include "common/math_base/slam_math.hpp"

namespace cvte_lidar_slam{

class LoamRegistration {
public:
    LoamRegistration();
    LoamRegistration(const LoamRegistration& obj) = delete;
    LoamRegistration& operator=(const LoamRegistration& obj) = delete;

    bool loam_registration(const laserCloud::Ptr cur_corner_cloud,const laserCloud::Ptr cur_surf_cloud,
        const laserCloud::Ptr map_corner_cloud,const laserCloud::Ptr map_surf_cloud,Mat34d &cur_pose);

private:

    bool scanToLocalMap();

    void resetVariable();

    void findCorrespondingFeature();

    laserCloud::Ptr ds_map_corner_cloud_;
    laserCloud::Ptr ds_map_surf_cloud_;
    laserCloud::Ptr ds_cur_corner_cloud_;
    laserCloud::Ptr ds_cur_surf_cloud_;

    pclDownsampler downsize_filter_map_corner_; ///< 下采样当前帧corner点云
    pclDownsampler downsize_filter_map_surf_;  ///< 下采样当前帧surf点云
    pclDownsampler downsize_filter_cur_corner_;///< 下采样局部地图corner点云
    pclDownsampler downsize_filter_cur_surf_;///< 下采样局部地图surf点云

    pclKdTree::Ptr ptr_kdtree_corner_from_map_;
    pclKdTree::Ptr ptr_kdtree_surf_from_map_;

    Mat34d keyframe_pose_;

};

}

#endif