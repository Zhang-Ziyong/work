/*
 * @Author: your name
 * @Date: 2020-08-14 15:48:10
 * @LastEditTime: 2020-08-17 20:46:23
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /new_cvte_lidar_slam/slam_core/common/registration/loam_registration.cpp
 */
#include "loam_registration.hpp"
#include "msf/pose_graph_error_term.hpp"
#include "common/math_base/slam_transform.hpp"
#include "glog/logging.h"
namespace cvte_lidar_slam {
LoamRegistration::LoamRegistration() {
    ds_map_corner_cloud_.reset(new laserCloud());
    ds_map_surf_cloud_.reset(new laserCloud());
    ds_cur_corner_cloud_.reset(new laserCloud());
    ds_cur_surf_cloud_.reset(new laserCloud());
    downsize_filter_map_corner_.setLeafSize(0.2,0.2,0.2);
    downsize_filter_map_surf_.setLeafSize(0.4,0.4,0.4);
    downsize_filter_cur_corner_.setLeafSize(0.2,0.2,0.2);
    downsize_filter_cur_surf_.setLeafSize(0.4,0.4,0.4);
    ptr_kdtree_corner_from_map_.reset(new pclKdTree());
    ptr_kdtree_surf_from_map_.reset(new pclKdTree());
    resetVariable();
}

void LoamRegistration::resetVariable() {
    ds_map_corner_cloud_->clear();
    ds_map_surf_cloud_->clear();
    ds_cur_corner_cloud_->clear();
    ds_cur_surf_cloud_->clear();
}

bool LoamRegistration::loam_registration(const laserCloud::Ptr cur_corner_cloud,const laserCloud::Ptr cur_surf_cloud,
    const laserCloud::Ptr map_corner_cloud,const laserCloud::Ptr map_surf_cloud,Mat34d &cur_pose) {
    if(cur_corner_cloud == nullptr || cur_surf_cloud == nullptr || map_corner_cloud == nullptr || map_surf_cloud == nullptr) {
        LOG(ERROR) << "input cloud pointer is nullptr";
        return false;
    }
    downsize_filter_map_corner_.setInputCloud(map_corner_cloud);
    downsize_filter_map_corner_.filter(*ds_map_corner_cloud_);
    downsize_filter_map_surf_.setInputCloud(map_surf_cloud);
    downsize_filter_map_surf_.filter(*ds_map_surf_cloud_);
    downsize_filter_cur_corner_.setInputCloud(cur_corner_cloud);
    downsize_filter_cur_corner_.filter(*ds_cur_corner_cloud_);
    downsize_filter_cur_surf_.setInputCloud(cur_surf_cloud);
    downsize_filter_cur_surf_.filter(*ds_cur_surf_cloud_);
    ptr_kdtree_corner_from_map_->setInputCloud(ds_map_corner_cloud_);
    ptr_kdtree_surf_from_map_->setInputCloud(ds_map_surf_cloud_);
    keyframe_pose_ = cur_pose;
    bool ret = scanToLocalMap();
    cur_pose = keyframe_pose_;
    return ret;
}

bool LoamRegistration::scanToLocalMap() {
    if( ds_map_surf_cloud_->size() > 100){
        bool is_match_ok_ = true;
        for(int iter_count = 0;iter_count < 10;iter_count++) {
            Mat34d last_temp_pose = keyframe_pose_;
            findCorrespondingFeature();
            Mat34d delta_pose = Mathbox::deltaPose34d(last_temp_pose,keyframe_pose_);
            double delta_rotation = (Mathbox::rotationMatrixToEulerAngles(delta_pose.block<3,3>(0,0))).norm();
            double delta_trans = delta_pose.block<3,1>(0,3).norm();
            // LOG(INFO) << "iter time:  " << iter_count <<  "  rot:  " << delta_rotation << "    trans:   " << delta_trans;
            if(delta_trans < 0.01 && delta_rotation < 0.01){
                break;
            }else if(9 == iter_count){
                is_match_ok_ = false;
                LOG(ERROR) << "match failed" << std::endl;
            }
        }
        if(is_match_ok_){
            return true;
        }else{
            return false;
        }
    }else{
        LOG(ERROR) << "the size of map point cloud is too small";
        return false;
    }
}

void LoamRegistration::findCorrespondingFeature() {
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.2);
    ceres::LocalParameterization * pose_parameterization = new PosePara();
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);
    problem.AddParameterBlock(keyframe_pose_.data(),12,pose_parameterization);
    PointType point_ori,point_sel;
    float error_sum = 0.0;
    size_t valid_edge_point = 0;
    int cur_corner_cloud_num = ds_cur_corner_cloud_->points.size();
    vVec3d near_corners;
    Vec3d center(0.0,0.0,0.0);
    std::vector<int> point_search_ind;
    std::vector<float> point_search_sqdis;
    for(int i = 0;i < cur_corner_cloud_num;i++){
        float distance = 1.0;
        point_ori = ds_cur_corner_cloud_->points[i];
        Transformbox::pointAssociateToMap(&point_ori,&point_sel,keyframe_pose_);
        ptr_kdtree_corner_from_map_->nearestKSearch(point_sel,5,point_search_ind,point_search_sqdis);
        if(point_search_sqdis[4] < 1.0){
            center(0) = center(1) = center(2) = 0;
            for(int j = 0;j < 5;j++){
                Vec3d temp_point(
                ds_map_corner_cloud_->points[point_search_ind[j]].x,
                ds_map_corner_cloud_->points[point_search_ind[j]].y,
                ds_map_corner_cloud_->points[point_search_ind[j]].z);
                center += temp_point;
                near_corners.push_back(temp_point);
            }
            center = center / 5.0;
            Mat3d cov_mat = Mat3d::Zero();
            for(int j = 0;j < 5;j++){
                Vec3d temp_zero_mean = near_corners[j] - center;
                cov_mat += temp_zero_mean * temp_zero_mean.transpose();
            }
            cov_mat = 0.2 * cov_mat;
            Eigen::SelfAdjointEigenSolver<Mat3d> saes(cov_mat);
            Vec3d uint_direction = saes.eigenvectors().col(2);

            Vec3d curr_point(point_ori.x,point_ori.y,point_ori.z);

            if(saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]){
                Vec3d point_on_line = center;
                Vec3d point_a,point_b;
                point_a = 0.1 * uint_direction + point_on_line;
                point_b = -0.1 * uint_direction + point_on_line;

                Vec3d curr_map_point = Mathbox::multiplePoint(keyframe_pose_,curr_point);
                Vec3d l_pa_cross_l_pb =  (curr_map_point - point_a).cross(curr_map_point - point_b);
                Vec3d la_lb = point_a - point_b;
                distance = l_pa_cross_l_pb.norm() / la_lb.norm();
                double scale = 1.0 - 0.9 * fabs(distance);
                if(scale > 0.1){
                    valid_edge_point++;
                    ceres::CostFunction *cost_function = new EdgeFactor(curr_point,point_a,point_b,scale);
                    problem.AddResidualBlock(cost_function,loss_function,keyframe_pose_.data());
                }
            }
        }
        error_sum += distance;
    }
    int cur_surf_cloud_num = ds_cur_surf_cloud_->points.size();
    float distance = 1.0;
    Mat53d A0;
    Vec5d B0 = -1 * Vec5d::Ones();
    for(int i = 0; i < cur_surf_cloud_num;i++){
        point_ori = ds_cur_surf_cloud_->points[i];
        Transformbox::pointAssociateToMap(&point_ori,&point_sel,keyframe_pose_);
        ptr_kdtree_surf_from_map_->nearestKSearch(point_sel,5,point_search_ind,point_search_sqdis);
        if(point_search_sqdis[4] < 1.0){
            for(int j = 0;j < 5;j++){
                A0(j,0) = ds_map_surf_cloud_->points[point_search_ind[j]].x;
                A0(j,1) = ds_map_surf_cloud_->points[point_search_ind[j]].y;
                A0(j,2) = ds_map_surf_cloud_->points[point_search_ind[j]].z;
            }
            Vec3d norm = A0.colPivHouseholderQr().solve(B0);
            double negative_OA_dot_norm = 1.0 / norm.norm();
            norm.normalize();

            bool plane_valid = true;
            for(int j = 0;j < 5;j++){
                    if (fabs(
                        norm(0) * ds_map_surf_cloud_->points[point_search_ind[j]].x +
                        norm(1) * ds_map_surf_cloud_->points[point_search_ind[j]].y +
                        norm(2) * ds_map_surf_cloud_->points[point_search_ind[j]].z +
                        negative_OA_dot_norm) > 0.2) {
                plane_valid = false;
                break;
                }
            } 
            Vec3d curr_point(point_ori.x,point_ori.y,point_ori.z);
            if(plane_valid){
                Vec3d curr_map_point =
                Mathbox::multiplePoint(keyframe_pose_, curr_point);
                distance = fabs(norm.dot(curr_map_point) + negative_OA_dot_norm);
                double scale = 1. - 0.9 * distance;
                if (scale > 0.1) {
                    ceres::CostFunction *cost_function =
                        new PlaneFactor(curr_point, norm, scale, negative_OA_dot_norm);
                    problem.AddResidualBlock(cost_function, loss_function,
                        keyframe_pose_.data());
                }
            }
        }
        error_sum += distance;
    }
    double distance_error = error_sum / (cur_corner_cloud_num + cur_surf_cloud_num); 
    LOG(INFO) << "error distance :   " << distance_error << std::endl;        

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 5;
    options.num_threads = 1;
    options.minimizer_progress_to_stdout = false;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 1e-4;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
}

}