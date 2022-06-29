/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022, CVTE.
 * All rights reserved.
 *

 * @FilePath:
 /src/cvte_lidar_slam/slam_core/common/registration/cloud_registration.hpp
 * @brief:
 * @
 * @
 *@author Yun Su(robosu12@gmail.com)
 *@version 0.8
 *@data 2022-04-27
 ************************************************************************/
#ifndef CLOUD_REGISTRATION_HPP
#define CLOUD_REGISTRATION_HPP

#include <glog/logging.h>
#include <string>

// #include <pcl/features/from_meshes.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/organized_fast_mesh.h>

#include "common/data_struct/pc_base.hpp"
#include "common/math_base/slam_math.hpp"

namespace cvte_lidar_slam {
// class CloudRegistration{
//   static bool  operator ()()

// }; // end of class cloudRegistration

static double cloudRegistration(std::string type,
                                const laserCloud::Ptr ptr_source_cloud,
                                const laserCloud::Ptr ptr_target_cloud,
                                const double icp_fitness_score, float score,
                                Mat4d &transformation) {
  double cov = -1.0;
  if (ptr_source_cloud->empty()) {
    LOG(ERROR) << "source cloud is empty.";
    return cov;
  }
  if (ptr_target_cloud->empty()) {
    LOG(ERROR) << "Target cloud is empty.";
    return cov;
  }

  if (type == "icp") {
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    // icp.setMaxCorrespondenceDistance(3.0);
    // icp.setMaximumIterations(100);
    icp.setMaxCorrespondenceDistance(1.0);  // 2.0;
    icp.setMaximumIterations(60);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    icp.setInputSource(ptr_source_cloud);
    icp.setInputTarget(ptr_target_cloud);
    pcl::PointCloud<PointType>::Ptr unused_result(
        new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    // cov = icp.getFitnessScore();
    // cov = icp.getFitnessScore(icp_fitness_score);

    Eigen::Isometry3f dT_correct = Eigen::Isometry3f::Identity();
    dT_correct = icp.getFinalTransformation();

    Eigen::Vector3d _dp_correct;
    Eigen::Matrix3d _dr_correct;
    _dp_correct = dT_correct.translation().cast<double>();
    _dr_correct = dT_correct.rotation().cast<double>();

    // Transform the input cloud using the final transformation
    pcl::PointCloud<PointType>::Ptr aligned_cloud(
        new pcl::PointCloud<PointType>());
    Mat34d dT_correct_34 = Mathbox::Identity34();
    dT_correct_34.block<3, 3>(0, 0) = _dr_correct;
    dT_correct_34.block<3, 1>(0, 3) = _dp_correct;
    aligned_cloud =
        Transformbox::transformPointCloud(dT_correct_34, ptr_source_cloud);

    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);

    auto KdTreePtr = icp.getSearchMethodTarget();

    size_t inlier_num = 0;
    size_t overlap_num = 0;
    float match_dis_sum = 0.0;
    float match_dis_ave = 0.99;
    float inlier_dis = icp_fitness_score;
    size_t cloud_size = aligned_cloud->points.size();
    for (size_t i = 0; i < cloud_size; ++i) {
      // Find its nearest neighbor in the target
      KdTreePtr->nearestKSearch(aligned_cloud->points[i], 1, nn_indices,
                                nn_dists);

      // Deal with occlusions (incomplete targets)
      if (nn_dists[0] < inlier_dis) {
        match_dis_sum += nn_dists[0];
        inlier_num++;
      }
      if (nn_dists[0] < inlier_dis * 0.5) {
        overlap_num++;
      }
    }

    if (inlier_num > 10) {
      match_dis_ave = match_dis_sum / inlier_num;
    }

    double overlap_ratio = 1.0 * overlap_num / cloud_size;
    if (overlap_ratio < 0.1) {
      overlap_ratio = 0.1;
    }

    // double overlap_trans = (overlap_ratio + 0.2) * (overlap_ratio + 0.2);
    double overlap_trans = overlap_ratio / 0.9;

    double overlap_score = overlap_trans * overlap_trans;

    LOG(INFO) << "cloudRegistration: overlap_ratio: " << overlap_ratio;
    LOG(INFO) << "cloudRegistration: overlap_score: " << overlap_score;

    cov = match_dis_ave / overlap_score;

    if (icp.hasConverged() == false || cov > score) {
      LOG(ERROR) << "icp not converged: " << icp.hasConverged() << "  "
                 << "icp fitness score: " << cov;
      return -1.;
    } else {
      transformation = icp.getFinalTransformation().cast<double>();
      LOG(WARNING) << "icp converged: " << icp.hasConverged() << "  "
                   << "icp fitness score: " << cov;
      return cov;
    }
  } else if (type == "ndt") {
    // pclomp::NormalDistributionsTransform<PointType, PointType>::Ptr ndt_omp(
    //     new pclomp::NormalDistributionsTransform<PointType, PointType>());
    // ndt_omp->setStepSize(0.1);
    // ndt_omp->setMaximumIterations(100);
    // ndt_omp->setTransformationEpsilon(0.01);
    // ndt_omp->setResolution(0.3);
    // ndt_omp->setNumThreads(4);
    // ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    // ndt_omp->setInputSource(ptr_source_cloud);
    // ndt_omp->setInputTarget(ptr_target_cloud);
    // pcl::PointCloud<PointType>::Ptr unused_result(
    //     new pcl::PointCloud<PointType>());
    // ndt_omp->align(*unused_result);
    // cov = ndt_omp->getFitnessScore();
    // std::cout << "ndt has converged:" << ndt_omp->hasConverged() << " "
    //           << "ndt fitness score:" << ndt_omp->getFitnessScore()
    //           << std::endl;
    // if (ndt_omp->hasConverged() == false ||
    //     ndt_omp->getFitnessScore() > score) {
    //   transformation = Mat4d::Identity();
    //   return -1.;
    // } else {
    //   transformation = ndt_omp->getFinalTransformation().cast<double>();
    //   return ndt_omp->getFitnessScore();
    // }
  } else if (type == "gicp") {
    // pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_source_keypoints(new
    // pcl::PointCloud<pcl::PointXYZ>); pcl::PointCloud<pcl::PointXYZ>::Ptr
    // ptr_target_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::copyPointCloud(*ptr_source_cloud, *ptr_source_keypoints);
    // pcl::copyPointCloud(*ptr_target_cloud, *ptr_target_keypoints);
    // // Normal estimation*
    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> source_normal_est;
    // pcl::PointCloud<pcl::Normal>::Ptr source_normals(new
    // pcl::PointCloud<pcl::Normal>); pcl::search::KdTree<pcl::PointXYZ>::Ptr
    // source_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // source_tree->setInputCloud(ptr_source_keypoints);
    // source_normal_est.setInputCloud(ptr_source_keypoints);
    // source_normal_est.setSearchMethod(source_tree);
    // source_normal_est.setKSearch(20);
    // source_normal_est.compute(*source_normals);
    // pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud_with_normals(new
    // pcl::PointCloud<pcl::PointNormal>);
    // pcl::concatenateFields(*ptr_source_keypoints, *source_normals,
    // *source_cloud_with_normals); pcl::search::KdTree<pcl::PointNormal>::Ptr
    // source_tree2(new pcl::search::KdTree<pcl::PointNormal>);
    // source_tree2->setInputCloud(source_cloud_with_normals);
    // pcl::GreedyProjectionTriangulation<pcl::PointNormal> source_gp3;
    // pcl::PolygonMesh source_triangles;
    // source_gp3.setSearchRadius(0.5);
    // source_gp3.setMu(2.5);
    // source_gp3.setMaximumNearestNeighbors(100);
    // source_gp3.setMaximumSurfaceAngle(M_PI / 4);  // 45 degrees
    // source_gp3.setMinimumAngle(M_PI / 18);        // 10 degrees
    // source_gp3.setMaximumAngle(2 * M_PI / 3);     // 120 degrees
    // source_gp3.setNormalConsistency(false);
    // source_gp3.setInputCloud(source_cloud_with_normals);
    // source_gp3.setSearchMethod(source_tree2);
    // source_gp3.reconstruct(source_triangles);

    // // Normal estimation*
    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> target_normal_est;
    // pcl::PointCloud<pcl::Normal>::Ptr target_normals(new
    // pcl::PointCloud<pcl::Normal>); pcl::search::KdTree<pcl::PointXYZ>::Ptr
    // target_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // target_tree->setInputCloud(ptr_target_keypoints);
    // target_normal_est.setInputCloud(ptr_target_keypoints);
    // target_normal_est.setSearchMethod(target_tree);
    // target_normal_est.setKSearch(20);
    // target_normal_est.compute(*target_normals);
    // pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(new
    // pcl::PointCloud<pcl::PointNormal>);
    // pcl::concatenateFields(*ptr_target_keypoints, *target_normals,
    // *target_cloud_with_normals); pcl::search::KdTree<pcl::PointNormal>::Ptr
    // target_tree2(new pcl::search::KdTree<pcl::PointNormal>);
    // target_tree2->setInputCloud(target_cloud_with_normals);
    // pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3_target;
    // pcl::PolygonMesh target_triangles;
    // gp3_target.setSearchRadius(0.5);
    // gp3_target.setMu(2.5);
    // gp3_target.setMaximumNearestNeighbors(100);
    // gp3_target.setMaximumSurfaceAngle(M_PI / 4);  // 45 degrees
    // gp3_target.setMinimumAngle(M_PI / 18);        // 10 degrees
    // gp3_target.setMaximumAngle(2 * M_PI / 3);     // 120 degrees
    // gp3_target.setNormalConsistency(false);
    // gp3_target.setInputCloud(target_cloud_with_normals);
    // gp3_target.setSearchMethod(target_tree2);
    // gp3_target.reconstruct(target_triangles);

    // boost::shared_ptr<std::vector<Eigen::Matrix<double, 3, 3>,
    // Eigen::aligned_allocator<Eigen::Matrix<double, 3, 3>>>> source_covs(new
    // std::vector<Eigen::Matrix<double, 3, 3>,
    // Eigen::aligned_allocator<Eigen::Matrix<double, 3, 3>>>);
    // boost::shared_ptr<std::vector<Eigen::Matrix<double, 3, 3>,
    // Eigen::aligned_allocator<Eigen::Matrix<double, 3, 3>>>> target_covs(new
    // std::vector<Eigen::Matrix<double, 3, 3>,
    // Eigen::aligned_allocator<Eigen::Matrix<double, 3, 3>>>);
    // pcl::features::computeApproximateCovariances(*ptr_source_keypoints,
    // *source_normals, *source_covs);
    // pcl::features::computeApproximateCovariances(*ptr_target_keypoints,
    // *target_normals, *target_covs);
    // pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    // gicp.setMaxCorrespondenceDistance(1);
    // gicp.setMaximumIterations(100);
    // // gicp.setTransformationEpsilon(1e-6);
    // gicp.setInputSource(ptr_source_keypoints);
    // gicp.setInputTarget(ptr_target_keypoints);
    // gicp.setSourceCovariances(source_covs);
    // gicp.setTargetCovariances(target_covs);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr unused_result(new
    // pcl::PointCloud<pcl::PointXYZ>()); gicp.align(*unused_result);
    // // std::cout << gicp.getFinalTransformation() << std::endl << std::endl;
    // // std::cout << __LINE__ << std::endl;
    // cov = gicp.getFitnessScore();
    // // std::cout << __LINE__ << std::endl;
    // // std::cout << "icp has converged: " << gicp.hasConverged() << "  "
    // //          << "icp fitness score: " << gicp.getFitnessScore() <<
    // std::endl; if (gicp.hasConverged() == false || gicp.getFitnessScore() >
    // score)
    // {
    //   //   std::cout << __LINE__ << std::endl;
    //   transformation = gicp.getFinalTransformation().cast<double>();
    //   return -1.;
    // }
    // else
    // {
    //   transformation = gicp.getFinalTransformation().cast<double>();
    //   //   std::cout << __LINE__ << std::endl;
    //   return cov;
    // }
  } else {
    return -1.;
  }
  return -1.;
}

static double cloudMatchByICP(const laserCloud::Ptr ptr_source_cloud,
                              const laserCloud::Ptr ptr_target_cloud,
                              float max_match_dis, int max_iteration,
                              float score, Mat4d &transformation) {
  double cov = -1.0;
  if (ptr_source_cloud->empty()) {
    LOG(ERROR) << "source cloud is empty.";
    return cov;
  }
  if (ptr_target_cloud->empty()) {
    LOG(ERROR) << "Target cloud is empty.";
    return cov;
  }

  pcl::IterativeClosestPoint<PointType, PointType> icp;

  icp.setMaxCorrespondenceDistance(max_match_dis);
  icp.setMaximumIterations(max_iteration);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);
  icp.setInputSource(ptr_source_cloud);
  icp.setInputTarget(ptr_target_cloud);
  pcl::PointCloud<PointType>::Ptr unused_result(
      new pcl::PointCloud<PointType>());
  icp.align(*unused_result);

  cov = icp.getFitnessScore(2.0);

  if (icp.hasConverged() == false || cov > score) {
    LOG(ERROR) << "icp not converged: " << icp.hasConverged() << "  "
               << "icp fitness score: " << cov;
    return -1.;
  } else {
    transformation = icp.getFinalTransformation().cast<double>();
    LOG(WARNING) << "icp converged: " << icp.hasConverged() << "  "
                 << "icp fitness score: " << cov;
    return cov;
  }
}

static double cloudMatchByIterationICP(const laserCloud::Ptr ptr_source_cloud,
                                       const laserCloud::Ptr ptr_target_cloud,
                                       float max_match_dis, int max_iteration,
                                       float score, Mat4d &d_transformation) {
  double cov = -1.0;
  if (ptr_source_cloud->empty()) {
    LOG(ERROR) << "source cloud is empty.";
    return cov;
  }
  if (ptr_target_cloud->empty()) {
    LOG(ERROR) << "Target cloud is empty.";
    return cov;
  }

  Eigen::Matrix<float, 4, 4> dT_correct =
      Eigen::Matrix<float, 4, 4>::Identity();

  double max_correspondence_dis = max_match_dis;
  double max_iter = max_iteration;

  pcl::IterativeClosestPoint<PointType, PointType> icp_tmp;

  for (int i = 0; i < 3; i++) {
    // ICP Settings
    icp_tmp.setMaxCorrespondenceDistance(max_correspondence_dis);
    icp_tmp.setMaximumIterations(max_iter);
    icp_tmp.setTransformationEpsilon(1e-6);
    icp_tmp.setEuclideanFitnessEpsilon(1e-6);
    // icp_tmp.setRANSACIterations(0);

    // Align clouds
    pcl::PointCloud<PointType>::Ptr align_result(
        new pcl::PointCloud<PointType>());
    icp_tmp.setInputSource(ptr_source_cloud);
    icp_tmp.setInputTarget(ptr_target_cloud);
    icp_tmp.align(*align_result, dT_correct);

    dT_correct = icp_tmp.getFinalTransformation();
    max_correspondence_dis = max_correspondence_dis * 0.3;
  }

  // cov = icp_tmp.getFitnessScore();
  cov = icp_tmp.getFitnessScore(2.0);  // 3.0

  if (icp_tmp.hasConverged() == false || cov > score) {
    LOG(ERROR) << "icp not converged: " << icp_tmp.hasConverged() << "  "
               << "icp fitness score: " << cov;
    return -1.;
  } else {
    d_transformation = icp_tmp.getFinalTransformation().cast<double>();
    LOG(WARNING) << "icp converged: " << icp_tmp.hasConverged() << "  "
                 << "icp fitness score: " << cov;
    return cov;
  }
}

}  // namespace cvte_lidar_slam

#endif  // endif