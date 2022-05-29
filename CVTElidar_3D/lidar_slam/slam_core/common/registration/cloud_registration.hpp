/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *

 * @FilePath:
 /src/cvte_lidar_slam/slam_core/common/registration/cloud_registration.hpp
 * @brief:
 * @
 * @
 * @author: Wei Zhang(zhangwei@cvte.com)
 * @version: v 1.0
 * @Date: 2020-07-15 09:39:53
 ************************************************************************/
#ifndef CLOUD_REGISTRATION_HPP
#define CLOUD_REGISTRATION_HPP

#include <glog/logging.h>
#include <string>

#include <pcl/features/from_meshes.h>
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
                                float score, Mat4d &transformation) {
  static int cloud_count = 0;
  double cov = -1.0;
  if (ptr_target_cloud->empty()) {
    LOG(WARNING) << "Target cloud is empty.";
    return false;
  }
  if (type == "icp") {
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    icp.setInputSource(ptr_source_cloud);
    icp.setInputTarget(ptr_target_cloud);
    pcl::PointCloud<PointType>::Ptr unused_result(
        new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    cov = icp.getFitnessScore();
    std::cout << "icp has converged: " << icp.hasConverged() << "  "
              << "icp fitness score: " << icp.getFitnessScore() << std::endl;
    LOG(INFO) << "icp has converged: " << icp.hasConverged() << "  "
              << "icp fitness score: " << icp.getFitnessScore();
    cloud_count++;
    // pcl::io::savePCDFileASCII(
    //     "/home/caoyong/source_cloud_" + std::to_string(cloud_count) + ".pcd",
    //     *ptr_source_cloud);
    // pcl::io::savePCDFileASCII(
    //     "/home/caoyong/target_cloud_" + std::to_string(cloud_count) + ".pcd",
    //     *ptr_target_cloud);
    // pcl::io::savePCDFileASCII(
    //     "/home/caoyong/unused_cloud_" + std::to_string(cloud_count) + ".pcd",
    //     *unused_result);

    if (icp.hasConverged() == false || icp.getFitnessScore() > score) {
      transformation = icp.getFinalTransformation().cast<double>();
      //  std::cout << "transformation: " << transformation << std::endl;
      return -1.;
    } else {
      transformation = icp.getFinalTransformation().cast<double>();
      LOG(INFO) << "successfully transformation:\n " << transformation;
      return icp.getFitnessScore();
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_source_keypoints(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_target_keypoints(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*ptr_source_cloud, *ptr_source_keypoints);
    pcl::copyPointCloud(*ptr_target_cloud, *ptr_target_keypoints);
    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> source_normal_est;
    pcl::PointCloud<pcl::Normal>::Ptr source_normals(
        new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr source_tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    source_tree->setInputCloud(ptr_source_keypoints);
    source_normal_est.setInputCloud(ptr_source_keypoints);
    source_normal_est.setSearchMethod(source_tree);
    source_normal_est.setKSearch(20);
    source_normal_est.compute(*source_normals);
    pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud_with_normals(
        new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*ptr_source_keypoints, *source_normals,
                           *source_cloud_with_normals);
    pcl::search::KdTree<pcl::PointNormal>::Ptr source_tree2(
        new pcl::search::KdTree<pcl::PointNormal>);
    source_tree2->setInputCloud(source_cloud_with_normals);
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> source_gp3;
    pcl::PolygonMesh source_triangles;
    source_gp3.setSearchRadius(0.5);
    source_gp3.setMu(2.5);
    source_gp3.setMaximumNearestNeighbors(100);
    source_gp3.setMaximumSurfaceAngle(M_PI / 4);  // 45 degrees
    source_gp3.setMinimumAngle(M_PI / 18);        // 10 degrees
    source_gp3.setMaximumAngle(2 * M_PI / 3);     // 120 degrees
    source_gp3.setNormalConsistency(false);
    source_gp3.setInputCloud(source_cloud_with_normals);
    source_gp3.setSearchMethod(source_tree2);
    source_gp3.reconstruct(source_triangles);

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> target_normal_est;
    pcl::PointCloud<pcl::Normal>::Ptr target_normals(
        new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr target_tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    target_tree->setInputCloud(ptr_target_keypoints);
    target_normal_est.setInputCloud(ptr_target_keypoints);
    target_normal_est.setSearchMethod(target_tree);
    target_normal_est.setKSearch(20);
    target_normal_est.compute(*target_normals);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud_with_normals(
        new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*ptr_target_keypoints, *target_normals,
                           *target_cloud_with_normals);
    pcl::search::KdTree<pcl::PointNormal>::Ptr target_tree2(
        new pcl::search::KdTree<pcl::PointNormal>);
    target_tree2->setInputCloud(target_cloud_with_normals);
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3_target;
    pcl::PolygonMesh target_triangles;
    gp3_target.setSearchRadius(0.5);
    gp3_target.setMu(2.5);
    gp3_target.setMaximumNearestNeighbors(100);
    gp3_target.setMaximumSurfaceAngle(M_PI / 4);  // 45 degrees
    gp3_target.setMinimumAngle(M_PI / 18);        // 10 degrees
    gp3_target.setMaximumAngle(2 * M_PI / 3);     // 120 degrees
    gp3_target.setNormalConsistency(false);
    gp3_target.setInputCloud(target_cloud_with_normals);
    gp3_target.setSearchMethod(target_tree2);
    gp3_target.reconstruct(target_triangles);

    boost::shared_ptr<
        std::vector<Eigen::Matrix<double, 3, 3>,
                    Eigen::aligned_allocator<Eigen::Matrix<double, 3, 3>>>>
    source_covs(
        new std::vector<Eigen::Matrix<double, 3, 3>,
                        Eigen::aligned_allocator<Eigen::Matrix<double, 3, 3>>>);
    boost::shared_ptr<
        std::vector<Eigen::Matrix<double, 3, 3>,
                    Eigen::aligned_allocator<Eigen::Matrix<double, 3, 3>>>>
    target_covs(
        new std::vector<Eigen::Matrix<double, 3, 3>,
                        Eigen::aligned_allocator<Eigen::Matrix<double, 3, 3>>>);
    pcl::features::computeApproximateCovariances(*ptr_source_keypoints,
                                                 *source_normals, *source_covs);
    pcl::features::computeApproximateCovariances(*ptr_target_keypoints,
                                                 *target_normals, *target_covs);
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaxCorrespondenceDistance(1);
    gicp.setMaximumIterations(100);
    // gicp.setTransformationEpsilon(1e-6);
    gicp.setInputSource(ptr_source_keypoints);
    gicp.setInputTarget(ptr_target_keypoints);
    gicp.setSourceCovariances(source_covs);
    gicp.setTargetCovariances(target_covs);
    pcl::PointCloud<pcl::PointXYZ>::Ptr unused_result(
        new pcl::PointCloud<pcl::PointXYZ>());
    gicp.align(*unused_result);
    // std::cout << gicp.getFinalTransformation() << std::endl << std::endl;
    // std::cout << __LINE__ << std::endl;
    cov = gicp.getFitnessScore();
    // std::cout << __LINE__ << std::endl;
    // std::cout << "icp has converged: " << gicp.hasConverged() << "  "
    //          << "icp fitness score: " << gicp.getFitnessScore() << std::endl;
    if (gicp.hasConverged() == false || gicp.getFitnessScore() > score) {
      //   std::cout << __LINE__ << std::endl;
      transformation = gicp.getFinalTransformation().cast<double>();
      return false;
    } else {
      transformation = gicp.getFinalTransformation().cast<double>();
      //   std::cout << __LINE__ << std::endl;
      return gicp.getFitnessScore();
    }
  } else {
    return -1.;
  }
}

}  // namespace cvte_lidar_slam

#endif  // endif