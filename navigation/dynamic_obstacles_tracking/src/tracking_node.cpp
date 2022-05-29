/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file tracking_node.cpp
 *
 *@brief cpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.2.4
 *@data 2020-05-25
 ************************************************************************/
#include "log.hpp"

#include "classifier/classifier.hpp"
#include "clusters/cluster.hpp"
// #include "common/bounding_box.hpp"
#include "common/bounding.hpp"
#include "object_builders/min_box_object_builder.hpp"
#include "tracking_node.hpp"
#include "tracking_worker.hpp"

namespace CVTE_BABOT {

TrackingNode::TrackingNode()
    : ptr_point_cluster_(std::make_shared<PointCloudCluster>()),
      ptr_tracking_worker_(std::make_shared<TrackingWorker>()),
      ptr_classifier_(std::make_shared<Classifier>()) {
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  node_ = std::make_shared<rclcpp::Node>("dynamic_obstacles_tracking", options);
  ptr_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  updateParameters();

  point_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud>(
      s_cloud_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&TrackingNode::pointCloudCallback, this,
                std::placeholders::_1));

  // the whole tracking output
  tracking_output_objects_pub_ =
      node_->create_publisher<lidar_perception_msgs::msg::TrackingObjectArray>(
          "/tracking_objects", rclcpp::QoS(1).best_effort());
  static_points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "static_points", rclcpp::QoS(10).best_effort());

  boundmsgs_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "people", rclcpp::QoS(10).best_effort());
  tracking_objects_velocity_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/tracking/objects_vel", rclcpp::QoS(1).best_effort());
}

void TrackingNode::updateParameters() {
  node_->get_parameter_or("print_log", b_print_log_, false);

  node_->get_parameter_or("cloud_topic", s_cloud_topic_,
                          std::string("camera_navigation_cloud"));

  int filter_num = 0;
  node_->get_parameter_or("filter_num", filter_num, 5);

  node_->get_parameter_or("threshold_contian_IoU", d_threshold_contian_IoU_,
                          0.5);

  // 聚类器的参数
  ClusterParams cluster_params;
  node_->get_parameter_or("cluster_radius", cluster_params.cluster_radius,
                          0.15);
  node_->get_parameter_or("increment_radius", cluster_params.increment_radius,
                          0.15);
  // 无法从参数文件读取无符号整型.
  int min_cluster_size = 0;
  node_->get_parameter_or("min_cluster_size", min_cluster_size, 8);
  assert(min_cluster_size >= 0);
  cluster_params.min_cluster_size = static_cast<uint16_t>(min_cluster_size);
  ptr_point_cluster_->updateParams(cluster_params);

  // 分类器的参数
  ClassifierParams classifier_params;
  node_->get_parameter_or("max_length", classifier_params.max_length, 0.5);
  node_->get_parameter_or("max_width", classifier_params.max_width, 0.5);
  node_->get_parameter_or("max_area", classifier_params.max_area, 0.5);
  node_->get_parameter_or("max_velocity", classifier_params.max_velocity, 3.0);
  node_->get_parameter_or("min_velocity", classifier_params.min_velocity, 1.0);
  node_->get_parameter_or("low_obs_height_threshhold",
                          classifier_params.low_obs_height_threshhold, 0.3);
  ptr_classifier_->updateParams(classifier_params);
}

void TrackingNode::fromPointCloudToVpoints(
    const sensor_msgs::msg::PointCloud::SharedPtr &ptr_point_cloud_msg,
    const VPointsPtr &ptr_vp_cloud) {
  if (ptr_point_cloud_msg->points.empty()) {
    return;
  }

  ptr_vp_cloud->reserve(ptr_point_cloud_msg->points.size());
  VPoint vpoint;
  for (size_t i = 0; i < ptr_point_cloud_msg->points.size(); i++) {
    const auto &point = ptr_point_cloud_msg->points[i];
    vpoint.x = point.x;
    vpoint.y = point.y;
    vpoint.z = point.z;
    ptr_vp_cloud->push_back(vpoint);
  }
}

void TrackingNode::pointCloudCallback(
    const sensor_msgs::msg::PointCloud::SharedPtr ptr_point_cloud_msg) {
  if (ptr_point_cloud_msg->points.size() == 0) {
    return;
  }

  // 1.转换数据
  VPointsPtr ptr_cloud_raw(new pcl::PointCloud<VPoint>);
  ptr_cloud_raw->reserve(ptr_point_cloud_msg->points.size());
  fromPointCloudToVpoints(ptr_point_cloud_msg, ptr_cloud_raw);

  // 2.聚类
  ptr_point_cluster_->setInputCloud(ptr_cloud_raw);
  auto v_ptr_point_cloud = ptr_point_cluster_->computeClustersPointCloud();

  // 3.计算几何特征
  std::vector<ObjectPtr> objects;
  MinBoxObjectBuilder::buildObjects(v_ptr_point_cloud, objects);

  // 4.去除过分割的对象(欠分割未处理)
  // 从大到小，可以让小物体被归类到大物体中
  std::vector<ObjectPtr> objects_obsved(objects.begin(), objects.end());
  auto objects_expected = ptr_tracking_worker_->collectExpectedObjects();
  sort(objects_expected.begin(), objects_expected.end(), areaCmp);

  // objects_expected.clear();
  if (!objects_expected.empty()) {
    /// @note 通过Tracking信息提升分割：过分割,
    /// 主要目的是因为遮挡等造成的过分割
    for (size_t expected_idx = 0u; expected_idx < objects_expected.size();
         ++expected_idx) {
      GroundBox gbox_expected;
      toGroundBox(objects_expected[expected_idx], &gbox_expected);

      auto cloud_merged =
          pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
      int count = 0;
      for (size_t obsv_idx = 0u; obsv_idx < objects.size(); ++obsv_idx) {
        GroundBox gbox_obsv;
        toGroundBox(objects[obsv_idx], &gbox_obsv);

        // combining all connected components within an expected
        // object’s bounding box into a new one
        if (groundBoxOverlap(gbox_expected, gbox_obsv,
                             d_threshold_contian_IoU_)) {
          *cloud_merged += *objects[obsv_idx]->ptr_point_cloud;
          objects_obsved[obsv_idx]->ptr_point_cloud->clear();
          count++;
        }
      }

      // build merged object
      ObjectPtr ptr_object_merged = std::make_shared<Object>();
      bool build_status =
          MinBoxObjectBuilder::buildObject(cloud_merged, *ptr_object_merged);
      if (build_status) {
        objects_obsved.push_back(ptr_object_merged);
        // 如果没有分配id，以匹配上的id作为其id
        if (!ptr_object_merged->ifAssignedId()) {
          ptr_object_merged->id = objects_expected[expected_idx]->id;
        }
      }

      // maintain tracking-help segmented objects
    }
    // remove all connected components at once
    auto iter = objects_obsved.begin();
    for (; iter != objects_obsved.end();) {
      if ((*iter)->ptr_point_cloud->empty()) {
        iter = objects_obsved.erase(iter);
      } else {
        ++iter;
      }
    }
  }
  // LOG(INFO) << "merge finished.";
  // 5.跟踪
  std::vector<ObjectPtr> objects_tracked;
  ptr_tracking_worker_->track(objects_obsved, objects_tracked);

  std::vector<ConstObjectPtr> objects_classified;
  ptr_classifier_->classify(objects_tracked, objects_classified);

  // 发布结果,保持时间戳一致
  computeAndPubObjects(objects_classified, ptr_point_cloud_msg->header.stamp);

  // 6.可视化
  computeAndPubBoxMsg(objects_classified);

  computeAndPubVelMsg(objects_classified);
}

void TrackingNode::computeAndPubObjects(
    const std::vector<ConstObjectPtr> &v_ptr_object,
    const builtin_interfaces::msg::Time &time) {
  if (v_ptr_object.empty()) {
    return;
  }

  lidar_perception_msgs::msg::TrackingObjectArray msg_tracking_objects;
  msg_tracking_objects.header.stamp = time;
  msg_tracking_objects.header.frame_id = "map";

  auto ptr_static_points =
      pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
  for (const auto &ptr_object : v_ptr_object) {
    // 异常处理,不应该把TEMPORARY_OBJECT的对象传入
    if (ptr_object->tracking_state == TEMPORARY_OBJECT) {
      LOG(ERROR) << "unexpected object tracking state!";
      continue;
    }

    // 以下三种情况当作静态障碍处理
    // 1.大小超出正常范围
    if (ptr_object->object_type == STATIC_OBJECT) {
      *ptr_static_points += *ptr_object->ptr_point_cloud;
      continue;
    }

    if (ptr_object->object_type == CAR_OBJECT) {
      msg_tracking_objects.type.push_back(2);
    } else if (ptr_object->object_type == OTHER_OBJECT) {
      msg_tracking_objects.type.push_back(1);
    } else {
      LOG(ERROR) << "unexpected object type!";
      continue;
    }

    // velocity
    geometry_msgs::msg::Point velocity;
    velocity.x = ptr_object->velocity(0);
    velocity.y = ptr_object->velocity(1);
    velocity.z = 0.0;
    msg_tracking_objects.velocities.push_back(velocity);

    // position
    geometry_msgs::msg::Point position;
    position.x = ptr_object->trajectory.back()(0);
    position.y = ptr_object->trajectory.back()(1);
    position.z = 0;
    msg_tracking_objects.positions.push_back(position);

    // direction
    geometry_msgs::msg::Point direction;
    direction.x = 0.0;
    direction.y = 0.0;
    direction.z = 0.0;
    msg_tracking_objects.directions.push_back(direction);

    // size
    geometry_msgs::msg::Point geometry;
    geometry.x = ptr_object->length;
    geometry.y = ptr_object->width;
    geometry.z = ptr_object->height;
    msg_tracking_objects.sizes.push_back(geometry);

    // segment
    sensor_msgs::msg::PointCloud2 msg_segment;
    pcl::toROSMsg(*ptr_object->ptr_point_cloud, msg_segment);
    msg_tracking_objects.segments.push_back(msg_segment);

    msg_tracking_objects.ids.push_back(ptr_object->id);
  }

  tracking_output_objects_pub_->publish(msg_tracking_objects);

  publishCloudMsg(ptr_static_points);
}

void TrackingNode::computeAndPubVelMsg(
    const std::vector<ConstObjectPtr> &v_ptr_object) {
  // 清除没有被跟踪到的对象的marker.
  std::vector<int> v_valid_id;
  if (!v_ptr_object.empty()) {
    v_valid_id.reserve(v_ptr_object.size());
  }

  for (const auto &ptr_object : v_ptr_object) {
    if (ptr_object->motion_state != MOBILE_OBJECT) {
      continue;
    }
    v_valid_id.push_back(ptr_object->id);
  }

  visualization_msgs::msg::MarkerArray vel_marker_array;
  // 清除所有vel, vel_text
  computeClearingVelMarkers(v_valid_id, vel_marker_array);

  for (const auto &ptr_object : v_ptr_object) {
    assert(!ptr_object->trajectory.empty());
    if (ptr_object->motion_state != MOBILE_OBJECT) {
      continue;
    }

    // 1.可视化速度的箭头
    visualization_msgs::msg::Marker vel_arrow;
    vel_arrow.header.frame_id = "map";
    vel_arrow.ns = "tracking_vels";
    vel_arrow.id = ptr_object->id;
    vel_arrow.type = visualization_msgs::msg::Marker::ARROW;
    vel_arrow.scale.x = 0.2;
    vel_arrow.scale.y = 0.2;
    vel_arrow.scale.z = 0.2;
    vel_arrow.color.r = 0.0f;
    vel_arrow.color.g = 1.0f;
    vel_arrow.color.b = 0.0f;
    vel_arrow.color.a = 1.0f;

    // Fill in velocity's arrow
    Eigen::Vector3d start_vector;
    start_vector(0) = ptr_object->trajectory.back()(0);
    start_vector(1) = ptr_object->trajectory.back()(1);
    start_vector(2) = 0.00;
    Eigen::Vector3d velocity_vector;
    velocity_vector(0) = ptr_object->velocity(0);
    velocity_vector(1) = ptr_object->velocity(1);
    velocity_vector(2) = 0.00;

    Eigen::Vector3d end_vector;
    end_vector = start_vector + velocity_vector;

    geometry_msgs::msg::Point start_point, end_point;
    start_point.x = start_vector(0);
    start_point.y = start_vector(1);
    start_point.z = 0.3;
    end_point.x = end_vector(0);
    end_point.y = end_vector(1);
    end_point.z = 0.3;

    vel_arrow.points.push_back(start_point);
    vel_arrow.points.push_back(end_point);
    vel_marker_array.markers.push_back(vel_arrow);

    // 2.可视化速度的文本
    visualization_msgs::msg::Marker vel_text;
    vel_text.header.frame_id = "map";
    vel_text.ns = "tracking_texts";
    vel_text.id = ptr_object->id;
    vel_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    std::stringstream vel_str;
    // fixed：表示普通方式输出，不采用科学计数法。
    double vel_scalar = sqrt(pow(ptr_object->velocity(0), 2.0) +
                             pow(ptr_object->velocity(1), 2.0));
    vel_str << std::fixed << std::setprecision(2) << std::setfill('0')
            << vel_scalar;
    vel_text.text = vel_str.str() + " m/s";
    vel_text.scale.z = 0.7;
    vel_text.color = vel_arrow.color;

    // Fill in velocity's label
    vel_text.pose.position.x = start_vector(0);
    vel_text.pose.position.y = start_vector(1);
    vel_text.pose.position.z = 1.2;
    vel_marker_array.markers.push_back(vel_text);
  }

  tracking_objects_velocity_pub_->publish(vel_marker_array);
}

void TrackingNode::computeClearingVelMarkers(
    const std::vector<int> &v_valid_index,
    visualization_msgs::msg::MarkerArray &clear_markers) {
  for (size_t i = 0; i < MAX_LABELS_NUM; i++) {
    if (isElementInVector(v_valid_index, i)) {
      continue;
    }

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.id = i;
    clear_marker.action = clear_marker.DELETE;
    clear_marker.ns = "tracking_vels";
    clear_markers.markers.push_back(clear_marker);
    clear_marker.ns = "tracking_texts";
    clear_markers.markers.push_back(clear_marker);
  }
}

void TrackingNode::computeClearingMarkers(
    const std::vector<int> &v_valid_id,
    visualization_msgs::msg::MarkerArray &boxset_msgs) {
  // clearing dated boxes.
  for (size_t id = 0; id < MAX_LABELS_NUM; id++) {
    if (isElementInVector(v_valid_id, id)) {
      continue;
    }

    visualization_msgs::msg::Marker clearing_bounding_marker;
    clearing_bounding_marker.header.frame_id = "map";
    clearing_bounding_marker.ns = "box";
    clearing_bounding_marker.id = id;
    clearing_bounding_marker.action = visualization_msgs::msg::Marker::DELETE;
    boxset_msgs.markers.push_back(clearing_bounding_marker);

    visualization_msgs::msg::Marker clearing_text_marker;
    clearing_text_marker.header.frame_id = "map";
    clearing_text_marker.ns = "id";
    clearing_text_marker.id = id;
    clearing_text_marker.action = visualization_msgs::msg::Marker::DELETE;
    boxset_msgs.markers.push_back(clearing_text_marker);

    visualization_msgs::msg::Marker clearing_trajectory_marker;
    clearing_trajectory_marker.header.frame_id = "map";
    clearing_trajectory_marker.ns = "trajectory";
    clearing_trajectory_marker.id = id;
    clearing_trajectory_marker.action = visualization_msgs::msg::Marker::DELETE;
    boxset_msgs.markers.push_back(clearing_trajectory_marker);

    visualization_msgs::msg::Marker clearing_vel_marker;
    clearing_vel_marker.header.frame_id = "map";
    clearing_vel_marker.id = id;
    clearing_vel_marker.action = visualization_msgs::msg::Marker::DELETE;
    clearing_vel_marker.ns = "tracking_vels";
    boxset_msgs.markers.push_back(clearing_vel_marker);
    clearing_vel_marker.ns = "tracking_texts";
    boxset_msgs.markers.push_back(clearing_vel_marker);
  }

  boundmsgs_pub_->publish(boxset_msgs);
}

void TrackingNode::computeAndPubBoxMsg(
    const std::vector<ConstObjectPtr> &v_ptr_object) {
  std::vector<int> v_valid_id;
  // 清除没有被跟踪到的对象的marker.
  // 即使没有新对象，也要清除掉旧的marker,所以不能直接return.
  if (!v_ptr_object.empty()) {
    v_valid_id.reserve(v_ptr_object.size());
  }

  for (const auto &ptr_object : v_ptr_object) {
    v_valid_id.push_back(ptr_object->id);
  }

  visualization_msgs::msg::MarkerArray boxset_msgs;
  // 清除所有Box,Text,Trajactory
  computeClearingMarkers(v_valid_id, boxset_msgs);

  for (const auto &ptr_object : v_ptr_object) {
    visualization_msgs::msg::Marker bounding_box_marker;
    computeBoundingBoxMarker(ptr_object, bounding_box_marker);
    boxset_msgs.markers.push_back(bounding_box_marker);

    visualization_msgs::msg::Marker text_marker;
    computeTextMarker(ptr_object, text_marker);
    boxset_msgs.markers.push_back(text_marker);

    visualization_msgs::msg::Marker trajectory_marker;
    computeTrajMarker(ptr_object, trajectory_marker);
    boxset_msgs.markers.push_back(trajectory_marker);
  }

  boundmsgs_pub_->publish(boxset_msgs);
}

void TrackingNode::publishCloudMsg(const VPointsPtr &ptr_output_cloud) {
  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*ptr_output_cloud, output_msg);
  output_msg.header.stamp = ptr_clock_->now();
  output_msg.header.frame_id = "map";
  static_points_pub_->publish(output_msg);
}

void TrackingNode::computeBoundingBoxMarker(
    const ConstObjectPtr &ptr_object,
    visualization_msgs::msg::Marker &bounding_box_marker) {
  uint32_t shape;
  if (ptr_object->tracking_state == TRACKED_OBJECT) {
    shape = visualization_msgs::msg::Marker::CUBE;

    if (ptr_object->object_type == CAR_OBJECT) {
      bounding_box_marker.color.r = 1.0f;
      bounding_box_marker.color.g = 1.0f;
      bounding_box_marker.color.b = 0.0f;
    } else if (ptr_object->motion_state == MOBILE_OBJECT) {
      bounding_box_marker.color.r = 1.0f;
      bounding_box_marker.color.g = 0.0f;
      bounding_box_marker.color.b = 0.0f;
    } else {
      bounding_box_marker.color.r = 0.0f;
      bounding_box_marker.color.g = 0.0f;
      bounding_box_marker.color.b = 1.0f;
    }
  } else {
    shape = visualization_msgs::msg::Marker::SPHERE;
    bounding_box_marker.color.r = 0.0f;
    bounding_box_marker.color.g = 0.0f;
    bounding_box_marker.color.b = 1.0f;
  }

  bounding_box_marker.header.frame_id = "map";
  bounding_box_marker.ns = "box";
  bounding_box_marker.type = shape;
  bounding_box_marker.id = ptr_object->id;
  bounding_box_marker.lifetime = rclcpp::Duration(1.0);
  bounding_box_marker.pose.orientation.x = 0.0;
  bounding_box_marker.pose.orientation.y = 0.0;
  bounding_box_marker.pose.orientation.z = 0.0;
  bounding_box_marker.pose.orientation.w = 1.0;
  bounding_box_marker.color.a = 0.5;

  bounding_box_marker.pose.position.x = ptr_object->object_center(0);
  bounding_box_marker.pose.position.y = ptr_object->object_center(1);
  bounding_box_marker.pose.position.z = ptr_object->object_center(2);

  bounding_box_marker.scale.x = abs(ptr_object->length);
  bounding_box_marker.scale.y = abs(ptr_object->width);
  bounding_box_marker.scale.z = abs(ptr_object->height);
}

void TrackingNode::computeTextMarker(
    const ConstObjectPtr &ptr_object,
    visualization_msgs::msg::Marker &text_marker) {
  text_marker.ns = "id";
  text_marker.id = ptr_object->id;
  text_marker.header.frame_id = "map";
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.pose.position.x = ptr_object->object_center(0);
  text_marker.pose.position.y = ptr_object->object_center(1);
  text_marker.pose.position.z = 2.0;
  text_marker.scale.z = 0.5;
  text_marker.color.r = 0.0f;
  text_marker.color.g = 0.0f;
  text_marker.color.b = 1.0f;
  text_marker.color.a = 1.0f;
  text_marker.text = std::to_string(ptr_object->id);
  text_marker.lifetime = rclcpp::Duration(0.1);
}

void TrackingNode::computeTrajMarker(
    const ConstObjectPtr &ptr_object,
    visualization_msgs::msg::Marker &trajectory_marker) {
  trajectory_marker.header.frame_id = "map";
  trajectory_marker.ns = "trajectory";
  trajectory_marker.id = ptr_object->id;
  trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  trajectory_marker.scale.x = 0.1;
  trajectory_marker.color.a = 1.0;
  trajectory_marker.color.r =
      std::max(0.3, static_cast<double>(ptr_object->id % 3) / 3.0);
  trajectory_marker.color.g =
      std::max(0.3, static_cast<double>(ptr_object->id % 6) / 6.0);
  trajectory_marker.color.b =
      std::max(0.3, static_cast<double>(ptr_object->id % 9) / 9.0);
  trajectory_marker.lifetime = rclcpp::Duration(1.0);
  trajectory_marker.pose.orientation.x = 0.0;
  trajectory_marker.pose.orientation.y = 0.0;
  trajectory_marker.pose.orientation.z = 0.0;
  trajectory_marker.pose.orientation.w = 1.0;

  const auto &trajectory = ptr_object->trajectory;
  for (size_t point_it = 0; point_it < trajectory.size(); point_it++) {
    geometry_msgs::msg::Point point;

    point.x = trajectory[point_it](0);
    point.y = trajectory[point_it](1);
    trajectory_marker.points.push_back(point);
  }
}

}  // namespace CVTE_BABOT

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  CVTE_BABOT::initGoogleLog("dynamic_obstacles_tracking", "info");

  CVTE_BABOT::TrackingNode tracking_node;

  rclcpp::spin(tracking_node.getNodeHander());

  rclcpp::shutdown();

  return 0;
}
