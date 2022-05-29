#include "slam_system/mapping.hpp"
#include "frontend/submap_painter.hpp"

#include <sys/file.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fstream>

namespace slam2d_core {
namespace slam_system {

using namespace std::chrono_literals;

Mapping::Mapping() {}

Mapping::~Mapping() {}

bool Mapping::startMapping(const std::string &realtime_map_file) {
  realtime_map_file_ = realtime_map_file;
  scan_sampler_ptr_ = std::make_unique<FixedRatioSampler>(
      pose_graph_options_.scan_sampling_ratio);
  odometry_sampler_ptr_ = std::make_unique<FixedRatioSampler>(
      pose_graph_options_.odometry_sampling_ratio);
  LOG(INFO) << " the ratio of scan " << pose_graph_options_.scan_sampling_ratio
            << " the ration of odom "
            << pose_graph_options_.odometry_sampling_ratio;

  global_trajectory_builder_ptr_ =
      std::make_unique<slam2d_core::backend::GlobalTrajectoryBuilder>(
          local_trajectory_options_, pose_graph_options_);

  stop_thread_ = false;
  draw_map_thread_ = std::thread(&Mapping::drawMapCallback, this);
  LOG(INFO) << "-------------startMapping.";

  submap_slices_.clear();
  return true;
}

void Mapping::setParams(
    const slam2d_core::backend::PoseGraphOptions &pose_graph_options,
    const slam2d_core::frontend::LocalTrajectoryBuilderOptions
        &local_trajectory_options) {
  pose_graph_options_ = pose_graph_options;
  local_trajectory_options_ = local_trajectory_options;
}

bool Mapping::canceMapping() {
  scan_sampler_ptr_.reset();
  odometry_sampler_ptr_.reset();

  stop_thread_ = true;
  draw_map_thread_.join();

  global_trajectory_builder_ptr_.reset();
  LOG(INFO) << "-------------canceMapping.";
  return true;
}

void Mapping::runFinalOptimization() {
  global_trajectory_builder_ptr_->runFinalOptimization();
  drawMap();
}

void Mapping::drawMapCallback() {
  while (!stop_thread_) {
    std::this_thread::sleep_for(2s);
    drawMap();
  }
}

void Mapping::addOdomData(const common::OdometryData &odom_data) {
  if (!odometry_sampler_ptr_->Pulse()) {
    return;
  }
  global_trajectory_builder_ptr_->addOdomData(odom_data);
}

void Mapping::addImuData(const common::ImuData &imu_data) {
  global_trajectory_builder_ptr_->addImuData(imu_data);
}

void Mapping::addScanData(
    const common::TimedPointCloudData &time_point_cloud_data) {
  if (!scan_sampler_ptr_->Pulse()) {
    return;
  }
  global_trajectory_builder_ptr_->addRangeData(time_point_cloud_data);
}

bool Mapping::getLocalizationResult(const common::Time &time,
                                    common::Rigid3 &pose) {
  return global_trajectory_builder_ptr_->getGlobalPose(time, pose);
}

void Mapping::drawMap() {
  std::lock_guard<std::mutex> guard(submap_slices_mutex_);
  const auto submap_poses = global_trajectory_builder_ptr_->getAllSubmapPoses();

  std::set<int> submap_ids_to_delete;
  for (const auto &pair : submap_slices_) {
    submap_ids_to_delete.insert(pair.first);
  }

  for (const auto &submap_id_pose : submap_poses) {
    submap_ids_to_delete.erase(submap_id_pose.first);

    frontend::SubmapSlice &submap_slice = submap_slices_[submap_id_pose.first];

    submap_slice.pose = submap_id_pose.second.pose;
    submap_slice.metadata_version = submap_id_pose.second.version;
    if (submap_slice.surface != nullptr &&
        submap_slice.version == submap_id_pose.second.version) {
      continue;
    }
    common::SubmapTexture texture;
    if (!global_trajectory_builder_ptr_->submapToTexture(submap_id_pose.first,
                                                         &texture)) {
      continue;
    }

    submap_slice.version = texture.submap_version;
    submap_slice.width = texture.width;
    submap_slice.height = texture.height;
    submap_slice.slice_pose = texture.slice_pose;
    submap_slice.resolution = texture.resolution;
    submap_slice.cairo_data.clear();
    submap_slice.surface = frontend::drawTexture(
        texture.pixels.intensity, texture.pixels.alpha, texture.width,
        texture.height, &submap_slice.cairo_data);
  }

  for (const auto &id : submap_ids_to_delete) { submap_slices_.erase(id); }

  if (0 == submap_slices_.size()) {
    return;
  }

  auto painted_slices = frontend::paintSubmapSlices(submap_slices_, 0.1);
  // saveMapToFile(painted_slices, 0.1, realtime_map_file_);
  saveMapYaml(painted_slices, 0.1, realtime_map_file_);
}

frontend::PaintSubmapSlicesResult Mapping::paintSubmapSlices() {
  std::lock_guard<std::mutex> guard(submap_slices_mutex_);

  if (0 == submap_slices_.size()) {
    LOG(WARNING) << " submap_slices_.size() = 0";
  }
  return frontend::paintSubmapSlices(submap_slices_, 0.05);
}

frontend::PaintSubmapSlicesResult Mapping::paintSubmapSlices(
    double resolution) {
  std::lock_guard<std::mutex> guard(submap_slices_mutex_);

  if (0 == submap_slices_.size()) {
    LOG(WARNING) << " submap_slices_.size() = 0";
  }
  return frontend::paintSubmapSlices(submap_slices_, resolution);
}

bool Mapping::saveMapToFile(
    const slam2d_core::frontend::PaintSubmapSlicesResult &painted_slices,
    const double &resolution, const std::string &file_name) {
  const int width = cairo_image_surface_get_width(painted_slices.surface.get());
  const int height =
      cairo_image_surface_get_height(painted_slices.surface.get());
  const int threshold_free = 49;
  const int threshold_occupied = 55;
  std::string mapdatafile = file_name + ".pgm";
  FILE *out = fopen(mapdatafile.c_str(), "w");

  if (!out) {
    LOG(ERROR) << "Couldn't save map file to " << mapdatafile.c_str()
               << std::endl;
    return false;
  }

  fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
          resolution, width, height);

  std::vector<int8_t> data;
  const uint32_t *pixel_data = reinterpret_cast<uint32_t *>(
      cairo_image_surface_get_data(painted_slices.surface.get()));
  data.reserve(width * height);
  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      const uint32_t packed = pixel_data[y * width + x];
      const unsigned char color = packed >> 16;
      const unsigned char observed = packed >> 8;
      const int value =
          observed == 0
              ? -1
              : slam2d_core::common::roundToInt((1. - color / 255.) * 100.);
      CHECK_LE(-1, value);
      CHECK_GE(100, value);
      data.push_back(value);
    }
  }

  int x, y, i;
  for (y = 0; y < height; ++y) {
    for (x = 0; x < width; ++x) {
      i = x + (height - y - 1) * width;
      if (data[i] >= 0 && data[i] <= threshold_free) {
        fputc(254, out);
      } else if (data[i] >= threshold_occupied) {
        fputc(000, out);
      } else {
        fputc(205, out);
      }
    }
  }

  fflush(out);
  fclose(out);

  std::string mapmetadatafile = file_name + ".yaml";
  FILE *yaml = fopen(mapmetadatafile.c_str(), "w");

  flock(yaml->_fileno, LOCK_EX);

  Eigen::Quaterniond quaternion(1.0, 0.0, 0.0, 0.0);

  double yaw = slam2d_core::common::quaternionToYaw(quaternion);

  std::string map_name;
  auto pos = mapdatafile.find_last_of("/");
  if (pos == std::string::npos) {
    map_name = mapdatafile;
  } else if (pos != (mapdatafile.size() - 1)) {
    map_name = mapdatafile.substr(pos + 1);
  } else {
    LOG(ERROR) << "map name error!";
    return false;
  }

  fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\n",
          mapdatafile.c_str(), resolution,
          -painted_slices.origin.x() * resolution,
          (-height + painted_slices.origin.y()) * resolution, yaw);

  fprintf(yaml,
          "negate: 0\noccupied_thresh: 0.65\nfree_thresh: "
          "0.196\nheight: %d\nwidth: %d\n",
          height, width);
  fflush(yaml);
  flock(yaml->_fileno, LOCK_UN);
  fclose(yaml);

  return true;
}

bool Mapping::saveMapYaml(
    const slam2d_core::frontend::PaintSubmapSlicesResult &painted_slices,
    const double &resolution, const std::string &file_name) {
  const int width = cairo_image_surface_get_width(painted_slices.surface.get());
  const int height =
      cairo_image_surface_get_height(painted_slices.surface.get());

  std::string mapmetadatafile = file_name + ".yaml";
  FILE *yaml = fopen(mapmetadatafile.c_str(), "w");
  if (nullptr == yaml) {
    LOG(ERROR) << "fail open yaml";
    return false;
  }

  flock(yaml->_fileno, LOCK_EX);
  // LOG(ERROR) << "flock yaml";

  Eigen::Quaterniond quaternion(1.0, 0.0, 0.0, 0.0);

  double yaw = slam2d_core::common::quaternionToYaw(quaternion);

  fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\n",
          mapmetadatafile.c_str(), resolution,
          -painted_slices.origin.x() * resolution,
          (-height + painted_slices.origin.y()) * resolution, yaw);
  // LOG(ERROR) << "fprintf yaml";

  fprintf(yaml,
          "negate: 0\noccupied_thresh: 0.65\nfree_thresh: "
          "0.196\nheight: %d\nwidth: %d\n",
          height, width);
  // LOG(ERROR) << "fprintf yaml";
  fflush(yaml);
  flock(yaml->_fileno, LOCK_UN);
  fclose(yaml);

  return true;
}

bool Mapping::requestSaveMap(const std::string &file_name) {
  std::lock_guard<std::mutex> guard(submap_slices_mutex_);
  if (0 == submap_slices_.size()) {
    LOG(ERROR) << " submap_slices_.size() = 0";
  }
  auto painted_slices = frontend::paintSubmapSlices(submap_slices_, 0.05);
  saveMapToFile(painted_slices, 0.05, file_name);
  return true;
}

bool Mapping::hasTrajectoryNode() {
  std::lock_guard<std::mutex> guard(get_node_mutex_);
  const auto node_poses =
      global_trajectory_builder_ptr_->getTrajectoryNodePoses();
  if (0 == node_poses.size()) {
    LOG(WARNING) << " submap_slices_.size() = 0";
    return false;
  }
  return true;
}

bool Mapping::getLastTrajectoryNode(int &id, common::Rigid3 &node_pose) {
  std::lock_guard<std::mutex> guard(get_node_mutex_);
  const auto node_poses =
      global_trajectory_builder_ptr_->getTrajectoryNodePoses();
  if (0 == node_poses.size()) {
    LOG(WARNING) << "node_poses.size() = 0.";
    return false;
  }

  auto last_node = --(node_poses.end());
  id = last_node->first;
  node_pose = last_node->second.global_pose;
  return true;
}

bool Mapping::getTrajectoryNodePose(const int &id, common::Rigid3 &node_pose) {
  std::lock_guard<std::mutex> guard(get_node_mutex_);
  const auto node_poses =
      global_trajectory_builder_ptr_->getTrajectoryNodePoses();
  if (0 == node_poses.size()) {
    LOG(WARNING) << "node_poses.size() = 0.";
    return false;
  }

  auto node_data = node_poses.find(id);
  if (node_data == node_poses.end()) {
    LOG(ERROR) << "can not find node id: " << id;
    return false;
  }

  node_pose = node_data->second.global_pose;
  return true;
}

std::map<int, common::TrajectoryNodePose> Mapping::getTrajectoryNodePoses() {
  std::lock_guard<std::mutex> guard(get_node_mutex_);

  if (nullptr == global_trajectory_builder_ptr_) {
    LOG(WARNING) << "global_trajectory_builder_ptr_ is nullptr.";
    return {};
  }

  const auto node_poses =
      global_trajectory_builder_ptr_->getTrajectoryNodePoses();
  return node_poses;
}

std::map<int, backend::SubmapPose> Mapping::getAllSubmapPoses() const {
  if (nullptr == global_trajectory_builder_ptr_) {
    LOG(WARNING) << "global_trajectory_builder_ptr_ is nullptr.";
    return {};
  }
  return global_trajectory_builder_ptr_->getAllSubmapPoses();
}

std::vector<backend::Constraint> Mapping::constraints() const {
  if (nullptr == global_trajectory_builder_ptr_) {
    LOG(WARNING) << "global_trajectory_builder_ptr_ is nullptr.";
    return {};
  }
  return global_trajectory_builder_ptr_->constraints();
}

void Mapping::setReturnMappingPose(const common::Rigid3 &global_pose) {
  global_trajectory_builder_ptr_->setReturnMappingPose(global_pose);
}

bool Mapping::serializeStateToFile(bool include_unfinished_submaps,
                                   const std::string &filename) {
  return global_trajectory_builder_ptr_->serializeStateToFile(
      include_unfinished_submaps, filename);
}

void Mapping::loadSerializeStateFromFile(const std::string &state_filename) {
  global_trajectory_builder_ptr_->loadSerializeStateFromFile(state_filename);
}

}  // namespace slam_system
}  // namespace slam2d_core
