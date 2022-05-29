#include "lidar_iris_system.hpp"

namespace cvte_lidar_slam {

std::shared_ptr<LidarIrisSystem> LidarIrisSystem::ptr_lidar_iris_system_ =
    nullptr;

LidarIrisSystem::LidarIrisSystem() {
  ptr_lidar_iris_ = std::make_shared<LidarIris>(4, 18, 1.6, 0.75, 50);
}

std::shared_ptr<LidarIrisSystem> LidarIrisSystem::getInstance() {
  if (ptr_lidar_iris_system_ == nullptr) {
    ptr_lidar_iris_system_.reset(new LidarIrisSystem());
  }
  return ptr_lidar_iris_system_;
}

void LidarIrisSystem::insertFeatureCloud(const laserCloud::Ptr ori_cloud,
                                         const size_t frame_id) {
  IrisFeatureData feature_data;
  feature_data.frame_id = frame_id;
  cv::Mat1b f_img = LidarIris::getIris(*ori_cloud);
  LidarIris::FeatureDesc f_desc = ptr_lidar_iris_->getFeature(f_img);
  feature_data.feature_desc = f_desc;
  feature_mutex_.lock();
  v_feature_data_.push_back(feature_data);
  feature_mutex_.unlock();
}

bool LidarIrisSystem::saveFeatures(const std::string &map_dir) {
  LOG(INFO) << "LidarIrisSystem::saveFeatures ";
  feature_mutex_.lock();
  clock_t startTime = clock();
  if (v_feature_data_.empty()) {
    feature_mutex_.unlock();
    LOG(ERROR) << "v_feature_data_.empty()";
    return false;
  }
  std::ofstream feature_info_file;
  const std::string &info_file_name = map_dir + "feature_info.txt";
  feature_info_file.open(info_file_name, std::ios::out | std::ios::trunc);
  feature_info_file << std::fixed;
  if (!feature_info_file.is_open()) {
    feature_mutex_.unlock();
    LOG(ERROR) << " Can not open feature file";
    return false;
  }
  for (const auto &iter : v_feature_data_) {
    const size_t &frame_id = iter.frame_id;
    std::string feature_img_file =
        map_dir + "featur_img" + std::to_string(frame_id) + ".png";
    std::string feature_t_file =
        map_dir + "featur_T" + std::to_string(frame_id) + ".png";
    std::string feature_m_file =
        map_dir + "featur_M" + std::to_string(frame_id) + ".png";

    cv::imwrite(feature_img_file, iter.feature_desc.img);
    cv::imwrite(feature_t_file, iter.feature_desc.T);
    cv::imwrite(feature_m_file, iter.feature_desc.M);
    feature_info_file << std::setprecision(0) << frame_id << std::endl;
  }
  feature_info_file.close();
  clock_t endTime = clock();
  std::cout << "v_feature_data_ " << v_feature_data_.size() << ","
            << (endTime - startTime) / (double)CLOCKS_PER_SEC << "s."
            << std::endl;
  feature_mutex_.unlock();
  return true;
}

bool LidarIrisSystem::loadFeature(const std::string &map_dir) {
  std::string feature_file = map_dir;
  if (feature_file.back() != '/') {
    feature_file.push_back('/');
  }
  std::ifstream feature_info_file;
  const std::string &info_data_dir = map_dir + "feature_info.txt";
  feature_info_file.open(info_data_dir);
  if (!feature_info_file.is_open()) {
    LOG(ERROR) << " Can not open feature info file";
    return false;
  }
  v_feature_data_.clear();
  um_feature_data_.clear();
  std::vector<IrisFeatureData>().swap(v_feature_data_);
  clock_t startTime = clock();
  while (!feature_info_file.eof()) {
    IrisFeatureData feature_data;
    size_t frame_id;
    std::string s;
    std::getline(feature_info_file, s);
    std::stringstream ss;
    ss << s;
    ss >> frame_id;
    feature_data.frame_id = frame_id;

    std::string feature_img_file =
        map_dir + "featur_img" + std::to_string(frame_id) + ".png";
    std::string feature_t_file =
        map_dir + "featur_T" + std::to_string(frame_id) + ".png";
    std::string feature_m_file =
        map_dir + "featur_M" + std::to_string(frame_id) + ".png";

    if (access(feature_img_file.c_str(), 0) != 0 ||
        access(feature_t_file.c_str(), 0) != 0 ||
        access(feature_m_file.c_str(), 0) != 0) {
      LOG(ERROR) << feature_img_file << "feature file is not exist";
      return false;
    }

    feature_data.feature_desc.img = cv::imread(feature_img_file, 0);
    feature_data.feature_desc.T = cv::imread(feature_t_file, 0);
    feature_data.feature_desc.M = cv::imread(feature_m_file, 0);

    v_feature_data_.push_back(feature_data);
    um_feature_data_.insert(std::make_pair(frame_id, feature_data));
  }
  feature_info_file.close();
  clock_t endTime = clock();
  LOG(INFO) << "load feature data " << map_dir << ", " << v_feature_data_.size()
            << "," << (endTime - startTime) / (double)CLOCKS_PER_SEC << "s.";
  if (v_feature_data_.empty() || um_feature_data_.empty()) {
    return false;
  }
  return true;
}

bool LidarIrisSystem::findNearestFrame(const laserCloud::Ptr ori_cloud,
                                       std::pair<size_t, float> &compare_result,
                                       int &angle_bias) {
  feature_mutex_.lock();
  if (v_feature_data_.empty()) {
    feature_mutex_.unlock();
    LOG(ERROR) << "v_feature_data_.empty()";
    return false;
  }

  cv::Mat1b f_img = LidarIris::getIris(*ori_cloud);
  LidarIris::FeatureDesc f_desc = ptr_lidar_iris_->getFeature(f_img);

  float min_dist = 10000000;  // init with somthing large

  for (const auto &iter : v_feature_data_) {
    const size_t &frame_id = iter.frame_id;

    int bias;
    auto dis = ptr_lidar_iris_->compare(f_desc, iter.feature_desc, &bias);

    if (dis < min_dist) {
      min_dist = dis;
      compare_result.first = frame_id;
      compare_result.second = min_dist;
      angle_bias = bias;
    }
    if (min_dist < 0.3) {
      std::cout << "find a frame " << min_dist << " < 0.3" << std::endl;
      break;
    }
  }
  feature_mutex_.unlock();
  return true;
}

bool LidarIrisSystem::findNearestFramebyFrameids(
    const laserCloud::Ptr ori_cloud, const std::vector<size_t> &v_frame_id,
    std::pair<size_t, float> &compare_result, int &angle_bias) {
  feature_mutex_.lock();

  if (um_feature_data_.empty()) {
    feature_mutex_.unlock();
    LOG(ERROR) << "um_feature_data_.empty()";
    return false;
  }

  if (ori_cloud->points.empty()) {
    feature_mutex_.unlock();
    LOG(ERROR) << "ori_cloud.empty()";
    return false;
  }
  LOG(INFO) << "ori_cloud" << ori_cloud->points.size();
  cv::Mat1b f_img = LidarIris::getIris(*ori_cloud);
  LidarIris::FeatureDesc f_desc = ptr_lidar_iris_->getFeature(f_img);

  float min_dist = 10000000;  // init with somthing large

  for (size_t i = 0; i < v_frame_id.size(); i++) {
    LOG(INFO) << "LidarIrisSystem v_frame_id " << i << "," << v_frame_id[i];
    auto feature_data = um_feature_data_.find(v_frame_id[i]);
    if (feature_data == um_feature_data_.end()) {
      LOG(ERROR) << "Wrong frame id: " << v_frame_id[i];
      continue;
    }
    int bias;
    auto dis = ptr_lidar_iris_->compare(
        f_desc, (feature_data->second).feature_desc, &bias);

    if (dis < min_dist) {
      min_dist = dis;
      compare_result.first = feature_data->first;
      compare_result.second = min_dist;
      angle_bias = bias;
    }
    LOG(INFO) << "min_dist " << dis << "," << min_dist;
    if (min_dist < 0.35) {
      LOG(INFO) << "find a frame " << min_dist;
      break;
    }
    if(i > 30){
      LOG(INFO) << "too much key frame " << min_dist;
      break;
    }
  }
  feature_mutex_.unlock();
  return true;
}
}  // namespace cvte_lidar_slam