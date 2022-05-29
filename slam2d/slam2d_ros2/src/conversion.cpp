/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, CVTE.
 * All rights reserved.
 *
 *@file conversion.cpp
 *
 *@brief
 * 与slam2d_core与ros2的一些数据类型转换相关函数的具体实现
 *
 *@author chenmingjian(chenmingjian@cvte.com)
 *@version v1.0
 *@data 2021-04-01
 ************************************************************************/
#include "conversion.hpp"

#include <sys/file.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

namespace slam2d_ros2 {

builtin_interfaces::msg::Time toRos(const slam2d_core::common::Time &time) {
  static int64_t uts_timestamp, ns_since_unix_epoch;
  static builtin_interfaces::msg::Time ros_time;

  uts_timestamp = slam2d_core::common::toUniversal(time);
  ns_since_unix_epoch =
      (uts_timestamp -
       slam2d_core::common::kUtsEpochOffsetFromUnixEpochInSeconds *
           10000000ll) *
      100ll;

  ros_time.sec = static_cast<builtin_interfaces::msg::Time::_sec_type>(
      ns_since_unix_epoch / 1000000000);
  ros_time.nanosec = ns_since_unix_epoch % 1000000000;
  return ros_time;
}

slam2d_core::common::Time fromRos(const builtin_interfaces::msg::Time &time) {
  return slam2d_core::common::fromUniversal(
      (time.sec + slam2d_core::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
          10000000ll +
      (time.nanosec + 50) / 100);  // + 50 to get the rounding correct.
}

void scanToPointCloud(
    const sensor_msgs::msg::LaserScan &msg,
    const slam2d_core::common::Rigid3 &laser_tf,
    slam2d_core::common::TimedPointCloudData &time_point_cloud,
    sensor_msgs::msg::PointCloud &cloud_out) {
  CHECK_GE(msg.range_min, 0.f);
  CHECK_GE(msg.range_max, msg.range_min);

  if (msg.angle_increment > 0.f) {
    CHECK_GT(msg.angle_max, msg.angle_min);
  } else {
    CHECK_GT(msg.angle_min, msg.angle_max);
  }

  slam2d_core::common::PointCloud point_cloud;
  double angle = msg.angle_min;

  Eigen::Vector3d point_front;
  Eigen::Vector3d point_mid;
  Eigen::Vector3d point_back;
  Eigen::Vector3d point_add;

  double length_front = 0;
  double length_mid = 0;
  double length_back = 0;

  for (size_t i = 0; i < msg.ranges.size(); ++i) {
    const auto &echoes = msg.ranges[i];
    if (msg.range_min <= echoes && echoes <= msg.range_max) {
      const Eigen::AngleAxisd rotation(angle, Eigen::Vector3d::UnitZ());
      slam2d_core::common::RangePoint point{
          rotation * (echoes * Eigen::Vector3d::UnitX())};

      point_front = point_mid;
      point_mid = point_back;
      point_back = point.position;

      length_front = length_mid;
      length_mid = length_back;
      length_back = echoes;

      if (i >= 3 && i <= msg.ranges.size() - 1) {
        auto next_echoes = std::min(msg.ranges[i + 1], msg.range_max);
        const Eigen::AngleAxisd next_rotation(angle + msg.angle_increment,
                                              Eigen::Vector3d::UnitZ());
        Eigen::Vector3d next_point =
            next_rotation * (next_echoes * Eigen::Vector3d::UnitX());

        double dis_f_m = 0, dis_m_b = 0, dist_b_n = 0;
        dis_f_m =
            hypot(point_front[0] - point_mid[0], point_front[1] - point_mid[1]);
        dis_m_b =
            hypot(point_mid[0] - point_back[0], point_mid[1] - point_back[1]);

        dist_b_n =
            hypot(next_point[0] - point_back[0], next_point[1] - point_back[1]);

        if (dist_b_n > 0.3 && dis_m_b > 0.3) {
          angle += msg.angle_increment;
          continue;
        }

        double delta_k = fabs((length_back - length_mid) / length_mid -
                              (length_mid - length_front) / length_front) /
                         msg.angle_increment;

        //判断是否递增或递减
        bool flag_0 = (dist_b_n - dis_m_b) * (dis_m_b - dis_f_m) > 0;
        bool flag_1 = dis_f_m > 0.05 && dis_m_b > 0.05;
        bool flag_2 = delta_k < 2.0;

        if (flag_0 && flag_1 && flag_2) {
          int num = ceil(dis_m_b / 0.05);
          double a_x = (point_back[0] - point_mid[0]) / num;
          double a_y = (point_back[1] - point_mid[1]) / num;
          for (int j = 0; j < num; ++j) {
            point_add[0] = point_mid[0] + a_x * j;
            point_add[1] = point_mid[1] + a_y * j;
            point_cloud.push_back(slam2d_core::common::RangePoint{point_add});
            time_point_cloud.lengths.push_back(
                hypot(point_add[0], point_add[1]));
          }
        }
      }
      point_cloud.push_back(point);
      time_point_cloud.lengths.push_back(echoes);
    }
    angle += msg.angle_increment;
  }

  time_point_cloud.time = fromRos(msg.header.stamp);
  time_point_cloud.origin = laser_tf.translation().cast<double>();
  time_point_cloud.ranges =
      slam2d_core::common::transformPointCloud(point_cloud, laser_tf.cast());

  cloud_out.header = msg.header;
  cloud_out.points.resize(point_cloud.size());
  cloud_out.channels.resize(0);

  int idx_index = -1;

  int chan_size = cloud_out.channels.size();
  cloud_out.channels.resize(chan_size + 1);
  cloud_out.channels[chan_size].name = "index";
  cloud_out.channels[chan_size].values.resize(point_cloud.size());
  idx_index = chan_size;

  unsigned int count = 0;

  for (unsigned int index = 0; index < point_cloud.size(); index++) {
    cloud_out.points[count].x = point_cloud[index].position[0];
    cloud_out.points[count].y = point_cloud[index].position[1];
    cloud_out.points[count].z = 0.0;

    cloud_out.channels[idx_index].values[count] = index;

    count++;
  }

  cloud_out.points.resize(count);
  for (unsigned int d = 0; d < cloud_out.channels.size(); d++)
    cloud_out.channels[d].values.resize(count);

  return;
}

Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3 &vector3) {
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

Eigen::Quaterniond toEigen(const geometry_msgs::msg::Quaternion &quaternion) {
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}

slam2d_core::common::Rigid3 toRigid3(
    const geometry_msgs::msg::TransformStamped &transform) {
  return slam2d_core::common::Rigid3(toEigen(transform.transform.translation),
                                     toEigen(transform.transform.rotation));
}

slam2d_core::common::Rigid3 toRigid3(const geometry_msgs::msg::Pose &pose) {
  return slam2d_core::common::Rigid3(
      {pose.position.x, pose.position.y, pose.position.z},
      toEigen(pose.orientation));
}

geometry_msgs::msg::Transform toGeometryMsgTransform(
    const slam2d_core::common::Rigid3 &rigid3) {
  geometry_msgs::msg::Transform transform;
  transform.translation.x = rigid3.translation().x();
  transform.translation.y = rigid3.translation().y();
  transform.translation.z = rigid3.translation().z();
  transform.rotation.w = rigid3.rotation().w();
  transform.rotation.x = rigid3.rotation().x();
  transform.rotation.y = rigid3.rotation().y();
  transform.rotation.z = rigid3.rotation().z();
  return transform;
}

geometry_msgs::msg::Pose toGeometryMsgPose(
    const slam2d_core::common::Rigid3 &rigid3) {
  geometry_msgs::msg::Pose pose;
  pose.position = toGeometryMsgPoint(rigid3.translation());
  pose.orientation.w = rigid3.rotation().w();
  pose.orientation.x = rigid3.rotation().x();
  pose.orientation.y = rigid3.rotation().y();
  pose.orientation.z = rigid3.rotation().z();
  return pose;
}

geometry_msgs::msg::Point toGeometryMsgPoint(const Eigen::Vector3d &vector3d) {
  geometry_msgs::msg::Point point;
  point.x = vector3d.x();
  point.y = vector3d.y();
  point.z = vector3d.z();
  return point;
}

std_msgs::msg::ColorRGBA colorBar(double xbrt) {
  assert(xbrt >= 0 && xbrt <= 1);
  xbrt = 1.0 - xbrt;
  std_msgs::msg::ColorRGBA color;
  if (xbrt >= 0 && xbrt <= 0.25) {
    color.b = 1;
    color.g = (xbrt / 0.25);
    color.r = 0;
  } else if (xbrt > 0.25 && xbrt <= 0.5) {
    color.b = (2 - (xbrt / 0.25));
    color.g = 1;
    color.r = 0;
  } else if (xbrt > 0.5 && xbrt <= 0.75) {
    color.b = 0;
    color.g = 1;
    color.r = (xbrt / 0.25 * 1 - 2);
  } else {
    color.b = 0;
    color.g = (4 - (xbrt / 0.25));
    color.r = 1;
  }
  color.a = 1.0;
  return color;
}

std_msgs::msg::ColorRGBA getColor(int id) {
  constexpr float kGoldenRatioConjugate = 0.6180339887498949f;
  const float hue = std::fmod(0.69f + kGoldenRatioConjugate * id, 1.f);

  float h = hue;
  float s = 0.85f;
  float v = 0.77f;

  const float h_6 = (h == 1.f) ? 0.f : 6 * h;
  const int h_i = std::floor(h_6);
  const float f = h_6 - h_i;

  const float p = v * (1.f - s);
  const float q = v * (1.f - f * s);
  const float t = v * (1.f - (1.f - f) * s);

  std_msgs::msg::ColorRGBA result;
  result.a = 1.f;
  if (h_i == 0) {
    result.r = v;
    result.g = t;
    result.b = p;
  } else if (h_i == 1) {
    result.r = q;
    result.g = v;
    result.b = p;
  } else if (h_i == 2) {
    result.r = p;
    result.g = v;
    result.b = t;
  } else if (h_i == 3) {
    result.r = p;
    result.g = q;
    result.b = v;
  } else if (h_i == 4) {
    result.r = t;
    result.g = p;
    result.b = v;
  } else if (h_i == 5) {
    result.r = v;
    result.g = p;
    result.b = q;
  } else {
    result.r = 0.f;
    result.g = 0.f;
    result.b = 0.f;
  }

  return result;
}

std::unique_ptr<nav_msgs::msg::OccupancyGrid> createOccupancyGridMsg(
    const slam2d_core::frontend::PaintSubmapSlicesResult &painted_slices,
    const double &resolution, const std::string &frame_id,
    const builtin_interfaces::msg::Time &time) {
  auto occupancy_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>();

  const int width = cairo_image_surface_get_width(painted_slices.surface.get());
  const int height =
      cairo_image_surface_get_height(painted_slices.surface.get());

  occupancy_grid->header.stamp = time;
  occupancy_grid->header.frame_id = frame_id;
  occupancy_grid->info.map_load_time = time;
  occupancy_grid->info.resolution = resolution;
  occupancy_grid->info.width = width;
  occupancy_grid->info.height = height;
  occupancy_grid->info.origin.position.x =
      -painted_slices.origin.x() * resolution;
  occupancy_grid->info.origin.position.y =
      (-height + painted_slices.origin.y()) * resolution;
  occupancy_grid->info.origin.position.z = 0.;
  occupancy_grid->info.origin.orientation.w = 1.;
  occupancy_grid->info.origin.orientation.x = 0.;
  occupancy_grid->info.origin.orientation.y = 0.;
  occupancy_grid->info.origin.orientation.z = 0.;

  const uint32_t *pixel_data = reinterpret_cast<uint32_t *>(
      cairo_image_surface_get_data(painted_slices.surface.get()));
  occupancy_grid->data.reserve(width * height);
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
      occupancy_grid->data.push_back(value);
    }
  }

  return occupancy_grid;
}

void createOccupancyGridMsg(
    nav_msgs::msg::OccupancyGrid &occupancy_grid,
    std::shared_ptr<slam2d_core::amcl::OccupancyGrid> map_ptr,
    const builtin_interfaces::msg::Time &time) {
  occupancy_grid.header.frame_id = "map";
  occupancy_grid.info.width = map_ptr->getSizeX();
  occupancy_grid.info.height = map_ptr->getSizeY();
  occupancy_grid.data.resize(map_ptr->getSizeX() * map_ptr->getSizeY());
  occupancy_grid.header.stamp = time;
  occupancy_grid.info.map_load_time = time;
  occupancy_grid.info.resolution = map_ptr->getResolution();
  occupancy_grid.info.origin.position.x = map_ptr->getOriginX();
  occupancy_grid.info.origin.position.y = map_ptr->getOriginY();
  occupancy_grid.info.origin.position.z = 0.;
  occupancy_grid.info.origin.orientation.w = 1.;
  occupancy_grid.info.origin.orientation.x = 0.;
  occupancy_grid.info.origin.orientation.y = 0.;
  occupancy_grid.info.origin.orientation.z = 0.;

  for (unsigned int j = 0; j < occupancy_grid.info.height; j++) {
    for (unsigned int i = 0; i < occupancy_grid.info.width; i++) {
      if (map_ptr->getCells()[i][j].occ_state == 1) {
        occupancy_grid.data[occupancy_grid.info.width * j + i] =
            100;  // 100显示为黑色，表示占用
      } else if (map_ptr->getCells()[i][j].occ_state == -1) {
        occupancy_grid.data[occupancy_grid.info.width * j + i] =
            0;  // 0表示空闲
      } else {
        occupancy_grid.data[occupancy_grid.info.width * j + i] = -1;  //表示未知
      }
    }
  }
}

bool saveOccupancyGridToMapfile(
    std::unique_ptr<nav_msgs::msg::OccupancyGrid> ptr_map,
    const std::string &file_name) {
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
          ptr_map->info.resolution, ptr_map->info.width, ptr_map->info.height);

  unsigned int x, y, i;
  for (y = 0; y < ptr_map->info.height; ++y) {
    for (x = 0; x < ptr_map->info.width; ++x) {
      i = x + (ptr_map->info.height - y - 1) * ptr_map->info.width;
      if (ptr_map->data[i] >= 0 && ptr_map->data[i] <= threshold_free) {
        fputc(254, out);
      } else if (ptr_map->data[i] >= threshold_occupied) {
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

  Eigen::Quaterniond quaternion(
      ptr_map->info.origin.orientation.w, ptr_map->info.origin.orientation.x,
      ptr_map->info.origin.orientation.y, ptr_map->info.origin.orientation.z);

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
          mapdatafile.c_str(), ptr_map->info.resolution,
          ptr_map->info.origin.position.x, ptr_map->info.origin.position.y,
          yaw);
  fprintf(yaml,
          "negate: 0\noccupied_thresh: 0.65\nfree_thresh: "
          "0.196\nheight: %d\nwidth: %d\n",
          ptr_map->info.height, ptr_map->info.width);
  fflush(yaml);
  flock(yaml->_fileno, LOCK_UN);
  fclose(yaml);

  return true;
}

void base64Encode(const std::string &input, std::string &output) {
  //编码表
  const char encode_table[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  //返回值
  size_t data_byte = input.size();
  const char *data = input.data();
  std::string str_encode;
  unsigned char Tmp[4] = {0};
  int line_length = 0;
  for (int i = 0; i < (int) (data_byte / 3); i++) {
    Tmp[1] = *data++;
    Tmp[2] = *data++;
    Tmp[3] = *data++;
    str_encode += encode_table[Tmp[1] >> 2];
    str_encode += encode_table[((Tmp[1] << 4) | (Tmp[2] >> 4)) & 0x3F];
    str_encode += encode_table[((Tmp[2] << 2) | (Tmp[3] >> 6)) & 0x3F];
    str_encode += encode_table[Tmp[3] & 0x3F];
    if (line_length += 4, line_length == 76) {
      str_encode += "\r\n";
      line_length = 0;
    }
  }

  //对剩余数据进行编码
  int mod = data_byte % 3;
  if (mod == 1) {
    Tmp[1] = *data++;
    str_encode += encode_table[(Tmp[1] & 0xFC) >> 2];
    str_encode += encode_table[((Tmp[1] & 0x03) << 4)];
    str_encode += "==";
  } else if (mod == 2) {
    Tmp[1] = *data++;
    Tmp[2] = *data++;
    str_encode += encode_table[(Tmp[1] & 0xFC) >> 2];
    str_encode += encode_table[((Tmp[1] & 0x03) << 4) | ((Tmp[2] & 0xF0) >> 4)];
    str_encode += encode_table[((Tmp[2] & 0x0F) << 2)];
    str_encode += "=";
  }

  output = str_encode;
}

bool getPngFromOccupancyGrid(nav_msgs::msg::OccupancyGrid *ptr_map,
                             chassis_interfaces::msg::MappingPng &mapping_png,
                             const std::string &map_temp_path) {
  const int threshold_free = 49;
  const int threshold_occupied = 55;

  if (ptr_map == nullptr) {
    LOG(ERROR) << "ptr_map is nullptr";
    return false;
  }

  unsigned int x, y, i;
  cv::Mat png_grey_mat = cv::Mat::zeros(
      ptr_map->info.height, ptr_map->info.width, CV_8UC1);  // 8位无符号灰度图像

  for (y = 0; y < ptr_map->info.height; ++y) {
    for (x = 0; x < ptr_map->info.width; ++x) {
      i = x + (ptr_map->info.height - y - 1) * ptr_map->info.width;
      if (ptr_map->data[i] >= 0 && ptr_map->data[i] <= threshold_free) {
        png_grey_mat.at<unsigned char>(y, x) = (unsigned char) 254;
      } else if (ptr_map->data[i] >= threshold_occupied) {
        png_grey_mat.at<unsigned char>(y, x) = (unsigned char) 0;
      } else {
        png_grey_mat.at<unsigned char>(y, x) = (unsigned char) 205;
      }
    }
  }

  if (png_grey_mat.rows <= 0) {
    LOG(ERROR) << "image_rgb_png row <= 0";
    return false;
  }
  cv::imwrite(map_temp_path, png_grey_mat);

  std::ifstream image_fstream;
  std::stringstream map_png_ss;
  image_fstream.open(map_temp_path, std::ios::in | std::ios::binary);
  if (!image_fstream.is_open()) {
    LOG(ERROR) << " open file" << map_temp_path << " fail!";
    boost::filesystem::remove(map_temp_path);
    return false;
  }
  map_png_ss << image_fstream.rdbuf();
  image_fstream.close();

  std::string base64_str;
  base64Encode(map_png_ss.str(), base64_str);
  mapping_png.width = ptr_map->info.width;
  mapping_png.height = ptr_map->info.height;
  mapping_png.origin_x = ptr_map->info.origin.position.x;
  mapping_png.origin_y = ptr_map->info.origin.position.y;
  mapping_png.resolution = ptr_map->info.resolution;
  mapping_png.png_image = base64_str;
  mapping_png.file_size = base64_str.size();

  // shunping **************************************************************
  // std::string mapdatafile = "./mapping.pgm";
  // FILE *out = fopen(mapdatafile.c_str(), "w");

  // if (!out) {
  //   LOG(ERROR) << "Couldn't open file  " << mapdatafile.c_str() << std::endl;
  //   return false;
  // }

  // if (ptr_map == nullptr) {
  //   LOG(ERROR) << "ptr_map is nullptr";
  //   fclose(out);
  //   return false;
  // }

  // fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
  //         ptr_map->info.resolution, ptr_map->info.width,
  //         ptr_map->info.height);

  // unsigned int x, y, i;
  // for (y = 0; y < ptr_map->info.height; ++y) {
  //   for (x = 0; x < ptr_map->info.width; ++x) {
  //     i = x + (ptr_map->info.height - y - 1) * ptr_map->info.width;
  //     if (ptr_map->data[i] >= 0 && ptr_map->data[i] <= threshold_free) {
  //       fputc(254, out);
  //     } else if (ptr_map->data[i] >= threshold_occupied) {
  //       fputc(000, out);
  //     } else {
  //       fputc(205, out);
  //     }
  //   }
  // }

  // fflush(out);
  // fclose(out);

  // std::ifstream image_fstream;
  // std::stringstream map_png_ss;
  // boost::filesystem::path pgm_file = "./mapping.pgm";
  // if (boost::filesystem::exists(pgm_file.string())) {
  //   cv::Mat img = cv::imread(pgm_file.string());
  //   if (img.rows <= 0) {
  //     LOG(ERROR) << "img row <= 0";
  //     boost::filesystem::remove(pgm_file.string());
  //     return false;
  //   }
  //   boost::filesystem::remove(pgm_file.string());

  //   cv::imwrite(pgm_file.replace_extension("png").string(), img);
  //   image_fstream.open(pgm_file.string(), std::ios::in | std::ios::binary);
  //   if (!image_fstream.is_open()) {
  //     LOG(ERROR) << "open file" << pgm_file.string() << " fail!";
  //     boost::filesystem::remove(pgm_file.string());
  //     return false;
  //   }
  //   map_png_ss << image_fstream.rdbuf();
  //   image_fstream.close();

  //   std::string base64_str;
  //   base64Encode(map_png_ss.str(), base64_str);
  //   mapping_png.width = ptr_map->info.width;
  //   mapping_png.height = ptr_map->info.height;
  //   mapping_png.origin_x = ptr_map->info.origin.position.x;
  //   mapping_png.origin_y = ptr_map->info.origin.position.y;
  //   mapping_png.resolution = ptr_map->info.resolution;
  //   mapping_png.png_image = base64_str;
  //   mapping_png.file_size = base64_str.size();

  //   boost::filesystem::remove(pgm_file.string());
  // }

  return true;
}

bool getPngFromDepthOccupancyGrid(
    const slam2d_core::occupancy_map::UserOccupancyGrid &globalmap,
    chassis_interfaces::msg::MappingPng &mapping_png,
    const std::string &map_temp_path) {
  const int threshold_free = 49;
  const int threshold_occupied = 55;

  if (0 == globalmap.data.size()) {
    LOG(ERROR) << "globalmap is empty";
    return false;
  }

  unsigned int x, y, i;
  cv::Mat png_grey_mat =
      cv::Mat::zeros(globalmap.info.height, globalmap.info.width,
                     CV_8UC1);  // 8位无符号灰度图像

  for (y = 0; y < globalmap.info.height; ++y) {
    for (x = 0; x < globalmap.info.width; ++x) {
      i = x + (globalmap.info.height - y - 1) * globalmap.info.width;
      if (globalmap.data[i] >= 0 && globalmap.data[i] <= threshold_free) {
        png_grey_mat.at<unsigned char>(y, x) = (unsigned char) 254;
      } else if (globalmap.data[i] >= threshold_occupied) {
        png_grey_mat.at<unsigned char>(y, x) = (unsigned char) 0;
      } else {
        png_grey_mat.at<unsigned char>(y, x) = (unsigned char) 205;
      }
    }
  }

  if (png_grey_mat.rows <= 0) {
    LOG(ERROR) << "image_rgb_png row <= 0";
    return false;
  }
  cv::imwrite(map_temp_path, png_grey_mat);

  std::ifstream image_fstream;
  std::stringstream map_png_ss;
  image_fstream.open(map_temp_path, std::ios::in | std::ios::binary);
  if (!image_fstream.is_open()) {
    LOG(ERROR) << " open file" << map_temp_path << " fail!";
    boost::filesystem::remove(map_temp_path);
    return false;
  }
  map_png_ss << image_fstream.rdbuf();
  image_fstream.close();
  slam2d_core::occupancy_map::Pose2d laser_pose =
      slam2d_core::occupancy_map::Mathbox::Mat34d2Pose2d(globalmap.info.origin);

  std::string base64_str;
  base64Encode(map_png_ss.str(), base64_str);
  mapping_png.width = globalmap.info.width;
  mapping_png.height = globalmap.info.height;
  mapping_png.origin_x = laser_pose.x();
  mapping_png.origin_y = laser_pose.y();
  mapping_png.resolution = globalmap.info.resolution;
  mapping_png.png_image = base64_str;
  mapping_png.file_size = base64_str.size();

  return true;
}

}  // namespace slam2d_ros2