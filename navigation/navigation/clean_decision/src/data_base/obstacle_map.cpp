/*
 * @Author: your name
 * @Date: 2021-10-21 10:57:59
 * @LastEditTime: 2021-10-21 16:52:59
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /navigation/navigation/clean_decision/src/obstacle_map.cpp
 */
#include "data_base/obstacle_map.hpp"
#include "glog/logging.h"
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "SDL/SDL_image.h"
#include <sys/file.h>
#include <libgen.h>

namespace CVTE_BABOT {
ObstacleMap::ObstacleMap()
    : d_origin_x_(0.0),
      d_origin_y_(0.0),
      d_resolution_(0.1),
      ui_size_x_(0),
      ui_size_y_(0),
      uc_default_value_(0),
      free_thresh_(0),
      occupied_thresh_(0) {
  obstacle_map_.clear();
}

ObstacleMap::ObstacleMap(const ObstacleMap &map) {
  std::lock_guard<std::recursive_mutex> lock(map_mutex_);
  ui_size_x_ = map.getSizeInCellsX();
  ui_size_y_ = map.getSizeInCellsY();
  d_resolution_ = map.getResolution();
  d_origin_x_ = map.getOriginX();
  d_origin_y_ = map.getOriginY();
  obstacle_map_ = map.getCharMap();
}
ObstacleMap::ObstacleMap(unsigned int ui_size_x, unsigned int ui_size_y,
                         double d_resolution, double d_origin_x,
                         double d_origin_y, unsigned char uc_default_value)
    : ui_size_x_(ui_size_x),
      ui_size_y_(ui_size_y),
      d_resolution_(d_resolution),
      d_origin_x_(d_origin_x),
      d_origin_y_(d_origin_y),
      uc_default_value_(uc_default_value) {
  initMaps(ui_size_x_, ui_size_y_);
  resetMaps();
  for (auto &data : obstacle_map_) { data = uc_default_value; }
}

ObstacleMap &ObstacleMap::operator=(const ObstacleMap &map) {
  if (this == &map) {
    return *this;
  }
  std::lock_guard<std::recursive_mutex> lock(map_mutex_);
  ui_size_x_ = map.getSizeInCellsX();
  ui_size_y_ = map.getSizeInCellsY();
  d_resolution_ = map.getResolution();
  d_origin_x_ = map.getOriginX();
  d_origin_y_ = map.getOriginY();
  obstacle_map_ = map.getCharMap();
}

bool ObstacleMap::readMap(const std::string &map_path) {
  map_filename_ = map_path;
  std::ifstream fin(map_path.c_str());
  if (fin.fail()) {
    LOG(ERROR) << "Could not open '" + map_path + "': file not found"
               << std::endl;
    return false;
  }
  YAML::Node doc = YAML::LoadFile(map_path);

  try {
    map_filename_ = doc["image"].as<std::string>();
    if (0 == map_filename_.size()) {
      LOG(ERROR) << "The image tag cannot be an empty string" << std::endl;
      return false;
    }

    if (map_filename_[0] != '/') {
      char *fname_copy = strdup(map_path.c_str());
      map_filename_ = std::string(dirname(fname_copy)) + '/' + map_filename_;
      free(fname_copy);
    }
  } catch (YAML::Exception) {
    LOG(ERROR) << "'" + map_path +
                      "' does not contain an image tag or it is invalid"
               << std::endl;
    return false;
  }

  std::string map_type;
  try {
    map_type = doc["map_type"].as<std::string>();
  } catch (YAML::Exception) { map_type = "occupancy"; }

  if ("occupancy" != map_type) {
    LOG(ERROR) << "Cannot load unknown map type: '" + map_type + "'"
               << std::endl;
    return false;
  }

  try {
    d_resolution_ = doc["resolution"].as<double>();
  } catch (YAML::Exception) {
    LOG(ERROR) << "The map does not contain a resolution tag or it is invalid"
               << std::endl;
    return false;
  }

  try {
    d_origin_x_ = doc["origin"][0].as<double>();
    d_origin_y_ = doc["origin"][1].as<double>();
  } catch (YAML::Exception) {
    LOG(ERROR) << "The map does not contain an origin tag or it is invalid"
               << std::endl;
    return false;
  }

  double free_thresh;
  try {
    free_thresh = doc["free_thresh"].as<double>();
    free_thresh_ = 255 * free_thresh;
  } catch (YAML::Exception) {
    LOG(ERROR) << "The map does not contain a free_thresh tag or it is invalid"
               << std::endl;
    return false;
  }

  double occupied_thresh;
  try {
    occupied_thresh = doc["occupied_thresh"].as<double>();
    occupied_thresh_ = 255 * occupied_thresh;
  } catch (YAML::Exception) {
    LOG(ERROR)
        << "The map does not contain an occupied_thresh tag or it is invalid"
        << std::endl;
    return false;
  }

  SDL_Surface *img;
  if (!(img = IMG_Load(map_filename_.c_str()))) {
    LOG(ERROR) << "failed to open image file: " + map_filename_ << std::endl;
    return false;
  }

  ui_size_x_ = img->w;
  ui_size_y_ = img->h;
  // origin_x_ += size_x_ / 2 * scale_;
  // origin_y_ += size_y_ / 2 * scale_;
  LOG(INFO) << "reading map: " << ui_size_x_ << "*" << ui_size_y_
            << "  scale:" << d_resolution_ << "  origin:(" << d_origin_x_ << ","
            << d_origin_y_ << ")";
  // init the cells_
  obstacle_map_.resize(ui_size_x_ * ui_size_y_);

  // Get values that we'll need to iterate through the pixels
  int rowstride = img->pitch;
  int n_channels = img->format->BytesPerPixel;

  // NOTE: Trinary mode still overrides here to preserve existing behavior.
  // Alpha will be averaged in with color channels when using trinary mode.
  int avg_channels = n_channels;

  // Copy pixel data into the map structure
  unsigned char *pixels = (unsigned char *) (img->pixels);
  int color_sum;

  unsigned char *p = nullptr;
  int color_avg = 0.0;
  double occ = 0.0;
  for (unsigned int j = 0; j < ui_size_y_; j++) {
    for (unsigned int i = 0; i < ui_size_x_; i++) {
      p = pixels + j * rowstride + i * n_channels;
      color_sum = 0;
      for (int k = 0; k < avg_channels; k++) { color_sum += *(p + (k)); }
      color_avg = color_sum / avg_channels;
      // unsigned int index = getIndex(i, j);
      unsigned char value = 255 - color_avg;
      if (value > free_thresh_) {
        setMapPointValue(GridmapPoint(i, ui_size_y_ - j - 1), 254);
      } else {
        setMapPointValue(GridmapPoint(i, ui_size_y_ - j - 1), 0);
      }
      // occ = (255 - color_avg) / 255.0;
      // if (occ >= occupied_thresh) {
      //   cells_[i][size_y_ - j - 1].occ_state = +1;
      // } else if (occ <= free_thresh) {
      //   cells_[i][size_y_ - j - 1].occ_state = -1;
      // } else {
      //   cells_[i][size_y_ - j - 1].occ_state = 0;
      // }
    }
  }
  SDL_FreeSurface(img);
  return true;
}

bool ObstacleMap::saveMap(const std::string &map_path) {
  // LOG(INFO) << "Received a " << map.info.width << "X" << map.info.height
  //           << "map " << map.info.resolution << "m/pix";
  // const int threshold_free = 49;
  // const int threshold_occupied = 55;
  if (obstacle_map_.empty() || ui_size_x_ == 0 || ui_size_y_ == 0) {
    LOG(ERROR) << "no data to save";
    return false;
  }
  std::string mapdatafile = map_path + ".pgm";

  LOG(INFO) << "Writing map occupancy data to " << mapdatafile;
  FILE *out = fopen(mapdatafile.c_str(), "w");
  if (!out) {
    LOG(ERROR) << "Couldn't save map file to " << mapdatafile;
    return false;
  }

  fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
          d_resolution_, ui_size_x_, ui_size_y_);
  LOG(INFO) << "map info: " << std::endl
            << "resolution: " << d_resolution_ << std::endl
            << "origin_x: " << d_origin_x_ << std::endl
            << "origin_y: " << d_origin_y_ << std::endl
            << "free_thresh: " << free_thresh_ << std::endl
            << "occupy_thresh: " << occupied_thresh_ << std::endl
            << "size_x: " << ui_size_x_ << std::endl
            << "size_y: " << ui_size_y_;
  for (unsigned int y = 0; y < ui_size_y_; y++) {
    for (unsigned int x = 0; x < ui_size_x_; x++) {
      // unsigned int i = x + (ui_size_y_ - y - 1) * ui_size_x_;
      unsigned int i = getIndex(x, ui_size_y_ - y - 1);
      if (obstacle_map_[i] >= 0 && obstacle_map_[i] <= free_thresh_) {
        // [0,free)
        fputc(254, out);
      } else if (obstacle_map_[i] >= occupied_thresh_) {
        // (occ,255]
        fputc(000, out);
      } else {
        //  occ[0.25,0.65]
        fputc(205, out);
      }
    }
  }

  fclose(out);

  std::string mapmetadatafile = map_path + ".yaml";
  LOG(INFO) << "Writing map occupancy data to " << mapmetadatafile;
  FILE *yaml = fopen(mapmetadatafile.c_str(), "w");
  flock(yaml->_fileno, LOCK_EX);

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

  // Pose2d laser_pose = Mathbox::Mat34d2Pose2d(map.info.origin);

  // TODO: use ours Math tools
  fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\n",
          mapdatafile.c_str(), d_resolution_, d_origin_x_, d_origin_y_, 0.0);
  fprintf(yaml, "height: %d\nwidth: %d\n", ui_size_y_, ui_size_x_);
  fprintf(yaml, "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n");

  flock(yaml->_fileno, LOCK_UN);
  fclose(yaml);
  return true;
}

void ObstacleMap::resetMaps() {
  std::lock_guard<std::recursive_mutex> lock(map_mutex_);
  // obstacle_map_.clear();
  for (size_t index = 0; index < obstacle_map_.size(); index++) {
    obstacle_map_[index] = 0;
  }
}

void ObstacleMap::initMaps(unsigned int ui_size_x, unsigned int ui_size_y) {
  std::lock_guard<std::recursive_mutex> lock(map_mutex_);
  obstacle_map_.clear();
  obstacle_map_.resize(ui_size_x * ui_size_y);
}

void ObstacleMap::resizeMap(unsigned int ui_size_x, unsigned int ui_size_y,
                            double d_resolution, double d_origin_x,
                            double d_origin_y) {
  std::lock_guard<std::recursive_mutex> lock(map_mutex_);
  ui_size_x_ = ui_size_x;
  ui_size_y_ = ui_size_y;
  d_resolution_ = d_resolution;
  d_origin_x_ = d_origin_x;
  d_origin_y_ = d_origin_y;
  initMaps(ui_size_x, ui_size_y);
  resetMaps();
}

void ObstacleMap::mapToWorld(const GridmapPoint &cp_point,
                             GlobalmapPoint &wm_point) const {
  wm_point(0) = d_origin_x_ + (cp_point(0) + 0.5) * d_resolution_;
  wm_point(1) = d_origin_y_ + (cp_point(1) + 0.5) * d_resolution_;
}

bool ObstacleMap::worldToMap(const GlobalmapPoint &wm_point,
                             GridmapPoint &cp_point) const {
  if (wm_point(0) < d_origin_x_ || wm_point(1) < d_origin_y_) {
    return false;
  }
  cp_point(0) = static_cast<int>((wm_point(0) - d_origin_x_) / d_resolution_);
  cp_point(1) = static_cast<int>((wm_point(1) - d_origin_y_) / d_resolution_);
  if (cp_point(0) < ui_size_x_ && cp_point(1) < ui_size_y_) {
    return true;
  }
  return false;
}

void ObstacleMap::worldToMapNoBounds(const GlobalmapPoint &wm_point,
                                     GridmapPoint &cp_point) const {
  cp_point(0) = static_cast<int>((wm_point(0) - d_origin_x_) / d_resolution_);
  cp_point(1) = static_cast<int>((wm_point(1) - d_origin_y_) / d_resolution_);
}

void ObstacleMap::worldToMapEnforceBounds(const GlobalmapPoint &wm_point,
                                          GridmapPoint &cp_point) const {
  if (wm_point(0) < d_origin_x_) {
    cp_point(0) = 0;
  } else if (wm_point(0) > d_resolution_ * ui_size_x_ + d_origin_x_) {
    cp_point(0) = ui_size_x_ - 1;
  } else {
    cp_point(0) = static_cast<int>((wm_point(0) - d_origin_x_) / d_resolution_);
  }

  if (wm_point(1) < d_origin_y_) {
    cp_point(1) = 0;
  } else if (wm_point(1) > d_resolution_ * ui_size_y_ + d_origin_y_) {
    cp_point(1) = ui_size_y_ - 1;
  } else {
    cp_point(1) = static_cast<int>((wm_point(1) - d_origin_y_) / d_resolution_);
  }
}

}  // namespace CVTE_BABOT