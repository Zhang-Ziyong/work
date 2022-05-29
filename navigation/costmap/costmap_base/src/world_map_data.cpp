#include "world_map_data.hpp"
#include "SDL/SDL_image.h"
#include "glog/logging.h"
#include <fstream>
#include <iostream>
#include <libgen.h>
#include <yaml-cpp/yaml.h>

namespace CVTE_BABOT {
enum MAPMode { TRINARY, SCALE, RAW };

bool WorldmapData::readMapFromYaml(const std::string &map_path) {
  std::string map_filename = "";

  std::ifstream fin(map_path.c_str());
  if (fin.fail()) {
    LOG(ERROR) << "Could not open '" + map_path + "': file not found"
               << std::endl;
    return false;
  }
  YAML::Node doc = YAML::LoadFile(map_path);

  try {
    map_filename = doc["image"].as<std::string>();
    if (0 == map_filename.size()) {
      LOG(ERROR) << "The image tag cannot be an empty string" << std::endl;
      return false;
    }

    if (map_filename[0] != '/') {
      char *fname_copy = strdup(map_path.c_str());
      map_filename = std::string(dirname(fname_copy)) + '/' + map_filename;
      free(fname_copy);
    }
  } catch (YAML::Exception) {
    LOG(ERROR) << "'" + map_path +
                      "' does not contain an image tag or it is invalid"
               << std::endl;
    return false;
  }

  std::string map_type = "";
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

  // double origin_z = 0.0;
  try {
    origin_.d_x = doc["origin"][0].as<double>();
    origin_.d_y = doc["origin"][1].as<double>();
    // origin_z = doc["origin"][2].as<double>();
  } catch (YAML::Exception) {
    LOG(ERROR) << "The map does not contain an origin tag or it is invalid"
               << std::endl;
    return false;
  }

  double free_thresh = 0.0;
  try {
    free_thresh = doc["free_thresh"].as<double>();
  } catch (YAML::Exception) {
    LOG(ERROR) << "The map does not contain a free_thresh tag or it is invalid"
               << std::endl;
    return false;
  }

  double occupied_thresh = 0.0;
  try {
    occupied_thresh = doc["occupied_thresh"].as<double>();
  } catch (YAML::Exception) {
    LOG(ERROR)
        << "The map does not contain an occupied_thresh tag or it is invalid"
        << std::endl;
    return false;
  }

  std::string mode_str;
  MAPMode mode = MAPMode::TRINARY;

  try {
    mode_str = doc["mode"].as<std::string>();

    if (mode_str == "trinary") {
      mode = MAPMode::TRINARY;
    } else if (mode_str == "scale") {
      mode = MAPMode::SCALE;
    } else if (mode_str == "raw") {
      mode = MAPMode::RAW;
    } else {
      LOG(INFO) << "Mode parameter not recognized: '%s', using default value "
                   "(trinary)"
                << std::endl;
      mode = MAPMode::TRINARY;
    }
  } catch (YAML::Exception &) {
    LOG(ERROR) << "Mode parameter not set, using default value (trinary)"
               << std::endl;
    mode = MAPMode::TRINARY;
  }

  int negate = 0;
  try {
    negate = doc["negate"].as<int>();
  } catch (YAML::Exception) {
    LOG(ERROR) << "The map does not contain a negate tag or it is invalid"
               << std::endl;
  }

  SDL_Surface *img;
  if (!(img = IMG_Load(map_filename.c_str()))) {
    LOG(ERROR) << "failed to open image file: " + map_filename << std::endl;
    return false;
  }

  ui_width_ = img->w;
  ui_height_ = img->h;
  LOG(INFO) << "reading map: " << ui_width_ << "*" << ui_height_
            << "  resolution:" << d_resolution_ << "  origin:(" << origin_.d_x
            << "," << origin_.d_y << ")";
  // init the cells_
  ptr_data_->resize(ui_width_ * ui_height_);

  // Get values that we'll need to iterate through the pixels
  int rowstride = img->pitch;
  int n_channels = img->format->BytesPerPixel;

  // NOTE: Trinary mode still overrides here to preserve existing behavior.
  // Alpha will be averaged in with color channels when using trinary mode.
  int avg_channels;
  if (mode == MAPMode::TRINARY || !img->format->Amask) {
    avg_channels = n_channels;
  } else {
    avg_channels = n_channels - 1;
  }

  // Copy pixel data into the map structure
  unsigned char *pixels = (unsigned char *) (img->pixels);
  int color_sum;

  unsigned char *p;
  int alpha;
  for (unsigned int j = 0; j < ui_height_; j++) {
    for (unsigned int i = 0; i < ui_width_; i++) {
      p = pixels + j * rowstride + i * n_channels;
      color_sum = 0;
      for (int k = 0; k < avg_channels; k++) { color_sum += *(p + (k)); }
      double color_avg = color_sum / static_cast<double>(avg_channels);

      if (n_channels == 1) {
        alpha = 1;
      } else {
        alpha = *(p + n_channels - 1);
      }

      if (negate) {
        color_avg = 255 - color_avg;
      }

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
      unsigned char value;
      double occ = (255 - color_avg) / 255.0;
      // if (occ > occupied_thresh) {
      //   value = +100;
      // } else if (occ < free_thresh) {
      //   value = 0;
      // } else if (mode == MAPMode::TRINARY || alpha < 1.0) {
      //   value = -1;
      // } else {
      //   double ratio = (occ - free_thresh) / (occupied_thresh - free_thresh);
      //   value = 99 * ratio;
      // }
      if (occ > occupied_thresh) {
        value = +100;
      } else if (occ < free_thresh) {
        value = 0;
      } else {
        value = 10;
      }
      (*ptr_data_)[MAP_IDX(ui_width_, i, ui_height_ - j - 1)] = value;
    }
  }
  SDL_FreeSurface(img);
  return true;
}
}  // namespace CVTE_BABOT
