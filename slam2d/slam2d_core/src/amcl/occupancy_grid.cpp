#include "amcl/occupancy_grid.hpp"

#include <glog/logging.h>
#include <libgen.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>

#include "SDL/SDL_image.h"

namespace slam2d_core {
namespace amcl {

OccupancyGrid::OccupancyGrid() {}

OccupancyGrid::OccupancyGrid(const std::string &map_path,
                             const double max_occ_dist) {
  readFromFile(map_path);
  updataCspace(max_occ_dist);
}

OccupancyGrid::~OccupancyGrid() {
  clear();
}

void OccupancyGrid::clear() {
  cells_.clear();
}

bool OccupancyGrid::readFromFile(const std::string &map_path) {
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
    scale_ = doc["resolution"].as<double>();
  } catch (YAML::Exception) {
    LOG(ERROR) << "The map does not contain a resolution tag or it is invalid"
               << std::endl;
    return false;
  }

  try {
    origin_x_ = doc["origin"][0].as<double>();
    origin_y_ = doc["origin"][1].as<double>();
    origin_a_ = doc["origin"][2].as<double>();
  } catch (YAML::Exception) {
    LOG(ERROR) << "The map does not contain an origin tag or it is invalid"
               << std::endl;
    return false;
  }

  double free_thresh;
  try {
    free_thresh = doc["free_thresh"].as<double>();
  } catch (YAML::Exception) {
    LOG(ERROR) << "The map does not contain a free_thresh tag or it is invalid"
               << std::endl;
    return false;
  }

  double occupied_thresh;
  try {
    occupied_thresh = doc["occupied_thresh"].as<double>();
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

  size_x_ = img->w;
  size_y_ = img->h;
  // origin_x_ += size_x_ / 2 * scale_;
  // origin_y_ += size_y_ / 2 * scale_;
  LOG(INFO) << "reading map: " << size_x_ << "*" << size_y_
            << "  scale:" << scale_ << "  origin:(" << origin_x_ << ","
            << origin_y_ << ")";
  // init the cells_
  cells_.resize(size_x_);
  for (unsigned int i = 0; i < size_x_; i++) { cells_[i].resize(size_y_); }

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
  double color_avg = 0.0;
  double occ = 0.0;
  for (unsigned int j = 0; j < size_y_; j++) {
    for (unsigned int i = 0; i < size_x_; i++) {
      p = pixels + j * rowstride + i * n_channels;
      color_sum = 0;
      for (int k = 0; k < avg_channels; k++) { color_sum += *(p + (k)); }
      color_avg = color_sum / static_cast<double>(avg_channels);
      occ = (255 - color_avg) / 255.0;
      if (occ >= occupied_thresh) {
        cells_[i][size_y_ - j - 1].occ_state = +1;
      } else if (occ <= free_thresh) {
        cells_[i][size_y_ - j - 1].occ_state = -1;
      } else {
        cells_[i][size_y_ - j - 1].occ_state = 0;
      }
    }
  }
  SDL_FreeSurface(img);
  return true;
}

Cell OccupancyGrid::getCell(const double x, const double y) {
  unsigned int index_x = getMapCoordX(x);
  unsigned int index_y = getMapCoordY(y);
  if (mapValid(index_x, index_y)) {
    return cells_[index_x][index_y];
  }
  Cell cell;
  cell.occ_state = 0;
  cell.occ_dist = max_occ_dist_;
  // LOG(WARNING) << "get cell out of the map: " << x << " " << y << std::endl;
  return cell;
}

void OccupancyGrid::enqueue(const unsigned int i, const unsigned int j,
                            const unsigned int src_i, const unsigned int src_j,
                            std::vector<std::vector<double>> &cdm,
                            std::priority_queue<CellData> &Q,
                            std::vector<std::vector<unsigned char>> &marked) {
  unsigned int di;
  unsigned int dj;
  if (marked[i][j] == 1) {
    return;
  }
  if (i > src_i) {
    di = i - src_i;
  } else {
    di = src_i - i;
  }
  if (j > src_j) {
    dj = j - src_j;
  } else {
    dj = src_j - j;
  }

  double distance = cdm[di][dj];

  if (distance > cell_redius_) {  // this not a distance in real world, but in
    // the map image
    return;
  }
  cells_[i][j].occ_dist = distance * scale_;

  CellData cell;
  cell.occ_dist = cells_[i][j].occ_dist;
  cell.i = i;
  cell.j = j;
  cell.src_i = src_i;
  cell.src_j = src_j;
  Q.push(cell);

  if (!mapValid(i, j)) {
    LOG(ERROR) << "enqueue: parameter error";
  } else {
    marked[i][j] = 1;
  }
}

bool OccupancyGrid::updataCspace(const double max_occ_dist) {
  if (max_occ_dist <= 0) {
    LOG(ERROR) << "parameter max_occ_dist <0";
    return false;
  }
  max_occ_dist_ = max_occ_dist;
  cell_redius_ = max_occ_dist_ / scale_;
  std::priority_queue<CellData> Q;
  std::vector<std::vector<unsigned char>> marked;
  marked.resize(size_x_);
  for (unsigned int i = 0; i < size_x_; i++) { marked[i].resize(size_y_, 0); }

  std::vector<std::vector<double>> cdm;
  cdm.resize(cell_redius_ + 2);
  for (int i = 0; i < cell_redius_ + 2; i++) {
    cdm[i].resize(cell_redius_ + 2);
    for (int j = 0; j < cell_redius_ + 2; j++) {
      cdm[i][j] = std::sqrt(i * i + j * j);
    }
  }

  CellData cell;
  for (unsigned int i = 0; i < size_x_; i++) {
    cell.src_i = cell.i = i;
    for (unsigned int j = 0; j < size_y_; j++) {
      if (cells_[i][j].occ_state == +1) {
        cells_[i][j].occ_dist = 0.0;
        cell.occ_dist = 0.0;
        cell.src_j = cell.j = j;
        marked[i][j] = 1;
        Q.push(cell);
      } else {
        cells_[i][j].occ_dist = max_occ_dist;
      }
    }
  }

  if (Q.empty()) {
    LOG(WARNING) << " not occpuy cells";
    return false;
  }

  while (!Q.empty()) {
    CellData current_cell = Q.top();
    if (current_cell.i > 0) {
      enqueue(current_cell.i - 1, current_cell.j, current_cell.src_i,
              current_cell.src_j, cdm, Q, marked);
    }
    if (current_cell.j > 0) {
      enqueue(current_cell.i, current_cell.j - 1, current_cell.src_i,
              current_cell.src_j, cdm, Q, marked);
    }
    if (static_cast<unsigned int>(current_cell.i) < size_x_ - 1) {
      enqueue(current_cell.i + 1, current_cell.j, current_cell.src_i,
              current_cell.src_j, cdm, Q, marked);
    }
    if (static_cast<unsigned int>(current_cell.j) < size_y_ - 1) {
      enqueue(current_cell.i, current_cell.j + 1, current_cell.src_i,
              current_cell.src_j, cdm, Q, marked);
    }
    Q.pop();
  }
  return true;
}

bool OccupancyGrid::isInFreeSpace(const double x, const double y) {
  if (getCell(x, y).occ_state == -1) {
    return true;
  } else {
    return false;
  }
}

double OccupancyGrid::mapClacRange(const double ox, const double oy,
                                   const double oyaw, const double max_range) {
  int x0 = getMapCoordX(ox);
  int y0 = getMapCoordY(oy);
  int x1 = getMapCoordX(ox + max_range * std::cos(oyaw));
  int y1 = getMapCoordY(oy + max_range * std::sin(oyaw));

  bool steep;
  if (std::abs(y1 - y0) > std::abs(x1 - x0)) {
    steep = true;
  } else {
    steep = false;
  }

  int tmp;
  if (steep) {
    tmp = x0;
    x0 = y0;
    y0 = tmp;

    tmp = x1;
    x1 = y1;
    y1 = tmp;
  }

  int deltax = std::abs(x1 - x0);
  int deltay = std::abs(y1 - y0);

  int error = 0;
  int deltaerr = deltay;
  int x = x0;
  int y = y0;

  int xstep;
  int ystep;

  if (x0 < x1) {
    xstep = 1;
  } else {
    xstep = -1;
  }

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  if (steep) {
    if (!mapValid(y, x) || cells_[y][x].occ_state > -1) {
      return std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * scale_;
    }
  } else {
    if (!mapValid(x, y) || cells_[x][y].occ_state > -1) {
      return std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * scale_;
    }
  }

  while (x != x1 + xstep) {
    x += xstep;
    error += deltaerr;
    if (2 * error >= deltax) {
      y += ystep;
      error -= deltax;
    }

    if (steep) {
      if (!mapValid(y, x) || cells_[y][x].occ_state > -1) {
        return std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * scale_;
      }
    } else {
      if (!mapValid(x, y) || cells_[x][y].occ_state > -1) {
        return std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * scale_;
      }
    }
  }
  return max_range;
}
double OccupancyGrid::getMaxOccDist() {
  return max_occ_dist_;
}

}  // namespace amcl
}  // namespace slam2d_core