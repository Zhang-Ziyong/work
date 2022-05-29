#include "amcl/rbnn_cluster.hpp"
#include <math.h>

namespace slam2d_core {
namespace amcl {

unsigned int RBNN::findNeighbors(std::vector<int> &nidx,
                                 const unsigned int &idx,
                                 const std::vector<rbnn_point_t> &pl,
                                 const double &r, const unsigned int &nbn,
                                 const double &ANGLE_INC) {
  unsigned int si, ei;
  if (idx < nbn) {
    si = 0;
  } else {
    si = idx - nbn;
  }
  if (idx + nbn > pl.size()) {
    ei = pl.size();
  } else {
    ei = idx + nbn;
  }
  double distance;
  double dist_thd;
  double dmin = std::sqrt(pl[idx].x * pl[idx].x + pl[idx].y * pl[idx].y);

  unsigned int indx_count = 0;
  for (unsigned int i = si; i < ei; i++) {
    distance = std::sqrt((pl[i].x - pl[idx].x) * (pl[i].x - pl[idx].x) +
                         (pl[i].y - pl[idx].y) * (pl[i].y - pl[idx].y));

    dist_thd = r + dmin * tan(ANGLE_INC);

    if (distance <= dist_thd) {
      nidx.push_back(i);
      indx_count++;
    }
  }
  return indx_count;
}

void RBNN::mergeCluster(std::vector<int> &lbs, const int &l1, const int &l2) {
  if (l1 < l2) {
    for (unsigned int i = 0; i < lbs.size(); i++) {
      if (lbs[i] == l1) {
        lbs[i] = l2;
      }
    }
  } else {
    for (unsigned int i = 0; i < lbs.size(); i++) {
      if (lbs[i] == l2) {
        lbs[i] = l1;
      }
    }
  }
}

void RBNN::cluster(const std::vector<rbnn_point_t> &pl, int &clusters,
                   std::vector<int> &lbs, const double &r,
                   const int &min_cluster_num, const double &ANGLE_INC) {
  unsigned int pn = pl.size();
  unsigned int cid = 0;

  std::vector<int> nidx;
  double nb_size = 0;
  for (unsigned int i = 0; i < pn; i++) {
    nb_size = findNeighbors(nidx, i, pl, r, 5, ANGLE_INC);

    int j;
    for (unsigned int ni = 0; ni < nb_size; ni++) {
      j = nidx[ni];
      if (lbs[i] >= 0 && lbs[j] >= 0) {
        if (lbs[i] != lbs[j]) {
          mergeCluster(lbs, lbs[i], lbs[j]);
        }
      } else {
        if (lbs[j] >= 0) {
          lbs[i] = lbs[j];
        } else if (lbs[i] >= 0) {
          lbs[j] = lbs[i];
        }
      }
    }
    if (lbs[i] < 0) {
      lbs[i] = cid;
      cid++;
      for (unsigned int j = 0; j < nidx.size(); j++) { lbs[nidx[j]] = lbs[i]; }
    }
    nidx.clear();
  }
  std::vector<int> C;
  C.resize(cid, 0);
  for (unsigned int i = 0; i < lbs.size(); i++) { C[lbs[i]]++; }
  for (unsigned int i = 0; i < lbs.size(); i++) {
    if (C[lbs[i]] < min_cluster_num) {
      lbs[i] = -2;
    }
  }
  clusters = cid;
}

}  // namespace amcl
}  // namespace slam2d_core