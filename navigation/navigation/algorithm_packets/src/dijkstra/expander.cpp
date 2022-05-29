/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file expander.cpp
 *
 *@brief 搜索算法的基类实现
 *
*@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version Navigation-v2.0
 *@data 2019-12-17
 ************************************************************************/

#include "dijkstra/expander.hpp"
#include <glog/logging.h>
#include <math.h>
#include <iostream>


namespace CVTE_BABOT {

Expander::Expander(const unsigned int &nx, const unsigned int &ny){
    if (nx <= 0 || ny <= 0) {
        LOG(FATAL) << "[Expander] Constructer receive empty value.";
    }
    setSize(nx, ny);
}

void Expander::setSize(const unsigned int &nx, const unsigned int &ny){
    nx_ = nx;
    ny_ = ny;
    ns_ = nx * ny;
    if (gradx_.get() == NULL){
        gradx_ = boost::shared_array<float>(new float[nx * ny]);
    }
    if (grady_.get() == NULL){
        grady_ = boost::shared_array<float>(new float[nx * ny]);
    }
    if (potential_.get() == NULL) {
        potential_ = boost::shared_array<float>(new float[nx * ny]);
    }
    assert(gradx_.get() != NULL);
    assert(grady_.get() != NULL);
    assert(potential_.get() != NULL);
}

float Expander::gradCell(const unsigned int &n){
    // 判断n索引值是否超出地图边界
    if (n < nx_ || n > nx_ * ny_ - nx_){
        return 0.0;
    }
    
    if (gradx_[n] + grady_[n] > 0.0){
        return 1.0;
    }

    float cv = potential_[n];
    float dx = 0.0;
    float dy = 0.0;

    // 检查是否在障碍物里
    if (cv >= POT_HIGH){
        if (potential_[n - 1] < POT_HIGH) {
            dx = -lethal_cost_;
        } else if (potential_[n + 1] < POT_HIGH) {
            dx = lethal_cost_;
        }
        if (potential_[n - nx_] < POT_HIGH) {
            dy = -lethal_cost_;
        } else if (potential_[n + nx_] < POT_HIGH) {
            dy = lethal_cost_;
        }
    } else {
        if (potential_[n - 1] < POT_HIGH) {
            dx += potential_[n - 1] - cv;
        } else if (potential_[n + 1] < POT_HIGH) {
            dx += cv - potential_[n + 1];
        }
        if (potential_[n - nx_] < POT_HIGH) {
            dy += potential_[n - nx_] - cv;
        } else if (potential_[n + nx_] < POT_HIGH) {
            dy += cv - potential_[n + nx_];
        }
    }

    // 归一化
    float norm = hypot(dx, dy);
    if (norm > 0.0) {
        norm = 1.0 / norm;
        gradx_[n] = norm * dx;
        grady_[n] = norm * dy;
    }
    return norm;
}

bool Expander::getPath(const double &start_x, const double &start_y,
                    const double &goal_x, const double &goal_y,
                    std::vector<Pose2d> &path){
    Pose2d current;
    unsigned int stc = toIndex(goal_x, goal_y);

    float dx = goal_x - floorf(goal_x); // 目标点小数点后的偏移量
    float dy = goal_y - floorf(goal_y);
    int ns = nx_ * ny_;
    memset(gradx_.get(), 0.0, ns * sizeof(float)); // 将梯度置零
    memset(grady_.get(), 0.0, ns * sizeof(float));

    // 下面的变量是一些for循环时的临时变量，避免重复申请，提前声明
    double nx = 0.0; // 记录路径点的坐标
    double ny = 0.0;
    bool oscillation_detected = false; // 摆动检测
    int npath = 0; // 当前路径长度
    int stcnx = 0;
    int stcpx = 0;
    int minc = 0;
    int minp = 0;
    int st = 0;
    float x1 = 0.0; // 梯度计算的一些临时变量
    float x2 = 0.0;
    float x = 0.0;
    float y1 = 0.0;
    float y2 = 0.0;
    float y = 0.0;
    float ss = 0.0;

    for (int c = 0; c < ns * 4; c++){
        nx = stc % nx_ + dx;
        ny = stc / nx_ + dy;
        if (fabs(nx - start_x) <= 1.0 && fabs(ny - start_y) <= 1.0) {
            // current.setX(start_x);
            // current.setY(start_y);
            // path.push_back(current);
            return true;
        }
        
        // 检索越界
        if (stc < nx_ || stc > nx_ * ny_ - nx_) {
            LOG(ERROR) << "[Expander] Out of Bounds";
            return false;
        }
        current.setX(nx);
        current.setY(ny);
        path.push_back(current);
        oscillation_detected = false;
        npath = path.size();
        if (npath > 2 && path[npath - 1].getX() == path[npath - 3].getX() &&
          path[npath - 1].getY() == path[npath - 3].getY()){
            oscillation_detected = true;        
        }

        stcnx = stc + nx_;
        stcpx = stc - nx_;

        // 在起点与终点的时候，附近会有势场很大的值，会进入if语句
        if (potential_[stc] >= POT_HIGH || potential_[stc + 1] >= POT_HIGH ||
          potential_[stc - 1] >= POT_HIGH || potential_[stcnx] >= POT_HIGH ||
          potential_[stcnx + 1] >= POT_HIGH || potential_[stcnx - 1] >= POT_HIGH ||
          potential_[stcpx] >= POT_HIGH || potential_[stcpx + 1] >= POT_HIGH ||
          potential_[stcpx - 1] >= POT_HIGH || oscillation_detected) { 
            // 检查八个方向寻找梯度最小的
            minc = stc;
            minp = potential_[stc];
            st = stcpx - 1;
            if (potential_[st] < minp) {
                minp = potential_[st];
                minc = st;
            }
            st++;
            if (potential_[st] < minp) {
                minp = potential_[st];
                minc = st;
            }
            st++;
            if (potential_[st] < minp) {
                minp = potential_[st];
                minc = st;
            }
            st = stc - 1;
            if (potential_[st] < minp) {
                minp = potential_[st];
                minc = st;
            }
            st = stc + 1;
            if (potential_[st] < minp) {
                minp = potential_[st];
                minc = st;
            }
            st = stcnx - 1;
            if (potential_[st] < minp) {
                minp = potential_[st];
                minc = st;
            }
            st++;
            if (potential_[st] < minp) {
                minp = potential_[st];
                minc = st;
            }
            st++;
            if (potential_[st] < minp) {
                minp = potential_[st];
                minc = st;
            }
            stc = minc;
            dx = 0;
            dy = 0;
            if (potential_[stc] >= POT_HIGH) {
                return false;
            }
        } else { //找到梯度更好的方向
            // 从四个位置获取梯度值
            gradCell(stc);
            gradCell(stc + 1);
            gradCell(stcnx);
            gradCell(stcnx + 1);

            // 梯度插值 
            x1 = (1.0 - dx) * gradx_[stc] + dx * gradx_[stc + 1];
            x2 = (1.0 - dx) * gradx_[stcnx] + dx * gradx_[stcnx + 1];
            x = (1.0 - dy) * x1 + dy * x2; // interpolated x
            y1 = (1.0 - dx) * grady_[stc] + dx * grady_[stc + 1];
            y2 = (1.0 - dx) * grady_[stcnx] + dx * grady_[stcnx + 1];
            y = (1.0 - dy) * y1 + dy * y2; // interpolated y

            // 检查是否存在0梯度
            if (x == 0.0 && y == 0.0) {
                return false;
            }

            ss = pathStep_ / hypot(x, y);
            dx += x * ss;
            dy += y * ss;
            // 检查边界，渐变值超过1.0米时，跳转到临近的栅格点
            if (dx > 1.0) {
                stc++;
                dx -= 1.0;
            }
            if (dx < -1.0) {
                stc--;
                dx += 1.0;
            }
            if (dy > 1.0) {
                stc += nx_;
                dy -= 1.0;
            }
            if (dy < -1.0) {
                stc -= nx_;
                dy += 1.0;
            }    
        }
    }
    return false;
}
}