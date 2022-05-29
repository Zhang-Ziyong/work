/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file potential_calculator.hpp
 *
 *@brief 地图势场的计算方法
 *
 *@modified by Jiajun Liang(liangjiajun@cvte.com)
 *
 *@author Jiajun Liang(liangjiajun@cvte.com)
 *@version Navigation-v2.0
 *@data 2019-12-16
 ************************************************************************/

#ifndef __POTENTIAL_CALCULATOR_HPP
#define __POTENTIAL_CALCULATOR_HPP

#include <boost/smart_ptr.hpp>

namespace CVTE_BABOT{
/**
* PotentialCalculator
* @brief
*   用来计算地图的势场
* */

class PotentialCalculator {
public:
    PotentialCalculator() = default;
    PotentialCalculator(const PotentialCalculator &obj) = delete;
    PotentialCalculator &operator=(const PotentialCalculator &obj) = delete;

    /**
    * PotentialCalculator
    * @brief
    *   构造函数，设置势场的地图大小
    *
    * @param[in] nx-地图x轴长度
    * @param[in] ny-地图y轴长度
    * */
    PotentialCalculator(const int &nx, const int &ny);
    
    /**
    * setSize
    * @brief
    *   重新设置势场的地图大小
    *
    * @param[in] nx-地图x轴长度
    * @param[in] ny-地图y轴长度
    * */
    void setSize(const int &nx, const int &ny);

    /**
    * calculatePotential
    * @brief
    *   计算地图的势场值
    *
    * @param[in] potential-势场地图的内存
    * @param[in] cost-代价值
    * @param[in] n-二维数组索引的坐标
    * 
    * @return 需要计算点的当前时刻势场值
    * */
    float calculatePotential(boost::shared_array<float> potential, 
        const float &cost, const int &n);

    /**
    * toIndex
    * @brief
    *   二维坐标系到一维数组的索引转换
    *
    * @param[in] x-坐标点的x值
    * @param[in] y-坐标点的y值
    * */
    inline int toIndex(const int &x, const int &y) {return x + nx_ * y;}

private:
    int nx_ = 0; ///< 势场的长（单位：像素）
    int ny_ = 0; ///< 势场的宽（单位：像素）
    int ns_ = 0; ///< 势场的总大小（单位：像素）
};

}


#endif // end of __POTENTIAL_CALCULATOR_HPP