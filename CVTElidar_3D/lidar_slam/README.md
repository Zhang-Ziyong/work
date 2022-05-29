<!--
 * @FilePath: /src/lidar_slam/README.md
 * @brief: 
 *  
 * 
 * @author: Wei Zhang(zhangwei@cvte.com)
 * @version: v 1.0
 * @Date: 2020-04-21 09:28:13
--> 
## cvte_lidar_slam 
`cvte_lidar_slam`是一个多传感器融合的`lidar slam`算法包，本系统以多线激光雷达和里程计，以及可选的`GPS`作为输入，后续可能会加入图像的融合，它能实时输出高精度`6dof`的`pose`,用于机器人导航定位．　

#### **系统框架**     

<p align='center'>
    <img src="docs/flowchart.svg" alt="drawing" width="600"/>
</p>  

####  **内容**　
本系统采用了分离设计，即接口和算法分离，`ros_adapter`主要是系统对外`ros`的接口，`slam_core`是核心的算法部分，`params`是参数配置文件；slam算法根据功能划分为几个模块，`common`,`frontend`,`loopend`,`map`,`map_track`,`msf`,`occupancy_map`,`state_machine`,`system`几个子模块。　　

- 　**`common`**   

    通用模块主要是定义基本的数据结构，参数配置接口，数学运算库.

- 　**`frontend`** 

    前端主要包含两个子模块，一个是点云的分割和特征提取.
　

- **`loopend`**

    闭环检测模块对关键帧进行闭环检测，通过几何距离(或者`GPS`，语义识别结果)，在`map`中搜索最近的关键帧，再利用此关键帧临域内的关键帧，构成一个局部地图，进行闭环检测．　　

- **`map`**    

    地图管理模块，里面存放了所有的关键帧数据，提供各种数据调用接口．

- **`map_track`**    

    地图跟踪模块是地图跟踪模块，即`lidarodometry`的解算，同时生成关键帧插入地图模块．　

- **`msf`**    

    多传感器融合模块，进行图优化求解关键帧`pose`．

- **`occupancy_map`**    

    栅格地图模块，用于生成导航所需2d map．

- **`state_machine`**    

    状态机模块，用于slam系统的状态管理.

- **`system`**    

    系统模块是slam对外的封装接口.
　

#### **相关依赖**
- [ROS](http://wiki.ros.org/ROS/Installation) (仅使用`ros`接口，算法不依赖)
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [CERES](http://ceres-solver.org/)
- [PCL](https://github.com/PointCloudLibrary/pcl)
- [OpenCV](https://opencv.org/releases/page/2/)

#### **使用方法**
```
ros2 run cvte_lidar_slam lidar_slam_mission_manager __params:=./install/cvte_lidar_slam/params/xxxx.yaml
```

#### **版本发布计划**

- **lidar_slam V3.0**  

#### **开发人员**
曹永，林燕龙，张伟

#### **责任人**
张伟，林燕龙
