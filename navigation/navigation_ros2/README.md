# 导航系统

## 安装依赖

OMPL包: `sudo apt install libompl-dev`



## 编译方式

`colcon build --symlink-install --packages-up-to navigation_ros2 `



## 启动方式

### 有任务管理接收地图和路径：

`ros2 run navigation_ros2 navigation_ros2_mission_manager __params:=./src/ros2.0_core/navigation_ros2/params/navigation_param_6.yaml`

（注意 ： 参数文件需要按照对应车号启动，这里举例是用6号车的）


### 直接启动导航：

`ros2 run navigation_ros2 navigation_ros2 __params:=./src/ros2.0_core/navigation_ros2/params/navigation_param_6.yaml`

（注意 ： 参数文件需要按照对应车号启动，这里举例是用6号车的。

另外，需要将参数文件里面的第5行 “use_default_path” 参数改为true， 并且将“default_path_file”参数置为需要执行的路径

以及91行“init_static_map”参数改为true。并且将92行的“map_path”参数设置为读的默认2D地图 ）



