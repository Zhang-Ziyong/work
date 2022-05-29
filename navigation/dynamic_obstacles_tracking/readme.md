1. 把xhy里的地图拷贝到本地
2. 将wh/navigation_param_3.yaml覆盖到CVTE_Babot/src/ros2.0_core/navigation_ros2/params/navigation_param_3.yaml
2. CVTE_Babot/将src/ros2.0_core/navigation_ros2/params/navigation_param_3.yaml下的map_path参数路径, 修改为本地路径

运行以下节点, 分割及跟踪动态障碍

1. ros2 run ground_segment_ros2 ground_segment_node __params:=src/ros2.0_core/ground_segment_ros2/params/ground_segment_ros2_3.yaml
2. ros2 run navigation_ros2 navigation_ros2 __params:=src/ros2.0_core/navigation_ros2/params/navigation_param_3.yaml
3. ros2 run dynamic_obstacles_tracking tracking_node __params:=src/ros2.0_core/dynamic_obstacles_tracking/params/dynamic_obstacles_tracking.yaml

输入:/camera_navigation_cloud
输出: 可视化的跟踪结果 /people
