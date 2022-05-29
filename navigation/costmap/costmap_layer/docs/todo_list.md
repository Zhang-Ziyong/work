# costmap

------
1. 将每个层的传感器数据存储修改成一种效率更高的形式。
2. static_layer大小大于layeredCostmap时,更新范围是整张地图,
除了static_layer的updateBounds,其他层的updateBounds是否可优化
3. range_sensor_layer地图默认值是127,不是NO_INFORMATION,
可考虑改变其to_prob和to_cost的方式
4. inflation_layer的inflate_unknown功能未添加
5. obstacle_layer中获取到激光数据后设置raytrace_range_未写成配置化的
## 负责人
吴华勃， 陈明建