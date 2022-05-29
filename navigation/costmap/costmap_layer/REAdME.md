# costmap_layer

------
costmap的各个类型层的具体实现

## 内容

包含类，分别是：

### Layer
所有layer的基类

### CostmapLayer
继承了Layer和costmap特性，StaticLayer,ObstacleLayer,RangeSensorLayer都继承于它。

### StaticLayer
静态层，用于加载静态地图

### ObstacleLayer
障碍物层，融合激光雷达等点云数据

### RangeSensorLayer
超声层，融合超声数据

### LayerCreator
layer工厂模式的creator

## 开发人员
陈明建，吴华勃

## 责任人
陈明建，阳方平
