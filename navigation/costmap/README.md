# costmap

------
本项目用于机器保安项目的开发和测试，实现代价地图相关算法功能：
1. 融合激光雷达、超声、全局地图等数据
2. 根据需求生成不同的代价地图，以提供给路径规划和壁障使用

## 内容

本项目按照功能分为多个包，每一个包下面又包含多个文件夹，分别是doc,include,src,test,分别用来存放相关文档，头文件或模板文件，源文件以及测试文件。这些功能包分别是:

### costmap_base
主要实现costmap的接口
1. Costmap2d类
2. LayeredCostmap类
3. 以及其它一些公用数据接口

### costmap_builder
主要实现一个costmap的生成器，用于生成拥有不用层的costmap
1. CostmapBuilder基类
2. Costmap2dBuilder类
3. Costmap2dDirector类

### costmap_mediator
用于各个层的数据交互

### costmap_layer
用于不同类型层的具体实现

## 开发人员
陈明建，吴华勃

## 责任人
陈明建，阳方平
