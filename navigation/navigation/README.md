# 导航规划包

包含路径规划、控制算法以及逻辑控制接口，按照[通用框架文档](https://kb.cvte.com/pages/viewpage.action?pageId=165265408)编写，V2.0版本

1. local_planner: 局部路径规划器，使用普通工厂模式生产不同的规划算法
2. global_planner: 全局路径规划器，使用普通工厂模式生产不同的规划算法
3. local_controller: 用于根据任务路径，进行跟踪控制的算法包，含大部分通用控制算法
4. path_follower: 用于导航过程中避障的逻辑处理
5. logic_controller: 上层的应用逻辑处理

## 开发人员 

陈明建、梁嘉俊

