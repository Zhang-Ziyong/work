

## obstacle_layer 测试用例

> 开发人员： @wuhuabo 测试人员： @wuhuabo

### 1. 初始化obstacle_layer测试用例

> 功能描述： 初始化obstacle_layer, 设置点云topic, 测试获取数据是否正常

| 编号 | 输入/动作                                    | 期望的输出/响应              | 实际情况 |
| ---- | -------------------------------------------- | ---------------------------- | -------- |
| 1    | marking_cloud_sources设为scan back_scan, clearing_cloud_sources设为scan front_scan | v_clearing_cloud_buffer_names_和v_marking_cloud_buffer_names_有两个成员 | 符合     |
| 2    | 调用getMarkingClouds和getClearingClouds | 获取到点云为空 | 符合     |
| 3    |  更新scan, 调用getMarkingClouds和getClearingClouds | 获取到点云数据, 数量为1 | 符合     |
| 4    |  更新back_scan, 调用getMarkingClouds和getClearingClouds | 获取到marking_clouds数量为2, clearing_clouds数量为1 | 符合     |

### 2. updateRaytraceBounds 测试用例

> 功能描述： 根据激光束确定要更新的范围

传感器原点为{0, 0}
| 编号 | 输入/动作                                                              | 期望的输出/响应                                                         | 实际情况 |
| ---- | ---------------------------------------------------------------------- | ----------------------------------------------------------------------- | -------- |
| 1    | 初始范围为{1.0, 1.0 , 2.0, 3.0}, 更新范围2.5, 激光点{3.0, 4.0} | 更新范围保持{1.0, 1.0 , 2.0, 3.0} | 符合     |
| 2    | 初始范围为{6.0, 7.0, 10.0, 10.0}, 更新范围5.0, 激光点{3.0, 4.0} | 更新范围{3.0, 4.0 , 10.0, 10.0} | 符合     |
| 3    | 初始范围为{0.0, 0.0, 2.0, 3.0}, 更新范围5.0, 激光点{3.0, 4.0} | 更新范围{0.0, 0.0 , 3.0, 4.0} | 符合     |
| 4    | 初始范围为{0.0, 0.0, 2.0, 3.0}, 更新范围8.0, 激光点{2.5, 3.5} | 更新范围{0.0, 0.0, 2.5, 3.5} | 符合     |

### 3. raytraceFreespace 测试用例

> 功能描述： 根据点云确定要更新的范围

点云更新范围为5.0
| 编号 | 输入/动作                                                              | 期望的输出/响应                                                         | 实际情况 |
| ---- | ---------------------------------------------------------------------- | ----------------------------------------------------------------- | -------- |
| 1    | 初始范围为{0.0, 0.0 , 2.0, 2.0}, 传感器原点为{-10, -10}, 激光点{2.5, 3.5} | 更新范围保持{0.0, 0.0 , 2.0, 2.0} | 符合     |
| 2    | 初始范围为{6.0, 7.0, 10.0, 10.0}, 传感器原点为{5.0, 5.0}, 激光点{2.5, 3.5} | 更新范围保持{0.0, 0.0 , 2.0, 2.0} | 符合     |
| 3    | 初始范围为{0.0, 0.0 , 2.0, 2.0}, 传感器原点为{-0.5, -0.5}, 激光点{2.5, 3.5} | 更新范围{-0.5, -0.5 , 2.5, 3.5} | 符合     |
| 4    | 初始范围为{0.0, 0.0 , 2.0, 2.0}, 传感器原点为{-2.0, -1.0}, 激光点{-8.0, -9.0} | 更新范围{-5.0, -5.0 , 2.0, 2.0} | 符合     |
| 5    | 初始范围为{0.0, 0.0 , 2.0, 2.0}, 传感器原点为{1.0, 3.0}, 激光点{-8.0, -9.0} | 更新范围{-2.0, -1.0 , 2.0, 3.0} | 符合     |
| 6    | 设置地图默认值为LETHAL_OBSTACLE, 初始范围为{0.0, 0.0 , 2.0, 2.0}, 传感器原点为{-0.0, -0.0}, 激光点{3.0, 4.0}, {4.0, 1.0} | 更新范围{0.0, 0.0 , 4.0, 4.0}, 激光束扫过的格子变为FREE_SPACE, 总共有9个 | 符合     |

### 4. updateBounds 模型测试用例

> 功能描述： 根据obstacle_layer确定要更新的范围

设置地图默认值为NO_INFORMATION, marking_cloud_sources设为scan back_scan, clearing_cloud_sources设为scan front_scan, 
scan的raytrace_range5.0, obstacle_range6.0, front_scan的raytrace_range5.0, obstacle_range5.0, back_scan的
raytrace_range5.0, obstacle_range4.0
| 编号 | 输入/动作                                                              | 期望的输出/响应                                                         | 实际情况 |
| ---- | ---------------------------------------------------------------------- | ----------------------------------------------------- | -------- |
| 1    | scan激光点{3.0, 4.0, 0}, front_scan激光点{4.0, 1.0, 0}, back_scan激光点{3.0, 2.0, 0}, {0.0, 4.0, 5.0}, {4.0, 4.0, 0.0}, {-1.0, 4.0, 0.0}, 初始范围{0.0, 0.0, 2.0, 2.0}, 机器人坐标{5.0, 5.0} | 地图中FREE_SPACE有8个, LETHAL_OBSTACLE有2个, 更新范围{0.0, 0.0 , 4.0, 4.0} | 符合     |

### 5. updateSpeedLevel 测试用例

> 功能描述： 由激光点获取速度状态, 状态为Stop返回true, 否则为false

初始化obstacle_layer, 停障区{(-0.8, 1.0), (1.6, 1.0), (1.6, -1.0), (-0.8, -1.0)}, 减速区{(-1.0, 1.3), (1.8, 1.3), (1.8, -1.3), (-1.0, -1.3)}
| 编号 | 输入/动作                                                              | 期望的输出/响应                                                         | 实际情况 |
| ---- | ---------------------------------------------------------------------- | ----------------------------------------------------------------------- | -------- |
| 1    | 激光点(-0.7, 0.9) | 结果true, 速度状态Stop | 符合     |
| 2    | 激光点(1.5, 0.9) | 结果true, 速度状态Stop | 符合     |
| 3    | 激光点(-0.9, -1.1) | 结果false, 速度状态SpeedOne | 符合     |
| 4    | 激光点(0.9, -1.1) | 结果false, 速度状态SpeedOne | 符合     |
| 5    | 激光点(2.0, 2.0) | 结果false, 速度状态SpeedMax | 符合     |

### 5. getSpeedLevel 测试用例

> 功能描述： 由激光点云获取速度状态

初始化obstacle_layer, 停障区{(-0.8, 1.0), (1.6, 1.0), (1.6, -1.0), (-0.8, -1.0)}, 减速区{(-1.0, 1.3), (1.8, 1.3), (1.8, -1.3), (-1.0, -1.3)}, 
无说明额外减速区和减外停障区均为{0, 0}
| 编号 | 输入/动作                                                              | 期望的输出/响应                                                         | 实际情况 |
| ---- | ---------------------------------------------------------------------- | --------------------------------------------------- | -------- |
| 1    | 激光点(-0.7, 0.9, 0.00), (-0.9, 1.1, 0.00), (-2.0, 2.0, 0.00) | 速度状态Stop | 符合     |
| 2    | 激光点(1.7, 1.1, 0.00), (-2.0, 2.0, 0.00) | 速度状态SpeedOne | 符合     |
| 3    | 激光点(1.7, 1.1, 0.00), (-2.0, 2.0, 0.00), 额外停障区(0.15, 0.15) | 速度状态Stop | 符合     |
| 4    | 激光点(-0.9, 1.5, 0.00) | 速度状态SpeedMax | 符合     |
| 5    | 激光点(-0.9, 1.5, 0.00), 额外减速区(0.0, 0.3) | 速度状态SpeedOne | 符合     |
| 6    | 激光点(2.0, 1.5, 0.00), 额外减速区(0.3, 0.3) | 速度状态SpeedOne | 符合     |
| 7    | 激光点(2.0, -1.5, 0.00), 额外减速区(0.3, 0.3), 额外停障区(1.0, -1.0) | 速度状态Stop | 符合     |
| 8    | 激光点(-0.9, 1.1, 0.00) | 速度状态SpeedOne | 符合     |
| 9    | 激光点(-2.0, 2.0, 0.00), 输入速度状态为SpeedOne | 速度状态SpeedOne | 符合     |
| 10    | 激光点(-2.0, 2.0, 0.00), 输入速度状态为Stop | 速度状态Stop | 符合     |
