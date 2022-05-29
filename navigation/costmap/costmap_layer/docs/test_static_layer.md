

## static_layer 测试用例

> 开发人员： @wuhuabo 测试人员： @wuhuabo

### 1. 初始化static_layer 测试用例

> 功能描述： 初始化static_layer; CostmapParameters, LayeredCostmap, WorldmapData未初始化时, 或者没有使能时, 初始化失败

| 编号 | 输入/动作                                    | 期望的输出/响应              | 实际情况 |
| ---- | -------------------------------------------- | ---------------------------- | -------- |
| 1    | 初始化static_layer, CostmapParameters, LayeredCostmap, WorldmapData未初始化| 初始化失败 | 符合     |
| 2    | 初始化static_layer, LayeredCostmap, WorldmapData未初始化| 初始化失败 | 符合     |
| 3    | 初始化static_layer, WorldmapData未初始化| 初始化失败 | 符合     |
| 4    | 初始化static_layer, CostmapParameters, LayeredCostmap, WorldmapData均初始化| 初始化成功 | 符合     |
| 5    | 初始化static_layer, 没有使能| 初始化失败 | 符合     |

### 2. 初始化static_layer解析地图 测试用例

> 功能描述： 解析WorldmapData设置当前层的值

初始化static_layer, track_unknown_space为true, lethal_cost_threshold为100, unknown_cost_value为255, trinary_costmap为true
| 编号 | 输入/动作                                                              | 期望的输出/响应                                                         | 实际情况 |
| ---- | ---------------------------------------------------------------------- | ----------------------------------------------------------------------- | -------- |
| 1    | 解析255 | NO_INFORMATION | 符合     |
| 2    | 解析254 | LETHAL_OBSTACLE | 符合     |
| 3    | 解析101 | LETHAL_OBSTACLE | 符合     |
| 4    | 解析100 | LETHAL_OBSTACLE  | 符合     |
| 5    | 解析99 | FREE_SPACE | 符合     |
| 6    | 解析50 | FREE_SPACE | 符合     |
| 7    | 解析0 | FREE_SPACE | 符合     |

初始化static_layer, track_unknown_space为false, lethal_cost_threshold为99, unknown_cost_value为253, trinary_costmap为false

| 编号 | 输入/动作                                                              | 期望的输出/响应                                                         | 实际情况 |
| ---- | ---------------------------------------------------------------------- | ----------------------------------------------------------------------- | -------- |
| 1    | 解析255 | LETHAL_OBSTACLE | 符合     |
| 2    | 解析254 | FREE_SPACE | 符合     |
| 3    | 解析252 | LETHAL_OBSTACLE | 符合     |
| 4    | 解析100 | LETHAL_OBSTACLE  | 符合     |
| 5    | 解析99 | LETHAL_OBSTACLE | 符合     |
| 6    | 解析50 | 128 | 符合     |
| 7    | 解析0 | FREE_SPACE | 符合     |

### 3. loadMap 测试用例

> 功能描述： 从WorldmapData解析得到costmap

初始化static_layer, LayeredCostmap的rolling_window为true, lethal_cost_threshold为100, track_unknown_space为true
地图为

000   000   000   000   000   000   000   000   000   000
000   000   000   000   000   000   000   000   000   000
000   000   000   000   000   000   000   200   200   200
000   000   000   000   100   000   000   200   200   200
000   000   000   000   100   000   000   200   200   200
070   070   000   000   000   000   000   000   000   000
000   000   000   000   000   000   000   000   000   000
000   000   000   200   200   200   000   000   000   000
000   000   000   000   000   000   000   255   255   255
000   000   000   000   000   000   000   255   255   255
最左上角坐标 is (0, 0), 最右角坐标 9, 9

| 编号 | 输入/动作                                                              | 期望的输出/响应                                                         | 实际情况 |
| ---- | ---------------------------------------------------------------------- | ----------------------------------------------------------------------- | -------- |
| 1    | 检查x和y的大小 | 都为10 | 符合     |
| 2    | 检查LETHAL_OBSTACLE和NO_INFORMATION的cells数 | 14和6 | 符合     |
| 3    | 检查LETHAL_OBSTACLE的cell对应的map值 | 都大于等于100 | 符合     |
| 4    | 检查x和y的大小 | 都为10 | 符合     |
| 5    | 检查(7, 2) to (9, 4)是否在LETHAL_OBSTACLE的cells里 | 都在 | 符合     |
| 6    | 检查(4, 3) to (4, 4)是否在LETHAL_OBSTACLE的cells里 | 都在 | 符合     |
| 7    | 检查(3, 7) to (5, 7)是否在LETHAL_OBSTACLE的cells里 | 都在 | 符合     |

| 编号 | 输入/动作                                                              | 期望的输出/响应                                                         | 实际情况 |
| ---- | ---------------------------------------------------------------------- | ----------------------------------------------------------------------- | -------- |
| 1    | 检查(7, 8) to (9, 9)是否在NO_INFORMATION的cells里 | 都在 | 符合     |

初始化static_layer, LayeredCostmap的rolling_window为false, 其他不变
| 编号 | 输入/动作                                                              | 期望的输出/响应                                                         | 实际情况 |
| ---- | ---------------------------------------------------------------------- | ----------------------------------------------------------------------- | -------- |
| 1    | 初始化static_layer前, 检查LayeredCostmap的大小与其是否一致 | 不一致 | 符合     |
| 2    | 初始化static_layer后, 检查LayeredCostmap的大小与其是否一致 | rolling为false, 初始化static_layer会使LayeredCostmap 大小与其匹配 | 符合     |

### 4. updateCosts 测试用例

> 功能描述： 以static_layer更新layered_costmap

初始化static_layer, lethal_cost_threshold为100, track_unknown_space为true

| 编号 | 输入/动作                                                              | 期望的输出/响应                                                         | 实际情况 |
| ---- | ---------------------------------------------------------------------- | ----------------------------------------------------------------------- | -------- |
| 1    | LayeredCostmap为true, 检查LayeredCostmap LETHAL_OBSTACLE和NO_INFORMATION的cells数 | 全为NO_INFORMATION | 符合     |
| 2    | LayeredCostmap为true, use_maximum为true, 用static_layer更新(0, 0) to (5, 5), 检查LayeredCostmap的LETHAL_OBSTACLE和NO_INFORMATION的cells数 | 全为NO_INFORMATION | 符合     |
| 3    | LayeredCostmap为true, use_maximum为false, 用static_layer更新(0, 0) to (5, 5), 检查LETHAL_OBSTACLE和NO_INFORMATION的cells数 | 2个LETHAL_OBSTACLE, 75个NO_INFORMATION | 符合     |
| 4    | LayeredCostmap为false, use_maximum为false, 用static_layer更新(0, 0) to (5, 5), 检查LETHAL_OBSTACLE和NO_INFORMATION的cells数 | 2个LETHAL_OBSTACLE, 75个NO_INFORMATION | 符合     |
| 5    | LayeredCostmap为false, use_maximum为true, 用static_layer更新(3, 7) to (10, 10), 检查LETHAL_OBSTACLE和NO_INFORMATION的cells数 | 3个LETHAL_OBSTACLE, 85个NO_INFORMATION | 符合     |

### 5. updateBounds 测试用例

> 功能描述： 根据static_layer确定要更新的范围

初始化static_layer, static_map10*10,分辨率1
| 编号 | 输入/动作                                                              | 期望的输出/响应                                                         | 实际情况 |
| ---- | ---------------------------------------------------------------------- | ----------------------------------------------------------------------- | -------- |
| 1    | LayeredCostmap为true, pose{0, 0, 0}, 初始bound为{1e30, 1e30, -1e30, -1e30}, 确定更新范围 | (0.5, 0.5) to (10.5, 10.5) | 符合     |
| 2    | 初始bound为{1e30, 1e30, -1e30, -1e30}, 在1执行后再更新一次确定更新范围 | (0.5, 0.5) to (10.5, 10.5) | 符合     |
| 3    | LayeredCostmap为false, pose{0, 0, 0}, 初始bound为{1e30, 1e30, -1e30, -1e30}, 确定更新范围 | (0.5, 0.5) to (10.5, 10.5) | 符合     |
| 4    | 初始bound为{1e30, 1e30, -1e30, -1e30}, 在1执行后再更新一次确定更新范围 | 不再更新,保持原先的值{1e30, 1e30, -1e30, -1e30} | 符合     |