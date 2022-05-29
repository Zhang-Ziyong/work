## map_grid_cost_function测试用例
> 开发人员：@caoyong 测试人员：@caoyong

### 1.测试构造函数和复制
> 功能描述：测试MapGridCostFunction的构造和复制

| 编号 | 输入/动作 | 期望的输出/响应                 | 实际情况 |
| ---- | --------- | ------------------------------- | -------- |
| 1    | 构造      | 指针不为nullptr                 | 符合     |
| 2    | 复制      | 指针不为nullptr,且cellCosts相等 | 符合     |

### 2.测试路径的得分函数
> 功能描述：测试路径的得分函数

| 编号 | 输入/动作     | 期望的输出/响应 | 实际情况 |
| ---- | ------------- | --------------- | -------- |
| 1    | 轨迹评分情况1 | 结果不为0       | 符合     |
| 2    | 轨迹评分情况2 | 结果为-3.0      | 符合     |
| 3    | 轨迹评分情况3 | 结果为-3.0      | 符合     |

