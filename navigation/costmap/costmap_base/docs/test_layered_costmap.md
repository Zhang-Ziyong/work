## layered_costamp测试用例
> 开发人员：@chenmingjian 测试人员：@chenmingjian

### 1.构造函数测试用例
> 功能描述：测试InflationLayer初始化是否正常-LayeredCostmap构造函数

| 编号 | 输入/动作                                               | 期望的输出/响应                  | 实际情况 |
| ---- | ------------------------------------------------------- | -------------------------------- | -------- |
| 1    |设置rolling_window，track_unknow，LayeredCostmap会默认生成200*200，分辨率0.05，原点(-5,-5)的代价地图| 调用构造函数，再通过getCostmap获取costmap，通过getSizeInCellsX， getSizeInCellsY，getResolution，，getOriginX，getOriginY,判断是否和默认的相符| 符合     |
| 2    | 定义InflationLayer传入部分参数| 调用initialize，返回值为false | 符合     |
| 3    | 定义InflationLayer传入参数以及LayeredCostmap| 调用initialize，返回值为True | 符合     |
### 2.重新设置costmap测试用例
> 功能描述：测试重新设置后的costmap的大小分辨率等是否正确-resizeMap

| 编号 | 输入/动作                                               | 期望的输出/响应                  | 实际情况 |
| ---- | ------------------------------------------------------- | -------------------------------- | -------- |
| 1    |定义数据结构{size_x,size_y,reselution,origin_x,origin_}预设一组数据| 调用resizeMap函数，判断是否能resize成功，如果成功对比预设的数据| 符合     |

### 3.更新地图测试用例
> 功能描述：测试更新后的地图代价值是否正确-updateCosts
此部分涉及到其它layer，已放在costmap_ros2

| 编号 | 输入/动作                                               | 期望的输出/响应                  | 实际情况 |
| ---- | ------------------------------------------------------- | -------------------------------- | -------- |
| 1    || 符合     |
