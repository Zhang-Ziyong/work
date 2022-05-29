# costmap_ros2
## 优化任务
1. 根据架构文档要求，下列输入和输出是需要在发布版本去掉的
    a. tf相关：ptr_tf2_buffer_， ptr_tf_需要删掉 -》解决~~
    b. 发布相关：costmap_pub_、cloud_pub_，只与调试相关，可以删掉，或者使用debug开关 -》解决~~
    c. localization_sub_，定位更新与轨迹规划等同，可以考虑合并成一个 -》解决~~
2. 使用了大量的vector，需要注意使用规则
    a. parseVVF,返回了vec(vec)，这是个很废时的操作，内部使用reserve -》解决~~
    b. v_laser_scan_message_filter_; 等以v_开头的vectors使用前，需要使用reserve做初始化 -》解决~~
3. costmapTimerCallback，函数
    a. prepareGrid()。可以删掉，或者使用debug开关 -》解决~~
    b. 反复调用LayeredCostmap::updateMap，该函数有一些临时变量，如需要，可改为静态或成员，但要考虑是否被同时调用（加锁） -》500ms被调用一次,问题不大, 解决~~
4. rangeSensorCallback
    a. 被反复调用，函数有一些临时变量，如需要，可改为静态或成员，但要考虑是否被同时调用（加锁） -》解决~~
5. laserScanCallback
    a. 被反复调用，函数有一些临时变量，如需要，可改为静态或成员，但要考虑是否被同时调用（加锁） -》解决~~
    b. ptr_v_cloud_->resize,cloud_.points.resize,resize操作比较耗时，出现在反复调用的函数里不合适，如必要，需要去掉这种操作 -》解决~~
6. localizationDataCallback
    a. 被反复调用，函数有一些临时变量，如需要，可改为静态或成员，但要考虑是否被同时调用（加锁） -》解决~~
7. costmap_params_ros2中node_->get_parameter_or和ptr_costmap_params_->setParam可以通过一个函数接口统一调用
## 消除waring
1. /home/ovo/CVTE_Babot_new/src/ros2.0_core/costmap_ros2/include/message_filter/message_filter.hpp:70:37: warning: unused parameter ‘ui_cache_size’ [-Wunused-parameter]
                 const unsigned int &ui_cache_size)
这个waring需要去掉 -》解决~~
# 维护人
陈明建、吴华勃、林宇锋