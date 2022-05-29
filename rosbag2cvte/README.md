# rosbag2cvte

实际上ros2本身提供了bag录制功能，但是由于各种原因不适合在机器上常态化部署，因此花几天时间根据需求简单重写了此功能。

特性：

1. 实现了对rclcpp任意类型的接收发布封装，实现基本的record,info,play指令；
2. 实现zstd消息压缩解压，实现了rosbag2cvte的自动命令补全功能；
3. 重新设计了保存格式，完全没有数据库随机写操作，且header只有16字节，非常轻量省空间；
4. 增加了循环录制功能，通过-max_size_mb控制单个bag文件大小，通过-max_file_num控制bag文件个数；
5. 增加condition录制功能，通过/condition主题控制开启/暂停录制，从而避免录制无效数据（如充电时）；
6. 针对大规模部署后的异常排查需要，增加了异常触发录制功能，同时设计了环形缓冲区暂存数据，避免对flash的频繁读写；
7. 增加消息录制频率控制功能，可使用-resample '"topic_name":freqency‘ 对多个主题的录制频率进行控制；
8. 增加了目标播放时间段控制功能，通过-from,-to参数设置播放的时间段，实现了seek逻辑快速寻址；
9. 增加了播放速度控制功能，通过-play_speed参数控制播放速度，从而实现数据快速浏览和慢动作播放；
10. 增加了播放topic过滤功能，通过-skip可以控制屏蔽特定topic;

## 编译安装

安装ros2(foxy)之后,使用cmake进行编译安装：

```
git clone git@gitlab.gz.cvte.cn:zhaoyong/rosbag2cvte.git
cd rosbag2cvte && mkdir build
cd build && cmake ..
make && sudo make install
```

安装之后建议使用Svar给命令增加tab自动补全功能，首先安装Svar：

```
git clone https://github.com/zdzhaoyong/Svar
cd Svar && mkdir build
cd build && cmake ..
make && sudo make install

```

给rosbag2cvte增加命令补全：

```
sudo svar complete -bin rosbag2cvte
```

此时可尝试rosbag2cvte命令查看tab命令补全和帮助效果。

## 使用

大体和ros2 bag一致，有一些小区别


**注意**： 由于目前机器都使用best_effort进行发布，rosbag2cvte默认开启best_effort, 请使用'-best_effort false'进行关闭！

### 录制

* 简单录制,topics使用逗号分隔,默认根据时间保存到YYYYMMDD_HHmmss文件夹：

```
rosbag2cvte record -topics /scan,/odom 
```

* 录制频率控制

```
rosbag2cvte record -topics /scan,/odom -resample '"/odom":10,"/scan":2.5'
```

注意这里的引号也很重要，不能漏（偷懒用了json去解析，哪个大佬有兴趣可以改进一下）

* 循环录制(其实默认就是循环录制10GB数据的，这里其实是控制循环录制参数)：

```
rosbag2cvte record -topics /scan,/odom -output ./cvte_record -compression zstd -max_size_mb 512 -max_file_num 2
```

上述命令为保存包到cvte_record目录，开启流压缩，每个bagcvte文件最多为512MB，最多保存两个bagcvte文件。

* 条件录制和异常录制，对于用户部署机器，不希望平常读写flash，但希望出现特定异常后能将异常之前的数据录下，应使用此模式:

```
rosbag2cvte record -topics /scan,/odom -output ./cvte_record -compression zstd -condition /condition_topic -exception /exception_topic
```

上述命令中:
- /condition_topic的类型为std_msgs/msg/Bool,data代表是否开启录制
- /exception_topic的类型为std_msgs/msg/String,data代表异常类型，应尽可能短，因为会作为文件名保存

* 录制服务

为了让录制服务开机自启并长期运行（即使软件crash了也重新唤醒继续录制），可以通过提供的rosbag2cvte_service启动录制。首先请对脚本中的start函数内容进行编辑，调整成想录制的命令：

```
start() # Start the service
{
  ulimit -c 100 # 此行用于开启core自动保存，方便程序crash后查找原因，配合debug编译更佳
  # 下面这行就是想运行的命令，这里仅用于示例
  rosbag2cvte record -topics /color_image_front_up,/tf,/scan,/odom,/fusion_pose,/usound -compression zstd --resample -output /userdata/record '"/color_image_front_up":5.0,"/depth_image_front_up":5.0,"/depth_image_front_down":2.0,"/color_image_front_down":2.0,"/depth_image_back":2.0,"/color_image_back":2.0'
}
```

然后使用start_service即可开启服务,并使用*systemctl status rosbag2cvte*查看状态：

```
./rosbag2cvte_service start_service
```

### 查看信息

查看bagcvte文件集合信息：

```
rosbag2cvte info -bag 20220401_102249 -full
```

查看特定bagcvte文件信息：

```
rosbag2cvte info -bag 20220401_102249/20220401_102249.bagcvte -full
```

输出如下：

```
info.cpp:66 Load 20220401_102249/20220401_102249.bagcvte
{
  "duration": 14.266279697418213,
  "start": 1648779781.1737514,
  "end": 1648779795.440031,
  "topics": [
    {
      "count": 226,
      "topic": "/fusion_pose",
      "type": "geometry_msgs/msg/PoseWithCovarianceStamped"
    },
    {
      "count": 223,
      "topic": "/odom",
      "type": "nav_msgs/msg/Odometry"
    },
    {
      "count": 80,
      "topic": "/scan",
      "type": "sensor_msgs/msg/LaserScan"
    }
  ],
  "compression": "none"
}

```



### 播放


* 简单播放

```
rosbag2cvte play -bag 20220401_102249
```

* 控制播放时间段和速度

```
rosbag2cvte play -bag 20220401_102249 -from 1648779782.1 -to 1648779795.4 -play_speed 0.5
```

* 忽略部分topic, 不使用best_effort发布

```
rosbag2cvte play -bag 20220401_102249 -best_effort false -skip /odom
```

上述命令将不会播放/odom节点

## TODO

* 需要交叉编译部署到rk上实际测试性能表现
* 需要大量测试验证稳定性
* 在实际场景中发现新需求功能并导入


