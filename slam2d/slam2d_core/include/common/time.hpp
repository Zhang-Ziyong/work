#ifndef _COMMON_TIME_H_
#define _COMMON_TIME_H_

#include <chrono>
#include <ostream>
#include <ratio>

/*
预备知识:
c++11 提供了语言级别的时间函数.包括duration和time_point
duration是时间段,指的是某单位时间上的一个明确的tick(片刻数)：
3分钟->"3个1分钟",
1.5个"1/3秒" :1.5是tick,1/3秒是时间单位

time_point是一个duration和一个epoch(起点)的组合：
2017年5月4日是"自1970,01,01"以来的126200000秒数
common/time.h主要功能是提供时间转换函数：
 */
namespace slam2d_core {
namespace common {

// 719162 是0001年1月1日到1970年1月1日所经历的天数
constexpr int64_t kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock {
  using rep = int64_t;
  using period = std::ratio<1, 10000000>;  // 0.1us
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};

using Duration = UniversalTimeScaleClock::duration;
using Time = UniversalTimeScaleClock::time_point;

/*Time::min()是chrono自带的函数。返回一个低于1970.01.01的数。
编译运行cpp/cppstdlib_2nd/util/chrono1.cpp:
epoch: Thu Jan  1 08:00:00 1970
now:   Tue Jul  4 19:39:29 2017
min:   Tue Sep 21 08:18:27 1677
max:   Sat Apr 12 07:47:16 2262
*/

Duration fromSeconds(double seconds);
Duration fromMilliseconds(int64_t milliseconds);

double toSeconds(Duration duration);
double toSeconds(std::chrono::steady_clock::duration duration);

//将TUC时间(微秒)转化为c++的time_point对象
Time fromUniversal(int64_t ticks);

//将c++的time_point对象转为TUC时间,单位是us
int64_t toUniversal(Time time);

std::ostream &operator<<(std::ostream &os, Time time);

}  // namespace common
}  // namespace slam2d_core

#endif  // _COMMON_TIME_H_
