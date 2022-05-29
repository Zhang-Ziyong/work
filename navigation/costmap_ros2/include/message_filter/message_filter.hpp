/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file message_filter.hpp
 *
 *@brief
 * 1.消息过滤器,对订阅的消息与坐标转换做时间同步
 * 2.有转换源与消息源:消息源每次接收到消息去转换源的缓存区查找
 * 时间匹配的转换,有则调用callback,没有则存储到消息源缓存,转
 * 换源每次接收到转换去消息源的缓存区查找时间匹配的消息,有则调
 * 用callback,没有则存储到转换源缓存
 *
 *@modified by wuhuabo(wuhuabo@cvte.com)
 *
 *@author wuhuabo(wuhuabo@cvte.com)
 *@version current_algo.dev
 *@data 2019-05-05
 ************************************************************************/
#ifndef __MESSAGE_FILTER_HPP
#define __MESSAGE_FILTER_HPP
#include <glog/logging.h>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <vector>
#include "costmap_utils.hpp"
#include "debug_log.hpp"

namespace CVTE_BABOT {
typedef WorldmapPose Transform;                   /// 转换的类定义
typedef std::shared_ptr<Transform> PtrTransform;  /// 转换类型的指针
typedef double TimeType;                          /// 时间的类定义

/**
 * MessageFilter
 * @brief 消息过滤器实现类
 * 消息和转换的存储使用list,时间戳最近的放最前面
 **/
template <class Message>
class MessageFilter {
 public:
  /**
   * TransformResult
   * @brief 查找变换的结果
   **/
  enum TransformResult {
    /// can transform now
    CanTransform,
    /// can't transform now and could wait for the transform.
    WaitForFeasible,
    /// The timestamp on the message is more earlier than the cache length
    OutOfCache
  };
  typedef std::shared_ptr<Message> PtrMessage;  /// 消息类型的指针

  /**
   *MessageFilter
   *@brief
   *初始化MessageFilter
   *
   *@param[in] d_time_tolerance-消息与转换的容忍时间间隔
   *@param[in] d_time_cache-要保留的消息与当前已有转换的最长时间差
   *@param[in]
   *const unsigned int &
   *ui_msg_queue_size-消息缓存数量最大值,数据匹配异常时需要,保证缓存区不过多
   *@param[in]
   *const unsigned int &
   *ui_transform_cache_size-转换缓存数量最大值,用途与ui_msg_queue_size同
   **/
  MessageFilter(const double &d_time_tolerance, const double &d_time_cache,
                const unsigned int &ui_msg_queue_size,
                const unsigned int &ui_transform_cache_size)
      : d_time_tolerance_(d_time_tolerance),
        d_time_cache_(d_time_cache),
        ui_msg_queue_size_(ui_msg_queue_size),
        ui_transform_cache_size_(ui_transform_cache_size) {}

  MessageFilter(const MessageFilter &) = delete;
  MessageFilter &operator=(const MessageFilter &) = delete;
  ~MessageFilter() = default;

  /**
  *inputTransform
  *@brief
  *用于外界传入转换,查找并处理与传入转换时间匹配的消息
  *如果有可转换的消息,清除之前的转换因为已经过时,超过
  *ui_transform_cache_size_时清除最尾端的数据
  *
  *@param[in] time-转换对应的时间
  *@param[in] ptr_trans-传入的转换
  **/
  void inputTransform(const TimeType &time, const PtrTransform &ptr_trans) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (handleTransformableMsg(time, ptr_trans)) {
      l_transformInfo.clear();
    } else if (l_transformInfo.size() >= ui_transform_cache_size_) {
      l_transformInfo.pop_back();
    }
    l_transformInfo.push_front(TransformInfo(time, ptr_trans));
  }

  /**
  * handleTransformableMsg
  * @brief
  * 查找与传入转换时间匹配的消息并处理
  * 有能转换的消息则清除其之后的massage
  * 因为已过时
  *
  *@param[in] time-消息对应的时间
  *@param[in] ptr_trans-传入的转换
  *return true-有可转换的消息, false-无可转换的消息
  **/
  bool handleTransformableMsg(const TimeType &time,
                              const PtrTransform &ptr_trans) {
    for (auto it = l_messageInfo.begin(); it != l_messageInfo.end(); it++) {
      if (abs(it->time - time) < d_time_tolerance_) {
        transformableCB_(it->ptr_massage, ptr_trans);
        l_messageInfo.erase(it, l_messageInfo.end());
        return true;
      }
    }
    return false;
  }

  /**
  *inputMessage
  *@brief
  *用于外界传入消息,后续需要针对不同的frame_id做不同处理
  *
  *@param[in] time-消息对应的时间
  *@param[in] ptr_msg-传入的消息
  **/
  void inputMessage(const TimeType &time, const PtrMessage &ptr_msg) {
    if (ptr_msg->header.frame_id.empty()) {
      LOG(ERROR) << "massage's frame_id is empty.";
      return;
    }

    handleMessage(time, ptr_msg);
  }

  /**
  *handleMessage
  *@brief
  *对传入消息的消息做处理,去转换源的缓存区查找
  *时间匹配的转换,有则调用callback,没有则存储
  *到消息源缓存,如果大于ui_msg_queue_size_
  *则清除最尾端数据
  *
  *@param[in] time-消息对应的时间
  *@param[in] ptr_msg-传入的消息
  **/
  void handleMessage(const TimeType &time, const PtrMessage &ptr_msg) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    PtrTransform ptr_trans;
    TransformResult result = getLatestTransform(time, ptr_trans);
    if (CanTransform == result) {
      transformableCB_(ptr_msg, ptr_trans);
      return;
    } else if (OutOfCache == result) {
      LOG(WARNING) << "Could not transform for out of cache. ";
    }
    if (l_messageInfo.size() >= ui_msg_queue_size_) {
      l_messageInfo.pop_back();
    }
    l_messageInfo.push_front(MessageInfo(time, ptr_msg));
  }

  /**
  *getLatestTransform
  *@brief
  *获取与传入时间匹配的转换,返回获取的结果
  *
  *@param[in] time-消息对应的时间
  *@param[out] ptr_trans-对应的转换
  *@return
  *CanTransform-当前已可以转换,WaitForFeasible-尚未有对应的转换,
  *OutOfCache-传入的时间远早于已有的转换,认为是过时的不可转换
  **/
  TransformResult getLatestTransform(const TimeType &time,
                                     PtrTransform &ptr_trans) {
    double time_diff;
    double min_time_diff = d_time_tolerance_;
    auto min_it = l_transformInfo.begin();
    for (auto it = l_transformInfo.begin(); it != l_transformInfo.end(); it++) {
      time_diff = abs(it->time - time);
      if (time_diff < min_time_diff) {
        min_time_diff = time_diff;
        min_it = it;
        if (min_time_diff >= d_time_tolerance_) {
          break;
        }
      }
    }

    // if min_time_diff was changed, there is matching transform
    if (min_time_diff < d_time_tolerance_) {
      ptr_trans = min_it->ptr_transform;
      // the later transform is out of stale
      l_transformInfo.erase(++min_it, l_transformInfo.end());
      return CanTransform;
    } else if (l_transformInfo.size() != 0 &&
               l_transformInfo.begin()->time + d_time_cache_ < time) {
      // transform coming too later than message, the transform is useless.
      l_transformInfo.clear();
      return OutOfCache;
    } else {
      return WaitForFeasible;
    }
  }

  /**
  *registerTransformableCB
  *@brief
  *注册消息可转换时调用的处理函数
  *
  *@param[in] cb-处理函数
  **/
  void registerTransformableCB(
      const std::function<void(const PtrMessage &, const PtrTransform &)> &cb) {
    transformableCB_ = cb;
  }

  /**
  *MessageInfo
  *@brief
  *消息存储类型,存有消息与对应的时间戳
  *
  **/
  struct MessageInfo {
    /**
    *MessageInfo
    *@brief
    *MessageInfo构造函数
    *
    *@param[in] t-时间戳
    *@param[in] ptr_msg-消息
    **/
    MessageInfo(const TimeType &t, const PtrMessage &ptr_msg)
        : time(t), ptr_massage(ptr_msg) {}

    TimeType time;           ///消息对应的时间
    PtrMessage ptr_massage;  ///消息
  };

  /**
  *TransformInfo
  *@brief
  *转换存储类型,存有转换与对应的时间戳
  *
  **/
  struct TransformInfo {
    /**
    *TransformInfo
    *@brief
    *MessageInfo构造函数
    *
    *@param[in] t-时间戳
    *@param[in] ptr_msg-转换
    **/
    TransformInfo(const TimeType &t, const PtrTransform &ptr_trans)
        : time(t), ptr_transform(ptr_trans) {}

    TimeType time;               ///转换对应的时间
    PtrTransform ptr_transform;  ///转换
  };

  std::list<MessageInfo> l_messageInfo;      ///消息缓存区
  std::list<TransformInfo> l_transformInfo;  ///转换缓存区
  std::function<void(PtrMessage, PtrTransform)>
      transformableCB_;  ///消息与转换的时间匹配时调用的处理函数
  double
      d_time_tolerance_;  /// 消息与转换的容忍时间间隔,小于这个间隔视为两者的时间匹配
  double
      d_time_cache_;  /// 要保留的消息与当前已有转换的最长时间差,大于这个间隔视为消息过时,把消息丢弃
  unsigned int
      ui_msg_queue_size_;  /// 要保留的消息的最大数量,大于这个数量视为消息过时,把消息丢弃
  unsigned int
      ui_transform_cache_size_;  /// 要保留的转换的最大数量,大于这个数量视为转换过时,把转换丢弃
  std::recursive_mutex mutex_;  ///消息与转换的缓存区多线程访问,需要加锁
};

}  // namespace CVTE_BABOT

#endif  // __MESSAGE_FILTER_HPP
