/*
 * Copyright (c) 2020 Peking University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Rock Lee
 */

#ifndef __MEM_CACHE_PREFETCH_FILTER_SATURATED_COUNTER_HH__
#define __MEM_CACHE_PREFETCH_FILTER_SATURATED_COUNTER_HH__

#include <cstdint>

namespace prefetch_filter {

// 饱和计数器（最多支持31bit）
class SaturatedCounter {
public:
    // 赋值
    template<class T>
    int& operator= (const T& b) {
        // 说明一下可能的类型情况
        // 输入是uint64_t，比较会转换为uint64_t比较，不会出错
        // 输入是int64_t，比较会转换为int64_t比较，不会出错
        // 输入是uint32_t，比较会转换为uint32_t比较，不会出错
        // 输入是int32_t，比较会转换为int32_t比较，不会出错
        // 输入是长度更小的类型，比较会转换为int32_t比较，不会出错
        if (b >= maxValue_) {
            value_ = maxValue_;
        } else if (b <= minValue_) {
            value_ = minValue_;
        } else {
            value_ = b;
        }
        return value_;
    }

    // 加法
    template<class T>
    int& operator+ (const T& b) {
        // 扩展数值范围避免计算错误
        int64_t temp = value_;
        temp += b;
        if (temp > maxValue_) {
            return maxValue_;
        } else if (temp < minValue_) {
            return minValue_;
        } else {
            return temp;
        }
    }

    // 减法
    template<class T>
    int& operator- (const T& b) {
        return *this + (-b);
    }

    // 格式自动转换
    template<class T>
    const T operator() {
        // return static_cast<T>(value_);
        // 使用默认提供的转换函数处理
        return value_;
    }

    // 自增
    template<class T>
    int& operator+= (const T& b) {
        value_ = *this + b;
        return value_;
    }
    
    // 后递增
    template<class T>
    const int operator++ (const T& b) {
        int value = value_;
        value_ = *this + 1;
        return value;
    }
    
    // 前递增
    template<class T>
    int& operator++ () {
        value_ = *this + 1;
        return value_;
    }
    
    // 自减
    template<class T>
    int& operator-= (const T& b) {
        value_ = *this - b;
        return value_;
    }
    
    // 后递减
    template<class T>
    const int operator-- (const T& b) {
        int value = value_;
        value_ = *this - 1;
        return value;
    }
    
    // 前递减
    template<class T>
    int& operator-- () {
        value_ = *this - 1;
        return value_;
    }

    // 不等于
    template<class T>
    const bool operator!= (const T& b) {
        return value_ != b;
    }
    
    // 等于
    template<class T>
    const bool operator== (const T& b) {
        return value_ == b;
    }

    // 大于
    template<class T>
    const bool operator> (const T& b) {
        return value_ > b;
    }

    // 大于等于
    template<class T>
    const bool operator>= (const T& b) {
        return value_ >= b;
    }

    // 小于
    template<class T>
    const bool operator< (const T& b) {
        return value_ < b;
    }

    // 小于等于
    template<class T>
    const bool operator<= (const T& b) {
        return value_ <= b;
    }

    // 饱和计数器的位数
    uint8_t bits_;

private:
    // 构造函数
    SaturatedCounter(const uint8_t bits);

    // 初始化函数
    int init(const uint8_t bits);
    
    // 最小值
    const int minValue_ = 0;

    // 饱和计数器的最大数值
    int maxValue_;
    
    // 当前饱和计数器的数值
    int value_;
};

} // namespace prefetch_filter

#endif // __MEM_CACHE_PREFETCH_FILTER_SATURATED_COUNTER_HH__
