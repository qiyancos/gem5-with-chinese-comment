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
    SaturatedCounter& operator= (const T& b) {
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
        return *this;
    }
    
    SaturatedCounter& operator= (const SaturatedCounter& b) {
        int bValue = b.getValue();
        if (bValue >= maxValue_) {
            value_ = maxValue_;
        } else if (bValue <= minValue_) {
            value_ = minValue_;
        } else {
            value_ = bValue;
        }
        return *this;
    }
    
    // 加法
    template<class T>
    T operator+ (const T& b) const {
        return value_ + b;
    }

    int operator+ (const SaturatedCounter& b) const {
        return value_ + b.getValue();
    }
    
    // 减法
    template<class T>
    T operator- (const T& b) const {
        return *this + (-b);
    }

    int operator- (const SaturatedCounter& b) const {
        return *this + (-b.getValue());
    }

    // 格式自动转换
    #define TRANS_TYPE(Type) \
        operator Type() { \
            return value_; \
        }

    TRANS_TYPE(int8_t)
    TRANS_TYPE(uint8_t)
    TRANS_TYPE(int16_t)
    TRANS_TYPE(uint16_t)
    TRANS_TYPE(int32_t)
    TRANS_TYPE(uint32_t)
    TRANS_TYPE(int64_t)
    TRANS_TYPE(uint64_t)
    TRANS_TYPE(float)
    TRANS_TYPE(double)

    // 自增
    template<class T>
    SaturatedCounter operator+= (const T& b) {
        *this = *this + b;
        return *this;
    }
    
    // 后递增
    SaturatedCounter operator++ (int) {
        SaturatedCounter temp = *this;
        value_ = *this + 1;
        return temp;
    }
    
    // 前递增
    SaturatedCounter operator++ () {
        value_ = *this + 1;
        return *this;
    }
    
    // 自减
    template<class T>
    SaturatedCounter operator-= (const T& b) {
        *this = *this - b;
        return *this;
    }
    
    // 后递减
    SaturatedCounter operator-- (int) {
        SaturatedCounter temp = *this;
        value_ = *this - 1;
        return temp;
    }
    
    // 前递减
    SaturatedCounter operator-- () {
        value_ = *this - 1;
        return *this;
    }

    // 不等于
    template<class T>
    bool operator!= (const T& b) const {
        return *this - b != 0;
    }
    
    // 等于
    template<class T>
    bool operator== (const T& b) const {
        return *this - b == 0;
    }

    // 大于
    template<class T>
    bool operator> (const T& b) const {
        return *this - b > 0;
    }

    // 大于等于
    template<class T>
    bool operator>= (const T& b) const {
        return *this - b >= 0;
    }

    // 小于
    template<class T>
    bool operator< (const T& b) const {
        return *this - b < 0;
    }

    // 小于等于
    template<class T>
    bool operator<= (const T& b) const {
        return *this - b <= 0;
    }

    // 构造函数
    SaturatedCounter() : bits_(0), maxValue_(0) {};

    // 构造函数
    explicit SaturatedCounter(const uint8_t bits);

    // 构造函数（带初始化数值）
    SaturatedCounter(const uint8_t bits, const int value);

    // 复制构造函数
    SaturatedCounter(const SaturatedCounter &) = default;

    // 右值复制构造函数
    SaturatedCounter(SaturatedCounter&&) = default;
    
    // 初始化函数
    int init(const uint8_t bits);
    
    // 初始化函数（带初始化数值）
    int init(const uint8_t bits, const int value);
    
    // 用于获取实际数值
    int getValue() const {
        return value_;
    }

public:
    // 饱和计数器的位数
    uint8_t bits_;

private:
    // 最小值
    const int minValue_ = 0;

    // 饱和计数器的最大数值
    int maxValue_;
    
    // 当前饱和计数器的数值
    int value_ = 0;
};

} // namespace prefetch_filter

#endif // __MEM_CACHE_PREFETCH_FILTER_SATURATED_COUNTER_HH__
