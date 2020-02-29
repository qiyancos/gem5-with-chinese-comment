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

// 饱和计数器（最多支持32bit）
class SaturatedCounter {
public:
    // 赋值
    template<class T>
    uint32_t& operator= (const T& b) {
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
    uint32_t& operator+ (const T& b) {
        uint64_t temp = value_;
        if (temp + b > maxValue_) {
            return maxValue_;
        } else {
            return value_ + b;
        }
    }

    // 减法
    template<class T>
    uint32_t& operator- (const T& b) {
        if (value_ >= b) {
            return value_ - b;
        } else {
            return minValue_;
        }
    }

    // 格式自动转换
    template<class T>
    const T operator() {
        // return static_cast<T>(value_);
        return value_;
    }

    // 自增
    template<class T>
    uint32_t& operator+= (const T& b) {
        value_ = *this + b;
        return value_;
    }
    
    // 后递增
    template<class T>
    const uint32_t operator++ (const T& b) {
        uint32_t value = value_;
        value_ = *this + 1;
        return value;
    }
    
    // 前递增
    template<class T>
    uint32_t& operator++ () {
        value_ = *this + 1;
        return value_;
    }
    
    // 自减
    template<class T>
    uint32_t& operator-= (const T& b) {
        value_ = *this - b;
        return value_;
    }
    
    // 后递减
    template<class T>
    const uint32_t operator-- (const T& b) {
        uint32_t value = value_;
        value_ = *this - 1;
        return value;
    }
    
    // 前递减
    template<class T>
    uint32_t& operator-- () {
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
    SaturatedCounter();
    
    // 构造函数
    SaturatedCounter(const uint8_t bits);

    // 初始化函数
    int init(const uint8_t bits);
    
    // 最小值
    const uint32_t minValue_ = 0;

    // 饱和计数器的最大数值
    uint32_t maxValue_;
    
    // 当前饱和计数器的数值
    uint32_t value_;
};

} // namespace prefetch_filter

#endif // __MEM_CACHE_PREFETCH_FILTER_SATURATED_COUNTER_HH__
