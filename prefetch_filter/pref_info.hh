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

#ifndef __MEM_CACHE_PREFETCH_FILTER_PREF_INFO_HH__
#define __MEM_CACHE_PREFETCH_FILTER_PREF_INFO_HH__

#include <cstdint>

#include "arch/isa_traits.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "sim/byteswap.hh"
#include "sim/clocked_object.hh"
#include "sim/probe/probe.hh"

namespace prefetch_filter {

// 数据类型
enum DataType {Dmd, Pref};

// Miss以及Fill时候的信息
struct DataTypeInfo {
    // 插入的数据是什么属性
    DataType inserted;
    // 被替换的数据是什么属性
    DataType replaced;
}

// 一个预取信息项的索引和有效位数信息
struct IndexInfo {
    // 信息项索引
    uint8_t index_;
    // 信息项有效位数
    uint8_t bits_;
};

// 记录字符串到信息项映射的数据
extern std::map<std::string, IndexInfo> IndexMap;

// 注册一个新的信息项相关的函数
int addNewFeature(const std::string& name, const uint8_t bits);

// 注册新信息项的宏
#define DEF_FEATURE(STRING, BITS) \
    const int STRING = addNewFeature(#STRING, BITS);

// 预取信息类，用来从预取器传递预取信息到PPFE中
class PrefetchInfo {
private:
    // 信息项主体
    std::vector<uint32_t> info;

public:
    // 写入一个新的信息
    int setInfo(const uint8_t index, const uint32_t value);
    
    // 写入一个新的信息
    uint32_t getInfo(const uint8_t index);
};

} // namespace prefetch_filter

#endif // __MEM_CACHE_PREFETCH_FILTER_PREF_INFO_HH__
