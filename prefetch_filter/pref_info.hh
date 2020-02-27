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
    DataType source;
    // 被替换的数据是什么属性
    DataType target;
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
int addNewInfo(const std::string& name, const uint8_t bits);

// 注册新信息项的宏
#define DEF_INFO(STRING, BITS) \
    const int STRING = addNewInfo(#STRING, BITS);

// 预取信息类，用来从预取器传递预取信息到PPFE中
class PrefetchInfo {
private:
    // 信息项主体
    std::vector<uint32_t> info_;

public:
    // 写入一个新的信息
    int setInfo(const uint8_t index, const uint32_t value);
    
    // 写入一个新的信息
    uint32_t getInfo(const uint8_t index);
};

// 预取分类的结构体
struct PrefUsefulType {
    // 预取分类的名称
    const std::string name;
    // 进行判断的函数
    std::funtion<bool(const uint64_t&, const uint64_t&, const uint64_t&,
            const uint64_t&)> isType;
}

// 记录每一个预取分类的名称
extern std::vector<PrefUsefulType> PrefUsefulTypeList;

// 注册一个新的信息项相关的函数
int addNewPrefUsefulType(const std::string& name,
        std::funtion<bool(const uint64_t&, const uint64_t&, const uint64_t&,
        const uint64_t&)> judgeFunc);

class PrefetchUsefulInfo {
public:
    enum
    // 依据CPU的个数进行大小配置
    int resize(const uint8_t numCpus);
    
    // 更新一个预取有效命中，同时对命中统计数据进行更新
    int updateUse(const uint64_t& coreBitMap,
            std::vector<Stats::Vector*>& cacheStats);

    // 更新一个预取有害命中
    int updateHarm(const uint64_t& coreBitMap);

public:
    // 存放统计数据的结构体
    // 没有记录Cycle是因为实际上每一个有用或者有害带来的
    // 时钟周期损失/节省均为LLC的一次Miss Latency
    struct Stats {
        // 单核心预取产生的有效命中次数
        uint64_t singleCoreUsefulCount_;
        
        // 单核心预取有害统计的次数
        uint64_t singleCoreHarmCount_;
        
        // 多核心预取产生的有效命中次数
        uint64_t crossCoreUsefulCount_;
        
        // 多核心预取有害统计的次数
        uint64_t crossCoreHarmCount_;
        
        // 当前预取是否是一个新的预取，用于时间维度统计
        bool newPref_;
    };
    
    // 不同核心的Stats情况
    std::vector<Stats> stats_; 
    
    // 当前预取对应的替换数据地址
    uint64_t replacedAddress_;

private:
    // 所有相关的CPUID位图
    uint64_t relatedCpuBitMap_;
    
    // 所有相关的CPU和对应的预取相关Cache层级
    std::map<uint8_t, std::vector<unint8_t>> relatedCpusInfo_;
};

} // namespace prefetch_filter

#endif // __MEM_CACHE_PREFETCH_FILTER_PREF_INFO_HH__
