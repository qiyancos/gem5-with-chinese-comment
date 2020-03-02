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

#ifndef __MEM_CACHE_PREFETCH_FILTER_TABLE_HH__
#define __MEM_CACHE_PREFETCH_FILTER_TABLE_HH__

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

// 基于Cache结构设计的表格，使用理想的LRU替换算法
template<typename T>
class CacheTable {
private:
    // 表格的表项结构
    struct TableEntry {
        // Tag数值
        uint64_t tag_;
        // 用于LRU替换算法的数值, 0表示Invalid
        uint8_t priority_;
        // 存放的数值
        T value_;
    };

    class Set {
    private:
        // 数据实体
        std::vector<TableEntry> data_;
        // 当前优先替换的数据
        uint32_t replaceIndex_;
    public:
        // 构造函数
        Set(const uint8_t assoc);
        // 读取数据
        T& read(const uint64_t& tag);
        // 写入数据
        int write(const uint64_t& tag, const T& value);
    }
    
    // 表格的数据实体
    std::vector<Set> table_;

public:
    // 表格的大小
    const uint32_t size_;
    
    // 表格的组相联度
    const uint8_t assoc_;

public:
    // 初始化函数
    int init(const uint32_t size, const uint8_t assoc);

    // 初始化函数
    int init(const uint32_t size, const uint8_t assoc, const T& initVal);

    // 判断某一个地址是否有数据，0表示不存在，1表示存在，-1表示出现错误
    int touch(const uint64_t& addr);

    // 读取某一个地址对应的数据，0表示不存在，1表示成功，-1表示出现错误
    int read(const uint64_t& addr, T**);

    // 写入一个地址对应的数据，0表示不存在，1表示成功，-1表示出现错误
    int write(const uint64_t& addr, const T& value,
            uint64_t* replacedAddr = nullptr);
    
    // 无效化一个地址对应的表项0，表示不存在，1表示成功，-1表示出现错误
    int invalidate(const uint64_t& addr);
};

class IdealPrefetchUsefulTable {
public:
    // 析构函数
    ~IdealPrefetchUsefulTable();

    // 初始化表格
    int init(const uint8_t numCpus);

    // 在数据被命中的时候进行处理，同时会更新计数
    // cacheStats第一个是L1的发出预取对应的统计数据，
    // 第二个是L2发出预取对应的统计数据
    int updateHit(const uint64_t& addr, const DataType type, 
            std::vector<Stats::Vector*>& cacheStats);
    
    // 当一个预取替换之前的无用预取时进行更新，同时更新统计计数
    int replaceEvict(const PacketPtr& newPrefPkt, const uint64_t& oldPrefAddr,
            const Stats::Vector* totalUsefulValue[][],
            Stats::Vector* usefulDegree[][][],
            std::vector<Stats::Vector*>* usefulType[],
            uint32_t timingStatus[][][]);

    // 在预取被Demand Request替换掉的时候进行处理，同时更新统计计数
    int updateEvict(const uint64_t& addr, Stats::Vector* totalUsefulValue[][],
            Stats::Vector* usefulDegree[][][],
            std::vector<Stats::Vector*>* usefulType[],
            uint32_t timingStatus[][][]);
    
    // 对时间维度的信息进行更新，并重置相关信息项
    int updatePrefTiming(uint32_t timingStatus[][][]);

    // 当新的预取插入时添加表项
    int addPref(const PacketPtr& prefPkt, const uint64_t& evictAddr);

    // 判断一个预取是否被Demand Request覆盖了
    int isPrefHit(const uint64_t& addr);

private:
    // 当前系统中CPU的个数
    uint8_t numCpus_;

    // 依据地址到有用信息的映射，用于对Cache中的预取数据
    std::map<uint64_t, PrefetchUsefulInfo*> prefMap_;
    
    // 依据地址到有用信息的映射，用于对Cache中被预取替换的数据
    std::map<uint64_t, PrefetchUsefulInfo*> evictMap_;
};

} // namespace prefetch_filter

#endif // __MEM_CACHE_PREFETCH_FILTER_TABLE_HH__
