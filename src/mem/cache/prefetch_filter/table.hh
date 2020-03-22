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
#include <string>
#include <map>
#include <set>
#include <list>

#include "base/statistics.hh"
#include "mem/cache/prefetch_filter/pref_info.hh"

class BaseCache;
class Packet;

namespace prefetch_filter {

// 基于Cache结构设计的表格，使用理想的LRU替换算法
template<typename T>
class CacheTable {
public:
    // 初始化函数
    int init(const uint32_t size, const uint8_t assoc,
            const bool valid = false);

    // 初始化函数
    int init(const int8_t tagBits, const uint32_t size, const uint8_t assoc,
            const bool valid = false);

    // 初始化函数
    int init(const uint32_t size, const uint8_t assoc, const T& data,
            const bool valid = false);

    // 初始化函数
    int init(const int8_t tagBits, const uint32_t size, const uint8_t assoc,
            const T& data, const bool valid = false);

    // 判断某一个地址是否有数据，0表示不存在，1表示存在，-1表示出现错误
    int touch(const uint64_t& addr);

    // 读取某一个地址对应的数据，0表示不存在，1表示成功，-1表示出现错误
    int read(const uint64_t& addr, T** data);

    // 写入一个地址对应的数据，0表示命中并写入，1表示未命中且未替换，
    // 2表示未命中但是发生了替换，-1表示出现错误
    int write(const uint64_t& addr, const T& data,
            uint64_t* replacedAddr = nullptr, T* replacedData = nullptr);
    
    // 无效化一个地址对应的表项，0表示不存在，1表示成功，-1表示出现错误
    int invalidate(const uint64_t& addr);

    // 该函数会对所有的单元进行批量写操作
    int writeAll(const T& data, const bool valid = true);

public:
    // 表格的大小
    uint32_t size_;
    
    // Set的数目
    uint32_t sets_;

    // 表格的组相联度
    uint8_t assoc_;

private:
    // 表格的表项结构
    class CacheEntry {
    public:
        // 构造函数
        CacheEntry() {};
        
        // 复杂构造函数
        CacheEntry(const uint64_t& tag, const uint64_t& addr, bool valid) :
                tag_(tag), addr_(addr), valid_(valid) {}

        // 复杂构造函数
        CacheEntry(const uint64_t& tag, const uint64_t& addr, bool valid,
                const T& data) :
                tag_(tag), addr_(addr), valid_(valid), data_(data) {}
    
    public:
        // Tag数值
        uint64_t tag_ = 0;

        // 完整的地址
        uint64_t addr_ = 0;
        
        // valid bit
        bool valid_ = false;
        
        // 存放的数值
        T data_;
    };

    typedef std::list<CacheEntry> Set;

    // 表格的数据实体
    std::vector<Set> data_;
    
    // 依据Tag的大小，通过压缩地址空间节省开销，但是会出现部分错误识别
    // 该变量会屏蔽地址中的高位，压缩Tag空间
    uint64_t tagMask_;

    // Set对应的Mask
    uint64_t setMask_;
};

class IdealPrefetchUsefulTable {
typedef Packet *PacketPtr;

public:
    // 初始化函数
    IdealPrefetchUsefulTable() {}
    
    // 初始化函数
    IdealPrefetchUsefulTable(BaseCache* cache);
    
    // 初始化函数
    IdealPrefetchUsefulTable(BaseCache* cache, const bool valid);

    // 为一个新的预取生成信息项
    int newPref(const PacketPtr& prefPkt);

    // 当新的预取插入时添加表项，一次会更新多个对应的预取信息
    int addPref(const PacketPtr& prefPkt, const uint64_t& replacedAddr,
            const DataType type);

    // 判断一个预取是否被Demand Request覆盖了
    int isPrefHit(const uint64_t& addr);
    
    // 查找当前有用信息表中和某一个地址相关的信息，可以使Victim也可以是Pref
    int findSrcCaches(const uint64_t& addr, std::set<BaseCache*>* caches);

    // 查找当前有用信息表中和某一个地址相关的信息，可以使Victim也可以是Pref
    // 该函数不仅会查找Cache的预取，还会查找处理中的预取
    int findAllSrcCaches(BaseCache* cache, const PacketPtr& pkt,
            std::set<BaseCache*>* caches);

    // 在数据被命中的时候进行处理，同时会更新计数
    int updateHit(const PacketPtr& srcPkt, const uint64_t& hitAddr,
            const DataType srcType);
   
    // 在预取数据被预取命中的时候进行合并记录
    int combinePref(const PacketPtr& srcPkt, const uint64_t& hitAddr);
   
    // 当一个Demand发生Miss进行有害性信息更新
    int updateMiss(const PacketPtr& srcPkt);
    
    // 在预取被Demand Request替换掉的时候进行处理，同时更新统计计数
    int updateEvict(const uint64_t& addr);
    
    // 当一个预取替换之前的无用预取时进行更新，同时更新统计计数
    int replaceEvict(const PacketPtr& prefPkt, const uint64_t& oldPrefAddr);

    // 对时间维度的信息进行更新，并重置相关信息项
    int updatePrefTiming(
            std::vector<std::vector<Stats::Vector*>>& totalUsefulValue,
            std::vector<std::vector<std::vector<Stats::Vector*>>>&
            usefulDegree, std::vector<std::vector<Stats::Vector*>>& usefulType,
            std::vector<std::vector<std::vector<std::vector<uint32_t>>>*>
            timingStatus);

    // 设置当前Valid信息
    int setValidBit(const bool valid);

private:
    // 用于生成一个预取对应的Prefetch Info的Index
    uint64_t genIndex(BaseCache* cache, const uint8_t targetCacheLevel,
            const uint64_t& addr);
    
    // 用于生成一个预取对应的Prefetch Info的Index
    int genIndex(const PacketPtr& prefPkt, std::vector<uint64_t>* indexes);

private:
    // 表示当前是否实际有效
    bool valid_ = false;

    // 当前表格对应的Cache指针
    BaseCache* cache_ = nullptr;

    // 依据地址到有用信息的映射，用于对Cache中的预取数据
    std::map<uint64_t, std::set<PrefetchUsefulInfo*>> prefMap_;
    
    // 依据地址到有用信息的映射，用于对Cache中被预取替换的数据
    std::map<uint64_t, std::set<PrefetchUsefulInfo*>> evictMap_;

    // 存放全局有用信息的结构
    static std::map<uint64_t, PrefetchUsefulInfo> infoList_;
    
    // 存放时间维度统计周期内新被替换的预取
    std::vector<PrefetchUsefulInfo> evictedPref_;
};

} // namespace prefetch_filter

#endif // __MEM_CACHE_PREFETCH_FILTER_TABLE_HH__
