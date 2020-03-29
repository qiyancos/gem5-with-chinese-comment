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
#include <functional>
#include <map>
#include <set>
#include <list>
#include <queue>

#include "base/statistics.hh"
#include "base/types.hh"
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
            const uint8_t rightShiftBits, const bool valid = false);

    // 初始化函数
    int init(const int8_t tagBits, const uint32_t size, const uint8_t assoc,
            const uint8_t rightShiftBits, const bool valid = false);

    // 初始化函数
    int init(const uint32_t size, const uint8_t assoc,
            const uint8_t rightShiftBits, const T& data,
            const bool valid = false);

    // 初始化函数
    int init(const int8_t tagBits, const uint32_t size, const uint8_t assoc,
            const uint8_t rightShiftBits, const T& data,
            const bool valid = false);

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

    // 该结构用于Debug，记录所有不同Set中有效数据地址
    std::vector<std::set<uint64_t>> validAddr_;

    // 表示了右侧忽略的位数
    uint8_t rightShiftBits_ = 0;

    // 依据Tag的大小，通过压缩地址空间节省开销，但是会出现部分错误识别
    // 该变量会屏蔽地址中的高位，压缩Tag空间
    uint64_t tagMask_;

    // Set对应的Mask
    uint64_t setMask_;
};

class IdealPrefetchUsefulTable {
public:
    typedef Packet *PacketPtr;

public:
    // 初始化函数
    IdealPrefetchUsefulTable() {}
    
    // 初始化函数
    IdealPrefetchUsefulTable(BaseCache* cache);
    
    // 初始化函数
    IdealPrefetchUsefulTable(BaseCache* cache, const bool valid);
    
    // 判断一个数据是否是一个已知的预取
    int checkDataType(const uint64_t& addr, const uint64_t& prefIndex,
            const DataType type, const bool isMiss);

    // 判断一个预取是否被Demand Request覆盖了
    int isPrefValid(const uint64_t& addr);

    // 为一个新的预取生成信息项
    int newPref(PacketPtr& prefPkt);

    // 删除一个预取的所有信息香
    int deletePref(const PacketPtr& prefPkt);

    // 当新的预取插入时添加表项，一次会更新多个对应的预取信息
    int addPref(const PacketPtr& prefPkt, const uint64_t& replacedAddr,
            const DataType type);

    // 在预取数据被预取命中的时候进行合并记录
    int combinePref(const uint64_t& addr, const uint64_t& index,
            std::set<BaseCache*>* correlatedCaches = nullptr);
   
    // 在预取被Demand Request替换掉的时候进行处理，同时处理无效化
    int evictPref(const uint64_t& addr,
            std::set<BaseCache*>* invalidatingCaches = nullptr);
    
    // 查找当前有用信息表中和某一个地址相关的信息
    int findSrcCaches(const uint64_t& addr, std::set<BaseCache*>* caches);

    // 查找一个已经被无效化的预取的所有源Cache（该函数会在无效化之后使用）
    int findInvalidatedCaches(const uint64_t& addr,
            std::set<BaseCache*>* caches);

    // 查找当前有用信息表中和某一个地址相关的信息
    // 该函数不仅会查找Cache的预取，还会查找处理中的预取
    int findAllSrcCaches(BaseCache* cache, const PacketPtr& pkt,
            std::set<BaseCache*>* caches);

    // 在数据被命中的时候进行处理，同时会更新计数
    int updateHit(const PacketPtr& srcPkt, const uint64_t& hitAddr,
            const DataType srcType);
   
    // 当一个Demand发生Miss进行有害性信息更新
    int updateMiss(const PacketPtr& srcPkt);
    
    // 获取一个预取在当前Cache中的替换地址
    int getReplacedAddr(const uint64_t& addr, uint64_t* replacedAddr);

    // 设置预取无效化任务
    int addPrefInvalidation(BaseCache* cache, const Tick& completeTick,
            const uint64_t& addr);
    
    // 设置预取合并的任务
    int addPrefCombination(const Tick& completeTick, const uint64_t& addr,
            const uint64_t& index);
    
    // 依据给定的时钟周期将到点的预取无效化
    int updateInvalidation(const Tick& tickNow,
            std::set<uint64_t>* invalidatedPref);
    
    // 依据给定的时钟周期将到点的预取无效化
    int updateCombination(const Tick& tickNow);
    
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
    // 下面是基本数据结构

    // 表示当前是否实际有效
    bool valid_ = false;

    // 当前表格对应的Cache指针
    BaseCache* cache_ = nullptr;

    // 依据地址到有用信息的映射，用于对Cache中的预取数据
    std::map<uint64_t, std::set<PrefetchUsefulInfo*>> prefMap_;
    
    // 依据地址到有用信息的映射，用于对Cache中被预取替换的数据
    std::map<uint64_t, std::set<PrefetchUsefulInfo*>> evictMap_;

private:
    // 下面是和无效化有关的结构
    
    // 记录无效化信息的结构
    struct Invalidation {
        // 对应的无效化完成时间
        Tick completeTick_;
        // 对应的地址
        uint64_t addr_;
        // 发送请求的Cache
        BaseCache* cache_;
        // 比较大小排序函数
        bool operator< (const Invalidation& b) const {
            return completeTick_ > b.completeTick_;
        }
    };
    
    // 记录已经无效化的信息
    struct InvalidatedRecord {
        // 无效化的源头
        BaseCache* srcCache_;
        // 相关的预取信息
        std::set<PrefetchUsefulInfo*> infoList_;
    };

    // 用于快速找到下一个需要无效化的预取
    std::priority_queue<Invalidation, std::vector<Invalidation>>
            invalidatingTick_;
    
    // 当前正在被处于无效化计时的预取
    std::set<uint64_t> invalidatingPref_;

    // 无效化被打断，直接删除的预取
    std::set<uint64_t> forceDeletedPref_;

    // 已经被无效化的预取信息
    std::map<uint64_t, InvalidatedRecord> invalidatedMap_;
    
private:
    // 下面是和预取有效性合并传递相关的内容
    
    // 记录合并信息的结构
    struct Combination {
        // 对应的合并完成时间
        Tick completeTick_;
        // 对应的地址
        uint64_t addr_;
        // 需要合并的预取Index
        uint64_t index_;
        // 比较大小排序函数
        bool operator< (const Combination& b) const {
            return completeTick_ > b.completeTick_;
        }
    };
    
    // 用于快速找到下一个需要无效化的预取
    std::priority_queue<Combination, std::vector<Combination>> combiningTick_;
    
private:
    // 下面是全局记录的预取信息结构

    // 所有Cache均已经被删除的预取，准备用于统计
    static std::vector<PrefetchUsefulInfo> deletedPref_;
    
    // 存放全局有用信息的结构
    static std::map<uint64_t, PrefetchUsefulInfo> infoList_;
    
};

} // namespace prefetch_filter

#endif // __MEM_CACHE_PREFETCH_FILTER_TABLE_HH__
