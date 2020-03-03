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

#ifndef __MEM_CACHE_PREFETCH_FILTER_PPF_HH__
#define __MEM_CACHE_PREFETCH_FILTER_PPF_HH__

#include <cstdint>

#include "arch/isa_traits.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "sim/byteswap.hh"
#include "sim/clocked_object.hh"
#include "sim/probe/probe.hh"

class BaseCache;
struct PerceptronPrefetchFilterParams;

namespace prefetch_filter {

class Feature {
public:
    // 依据字符串初始化Feature
    int init(const std::string& feature);
    
    // 获取一个预取信息的索引
    uint16_t getIndex(const PrefetchInfo& info);

    // 获取当前Feature的表格大小
    uint16_t getSize();

public:
    // 特征的通用标识名称
    std::string name_;

private:
    // 当前Feature所使用的预取信息索引
    std::vector<uint8_t> infoIndexList_;

    // Feature索引对应结果的起始位次
    uint8_t startBits_;

    // Feature索引对应结果的位数
    uint8_t bits_;
};

class PrefetchUsefulTable {
public:
    // 进行表格的初始化
    int init(const uint8_t counterBits, const uint32_t CCsize,
            const uint8_t CCAssoc, const uint32_t VCSize,
            const uint8_t VCAssoc, BaseCache* cache = nullptr,
            int8_t CCTagBits = -1);

    // 对命中Pref的情况进行处理，不区分Pref还是替换数据地址
    int updateHit(const uint64_t& addr);
    
    // 对Miss的情况进行处理，不区分Pref还是替换数据地址
    int updateMiss(const uint64_t& addr);
    
    // 当新的Pref出现的时候，进行替换信息的更新
    int addPref(const uint64_t& prefAddr, const uint64_t evictedAddr);
    
    // 当前Cache中的预取被替换了
    int evictPref(const uint64_t& addr, int8_t* counterPtr);

public:
    // Counter的大小
    uint8_t counterBits_;

    // Counter Cache的Tag大小
    int8_t counterCacheTagBits_;
    
    // Counter Cache的大小
    uint32_t counterCacheSize_;

    // Counter Cache的组相联毒
    uint8_t counterCacheAssoc_;

    // Victim Cache的大小
    uint32_t victimCacheSize_;

    // Victim Cache的组相联度
    uint8_t victimCacheAssoc_;

private:
    // Counter Cache的存储表项
    class CounterEntry {
    public:
        // 默认初始化函数
        CounterEntry() {}
        // 初始化函数
        CounterEntry(const uint8_t counterBits, const uint64_t& prefAddr,
                const uint64_t& evictedAddr_);

    public:
        // 计数器大小
        SaturatedCounter counter_;
        // 对应的预取地址
        uint64_t prefAddr_;
        // 对应的被替换的地址
        uint64_t evictedAddr_;
    };

    // 当前处于Cache中的预取地址，此处不做压缩映射，实际硬件设计
    // Cache中存在压缩地址的相关记录结构
    std::set<uint64_t> prefInCache_;

    // 被替换数据到Counter Cache压缩地址的映射
    CacheTable<uint64_t> victimCache_;

    // Counter Cache
    CacheTable<CounterEntry> counterCache_;

    // 是否开启了统计
    bool valid_ = false;
};

// 基于感知器的预取过滤器
class PerceptronPrefetchFilter : public BasePrefetchFilter {

typedef CacheTable<std::vector<SaturatedCounter>> FeatureIndexTable;
typedef CacheTable<int8_t> FeatureWeightTable;
typedef CacheTable<BaseCache*> OldPrefTable;

public:
    // 构造函数 
    PerceptronPrefetchFilter(const PerceptronPrefetchFilterParams *p);
    
    // 析构函数
    ~PerceptronPrefetchFilter() {}

    // 通知发生了Hit事件
    int notifyCacheHit(BaseCache* cache, const PacketPtr& pkt,
            const DataTypeInfo& info);
    
    // 通知发生了Miss事件
    int notifyCacheMiss(BaseCache* cache, const PacketPtr& pkt,
            const DataTypeInfo& info, const uint64_t& combinedAddr);
    
    // 通知发生了Fill事件
    int notifyCacheFill(BaseCache* cache, const PacketPtr &pkt,
            const DataTypeInfo& info, const uint64_t& evictedAddr);

    // 对一个预取进行过滤，返回发送的Cache Level(0-3)或者不预取(4)
    int filterPrefetch(BaseCache* cache, const PacketPtr &pkt,
            const PrefetchInfo& info);
    
    // 注册统计变量
    void regStats() override;

private:
    // 针对不同类型预取的训练方法对应的enum类型
    enum TrainType {GoodPref, BadPref, UselessPref};

private:
    // 用于初始化数据的函数
    int init();

    // 依据给定的信息确定处理的目标表格
    Tables& getTable(BaseCache* cache);

    // 对一个给定的预取进行奖励或者惩罚，处理成功返回1，失败返回0，错误返回-1
    int train(Tables& workTable, const uint64_t& prefAddr,
            const uint8_t cacheLevel, const TrainType type);

    // 依据信息对Prefetch Table和Reject Table进行更新
    int updateTable(Tables& workTable, const uint64_t& prefAddr,
            const uint8_t targetCacheLevel, const uint8_t cpuId,
            const uint8_t srcCacheLevel, const std::vector<uint16_t>& indexes);

public:
    // 一直都未被过滤的预取请求个数，区分核心，编号区分缓存
    Stats::Vector* prefAccepted_[3];
    
    // 一直都被过滤的预取请求个数，区分核心，编号区分缓存
    Stats::Vector* prefRejected_[3];

    // 在Prefetch Table和Reject Table之间存在切换的预取
    Stats::Vector* prefThreshing_[3];
    
    // 因为表格不足，没有进行训练的预取个数
    Stats::Vector* prefNotTrained_[3];

    // 最终被处理成预取至L1的请求个数，编号对应源预取缓存等级
    Stats::Vector* prefToL1_[3];

    // 最终被处理成预取至L2的请求个数，编号对应源预取缓存等级
    Stats::Vector* prefToL2_[3];

    // 最终被处理成预取至L3的请求个数，编号对应源预取缓存等级
    Stats::Vector* prefToL3_[3];

    // 不同Feature不同权重的数值出现次数，用于计算Pearson相关因子
    std::vector<std::vector<Stats::Vector*>> featureWeightFrequency_;

private:
    // 初始化成功与否的标志
    bool initFailFlag = false;

    // 是否在不同CPU之间共享表格
    const bool cpuSharedTable_;
    
    // 是否在非LLC的Cache之间共享表格
    const bool cacheSharedTable_;
    
    // 是否在所有Cache之间共享表格
    const bool allSharedTable_;
    
    // 是否允许对预取请求进行等级提升
    const bool allowUpgrade_;
    
    // 将预取设置为到L1的可信度阈值
    const uint16_t l1PrefThreshold_;

    // 将预取设置为到L2的可信度阈值
    const uint16_t l2PrefThreshold_;

    // 将预取设置为到L3的可信度阈值
    const uint16_t l3PrefThreshold_;
    
    // Feature权重的bit数
    const uint8_t weightBits_;

    // 不同层级缓存相关反馈对应的训练幅度
    const uint8_t trainStep_[3];

    // 对于无用预取的训练Step，为0则表示不处理
    const uint8_t uselessPrefStep_;
    
    // 存放Feature的向量
    std::vector<Feature> featureList_;

private:
    // 一套基本表格的数据结构
    struct Tables {
        // 没有被过滤的预取索引表格，区分CPU
        FeatureIndexTable prefetchTable_;
    
        // 被过滤的预取索引表格，区分CPU
        FeatureIndexTable rejectTablePtr_;

        // 存放所有在Prefetch Table和Reject Table中预取在两个表格中出现次数
        // pair中第一个表示PT次数，第二个表示RT次数
        std::map<uint64_t, std::pair<uint8_t, uint8_t>> prefAppearTime_;

        // 存放Feature权重的表格，区分CPU
        std::vector<FeatureWeightTable> featureTable_;

        // 记录最近被剔除的预取
        OldPrefTable oldPrefTable_;
    };
    
    // 预取有害性统计相关的表格
    std::map<BaseCache*, PreftchUsefulTable> prefUsefulTable_;
    
    // 针对非LLC的结构使用的表格
    std::vector<std::vector<Tables>> noneLLCTables_;
    
    // 针对LLC的结构使用的表格
    Tables LLCTable_;
};

} // namespace prefetch_filter

#endif // __MEM_CACHE_PREFETCH_FILTER_PPF_HH__
