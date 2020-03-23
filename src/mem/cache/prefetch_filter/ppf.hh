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

#include "mem/cache/prefetch_filter/base.hh"
#include "mem/cache/prefetch_filter/saturated_counter.hh"

struct PerceptronPrefetchFilterParams;

namespace prefetch_filter {

class Feature {
public:
    // 依据字符串初始化Feature
    int init(const std::string& feature);
    
    // 获取一个预取信息的索引，如果获取的Feature无效，index被设置为最大值
    int getIndex(const PrefetchInfo& info, uint16_t* index);

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
    int init(const uint8_t counterBits, const uint32_t CCSize,
            const uint8_t CCAssoc, const uint32_t VCSize,
            const uint8_t VCAssoc, const int counterInit,
            BaseCache* cache = nullptr,
            const int8_t CCTagBits = -1, const int8_t VCTagBits = -1);

    // 对命中Pref的情况进行处理
    int updateHit(const uint64_t& addr);
    
    // 对Miss的情况进行处理，不区分Pref还是替换数据地址
    int updateMiss(const uint64_t& addr);
    
    // 当新的Pref出现的时候，进行替换信息的更新
    int addPref(const uint64_t& prefAddr, const uint64_t evictedAddr,
            const DataType type);
    
    // 当前Cache中的预取被替换了
    int evictPref(const uint64_t& addr, uint8_t* counterPtr);

public:
    // Counter的大小
    uint8_t counterBits_;

    // Counter的初始化数值
    int counterInit_;

    // Counter Cache的Tag大小
    int8_t counterCacheTagBits_;
    
    // Counter Cache的大小
    uint32_t counterCacheSize_;

    // Counter Cache的组相联毒
    uint8_t counterCacheAssoc_;

    // Counter Cache的Tag大小
    int8_t victimCacheTagBits_;
    
    // Victim Cache的大小
    uint32_t victimCacheSize_;

    // Victim Cache的组相联度
    uint8_t victimCacheAssoc_;

public:
    // Counter Cache的存储表项
    class CounterEntry {
    public:
        // 默认初始化函数
        CounterEntry() {}
        
        // 默认初始化函数
        CounterEntry(const uint8_t counterBits, const int counterInit) {
            counter_.init(counterBits, counterInit);
        }
        
        // 完整初始化函数
        CounterEntry(const uint8_t counterBits, const int counterInit,
                const uint64_t& prefAddr, const uint64_t& evictedAddr) :
                prefAddr_(prefAddr), evictedAddr_(evictedAddr) {
            counter_.init(counterBits, counterInit);
        }

    public:
        // 计数器大小
        SaturatedCounter counter_;
        // 对应的预取地址
        uint64_t prefAddr_;
        // 对应的被替换的地址
        uint64_t evictedAddr_;
    };

    // 是否开启了统计
    bool valid_ = false;

private:
    // 当前处于Cache中的预取地址，此处不做压缩映射，实际硬件设计
    // Cache中存在压缩地址的相关记录结构
    std::set<uint64_t> prefInCache_;

    // 被替换数据到Counter Cache压缩地址的映射
    CacheTable<uint64_t> victimCache_;

    // Counter Cache
    CacheTable<CounterEntry> counterCache_;
};

} // namespace prefetch_filter

// 基于感知器的预取过滤器
class PerceptronPrefetchFilter final : public BasePrefetchFilter {
private:
    typedef prefetch_filter::CacheTable<std::vector<uint16_t>>
            FeatureIndexTable;
    typedef prefetch_filter::CacheTable<prefetch_filter::SaturatedCounter>
            FeatureWeightTable;
    typedef prefetch_filter::Feature Feature;
    typedef prefetch_filter::PrefetchUsefulTable PrefetchUsefulTable;

public:
    // 构造函数 
    PerceptronPrefetchFilter(const PerceptronPrefetchFilterParams *p);
    
    // 析构函数
    ~PerceptronPrefetchFilter() {}
    
    // 创建全局唯一实例
    static PerceptronPrefetchFilter* create(
            const PerceptronPrefetchFilterParams *p);

    // 通知发生了Hit事件
    int notifyCacheHit(BaseCache* cache, const PacketPtr& pkt,
            const uint64_t& hitAddr, const DataTypeInfo& info) override;
    
    // 通知发生了Miss事件
    int notifyCacheMiss(BaseCache* cache, const PacketPtr& pkt,
            const PacketPtr& combinedPkt, const DataTypeInfo& info) override;
    
    // 通知发生了Fill事件
    int notifyCacheFill(BaseCache* cache, const PacketPtr &pkt,
            const uint64_t& evictedAddr, const DataTypeInfo& info) override;

    // 对一个预取进行过滤，返回发送的Cache Level(0-3)或者不预取(4)
    int filterPrefetch(BaseCache* cache, const uint64_t& prefAddr,
            const PrefetchInfo& info) override;
    
    // 删除一个被无效化的预取
    int invalidatePrefetch(BaseCache* cache, const uint64_t& prefAddr);
    
    // 默认初始化函数
    void init() override;

    // 注册统计变量
    void regStats() override;

private:
    // 针对不同类型预取的训练方法对应的enum类型
    enum TrainType {GoodPref, BadPref, UselessPref};
    
    // 一套PPF基本表格的数据结构
    class Tables {
    public:
        // 默认初始化函数
        Tables() {}

        class PrefInfoEntry {
        public:
            // 简单初始化函数
            PrefInfoEntry() {}

            // 带数值的初始化函数
            PrefInfoEntry(const std::set<BaseCache*>& caches,
                    const uint8_t timesPT = 0, const uint8_t timesRT = 0) :
                    caches_(caches), timesPT_(timesPT), timesRT_(timesRT) {}
            
            // 添加新的Cache信息
            int addCache(BaseCache* newCache) {
                caches_.insert(newCache);
                return 0;
            }

        public:
            // 和当前Prefetch有关的缓存
            std::set<BaseCache*> caches_;

            // Prefetch Table中的出现次数
            uint8_t timesPT_ = 0;

            // Reject Table中的出现次数
            uint8_t timesRT_ = 0;
        };
    
    public:
        // 没有被过滤的预取索引表格，区分CPU
        FeatureIndexTable prefetchTable_;
    
        // 被过滤的预取索引表格，区分CPU
        FeatureIndexTable rejectTable_;

        // 存放Feature权重的表格，区分CPU
        std::vector<FeatureWeightTable> featureTable_;

        // 存放所有在Prefetch Table和Reject Table中预取的信息，只用于统计
        std::map<uint64_t, PrefInfoEntry> localPrefTable_;
        
        // 存放所有最近被剔除的预取信息
        prefetch_filter::CacheTable<PrefInfoEntry> oldPrefTable_;
        
        // 表格相关结构的名称
        std::string name_;
        
        // 对应着相应统计数据的索引
        int statsIndex_;
    };

private:
    // 用于初始化数据的函数
    int initThis();

    // 用于初始化统计变量
    int initStats();

    // 依据给定的信息确定处理的目标表格
    Tables& getTable(BaseCache* cache);

    // 对一个给定的预取进行奖励或者惩罚，处理成功返回1，失败返回0，错误返回-1
    int train(Tables& workTable, const uint64_t& prefAddr,
            const uint8_t cacheLevel, const TrainType type);
    
    // 执行删除Prefetch记录操作的内部函数
    int removePrefetch(BaseCache* cache, const uint64_t& prefAddr,
            const bool isHit);

    // 依据信息对Prefetch Table和Reject Table进行更新
    int updateTable(Tables& workTable, const uint64_t& prefAddr,
            const uint8_t targetCacheLevel, BaseCache* srcCache,
            const std::vector<uint16_t>& indexes);

    // 更新权重的出现次数信息
    int updateFreqStats(Tables& workTable,
            const std::vector<uint16_t>& indexes);

    // 更新PPF表格相关情况的统计信息
    int updateWorkTableStats(Tables& workTable, const uint64_t& prefAddr);

public:
    // 一直都未被过滤的预取请求个数，区分核心，编号区分缓存
    std::vector<Stats::Vector*> prefAccepted_;
    
    // 一直都被过滤的预取请求个数，区分核心，编号区分缓存
    std::vector<Stats::Vector*> prefRejected_;

    // 在Prefetch Table和Reject Table之间存在切换的预取
    std::vector<Stats::Vector*> prefThreshing_;
    
    // 因为表格不足，没有进行训练的预取个数
    std::vector<Stats::Vector*> prefNotTrained_;

    // 最终被处理成预取至L1的请求个数，第一个维度表示源头Cache等级，
    // 第二个维度表示目标的Cache等级
    std::vector<std::vector<Stats::Vector*>> prefTarget_;

    // 不同Feature不同权重的数值出现次数，用于计算Pearson相关因子
    // 第一维度对应Workable， 第二维度表示Feature编号，第三维度表示Weight大小
    std::vector<std::vector<Stats::Vector*>> featureWeightFrequency_;

private:
    // 初始化成功与否的标志
    bool initFailFlag_ = false;

    // 是否在不同CPU之间共享表格
    const bool cpuSharedTable_;
    
    // 是否在非LLC的Cache之间共享表格
    const bool cacheSharedTable_;
    
    // 是否在所有Cache之间共享表格
    const bool allSharedTable_;
    
    // 是否允许对预取请求进行等级提升
    const bool allowUpgrade_;
    
    // Feature权重的bit数
    const uint8_t weightBits_;

    // Feature权重的初始化数值
    std::vector<uint8_t> weightInit_;

    // 有害表格计数器的初始化数值
    const uint8_t counterInit_;

    // 将预取设置为到L1的可信度阈值
    std::vector<uint16_t> prefThreshold_;

    // 不同层级缓存相关反馈对应的训练幅度(Miss)
    std::vector<uint8_t> missTrainStep_;

    // 不同层级缓存相关反馈对应的训练幅度(Hit)
    std::vector<uint8_t> hitTrainStep_;

    // 对于无用预取的训练Step，为0则表示不处理
    const uint8_t uselessPrefStep_;
    
    // 存放Feature的向量
    std::vector<Feature> featureList_;

    // 预取有害性统计相关的表格
    std::map<BaseCache*, PrefetchUsefulTable> prefUsefulTable_;
    
    // 针对非LLC的结构使用的表格（目前不对L1ICache设置表格）
    std::vector<std::vector<Tables>> workTables_;
};

#endif // __MEM_CACHE_PREFETCH_FILTER_PPF_HH__
