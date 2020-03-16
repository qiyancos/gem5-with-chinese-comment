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

#include "base/stats/info.hh"
#include "params/PerceptronPrefetchFilter.hh"
#include "mem/packet.hh"
#include "mem/cache/base.hh"
#include "mem/cache/cache.hh"
#include "mem/cache/prefetch_filter/ppf.hh"
#include "mem/cache/prefetch_filter/program_helper.hh"

namespace prefetch_filter {

inline bool isNumber(const std::string& str) {
    return str == "0" || atoi(str.c_str()) != 0;
}

int Feature::init(const std::string& feature) {
    // 将原始Feature依据空格进行拆分
    std::string word;
	std::vector<std::string> wordList;
    std::string::size_type start, end;
    start = 0;
    end = feature.find(' ');
    while (std::string::npos != end) {
        if (start != end) {
            word = feature.substr(start, end - start);
            wordList.push_back(word);
        }
        start = end + 1;
        end = feature.find(' ', start);
    }
    if (start != feature.length()) {
        word = feature.substr(start);
        wordList.push_back(word);
    }
    // 进行解析
    CHECK_WARN(wordList.size() > 2, "Not enough arguments for a feature");
    CHECK_WARN(isNumber(wordList.back()) &&
            isNumber(wordList[wordList.size() - 2]),
            "Not enough arguments for a feature");
    name_ = "";
    for (int i = 0; i < wordList.size() - 2; i++) {
        auto infoPair = PrefInfoIndexMap.find(wordList[i]);
        CHECK_WARN(infoPair != PrefInfoIndexMap.end(),
                "Unknow feature name \"%s\" in \"%s\"", wordList[i].c_str(),
                feature.c_str());
        infoIndexList_.push_back(infoPair->second.index_);
        name_ = name_ + wordList[i] + "_";
    }
    name_ = name_.substr(0, name_.size() - 1);
    startBits_ = atoi(wordList[wordList.size() - 2].c_str());
    bits_ = atoi(wordList.back().c_str());
    return 0;
}
    
int Feature::getIndex(const PrefetchInfo& info, uint16_t* index) {
    CHECK_ARGS(!infoIndexList_.empty(),
            "Can not get index before feature not initiated");
    uint32_t genIndex = 0;
    int result;
    for (int i = 0; i < infoIndexList_.size(); i++) {
        uint32_t temp;
        CHECK_ARGS(result = info.getInfo(infoIndexList_[i], &temp),
                "Failed to get a proper index");
        if (!result) {
            *index = 0xffff;
            return 0;
        } else if (i) {
            genIndex ^= temp;
        } else {
            genIndex = temp;
        }
    }
    uint32_t mask = -1;
    mask = mask >> startBits_ << (32 - bits_) >> (32 - bits_);
    *index = genIndex & mask;
    return 0;
}

uint16_t Feature::getSize() {
    return static_cast<uint16_t>(1) << bits_;
}

int PrefetchUsefulTable::init(const uint8_t counterBits, const uint32_t CCSize,
        const uint8_t CCAssoc, const uint32_t VCSize, const uint8_t VCAssoc,
        const int counterInit, BaseCache* cache,
        const int8_t CCTagBits, const int8_t VCTagBits) {
    counterBits_ = counterBits;
    counterInit_ = counterInit;
    counterCacheTagBits_ = CCTagBits;
    counterCacheSize_ = CCSize;
    counterCacheAssoc_ = CCAssoc;
    victimCacheTagBits_ = VCTagBits;
    victimCacheSize_ = VCSize;
    victimCacheAssoc_ = VCAssoc;
    if (!cache) {
        return 0;
    }
    valid_ = cache->enableHarmTable_;
    if (!valid_) {
        return 0;
    }
    CHECK_RET(counterCache_.init(CCTagBits, CCSize, CCAssoc,
            CounterEntry(counterBits, counterInit)),
            "Failed to init counter cache");
    CHECK_RET(victimCache_.init(VCSize, VCAssoc, VCTagBits),
            "Failed to init victim cache");
    return 0;
}

int PrefetchUsefulTable::updateHit(const uint64_t& addr) {
    if (!valid_) return 0;
    // 处理一个Demand Request命中了预取数据
    auto indexPair = prefInCache_.find(addr);
    CHECK_ARGS(indexPair != prefInCache_.end(),
            "Failed to find record for prefetch in the table");
    CounterEntry* entry;
    CHECK_ARGS(counterCache_.read(addr, &entry) == 1,
            "Failed to update counter in the counter cache when pref hit");
    entry->counter_++;
    return 0;
}

int PrefetchUsefulTable::updateMiss(const uint64_t& addr) {
    if (!valid_) return 0;
    // 如果发生了一个Demand Miss情况
    uint64_t* counterIndex;
    CHECK_ARGS(victimCache_.read(addr, &counterIndex),
            "Failed to find record for demand miss in the table");
    if (!counterIndex) {
        // 如果一个Demand Miss没有找到，那么可以跳过
        return 0;
    }
    CounterEntry* entry;
    CHECK_ARGS(counterCache_.read(*counterIndex, &entry) == 1,
            "Failed to update counter in the counter cache when pref hit");
    entry->counter_--;
    return 0;
}

int PrefetchUsefulTable::addPref(const uint64_t& prefAddr,
        const uint64_t evictedAddr) {
    if (!valid_) return 0;
    // 发生了新的预取替换Demand/Pref数据，相关记录被更新
    CHECK_ARGS(prefInCache_.find(prefAddr) == prefInCache_.end(),
            "Found prefetch already in record when adding prefetch");
    prefInCache_.insert(prefAddr);
    
    // 更新Victim Cache
    uint64_t replacedPrefAddr, replacedEvictedAddr;
    int result;
    CHECK_RET(result = victimCache_.write(evictedAddr, prefAddr,
            &replacedPrefAddr, &replacedEvictedAddr),
            "Failed to add prefetch victim info to victim cache");
    if (result == 0) {
        // 可能发生了问题，导致两个预取替换了同一个地址的数据
        // TODO 目前我们不处理，并报错
        CHECK_ARGS(false, "Two different prefetch replaced the %s",
                "same cache block");
    } else if (result == 2) {
        // 两个预取发生了冲突，导致新的预取替换信息和原来的替换信息发生冲突
        // 这时候应该将原来的预取删除，并进行更新；这会导致一个结果，
        // 被替换掉替换信息的预取之后只能进行有益性提升，无法进行有益性递减
        // TODO 我们暂时不处理该问题
    } else {
        // 新的预取替换的地址加入并且使用了空表项，可以继续执行
    }
    
    // 更新Counter Cache
    // 实际上由于Cache比Counter Cache大得多，必然会导致Counter冲突
    // TODO 这时候，我们应该将Counter的信息进行更新
    CHECK_ARGS((result = counterCache_.write(prefAddr,
            CounterEntry(counterBits_, counterInit_,
            prefAddr, evictedAddr))) == 1,
            "Failed to allocate new counter for prefetch (retcode: %d)",
            result);

    return 0;
}

int PrefetchUsefulTable::evictPref(const uint64_t& addr, uint8_t* counterPtr) {
    if (!valid_) return 0;
    // 进行删除的时候对应的Pref一定存在
    CHECK_ARGS(prefInCache_.find(addr) != prefInCache_.end(),
            "Can not find prefetch to be deleted in the prefetch record");
    prefInCache_.erase(addr);

    // 对应的Counter也一定的存在
    // TODO 结合上述冲突的情况，实际Counter可能不存在
    CounterEntry* counterEntry;
    CHECK_ARGS(counterCache_.read(addr, &counterEntry) == 1,
            "Counter correspond to prefetch not exists or error occurred");
    *counterPtr = counterEntry->counter_;

    // Victim可能不存在
    CHECK_RET(victimCache_.invalidate(counterEntry->evictedAddr_),
            "Failed to invalidate victim data in victim cache");
    
    CHECK_RET(counterCache_.invalidate(addr),
            "Failed to invalidate counter in counter cache");
    return 0;
}

} // namespace prefetch_filter

using namespace prefetch_filter;
using namespace Stats;

PerceptronPrefetchFilter::PerceptronPrefetchFilter(
        const PerceptronPrefetchFilterParams *p) :
        BasePrefetchFilter(p),
        cpuSharedTable_(p->cpu_shared_table),
        cacheSharedTable_(p->cache_shared_table),
        allSharedTable_(p->all_shared_table),
        allowUpgrade_(p->allow_upgrade),
        weightBits_(p->feature_weight_bits),
        weightInit_(p->feature_weight_init),
        counterInit_(p->counter_init_value),
        // 初始化预取的阈值设置
        prefThreshold_(p->prefetch_threshold),
        uselessPrefStep_(p->useless_prefetch_training_step) {
    // 初始化训练幅度
    trainStep_ = p->training_step;
    trainStep_.push_back(p->default_training_step);

    initFailFlag_ = false;
    int tempFlag;
    // 初始化Feature
    for (const std::string& featureStr : p->original_features) {
        featureList_.push_back(Feature());
        CHECK_WARN((tempFlag = featureList_.back().init(featureStr)) >= 0,
                "Failed to initiate original feature \"%s\" (Will be ignored)",
                featureStr.c_str());
        if (tempFlag < 0) {
            featureList_.pop_back();
        }
    }
    
    for (const std::string& featureStr : p->added_features) {
        featureList_.push_back(Feature());
        CHECK_WARN((tempFlag = featureList_.back().init(featureStr)) >= 0,
                "Failed to initiate added feature \"%s\" (Will be ignored)",
                featureStr.c_str());
        if (tempFlag < 0) {
            featureList_.pop_back();
        }
    }

    // 简单初始化信息表格
    workTables_.resize(1, std::vector<Tables>(1));
    Tables& LLCTable = workTables_[0][0];
    CHECK_WARN((tempFlag = LLCTable.prefetchTable_.init(p->prefetch_table_size,
            p->prefetch_table_assoc)) >= 0,
            "Failed to initiate prefetch table for init");
    initFailFlag_ |= tempFlag < 0;
    
    CHECK_WARN((tempFlag = LLCTable.rejectTable_.init(p->reject_table_size,
            p->reject_table_assoc)) >= 0,
            "Failed to initiate reject table for init");
    initFailFlag_ |= tempFlag < 0;
    
    for (int j = 0; j < featureList_.size(); j++) {
        CHECK_WARN((tempFlag = LLCTable.featureTable_[j].init(
                int8_t(-1), featureList_[j].getSize(), 1,
                SaturatedCounter(weightBits_, p->feature_weight_init))) >= 0,
                "Failed to initiate feature %s \"%s\" %s",
                "table for", featureList_[j].name_.c_str(), "in init");
        initFailFlag_ |= tempFlag < 0;
    }
    
    CHECK_WARN((tempFlag = LLCTable.oldPrefTable_.init(p->old_pref_table_size,
            p->old_pref_table_assoc)) >= 0,
            "Failed to initiate old prefetch table for init");
    initFailFlag_ |= tempFlag < 0;
    
    // 设置一个空的UsefulTable来存放配置信息
    prefUsefulTable_[nullptr] = PrefetchUsefulTable();
    CHECK_WARN((tempFlag = prefUsefulTable_[nullptr].init(p->counter_bits,
            p->counter_cache_size, p->counter_cache_assoc,
            p->victim_cache_size, p->victim_cache_assoc, counterInit_, 
            nullptr, p->counter_cache_tag_bits,
            p->victim_cache_tag_bits)) >= 0,
            "Failed to initiate useful table for LLC");
    initFailFlag_ |= tempFlag < 0;
}

int PerceptronPrefetchFilter::updateInstance(BasePrefetchFilter** ptr) {
    // 生成hash数值
    CHECK_RET_EXIT(BasePrefetchFilter::genHash("PerceptronPrefetchFilter"),
            "Failed to generate hash for a specific prefetch filter");
    // 调用父类函数更新指针
    CHECK_RET_EXIT(BasePrefetchFilter::updateInstance(ptr),
            "Failed to update pointer with global only instance");
    return 0;
}

int PerceptronPrefetchFilter::notifyCacheHit(BaseCache* cache,
        const PacketPtr& pkt, const uint64_t& hitAddr,
        const DataTypeInfo& info) {
    BasePrefetchFilter::notifyCacheHit(cache, pkt, hitAddr, info);
    // 对于指令Cache的预取不进行过滤和训练处理
    if (!cache->cacheLevel_) {
        return 0;
    }
    
    const uint64_t hitBlkAddr = hitAddr & cacheLineAddrMask_;
    const uint8_t cacheLevel = cache->cacheLevel_;
    if (info.source == Dmd && info.target == Pref) {
        std::set<BaseCache*> srcCaches;
        CHECK_RET_EXIT(usefulTable_[cache].findSrcCaches(hitBlkAddr,
                &srcCaches), "Failed to found prefetch recorded in cache");
        for (BaseCache* srcCache : srcCaches) {
            Tables& workTable = getTable(srcCache);
            // 一个Demand Request命中了预取数据，因此训练奖励
            CHECK_RET_EXIT(train(workTable, hitBlkAddr, cacheLevel, GoodPref),
                    "Failed to update training data when dmd hit pref");
        }
        // 更新预取有害性统计数据
        CHECK_RET_EXIT(prefUsefulTable_[cache].updateHit(hitBlkAddr),
                "Failed to update pref hit to prefetch useful table");
        // TODO 由于Demand命中，是否需要强制删除记录
    }
    /*
    else if (info.source == Pref && info.target == Dmd) {
        // 考虑预取会命中低等级的Cache，是正常现象，因此不属于无用预取
    }
    */
    return 0;
}

int PerceptronPrefetchFilter::notifyCacheMiss(BaseCache* cache,
        const PacketPtr& pkt, const uint64_t& combinedAddr,
        const DataTypeInfo& info) {
    BasePrefetchFilter::notifyCacheMiss(cache, pkt, combinedAddr, info);
    // 对于指令Cache的预取不进行过滤和训练处理
    if (!cache->cacheLevel_) {
        return 0;
    }
    
    const uint64_t combinedBlkAddr = combinedAddr & cacheLineAddrMask_;
    uint8_t cacheLevel = cache->cacheLevel_;
    if (info.source == Dmd) {
        if (info.target != Dmd) {
            // 对于一个Miss的Demand Request需要确定其有害性
            CHECK_RET_EXIT(prefUsefulTable_[cache].updateMiss(pkt->getAddr() &
                    cacheLineAddrMask_),
                    "Failed to update dmd miss in prefetch useful table");
        }

        // 如果一个预取Miss被Demand Request覆盖了，预取是无用的
        if (info.target == Pref) {
            std::set<BaseCache*> srcCaches;
            CHECK_RET_EXIT(usefulTable_[cache].findSrcCaches(pkt->getAddr() &
                    cacheLineAddrMask_, &srcCaches),
                    "Failed to found prefetch recorded in cache");
            for (BaseCache* srcCache : srcCaches) {
                Tables& workTable = getTable(srcCache);
                CHECK_RET_EXIT(train(workTable, combinedBlkAddr, cacheLevel,
                        UselessPref), "Failed to update training when dmd %s",
                        "miss added to prefetch miss");
            }
        }
    } else if (info.target == Dmd) {
        // 如果一个预取Miss并合并到了Demand Request Miss，属于无用预取
        for (BaseCache* srcCache : pkt->caches_) {
            Tables& workTable = getTable(srcCache);
            CHECK_RET_EXIT(train(workTable, pkt->getAddr() &
                    cacheLineAddrMask_, cacheLevel, UselessPref),
                    "Failed to update training %s",
                    "when pref miss added to demand miss"); 
        }
    }
    
    return 0;
}

int PerceptronPrefetchFilter::notifyCacheFill(BaseCache* cache,
        const PacketPtr &pkt, const uint64_t& evictedAddr,
        const DataTypeInfo& info) {
    // 对于指令Cache的预取不进行过滤和训练处理
    if (!cache->cacheLevel_) {
        return 0;
    }
    
    const uint64_t evictedBlkAddr = evictedAddr & cacheLineAddrMask_;
    uint8_t cacheLevel = cache->cacheLevel_;
    if (info.target == Pref) {
        std::set<BaseCache*> srcCaches;
        CHECK_RET_EXIT(usefulTable_[cache].findSrcCaches(evictedBlkAddr,
                &srcCaches), "Failed to found prefetch recorded in cache");
        
        uint8_t counter;
        // 说明一个预取数据被预取替换了，需要删除相应的记录
        CHECK_RET_EXIT(prefUsefulTable_[cache].evictPref(evictedBlkAddr,
                &counter), "Failed to remove old pref info when replaced");
        
        // 基于被替换的预取有害情况进行训练，只有开启有害统计，才会训练
        if (prefUsefulTable_[cache].valid_ && counter <= counterInit_) {
            for (BaseCache* srcCache : srcCaches) {
                Tables& workTable = getTable(srcCache);
                CHECK_RET_EXIT(train(workTable, evictedBlkAddr, cacheLevel,
                        counter == counterInit_ ? UselessPref : BadPref),
                        "Failed to update training when pref evicted"); 
            }
        }

        // 如果产生替换的也是预取，需要新增一个记录
        if (info.source == Pref) {
            CHECK_RET_EXIT(prefUsefulTable_[cache].addPref(pkt->getAddr() &
                    cacheLineAddrMask_, evictedBlkAddr),
                    "Failed to add new pref info when pref replaced pref");
        }
    } else if (info.source == Pref) {
        // 如果一个Demand Request数据被预取替换了需要新增一个记录
        CHECK_RET_EXIT(prefUsefulTable_[cache].addPref(pkt->getAddr() &
                cacheLineAddrMask_, evictedBlkAddr),
                "Failed to add new pref info when pref replaced dmd");
    }
    
    return 0;
}

int PerceptronPrefetchFilter::filterPrefetch(BaseCache* cache,
        const uint64_t& prefAddr, const PrefetchInfo& info) {
    if (!enableFilter_) {
        return BasePrefetchFilter::filterPrefetch(cache, prefAddr, info);
    }
    const int featureCount = featureList_.size();
    const uint8_t cacheLevel = cache->cacheLevel_;
    Tables& workTable = getTable(cache);

    // 计算权重加和
    uint16_t weightSum = 0;
    // 说明一下，如果我们标记某一个Index不可用，那么这个数值会被设置为
    // uint16_t的最大数值
    std::vector<uint16_t> indexes;
    int validFeatureCount = 0;
    for (int i = 0; i < featureCount; i++) {
        uint16_t featureIndex;
        CHECK_RET(featureList_[i].getIndex(info, &featureIndex),
                "Failed to get index for a feature");
        indexes.push_back(featureIndex);
        if (featureIndex != 0xffff) {
            validFeatureCount++;
            SaturatedCounter* weight;
            CHECK_RET(workTable.featureTable_[i].read(featureIndex, &weight),
                    "Failed to read weight for feature \"%s\" with index %d",
                    featureList_[i].name_.c_str(), featureIndex);
            weightSum = *weight + weightSum;
        }
    }

    // 取Weight均值，而非总和来避免可用Feature数量的影响
    // 该特性仅仅适用于软件，硬件设计无需要按照均值处理
    weightSum = double(weightSum) / validFeatureCount;
    
    // 依据权重加和进行预取更改和过滤
    uint8_t targetCacheLevel = 0;
    for (auto threshold : prefThreshold_) {
        if (weightSum >= threshold) {
            // 不会预取到L1Icache的
            if (!targetCacheLevel) {
                targetCacheLevel++;
                continue;
            }
            break;
        }
        targetCacheLevel++;
    }

    // 依据是否允许预取提升等级进行处理
    if (targetCacheLevel < cacheLevel && !allowUpgrade_) {
        targetCacheLevel = cacheLevel;
    }
    
    // 根据最后的结果更新统计数据
    if (targetCacheLevel <= maxCacheLevel_) {
        for (auto cpuId : cache->cpuIds_) {
            (*prefTarget_[cacheLevel][targetCacheLevel])[cpuId]++;
        }
    }
    
    // 依据最后的结果更新Prefetch Table和Reject Table
    CHECK_RET_EXIT(updateTable(workTable, prefAddr & cacheLineAddrMask_,
            targetCacheLevel, cache, indexes),
            "Failed to update prefetch table & reject table");
    return targetCacheLevel;
}

int PerceptronPrefetchFilter::invalidatePrefetch(BaseCache* cache,
        const uint64_t& prefAddr) {
    BasePrefetchFilter::invalidatePrefetch(cache, prefAddr);
    const uint64_t prefBlkAddr = prefAddr & cacheLineAddrMask_;
    
    // 首先查找相关的源Cache
    std::set<BaseCache*> srcCaches;
    CHECK_RET_EXIT(usefulTable_[cache].findSrcCaches(prefBlkAddr,
            &srcCaches), "Failed to found prefetch recorded in cache");
    
    uint8_t counter;
    // 说明一个预取数据被预取替换了，需要删除相应的记录
    CHECK_RET_EXIT(prefUsefulTable_[cache].evictPref(prefBlkAddr,
            &counter), "Failed to remove old pref info when replaced");
    
    // 基于被替换的预取有害情况进行训练，只有开启统计表格才会进行训练
    if (prefUsefulTable_[cache].valid_ && counter <= counterInit_) {
        for (BaseCache* srcCache : srcCaches) {
            Tables& workTable = getTable(srcCache);
            CHECK_RET_EXIT(train(workTable, prefBlkAddr, cache->cacheLevel_,
                    counter == counterInit_ ? UselessPref : BadPref),
                    "Failed to update training when pref evicted"); 
        }
    }
    
    return 0;
}

int PerceptronPrefetchFilter::initThis() {
    CHECK_ARGS(!initFailFlag_, "Failed when trying to initiate PPF");
    uint8_t cacheCount = caches_.size();
    // 重新初始化训练幅度
    CHECK_WARN(trainStep_.size() - 1 <= cacheCount,
            "Too many train steps are given.");
    trainStep_.resize(cacheCount, trainStep_.back());
    
    // 重新初始化预取阈值设置
    CHECK_WARN(prefThreshold_.size() <= cacheCount,
            "Too many threshold value are given");
    prefThreshold_.resize(cacheCount, weightInit_);

    // 初始化PPF表格
    if (!allSharedTable_ && maxCacheLevel_ > 1) {
        const Tables sample = workTables_[0][0];
        workTables_.clear();
        
        // 依据共享情况初始化表格大小
        int cpuSize = cpuSharedTable_ ? 1 : numCpus_;
        if (cacheSharedTable_) {
            // 针对Cache共享PPF表格的情况
            if (havePref_[maxCacheLevel_ - 1]) {
                workTables_.push_back(std::vector<Tables>(cpuSize, sample));
                for (int i = 0; i < cpuSize; i++) {
                    workTables_.back()[i].name_ = cpuSharedTable_ ?
                            "cpu_cache_shared" :
                            std::string("cache_shared_cpu_") +
                            std::to_string(i);
                }
            } else {
                workTables_.push_back(std::vector<Tables>());
            }
        } else {
            // 针对Cache不共享PPF表格的情况
            for (int i = 1; i < maxCacheLevel_; i++) {
                if (usePref_[i]) {
                    workTables_.push_back(std::vector<Tables>(cpuSize,
                            sample));
                    for (int j = 0; j < cpuSize; j++) {
                        workTables_.back()[j].name_ = cpuSharedTable_ ?
                                BaseCache::levelName_[j] + "_cpu_shared" :
                                BaseCache::levelName_[j] + "_cpu_" +
                                std::to_string(j);
                    }
                } else {
                    workTables_.push_back(std::vector<Tables>());
                }
            }
        }
        // 初始化LLC的表格
        if (usePref_[maxCacheLevel_]) {
            workTables_.push_back(std::vector<Tables>(1, sample));
            workTables_.back()[0].name_ =
                    BaseCache::levelName_[maxCacheLevel_];
        } else {
            workTables_.push_back(std::vector<Tables>());
        }
    } else if (allSharedTable_) {
        // 如果没有任何预取开启，则不生成表格
        if (havePref_[maxCacheLevel_]) {
            workTables_.clear();
        } else {
            workTables_[0][0].name_ = "all_shared";
        }
    }

    // 初始化有害性统计表格
    const uint8_t counterBits = prefUsefulTable_[nullptr].counterBits_;
    const int8_t CCTagBits = prefUsefulTable_[nullptr].counterCacheTagBits_;
    const uint32_t CCSize = prefUsefulTable_[nullptr].counterCacheSize_;
    const uint8_t CCAssoc = prefUsefulTable_[nullptr].counterCacheAssoc_;
    const int8_t VCTagBits = prefUsefulTable_[nullptr].victimCacheTagBits_;
    const uint32_t VCSize = prefUsefulTable_[nullptr].victimCacheSize_;
    const uint8_t VCAssoc = prefUsefulTable_[nullptr].victimCacheAssoc_;
    prefUsefulTable_.clear();
    bool usePrefNow = false;
    for (auto level : caches_) {
        usePrefNow |= usePref_[level.front()->cacheLevel_];
        for (BaseCache* cache : level) {
            prefUsefulTable_[cache] = PrefetchUsefulTable();
            if (usePrefNow) {
                CHECK_RET(prefUsefulTable_[cache].init(counterBits, CCSize,
                        CCAssoc, VCSize, VCAssoc, counterInit_, cache,
                        CCTagBits, VCTagBits),
                        "Failed to init prefetch useful table");
            }
        }
    }

    // 初始化部分统计变量
    for (int i = 0; i < cacheCount; i++) {
        prefTarget_.push_back(usePref_[i] ?
                std::vector<Stats::Vector*>(cacheCount) :
                std::vector<Stats::Vector*>());
    }

    return 0;
}

PerceptronPrefetchFilter::Tables& PerceptronPrefetchFilter::getTable(
        BaseCache* cache) {
    if (cache->cacheLevel_ == maxCacheLevel_) {
        CHECK_ARGS_EXIT(!workTables_.back().empty(),
                "Trying to get a worktable for cache without prefetch");
        return workTables_.back()[0];
    } else {
        uint8_t cpuIndex = cpuSharedTable_ ? 0 : *(cache->cpuIds_.begin());
        uint8_t cacheIndex = cacheSharedTable_ ? 0 : cache->cacheLevel_ - 1;
        CHECK_ARGS_EXIT(!workTables_[cacheIndex].empty(),
                "Trying to get a worktable for cache without prefetch");
        return workTables_[cacheIndex][cpuIndex];
    }
}

int PerceptronPrefetchFilter::train(Tables& workTable,
        const uint64_t& prefAddr, const uint8_t cacheLevel,
        const TrainType type) {
    // 查询Prefetch Table
    std::vector<uint16_t>* indexPtr = nullptr;
    CHECK_RET(workTable.prefetchTable_.read(prefAddr, &indexPtr),
            "Failed to find prefetch in prefetch table");
    
    // 查询Reject Table
    std::vector<uint16_t>* indexPtrTemp = nullptr;
    CHECK_RET(workTable.rejectTable_.read(prefAddr, &indexPtrTemp),
            "Failed to find prefetch in prefetch table");
    
    // 如果均不存在，则更新相关计数
    if (!(indexPtr || indexPtrTemp)) {
        Tables::PrefInfoEntry* prefInfo;
        int result;
        CHECK_RET(result = workTable.oldPrefTable_.read(prefAddr, &prefInfo),
                "Failed to find old prefetch info");
        if (result) {
            // 如果找到记录，则更新未训练的统计信息
            for (auto cache : prefInfo->caches_) {
                uint8_t cacheLevel = cache->cacheLevel_;
                for (auto cpuId : cache->cpuIds_) {
                    (*prefNotTrained_[cacheLevel])[cpuId]++;
                }
            }
            // 统计之后删除信息
            CHECK_RET(workTable.oldPrefTable_.invalidate(prefAddr),
                    "Failed to invalidate prefetch info");
        }
        return 0;
    }

    CHECK_ARGS((indexPtr != nullptr) ^ (indexPtrTemp != nullptr),
            "One prefetch should not exist %s",
            "both in prefetch table and reject table");
    indexPtr = indexPtr ? indexPtr : indexPtrTemp;
    
    int8_t step = 0;
    switch (type) {
    case GoodPref: step = trainStep_[cacheLevel]; break;
    case BadPref: step = -trainStep_[cacheLevel]; break;
    case UselessPref: step = -uselessPrefStep_; break;
    }

    // 进行权重更新
    SaturatedCounter* weight;
    for (int i = 0; i < featureList_.size(); i++) {
        /// 对于无效的Index不会进行训练
        if ((*indexPtr)[i] != 0xffff) {
            workTable.featureTable_[i].read((*indexPtr)[i], &weight);
            *weight += step;
        }
    }
    return 0;
}

int PerceptronPrefetchFilter::updateTable(Tables& workTable,
        const uint64_t& prefAddr, const uint8_t targetCacheLevel,
        BaseCache* srcCache, const std::vector<uint16_t>& indexes) {
    int result = 0;
    uint64_t evictedAddr;
    if (targetCacheLevel <= maxCacheLevel_) {
        // 如果进行了预取，则预取会被记录到Prefetch Table
        // 并从Reject Table删除
        std::vector<uint16_t> replacedIndexes;
        CHECK_RET(result = workTable.prefetchTable_.write(prefAddr, indexes,
                &evictedAddr, &replacedIndexes),
                "Failed to add new prefetch into prefetch table");
        
        // 如果对应的预取写入时命中，则生成新的表项包括替换，需要记录
        if (!result) {
            auto prefInfo = workTable.localPrefTable_.find(prefAddr);
            if (prefInfo != workTable.localPrefTable_.end()) {
                prefInfo->second.addCache(srcCache);
                prefInfo->second.timesPT_++;
            } else {
                std::set<BaseCache*> caches {srcCache};
                workTable.localPrefTable_[prefAddr] =
                        Tables::PrefInfoEntry(caches, 1, 0);
            }
        }

        // 如果替换的预取在另一个表格也不存在，删除记录表项
        if (result == 2 && workTable.rejectTable_.touch(evictedAddr) == 0) {
            // 依据表格出现次数更新统计信息
            CHECK_RET(updateWorkTableStats(workTable, evictedAddr),
                    "Failed to update work table statistics");
            // 预取被删除后更新权重统计
            CHECK_RET(updateFreqStats(workTable, replacedIndexes),
                    "Failed to update weight frequency stats");
        }
        CHECK_RET(workTable.rejectTable_.invalidate(prefAddr),
                "Failed to remove old prefetch from reject table");
    } else {
        // 如果预取被过滤，则预取会被记录到Reject Table
        // 并从Prefetch Table删除
        std::vector<uint16_t> replacedIndexes;
        CHECK_RET(workTable.rejectTable_.write(prefAddr, indexes,
                &evictedAddr, &replacedIndexes),
                "Failed to add new prefetch into reject table");
        
        // 如果对应的预取写入时命中，则生成新的表项包括替换，需要记录
        if (!result) {
            auto prefInfo = workTable.localPrefTable_.find(prefAddr);
            if (prefInfo != workTable.localPrefTable_.end()) {
                prefInfo->second.addCache(srcCache);
                prefInfo->second.timesRT_++;
            } else {
                std::set<BaseCache*> caches {srcCache};
                workTable.localPrefTable_[prefAddr] =
                        Tables::PrefInfoEntry(caches, 0, 1);
            }
        }
        
        // 如果替换的预取在另一个表格也不存在，删除记录表项
        if (result == 2 && workTable.prefetchTable_.touch(evictedAddr) == 0) {
            // 依据表格出现次数更新统计信息
            CHECK_RET(updateWorkTableStats(workTable, evictedAddr),
                    "Failed to update work table statistics");
            // 预取被删除后更新权重统计
            CHECK_RET(updateFreqStats(workTable, replacedIndexes),
                    "Failed to update weight frequency stats");
        }
        CHECK_RET(workTable.prefetchTable_.invalidate(prefAddr),
                "Failed to remove old prefetch from prefetch table");
    }
    return 0;
}

int PerceptronPrefetchFilter::updateFreqStats(Tables& workTable,
        const std::vector<uint16_t>& indexes) {
    // 进行权重查询和统计更新
    SaturatedCounter* weight;
    for (int i = 0; i < featureList_.size(); i++) {
        workTable.featureTable_[i].read(indexes[i], &weight);
        (*featureWeightFrequency_[workTable.statsIndex_][i])[*weight]++;
    }
    return 0;
}

int PerceptronPrefetchFilter::updateWorkTableStats(Tables& workTable,
        const uint64_t& prefAddr) {
    // 首先更新在不同表格的出现次数
    auto prefInfo = workTable.localPrefTable_.find(prefAddr);
    if (prefInfo != workTable.localPrefTable_.end()) {
        uint8_t PTTime = prefInfo->second.timesPT_;
        uint8_t RTTime = prefInfo->second.timesRT_;
        for (auto cache : prefInfo->second.caches_) {
            uint8_t level = cache->cacheLevel_;
            for (auto cpuId : cache->cpuIds_) {
                if (PTTime == 0) {
                    (*prefRejected_[level])[cpuId]++;
                } else if (RTTime == 0) {
                    (*prefAccepted_[level])[cpuId]++;
                } else {
                    (*prefThreshing_[level])[cpuId]++;
                }
            }
        }
    }
    // 将信息转移到被剔除信息表格中
    int result;
    CHECK_RET(result = workTable.oldPrefTable_.touch(prefAddr),
            "Failed to find evicted address in old prefetch table");
    if (result) {
        Tables::PrefInfoEntry* oldPrefInfo;
        CHECK_RET(workTable.oldPrefTable_.read(prefAddr, &oldPrefInfo),
                "Failed to read old prefetch in the table");
        oldPrefInfo->caches_.insert(prefInfo->second.caches_.begin(),
                prefInfo->second.caches_.end());
    } else {
        CHECK_RET(workTable.oldPrefTable_.write(prefAddr, prefInfo->second),
                "Failed to write old prefetch to the table");
    }
    // 更新完统计计数之后删除表项
    workTable.localPrefTable_.erase(prefAddr);
    return 0;
}

void PerceptronPrefetchFilter::regStats() {
    // 执行父类的统计数据初始化操作
    BasePrefetchFilter::regStats();
   
    // 初始化表格
    CHECK_RET_EXIT(initThis(), "Failed to initiate PPF.");
    
    uint8_t cacheCount = caches_.size();
    for (int i = 0; i < cacheCount; i++) {
        const std::string srcCacheName = BaseCache::levelName_[i];
        if (usePref_[i]) {
            prefAccepted_[i] = new Stats::Vector();
            prefAccepted_[i]
                    ->name(name() + ".accepted_prefetch_from_" + srcCacheName)
                    .desc(std::string("Number of prefetch requests from ") +
                            srcCacheName + " accepted.")
                    .flags(total);
            
            prefRejected_[i] = new Stats::Vector();
            prefRejected_[i]
                    ->name(name() + ".rejected_prefetch_from_" + srcCacheName)
                    .desc(std::string("Number of prefetch requests from ") +
                            srcCacheName + " rejected.")
                    .flags(total);
            
            prefThreshing_[i] = new Stats::Vector();
            prefThreshing_[i]
                    ->name(name() + ".threshing_prefetch_from_" + srcCacheName)
                    .desc(std::string("Number of threshing prefetch requests "
                            "from ") + srcCacheName + ".")
                    .flags(total);
            
            prefNotTrained_[i] = new Stats::Vector();
            prefNotTrained_[i]
                    ->name(name() + ".untrained_prefetch_from_" + srcCacheName)
                    .desc(std::string("Number of prefetch requests not trained"
                            "from ") + srcCacheName + ".")
                    .flags(total);
            
            for (int j = 0; j < cacheCount; j++) {
                const std::string tgtCacheName = BaseCache::levelName_[j];
                prefTarget_[i][j] = new Stats::Vector();
                prefTarget_[i][j]
                        ->name(name() + ".prefetch_sent_to_" + tgtCacheName +
                                "_from_" + srcCacheName)
                        .desc(std::string(
                                "Number of prefetch requests sent to")
                                + tgtCacheName + " from " + srcCacheName)
                        .flags(total);
            }

            // 按照CPU初始化
            for (int j = 0; j < numCpus_; j++) {
                const std::string cpuId = std::to_string(j);
                prefAccepted_[i]->subname(j, std::string("cpu") + cpuId);
                prefRejected_[i]->subname(j, std::string("cpu") + cpuId);
                prefThreshing_[i]->subname(j, std::string("cpu") + cpuId);
                prefNotTrained_[i]->subname(j, std::string("cpu") + cpuId);
                for (int k = 0; k < cacheCount; k++) {
                    prefTarget_[i][k]->subname(j, std::string("cpu") + cpuId);
                }
            }
        } else {
            prefAccepted_[i] = &emptyStatsVar_;
            prefRejected_[i] = &emptyStatsVar_;
            prefThreshing_[i] = &emptyStatsVar_;
            prefNotTrained_[i] = &emptyStatsVar_;
        }
    }
    
    CHECK_ARGS_EXIT(weightBits_ > 0, "Bit number of the feature weight must%s",
            " be greater than zero");
    int weightNum = 1 << (weightBits_ - 1);
    const std::vector<Stats::Vector*> sample(featureList_.size());
    // 为LLC设置Stats变量
    int i = 0;
    for (auto& cacheTables : workTables_) {
        for (auto& workTable : cacheTables) {
            featureWeightFrequency_.push_back(sample);
            workTable.statsIndex_ = i;
            for (int j = 0; j < featureList_.size(); j++) {
                featureWeightFrequency_[i][j] = new Stats::Vector();
                featureWeightFrequency_[i][j]
                        ->name(name() + ".feature_" + featureList_[i].name_ + 
                                "_weight_in_" + workTable.name_)
                        .desc(std::string("Time of appearence of a specific") +
                                " weight for a feature.")
                        .flags(total);
                for (int k = 0; k < weightNum; k++) {
                    featureWeightFrequency_[i][j]->subname(k,
                            std::to_string(k));
                }
            }
            i++;
        }
    }
}

PerceptronPrefetchFilter* PerceptronPrefetchFilterParams::create() {
    return new PerceptronPrefetchFilter(this);
}
