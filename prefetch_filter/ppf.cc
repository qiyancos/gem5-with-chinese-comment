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

#include <cstdint>

#include "mem/cache/prefetch_filter/ppf.hh"

namespace prefetch_filter {

#define isNumber(String) { \
    return String == "0" || atoi(String.c_str()) != 0; \
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
        word = str.substr(start);
        wordList.push_back(word);
    }
    // 进行解析
    CHECK_ARGS(wordList.size() > 2, "Not enough arguments for a feature");
    CHECK_ARGS(isNumber(wordList.back()) &&
            isNumber(wordList[wordList.size() - 2]),
            "Not enough arguments for a feature");
    name_ = "";
    for (int i = 0; i < wordList.size() - 2; i++) {
        auto infoPair = IndexMap.find(wordList[i]);
        CHECK_ARGS(infoPair != IndexMap.end(),
                "Unknow feature name \"%s\" in \"%s\"", wordList[i].c_str(),
                feature.c_str());
        infoIndexList_.push_back(infoPair->second);
        name_ = name_ + wordList[i] + "_";
    }
    name_ = name_.substr(0, name_.size() - 1);
    startBits_ = atoi(wordList[wordList.size() - 2].c_str());
    bits_ = atoi(wordList.back().c_str());
    return 0;
}
    
uint16_t Feature::getIndex(const PrefetchInfo& info) {
    CHECK_ARGS(!infoIndexList.empty(),
            "Can not get index before feature not initiated");
    uint32_t index = info.getInfo(infoIndexList_[0]);
    for (int i = 1; i < infoIndexList_.size(); i++) {
        index ^= info.getInfo(infoIndexList_[i]);
    }
    uint32_t mask = -1;
    mask = mask >> startBits_ << (32 - bits) >> (32 - bits);
    return index & mask;
}

uint16_t Feature::getSize() {
    return static_cast<uint16_t>(1) << bits_;
}

int PrefetchUsefulTable::init(const uint8_t counterBits, const uint32_t CCsize,
        const uint8_t CCAssoc, const uint32_t VCSize, const uint8_t VCAssoc,
        BaseCache* cache, const int8_t CCTagBits) {
    counterBits_ = counterBits;
    counterCacheTagBits = CCTagBits;
    counterCacheSize_ = CCSize;
    counterCacheAssoc_ = CCAssoc;
    victimCacheSize_ = VCSize;
    victimCacheAssoc_ = VCAssoc;
    if (!cache) {
        return 0;
    }
    valid_ = cache->enableHarmTable_;
    if (!valid_) {
        return 0;
    }
    CHECK_RET(counterCache_.init(CCSize, CCAssoc, CounterEntry(counterBits),
            CCTagBits), "Failed to init counter cache");
    CHECK_RET(victimCache_.init(VCSize, VCAssoc),
            "Failed to init victim cache");
    return 0;
}

int PrefetchUsefulTable::updateHit(const uint64_t& addr) {
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
    // 如果发生了一个Demand Miss情况
    uint64_t* counterIndex;
    CHECK_ARGS(victimCache_.read(addr, &counterIndex),
            "Failed to find record for demand miss in the table");
    if (!counterIndex) {
        // 如果一个Demand Miss没有找到，那么可以跳过
        return 0;
    }
    CounterEntry* entry;
    CHECK_ARGS(counterCache_.read(counterIndex, &entry) == 1,
            "Failed to update counter in the counter cache when pref hit");
    entry->counter_--;
    return 0;
}

int PrefetchUsefulTable::addPref(const uint64_t& prefAddr,
        const uint64_t evictedAddr) {
    // 发生了新的预取替换Demand/Pref数据，相关记录被更新
    CHECK_ARGS(prefInCache_.find(prefAddr) == prefInCache_.end(),
            "Found prefetch already in record when adding prefetch");
    prefInCache.insert(prefAddr);
    
    // 更新Victim Cache
    uint64_t replacedPrefAddr, replacedEvictedAddr;
    int result;
    CHECK_RET(result = victimCache_.write(evictedAddr, prefAddr,
            &repalcedPrefAddr, &replacedEvictedAddr),
            "Failed to add prefetch victim info to victim cache");
    if (result == 0) {
        // 可能发生了问题，导致两个预取替换了同一个地址的数据
        // TODO 目前我们不处理，并报错
        CHECK_ARGS(false, "Two different prefetch replaced the %s",
                "same cache block");
    } else if (result == 2) {
        // 两个预取发生了冲突，导致新的预取替换信息和原来的替换信息发生冲突
        // 这时候应该将原来的预取删除，并进行更新
        // TODO 我们暂时不处理该问题
    } else {
        // 新的预取替换的地址加入并且使用了空表项，可以继续执行
    }
    
    // 更新Counter Cache
    CHECK_ARGS((result = counterCache_.write(prefAddr,
            CounterEntry(counterBits_, prefAddr, evictedAddr))) == 1,
            "Failed to allocate new counter for prefetch (retcode: %d)",
            result);

    return 0;
}

int PrefetchUsefulTable::evictPref(const uint64_t& addr, int8_t* counterPtr) {
    // 进行删除的时候对应的Pref一定存在
    CHECK_ARGS(prefInCache_.find(addr) != prefInCache_.end(),
            "Can not find prefetch to be deleted in the prefetch record");
    prefInCache_.erase(addr);

    // 对应的Counter也一定的存在
    CounterEntry* counterEntry;
    CHECK_ARGS(counterCache_.read(addr, &counterEntry) == 1,
            "Counter correspond to prefetch not exists or error occurred");
    *counterPtr = counterEntry->counter_;

    // Victim可能不存在
    CHECK_RET(victimCache_.invalidate(counterEntry->evictedAddr),
            "Failed to invalidate victim data in victim cache");
    
    CHECK_RET(counterCache_.invalidate(addr),
            "Failed to invalidate counter in counter cache");
    return 0;
}

PerceptronPrefetchFilter::PerceptronPrefetchFilter(
        const PerceptronPrefetchFilterParams *p) :
        BasePrefetchFilter(p),
        cpuSharedTable_(p->cpu_shared_table),
        cacheSharedTable_(p->cache_shared_table),
        allSharedTable_(p->all_shared_table),
        allowUpgrade_(p->allow_upgrade),
        weightBits_(p->feature_weight_bits),
        l1PrefThreshold_(p->l1_threshold),
        l2PrefThreshold_(p->l1_threshold),
        l3PrefThreshold_(p->l1_threshold),
        uselssPrefStep_(p->useless_prefetch_training_step) {
    // 初始化训练幅度
    trainStep_[0] = p->l1_training_step;
    trainStep_[1] = p->l2_training_step;
    trainStep_[2] = p->l3_training_step;

    // 初始化Feature
    for (const std::string& featureStr : original_features) {
        featureList_.push_back(Feature());
        initFailFlag = featureList_.back().init(featureStr) < 0;
    }
    CHECK_WARN(initFailFlag, "Failed to initiate original feature \"%s\"",
            featureStr.c_str());
    
    for (const std::string& featureStr : added_features) {
        featureList_.push_back(Feature());
        initFailFlag = featureList_.back().init(featureStr) < 0;
    }
    CHECK_WARN(initFailFlag, "Failed to initiate added feature \"%s\"",
            featureStr.c_str());

    // 简单初始化信息表格
    initFailFlag = LLCTable_.prefetchTable_.init(p->prefetch_table_size,
            p->prefetch_table_assoc) < 0;
    CHECK_WARN(initFailFlag, "Failed to initiate prefetch table for LLC");
    
    initFailFlag = LLCTable_.rejectTable_.init(p->reject_table_size,
            p->reject_table_assoc) < 0;
    CHECK_WARN(initFailFlag, "Failed to initiate reject table for LLC");
    
    for (int j = 0; j < featureList_.size(); j++) {
        initFailFlag = LLCTable_.featureTable_[j].init(
                featureList_[j].getSize(), 1,
                SaturatedCounter(weightBits_)) < 0;
        CHECK_WARN(initFailFlag, "Failed to initiate feature %s \"%s\" %s",
                "table for", featureList_[j].name_.c_str(), "in LLC");
    }
    
    prefUsefulTable_[nullptr] = PrefetchUsefulTable();
    initFailFlag = prefUsefulTable_[nullptr].init(p->counter_bits,
            p->counter_cache_size, p->counter_cache_assoc,
            p->victim_cache_size, p->victim_cache_assoc,
            nullptr, p->counter_cache_tag_bits) < 0;
    CHECK_WARN(initFailFlag, "Failed to initiate useful table for LLC");
    
    initFailFlag = LLCTable_.oldPrefTable_.init(p->old_pref_table_size,
            p->old_pref_table_assoc) < 0;
    CHECK_WARN(initFailFlag, "Failed to initiate old prefetch table %s",
            "for LLC");
}

int PerceptronPrefetchFilter::notifyCacheHit(BaseCache* cache,
        const PacketPtr& pkt, const DataTypeInfo& info) {
    BasePrefetchFilter::notifyCacheHit(cache, pkt, info);
    // 对于指令Cache的预取不进行过滤和训练处理
    if (!cache->cacheLevel_) {
        return 0;
    }
    
    const uint8_t cacheLevel = cache->cacheLevel_;
    if (info.source == Dmd && info.target == Pref) {
        PrefetchUsefulInfo* info;
        CHECK_RET(usefulTable_[cache].findPrefInfo(pkt->addr, &info),
                "Failed to found prefetch recorded in cache");
        Tables& workTable = getTable(info->srcCache_);
        // 一个Demand Request命中了预取数据，因此训练奖励
        CHECK_RET(train(workTable, pkt->addr, cacheLevel, GoodPref),
                "Failed to update training data when dmd hit pref");
        
        // 更新预取有害性统计数据
        CHECK_RET(prefUsefulTable_[cache].updateHit(pkt->addr),
                "Failed to update pref hit to prefetch useful table");
    }
    // 考虑预取会命中低等级的Cache，是正常现象，因此不属于无用预取
    /*
    else if (info.source == Pref && info.target == Dmd) {
        PrefetchUsefulInfo* info;
        CHECK_RET(usefulTable_[cache].findPrefInfo(pkt->addr, &info),
                "Failed to found prefetch recorded in cache");
        Tables& workTable = getTable(info->srcCache_);
        // 一个预取命中了一个Demand数据，那么这个预取是一个无用的预取
        CHECK_RET(train(workTable, pkt->addr, cacheLevel, UselessPref),
                "Failed to update training data when pref hit dmd");
    }
    */
    return 0;
}

int PerceptronPrefetchFilter::notifyCacheMiss(BaseCache* cache,
        const PacketPtr& pkt, const DataTypeInfo& info,
        const uint64_t& combinedAddr) {
    BasePrefetchFilter::notifyCacheMiss(cache, pkt, info, combinedAddr);
    // 对于指令Cache的预取不进行过滤和训练处理
    if (!cache->cacheLevel_) {
        return 0;
    }
    
    uint8_t cacheLevel = cache->cacheLevel_;
    if (info.source == Dmd) {
        // 对于一个Miss的Demand Request需要确定其有害性
        CHECK_RET(prefUsefulTable_[cache].updateMiss(pkt->addr),
                "Failed to update dmd miss in prefetch useful table");
        
        // 如果一个预取Miss被Demand Request覆盖了，预取是无用的
        if (info.target == Pref) {
            PrefetchUsefulInfo* info;
            CHECK_RET(usefulTable_[cache].findPrefInfo(pkt->addr, &info),
                    "Failed to found prefetch recorded in cache");
            Tables& workTable = getTable(info->srcCache_);
            CHECK_RET(train(workTable, combinedAddr, cacheLevel, UselessPref),
                    "Failed to update training when dmd %s",
                    "miss added to prefetch miss");
            }
        }
    } else if (info.target == Dmd) {
        // 如果一个预取Miss并合并到了Demand Request Miss，属于无用预取
        Tables& workTable = getTable(pkt->srcCache_);
        CHECK_RET(train(workTable, pkt->addr, cacheLevel, UselessPref),
                "Failed to update training when pref miss %s",
                "added to demand miss"); 
    }
    
    return 0;
}

int PerceptronPrefetchFilter::notifyCacheFill(BaseCache* cache,
        const PacketPtr &pkt, const DataTypeInfo& info,
        const uint64_t& evictedAddr) {
    // 对于指令Cache的预取不进行过滤和训练处理
    if (!cache->cacheLevel_) {
        return 0;
    }
    
    Tables& workTable = getTable(cache);
    uint8_t cacheLevel = cache->cacheLevel_ - 1;
    if (info.target == Pref) {
        int8_t counter;
        // 说明一个预取数据被预取替换了，需要删除相应的记录
        CHECK_RET(prefUsefulTable_[cache].evictPref(evictedAddr, &counter),
                "Failed to remove old pref info when replaced");
        
        // 基于被替换的预取有害情况进行训练
        PrefetchUsefulInfo* info;
        CHECK_RET(usefulTable_[cache].findPrefInfo(evictedAddr, &info),
                "Failed to found prefetch recorded in cache");
        workTable = getTable(info->srcCache_);
        if (counter <= 0) {
            CHECK_RET(train(workTable, evictedAddr, cacheLevel,
                    counter == 0 ? UselessPref : BadPref),
                    "Failed to update training when pref evicted"); 
        }

        // 如果产生替换的也是预取，需要新增一个记录
        if (info.source == Pref) {
            CHECK_RET(prefUsefulTable_[cache].addPref(pkt->addr,
                    evictedAddr), "Failed to add new pref info when %s",
                    "pref replaced pref");
        }
    } else if (info.source == Pref) {
        // 如果一个Demand Request数据被预取替换了需要新增一个记录
        CHECK_RET(prefUsefulTable_[cache].addPref(pkt->addr,
                evictedAddr), "Failed to add new pref info when %s",
                "pref replaced dmd");
    }
    
    return 0;
}

int PerceptronPrefetchFilter::filterPrefetch(BaseCache* cache,
        const PacketPtr &pkt, const PrefetchInfo& info) {
    const int featureCount = featureList_.size();
    const uint8_t cpuId = sharedTable_ ? 1 : pkt->srcCpuId_;
    const uint8_t cacheLevel = cache->cacheLevel_;
    TablesTable& workTable = getTable(cache);

    // 计算权重加和
    uint16_t weightSum = 0;
    std::vector<uint16_t> indexes;
    for (int i = 0; i < featureCount; i++) {
        uint16_t featureIndex = featureList_[i].getIndex(info);
        indexes.push_back(featureIndex);
        weightSum += workTable.featureTable_.read(featureIndex);
    }

    // 依据权重加和进行预取更改和过滤
    uint8_t targetCacheLevel = 4;
    if (weightSum >= l1PrefThreshold_) {
        targetCacheLevel = 1;
    } else if (weightSum >= l2PrefThreshold) {
        targetCacheLevel = 2;
    } else if (weightSum >= l3PrefThreshold) {
        targetCacheLevel = 3;
    }

    // 依据是否允许预取提升等级进行处理
    if (targetCacheLevel < cacheLevel && !allowUpgrade_) {
        targetCacheLevel = cacheLevel;
    }
    
    // 根据最后的结果更新统计数据
    switch (targetCacheLevel) {
    case 1: (*prefToL1_[cacheLevel])[cpuId]++; break;
    case 2: (*prefToL2_[cacheLevel])[cpuId]++; break;
    case 3: (*prefToL3_[cacheLevel])[cpuId]++; break;
    }
    
    // 依据最后的结果更新Prefetch Table和Reject Table
    CHECK_RET(updateTable(workTable, pkt->addr, targetCacheLevel, cpuId,
            cacheLevel, indexes), "Failed to update prefetch table & %s",
            "reject table");
    return targetCacheLevel;
}

int PerceptronPrefetchFilter::init() {
    CHECK_ARGS(!initFailFlag, "Failed when trying to initiate PPF");
    if (!allSharedTable_) {
        if (maxCacheLevel_ == 1) {
            return 0;
        }
        
        // 依据共享情况初始化表格大小
        int cpuSize = cpuSharedTable_ ? 1 : numCpus_;
        int cacheSize = cacheSharedTable_ ? 1 : maxCacheLevel_ - 1;
        noneLLCTables_.resize(cpuSize, std::vector<Tables>(cacheSize,
                LLCTable_));
    }

    // 初始化有害性统计表格
    const uint8_t counterBits = prefUsefulTable_[nullptr].counterBits_;
    const int8_t CCTagBIts = prefUsefulTable_[nullptr].counterCacheTagBits_;
    const uint32_t CCsize = prefUsefulTable_[nullptr].counterCacheSize_;
    const uint8_t CCAssoc = prefUsefulTable_[nullptr].counterCacheAssoc_;
    const uint32_t VCsize = prefUsefulTable_[nullptr].victimCacheSize_;
    const uint8_t VCsize = prefUsefulTable_[nullptr].victimCacheAssoc_;
    prefUsefulTable_.clear();
    for (auto level : caches_) {
        for (BaseCache* cache : level) {
            prefUsefulTable_[cache] = PrefetchUsefulTable();
            CHECK_RET(prefUsefulTable_[cache].init(counterBits, CCsize,
                    CCAssoc, VCSize, VCAssoc, cache, CCTagBits),
                    "Failed to init prefetch useful table");
        }
    }
    return 0;
}

Tables& PerceptronPrefetchFilter::getTable(BaseCache* cache) {
    if (cache->cacheLevel_ == maxCacheLevel_) {
        return LLCTable_;
    } else {
        uint8_t cpuIdx = cpuSharedTable_ ? 0 : *(cache->cpuIds_.begin());
        uint8_t cacheIdx = cacheSharedTable_ ? 0 : cache->cacheLevel_ - 1;
        return noneLLCTable_[cpuIdx][cacheIdx];
    }
    return LLCTable_;
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
    CHECK_RET(workTable.rejctTable_.read(prefAddr, &indexPtrTemp),
            "Failed to find prefetch in prefetch table");
    
    // 如果均不存在，则更新相关计数
    if (!(indexPtr || indexPtrTemp)) {
        BaseCache* srcCache;
        int result = oldPrefTable_.read(prefAddr, &srcCache);
        CHECK_RET(result, "Failed to find old prefetch info");
        *(prefNotTrained_[srcCache->cacheLevel_ - 1]
                [*(srcCache->cpuIds_.begin())]) += result;
        return 0;
    }

    CHECK_ARGS(indexPtr ^ indexPtrTemp, "One prefetch should not exist %s",
            "both in prefetch table and reject table");
    indexPtr = indexPtr ? indexPtr : indexPtrTemp;
    
    int8_t step;
    switch (type) {
    case GoodPref: step = trainStep_[cacheLevel]; break;
    case BadPref: step = -trainStep_[cacheLevel]; break;
    case UselessPref: step = -uselessPrefStep_; break;
    }

    // 进行权重更新
    SaturatedCounter* weightPtr;
    for (int i = 0; i < featureList_.size(); i++) {
        workTable.featureTable_[i].read(*indexPtr[i], &weightPtr);
        *weightPtr += step;
    }
    return 0;
}

int PerceptronPrefetchFilter::updateTable(Tables& workTable,
        const uint8_t targetCacheLevel, const uint64_t& prefAddr,
        const uint8_t cpuId, const uint8_t srcCacheLevel,
        const std::vector<uint16_t>& indexes) {
    int result;
    uint64_t evictedAddr;
    if (targetCacheLevel < 4) {
        // 如果进行了预取，则预取会被记录到Prefetch Table
        // 并从Reject Table删除
        CHECK_RET(result = workTable.pefetchTable_.write(prefAddr, indexes,
                &evictedAddr), "Failed to add new prefetch into %s",
                "prefetch table");
        
        // 如果对应的预取写入时命中，则生成新的表项包括替换，需要记录
        if (!result) {
            if (workTable.prefAppearTime_.find(prefAddr) ==
                    prefAppearTime_.end()) {
                workTable.prefAppearTime_[prefAddr] =
                        std::pair<uint8_t, uint8_t>(1, 0);
            } else {
                workTable.prefAppearTime_[prefAddr]->first++;
            }
        }
        
        // 如果替换的预取在另一个表格也不存在，删除记录表项
        if (evictedAddr && workTable.rejectTable_.touch(evictedAddr) == 0) {
            auto evictedPref = workTable.prefAppearTime_.find(evictedAddr);
            if (evictedPref != workTable.prefAppearTime_.end()) {
                uint8_t PTTime = evictedPref->second.first;
                uint8_t RTTime = evictedPref->second.second;
                if (PTTime == 0) {
                    (*prefRejected_[srcCacheLevel])[cpuId]++;
                } else if (RTTime == 0) {
                    (*prefAccepted_[srcCacheLevel])[cpuId]++;
                } else {
                    (*prefThreshing_[srcCacheLevel])[cpuId]++;
                }
            }
            workTable.prefAppearTime_.erase(evictedAddr);
        }
        CHECK_RET(workTable.rejectTable_.invalidate(pkt->addr),
                "Failed to remove old prefetch from reject table");
    } else {
        // 如果预取被过滤，则预取会被记录到Reject Table
        // 并从Prefetch Table删除
        CHECK_RET(workTable.rejectTable_.write(pkt->addr, indexes, &evictedAddr),
                "Failed to add new prefetch into reject table");
        
        // 如果对应的预取写入时命中，则生成新的表项包括替换，需要记录
        if (!result) {
            if (workTable.prefAppearTime_.find(prefAddr) ==
                    prefAppearTime_.end()) {
                workTable.prefAppearTime_[prefAddr] =
                        std::pair<uint8_t, uint8_t>(0, 1);
            } else {
                workTable.prefAppearTime_[prefAddr].second++;
            }
        }
        
        // 如果替换的预取在另一个表格也不存在，删除记录表项
        if (evictedAddr && workTable.rejectTable_.touch(evictedAddr) == 0) {
            auto evictedPref = workTable.prefAppearTime_.find(evictedAddr);
            if (evictedPref != prefAppearTime_.end()) {
                uint8_t PTTime = evictedPref->second.first;
                uint8_t RTTime = evictedPref->second.second;
                if (PTTime == 0) {
                    (*prefRejected_[srcCacheLevel])[cpuId]++;
                } else if (RTTime == 0) {
                    (*prefAccepted_[srcCacheLevel])[cpuId]++;
                } else {
                    (*prefThreshing_[srcCacheLevel])[cpuId]++;
                }
            }
            workTable.prefAppearTime_.erase(evictedAddr);
        }
        CHECK_RET(workTable.prefetchTable_.invalidate(pkt->addr),
                "Failed to remove old prefetch from prefetch table");
    }
    return 0;
}

void PerceptronPrefetchFilter::regStats() {
    // 执行父类的统计数据初始化操作
    BasePrefetcherFilter::regStats();
   
    // 初始化表格
    CHECK_RET_EXIT(init(), "Failed to initiate PPF.");

    for (i = 0; i < 3; i++) {
        if (usePref_[i + 1]) {
            prefAccpeted_[i] = new Stats::Vector();
            prefAccpeted_[i]
                    ->name(name() + ".accepted_prefetch_from_" +
                            BaseCache::levelName_[i + 1])
                    .desc(std::string("Number of prefetch requests from ") +
                            BaseCache::levelName_[i] + " accepted.")
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefAccpeted_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefAccepted_[i] = &emptyStatsVar_;
        }
    }
    
    for (i = 0; i < 3; i++) {
        if (usePref_[i + 1]) {
            prefRejected_[i] = new Stats::Vector();
            prefRejected_[i]
                    ->name(name() + ".rejected_prefetch_from_" +
                            BaseCache::levelName_[i + 1])
                    .desc(std::string("Number of prefetch requests from ") +
                            BaseCache::levelName_[i] + " rejected.")
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefRejected_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefRejected_[i] = &emptyStatsVar_;
        }
    }
    
    for (i = 0; i < 3; i++) {
        if (usePref_[i + 1]) {
            prefThreshing_[i] = new Stats::Vector();
            prefThreshing_[i]
                    ->name(name() + ".threshing_prefetch_from_" +
                            BaseCache::levelName_[i + 1])
                    .desc(std::string("Number of threshing prefetch requests "
                            " from ") + BaseCache::levelName_[i] + ".")
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefThreshing_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefThreshing_[i] = &emptyStatsVar_;
        }
    }
    
    for (i = 0; i < 3; i++) {
        if (usePref_[i + 1]) {
            prefNotTrained_[i] = new Stats::Vector();
            prefNotTrained_[i]
                    ->name(name() + ".untrained_prefetch_from_" +
                            BaseCache::levelName_[i + 1])
                    .desc(std::string("Number of prefetch requests not trained"
                            " from ") + BaseCache::levelName_[i] + ".")
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefNotTrained_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefNotTrained_[i] = &emptyStatsVar_;
        }
    }

    for (i = 0; i < 3; i++) {
        if (usePref_[3 - i] && maxCacheLevel_ > 0) {
            prefToL1_[i] = new Stats::Vector();
            prefToL1_[i]
                    ->name(name() + ".prefetch_sent_to_L1_from" +
                            BaseCache::levelName_[3 - i])
                    .desc(std::string("Number of prefetch requests sent to " +
                            "L1 from " + BaseCache::levelName_[3 - i] + ".")
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefToL1_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefToL1_[i] = &emptyStatsVar_;
        }
    }
    
    for (i = 0; i < 3; i++) {
        if (usePref_[3 - i] && maxCacheLevel_ > 1) {
            prefToL2_[i] = new Stats::Vector();
            prefToL2_[i]
                    ->name(name() + ".prefetch_sent_to_L2_from" +
                            BaseCache::levelName_[3 - i])
                    .desc(std::string("Number of prefetch requests sent to " +
                            "L2 from " + BaseCache::levelName_[3 - i] + ".")
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefToL2_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefToL2_[i] = &emptyStatsVar_;
        }
    }

    for (i = 0; i < 3; i++) {
        if (usePref_[3 - i] && maxCacheLevel_ > 2) {
            prefToL3_[i] = new Stats::Vector();
            prefToL3_[i]
                    ->name(name() + ".prefetch_sent_to_L3_from" +
                            BaseCache::levelName_[3 - i])
                    .desc(std::string("Number of prefetch requests sent to " +
                            "L3 from " + BaseCache::levelName_[3 - i] + ".")
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefToL3_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefToL3_[i] = &emptyStatsVar_;
        }
    }
    
    CHECK_ARGS_EXIT(weightBits_ > 0, "Bit number of the feature weight must%s",
            " be greater than zero");
    int weightNum = 1 << (weightBits_ - 1);
    featureWeightFrequency_.resize(featureList.size(),
            std::vector<Stats::Vector*> (weightNum));
    for (int i = 0; i < featureList_.size(); i++) {
        for (int j = 0; j < weightNum; j++) {
            std::string weightStr = std::to_string(j);
            // 需要至少一个缓存层级开启了预取器
            if (usePref_[0] || usePref_[1] || usePref_[2] || usePref_[3]) {
                featureWeightFrequency_[i][j] = new Stats::Vector();
                featureWeightFrequency_[i][j]
                        ->name(name() + ".feature_" + featureList_[i].name_ + 
                                "_weight_" + weightStr)
                        .desc(std::string("Time of appearence of a specific") +
                                " weight for a feature.")
                        .flag(total);
                for (j = 0; j < numCpus_; j++) {
                    featureWeightFrequency[i]->subname(j, std::string("cpu") +
                            std::to_string(j));
                }
            } else {
                featureWeightFrequency_[i][j] = &emptyStatsVar_;
            }
        }
    }
}

} // namespace prefetch_filter
