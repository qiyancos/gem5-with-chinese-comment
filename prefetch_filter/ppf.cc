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
    
    initFailFlag = LLCTable_.prefUsefulTable_.init(p->counter_cache_size,
            p->counter_cache_assoc, p->victim_cache_size,
            p->victim_cache_assoc) < 0;
    CHECK_WARN(initFailFlag, "Failed to initiate useful table for LLC");
}

int PerceptronPrefetchFilter::notifyCacheHit(BaseCache* cache,
        const PacketPtr& pkt, const DataTypeInfo& info) {
    BasePrefetchFilter::notifyCacheHit(cache, pkt, info);
    // 对于指令Cache的预取不进行过滤和训练处理
    if (!cache->cacheLevel_) {
        return 0;
    }
    
    const uint8_t cpuId = pkt->srcCpuId_;
    TablesTable& workTable = getTable(cache);
    auto infoPair = workTable.prefSrcCacheMap_.find(pkt->addr);
    const uint8_t cacheLevel = cache->cacheLevel_;
    if (info.source == Dmd && info.target == Pref) {
        // 一个Demand Request命中了预取数据，因此训练奖励
        if (infoPair != workTable.prefSrcCacheMap_.end()) {
            CHECK_RET(train(pkt->addr, cacheLevel, infoPair->second, cpuId,
                    GoodPref), "Failed to update training data when dmd %s",
                    "hit pref");
        } else {
            prefNotTrained_++;
        }
    } else if (info.source == Pref && info.target == Dmd) {
        // 一个预取命中了一个Demand数据，那么这个预取是一个无用的预取
        CHECK_RET(train(pkt->addr, cacheLevel, infoPair->second, cpuId,
                UselessPref), "Failed to update training data when pref %s"
                "hit dmd");
    }

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
    
    uint8_t cpuId = sharedTable_ ? 1 : pkt->srcCpuId_;
    uint8_t cacheLevel = cache->cacheLevel_;
    TablesTable& workTable = getTable(cache);
    if (info.source == Dmd) {
        // 对于一个Miss的Demand Request需要确定其有害性
        CHECK_RET(workTable.preUsefulTable_.updateHit(pkt->addr),
                "Failed to update hit for a demand request");
        if (info.target == Pref) {
            auto infoPair = workTable.prefSrcCacheMap_.find(combinedAddr);
            if (infoPair != workTable.prefSrcCacheMap_.end()) {
                // 说明一个预取被Demand Request覆盖了，预取是无用的
                CHECK_RET(train(pkt->addr, cacheLevel, infoPair->second, cpuId,
                        UselessPref), "Failed to update training when dmd %s",
                        "miss added to prefetch miss");
            }
        }
    } else if (info.target == Dmd) {
        // 如果一个预取Miss并合并到了Demand Request Miss，属于无用预取
        CHECK_RET(train(pkt->addr, cacheLevel, pkt->srcCacheLevel_, cpuId,
                UselessPref), "Failed to update training when pref miss %s",
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
    
    uint8_t cpuId_ = sharedTable_ ? 1 : pkt->srcCpuId_;
    uint8_t cacheLevel = cache->cacheLevel_ - 1;
    TablesTable& workTable = getTable(cache);
    if (info.target == Pref) {
        // 说明一个预取数据被预取替换了，需要删除相应的记录
        CHECK_RET(workTable.preUsefulTable_.evictPref(evictedAddr),
                "Failed to remove old pref info when replaced");
        workTable.prefSrcCacheMap_.erase(evictedAddr);
        if (info.source == Pref) {
            // 如果产生替换的也是预取，需要新增一个记录
            CHECK_RET(workTable.prefUsefulTable_.addPref(pkt->addr,
                    evictedAddr), "Failed to add new pref info when %s",
                    "pref replaced pref");
            workTable.prefSrcCacheMap_[pkt->addr] = pkt->srcCacheLevel_;
        }
    } else if (info.source == Pref) {
        // 说明一个Demand Request数据被预取替换了需要新增一个记录
        CHECK_RET(workTable.preUsefulTable_.addPref(pkt->addr,
                evictedAddr), "Failed to add new pref info when %s",
                "pref replaced dmd");
        workTable.prefSrcCacheMap_[pkt->addr] = pkt->srcCacheLevel_;
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
    CHECK_RET(updateTable(workTable, pkt->addr, cpuId, srcCacheLevel, indexes),
            "Failed to update prefetch table & reject table");
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
        const uint8_t srcCacheLevel, const uint8_t cpuId,
        const TrainType type) {
    // 查询Prefetch Table
    std::vector<uint16_t>* indexPtr = nullptr;
    CHECK_RET(workTable.prefetchTable_.read(prefAddr, &indexPtr),
            "Failed to find prefetch in prefetch table");
    
    // 查询Reject Table
    std::vector<uint16_t>* indexPtrTemp = nullptr;
    CHECK_RET(workTable.rejctTable_.read(prefAddr, &indexPtrTemp),
            "Failed to find prefetch in prefetch table");
   
    CHECK_ARGS(indexPtr ^ indexPtrTemp, "One prefetch should not exist %s",
            "both or neither in prefetch table and reject table");
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
        const uint8_t cpuId, const uint8_t cacheLevel,
        const std::vector<uint16_t>& indexes) {
    int result;
    uint64_t evictedAddr;
    if (targetCacheLevel < 4) {
        // 如果进行了预取，则预取会被记录到Prefetch Table
        // 并从Reject Table删除
        CHECK_RET(result = workTable.pefetchTable_.write(prefAddr, indexes,
                &evictedAddr), "Failed to add new prefetch into %s",
                "prefetch table");
        // 如果对应的预取写入时没有命中，则生成新的表项，需要记录
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
                    *prefRejected_[srcCacheLevel]++;
                } else if (RTTime == 0) {
                    *prefAccepted_[srcCacheLevel]++;
                } else {
                    *prefThreshing_[srcCacheLevel]++;
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
        // 如果对应的预取写入时没有命中，则生成新的表项，需要记录
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
                    *prefRejected_[srcCacheLevel]++;
                } else if (RTTime == 0) {
                    *prefAccepted_[srcCacheLevel]++;
                } else {
                    *prefThreshing_[srcCacheLevel]++;
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
                    .desc(std::string("Number of prefetch requests from " +
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
                    .desc(std::string("Number of prefetch requests from " +
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
                            "from " + BaseCache::levelName_[i] + ".")
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefThreshing_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefThreshing_[i] = &emptyStatsVar_;
        }
    }
    
    if (usePref_[1] || usePref_[2] || usePref_[3]) {
        prefNotTrained_ = new Stats::Vector();
        prefNotTrained_
                ->name(name() + ".untrained_prefetch_from_caches")
                .desc(std::string("Number of untrained prefetch requests" +
                        " from cache due to lack of entries in table")
                .flag(total);
        for (j = 0; j < numCpus_; j++) {
            prefNotTrained_->subname(j, std::string("cpu") +
                    std::to_string(j));
        }
    } else {
        prefNotTrained_ = &emptyStatsVar_;
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
