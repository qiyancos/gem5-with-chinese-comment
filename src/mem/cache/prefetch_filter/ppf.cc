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

#include <cmath>

#include "base/stats/info.hh"
#include "params/PerceptronPrefetchFilter.hh"
#include "mem/packet.hh"
#include "mem/cache/base.hh"
#include "mem/cache/cache.hh"
#include "mem/cache/prefetch_filter/ppf.hh"
#include "mem/cache/prefetch_filter/debug_flag.hh"

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
        auto infoIter = PrefInfoIndexMap.find(wordList[i]);
        CHECK_WARN(infoIter != PrefInfoIndexMap.end(),
                "Unknow feature name \"%s\" in \"%s\"", wordList[i].c_str(),
                feature.c_str());
        infoIndexList_.push_back(infoIter->second.index_);
        name_ = name_ + infoIter->second.varName_ + "_";
    }
    startBits_ = atoi(wordList[wordList.size() - 2].c_str());
    name_ = name_ + wordList[wordList.size() - 2] + "_";
    bits_ = atoi(wordList.back().c_str());
    name_ = name_ + wordList.back();
    return 0;
}
    
int Feature::getIndex(const PrefetchInfo& info, uint16_t* index) {
    CHECK_ARGS(!infoIndexList_.empty(),
            "Can not get index before feature not initiated");
    uint32_t genIndex = 0;
    int result;
    for (int i = 0; i < infoIndexList_.size(); i++) {
        uint32_t temp;
        CHECK_RET(result = info.getInfo(infoIndexList_[i], &temp),
                "Failed to get a proper index");
        if (!result) {
            *index = 0xffff;
            return 0;
        } else if (i) {
            genIndex ^= temp;
        } else {
            genIndex = temp;
        }
        DEBUG_PF(3, "Feature %s: Info[%d], 0x%x", name_.c_str(),
                infoIndexList_[i], temp);
    }
    *index = genIndex >> startBits_ << (32 - bits_) >> (32 - bits_);
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
    cache_ = cache;
    valid_ = cache->enableHarmTable_;
    if (!valid_) {
        return 0;
    }
    CHECK_RET(counterCache_.init(CCTagBits, CCSize, CCAssoc,
            BasePrefetchFilter::cacheLineOffsetBits_,
            CounterEntry(counterBits, counterInit)),
            "Failed to init counter cache");
    CHECK_RET(victimCache_.init(VCTagBits, VCSize, VCAssoc,
            BasePrefetchFilter::cacheLineOffsetBits_),
            "Failed to init victim cache");
    return 0;
}

int PrefetchUsefulTable::isPrefValid(const uint64_t& addr) {
    if (prefInCache_.find(addr) == prefInCache_.end()) {
        CHECK_ARGS_EXIT(invalidatedPref_.find(addr) != invalidatedPref_.end(),
                "Can not find invalidation record when checking prefetch");
        return 0;
    } else {
        return 1;
    }
}

int PrefetchUsefulTable::getReplacedAddr(const uint64_t& addr,
        uint64_t* replacedAddr) {
    auto prefIter = prefInCache_.find(addr);
    if (prefIter == prefInCache_.end()) {
        CHECK_ARGS(invalidatedPref_.find(addr) != invalidatedPref_.end(),
                "Can not find prefetch record for given addr");
        *replacedAddr = invalidBlkAddr_;
        return 0;
    }
    if (prefIter->second == Trainable) {
        CounterEntry* entry;
        CHECK_ARGS(counterCache_.read(addr, &entry, true) == 1,
                "Failed to find counter in the counter cache for prefetch");
        *replacedAddr = entry->evictedAddr_;
    } else {
        *replacedAddr = invalidBlkAddr_;
    }
    return 0;
}

int PrefetchUsefulTable::updateHit(const uint64_t& addr) {
    if (!valid_) return 0;
    // 处理一个Demand Request命中了预取数据
    auto prefIter = prefInCache_.find(addr);
    CHECK_ARGS(prefIter != prefInCache_.end(),
            "Failed to find record for prefetch in the table");
    if (prefIter->second == Trainable) {
        CounterEntry* entry;
        CHECK_ARGS(counterCache_.read(addr, &entry, false) > 0,
                "Failed to update counter in the counter cache when pref hit");
        entry->counter_++;
        DEBUG_PF(2, "PPF update demand hit event for prefetch 0x%lx", addr);
    } else if (prefIter->second == CleanPref) {
        prefIter->second = UsefulCleanPref;
        DEBUG_PF(2, "PPF update demand hit event for clean prefetch 0x%lx",
                addr);
    }
    return 0;
}

int PrefetchUsefulTable::updateMiss(const uint64_t& addr) {
    if (!valid_) return 0;
    // 如果发生了一个Demand Miss情况
    uint64_t* counterIndex;
    int result;
    CHECK_RET(result = victimCache_.read(addr, &counterIndex, false),
            "Failed to find record for demand miss in the table");
    if (!result) {
        // 如果一个Demand Miss没有找到，那么可以跳过
        return 0;
    }
    CounterEntry* entry;
    CHECK_ARGS(counterCache_.read(*counterIndex, &entry, true) == 1,
            "Failed to update counter in the counter cache when pref hit");
    entry->counter_--;
    DEBUG_PF(2, "PPF update demand miss for prefetch @0x%lx",
            entry->prefAddr_);
    return 0;
}

int PrefetchUsefulTable::addPref(const uint64_t& prefAddr,
        const uint64_t evictedAddr, const DataType type,
        std::map<uint64_t, uint8_t>* conflictPref) {
    if (!valid_) return 0;
    
    conflictPref->clear();
    // 发生了新的预取替换Demand/Pref数据，相关记录被更新
    CHECK_ARGS(prefInCache_.find(prefAddr) == prefInCache_.end(),
            "Found prefetch already in record when adding prefetch");
    prefInCache_[prefAddr] = type == NullType ? CleanPref : Trainable;
    
    if (type == NullType) {
        DEBUG_PF(2, "PPF add clean prefetch @0x%lx", prefAddr);
        return 0;
    }

    int result;
    // 如果存在替换的地址，则进行Victim更新
    uint64_t replacedPrefAddr, replacedVictimAddr;
    CHECK_RET(result = victimCache_.write(evictedAddr, prefAddr,
            false, &replacedVictimAddr, &replacedPrefAddr),
            "Failed to add prefetch victim info to victim cache");
    if (result > 1 || result == 0) {
        // 两个预取发生了冲突，导致新的预取替换信息和原来的替换信息发生冲突
        auto replacedPrefIter = prefInCache_.find(replacedPrefAddr);
        CHECK_ARGS(replacedPrefIter != prefInCache_.end(),
                "Can not find prefetch record bound with victim @0x%lx",
                replacedVictimAddr);
        CHECK_ARGS(replacedPrefIter->second == Trainable,
                "Trainable prefetch can not replace non-trainable "
                "prefetch @0x%lx", replacedPrefIter->first);
        
        replacedPrefIter->second = NotTrainable;
        
        CounterEntry* counterEntry;
        CHECK_ARGS(counterCache_.read(replacedPrefAddr,
                &counterEntry, true) == 1,
                "Failed to get counter from counter cache");
        CHECK_ARGS(counterEntry->prefAddr_ == replacedPrefAddr &&
                counterEntry->evictedAddr_ == replacedVictimAddr,
                "Address in counter entry does not match [pref_addr %lx:%lx] "
                "[victim_addr %lx:%lx]", replacedPrefAddr,
                counterEntry->prefAddr_, replacedVictimAddr,
                counterEntry->evictedAddr_);
        (*conflictPref)[replacedPrefAddr] = counterEntry->counter_;
        
        // 这时候应该将原来的预取训练属性删除，并进行更新
        CHECK_ARGS(counterCache_.invalidate(replacedPrefAddr, true) == 1,
                "Failed to invalidate old prefetch counter");
        DEBUG_PF(2, "Delete prefetch @0x%lx due to victim cache conflict",
                replacedPrefIter->first);
    } else if (result == 1) {
        // 新的预取替换的地址加入并且使用了空表项，可以继续执行
        DEBUG_PF(2, "PPF add prefetch @0x%lx with new victim entry", prefAddr);
    }

    // 更新Counter Cache，这里也可能会出现冲突
    CounterEntry counterEntry;
    CHECK_RET(result = counterCache_.write(prefAddr,
            CounterEntry(counterBits_, counterInit_, prefAddr, evictedAddr),
            false, &replacedPrefAddr, &counterEntry),
            "Failed to allocate new counter for prefetch");
    if (result == 0) {
        // 可能发生了问题，之前的预取Counter没有正确的Invalidate
        CHECK_ARGS(false, "Counter for prefetch already exists");
    } else if (result > 1) {
        // 两个预取的Counter发生了冲突
        auto replacedPrefIter = prefInCache_.find(replacedPrefAddr);
        CHECK_ARGS(replacedPrefIter != prefInCache_.end(),
                "Can not find prefetch record bound with victim");
        CHECK_ARGS(replacedPrefIter->second == Trainable,
                "Trainable prefetch can not replace non-trainable "
                "prefetch @0x%lx", replacedPrefIter->first);
        replacedPrefIter->second = NotTrainable;
        
        CHECK_ARGS(counterEntry.prefAddr_ == replacedPrefAddr,
                "Address in counter entry does not match");
        (*conflictPref)[replacedPrefAddr] = counterEntry.counter_;
        
        // 这时候应该将原来的预取训练属性删除，并进行更新
        CHECK_ARGS(victimCache_.invalidate(
                counterEntry.evictedAddr_, true) == 1,
                "Failed to add prefetch victim info to victim cache");
        DEBUG_PF(2, "Delete prefetch @0x%lx due to counter cache conflict",
                replacedPrefIter->first);
    } else {
        // 新的预取插入Counter使用了空表项，可以继续执行
        DEBUG_PF(2, "PPF add prefetch @0x%lx with new counter entry", prefAddr);
    }
   
    return 0;
}

int PrefetchUsefulTable::evictPref(const uint64_t& addr, uint8_t* counterPtr) {
    *counterPtr = 255;
    if (!valid_) return 0;
    
    if (prefInCache_.find(addr) == prefInCache_.end()) {
        DEBUG_PF(2, "PPF evict prefetch @0x%lx already invalidated", addr);
        auto invalidatedIter = invalidatedPref_.find(addr);
        CHECK_ARGS(invalidatedIter != invalidatedPref_.end(),
                "Deleted prefetch can not find invalidation record")
        invalidatedPref_.erase(invalidatedIter);
        return 0;
    }
    
    DEBUG_PF(2, "PPF evict prefetch @0x%lx", addr);
    return deletePref(addr, counterPtr);
}

int PrefetchUsefulTable::invalidatePref(const uint64_t& addr,
        uint8_t* counterPtr) {
    *counterPtr = 255;
    if (!valid_) return 0;
    DEBUG_PF(2, "PPF invalidate prefetch @0x%lx", addr);
    CHECK_ARGS(prefInCache_.find(addr) != prefInCache_.end(),
            "Can not find invalidating prefetch in record table");
    CHECK_RET(deletePref(addr, counterPtr),
            "Failed to delete prefetch in the prefetch table");
    CHECK_ARGS(invalidatedPref_.find(addr) == invalidatedPref_.end(),
            "Can not invalidate prefetch already invalidated");
    invalidatedPref_.insert(addr);
    return 0;
}

void PrefetchUsefulTable::memCheck() {
    DEBUG_PF(1, "Size: %lu", prefInCache_.size());
    victimCache_.memCheck();
    counterCache_.memCheck();
    DEBUG_PF(1, "Size: %lu", invalidatedPref_.size());
}

int PrefetchUsefulTable::deletePref(const uint64_t& addr,
        uint8_t* counterPtr) {
    if (!valid_) return 0;
    DEBUG_PF(2, "PPF delete prefetch @0x%lx", addr);
    
    *counterPtr = 255;
    auto prefIter = prefInCache_.find(addr);
    // 进行删除的时候对应的Pref应该存在，不存在则应该被无效化过
    CHECK_ARGS(prefIter != prefInCache_.end(),
            "Can not delete prefetch not recorded in prefetch table");
    if (prefIter->second == Trainable) {
        // 如果可以训练，对应的Counter也一定的存在
        CounterEntry* counterEntry;
        CHECK_ARGS(counterCache_.read(addr, &counterEntry, true) == 1,
                "Counter correspond to prefetch not exists or error occurred");
        *counterPtr = counterEntry->counter_;

        if (counterEntry->evictedAddr_ != invalidBlkAddr_) {
            CHECK_ARGS(victimCache_.invalidate(
                    counterEntry->evictedAddr_, true) == 1,
                    "Failed to invalidate victim data @0x%lx in victim cache",
                    counterEntry->evictedAddr_);
        }

        CHECK_ARGS(counterCache_.invalidate(addr, true) == 1,
                "Failed to invalidate counter in counter cache");
    } else if (prefIter->second == CleanPref) {
        *counterPtr = counterInit_;
    }
    
    prefInCache_.erase(prefIter);

    return 0;
}

} // namespace prefetch_filter

using namespace prefetch_filter;
using namespace Stats;

PerceptronPrefetchFilter::PerceptronPrefetchFilter(
        const PerceptronPrefetchFilterParams *p) :
        BasePrefetchFilter(p),
        degreeUpdatePeriod_(p->degree_update_period),
        cpuSharedTable_(p->cpu_shared_table),
        cacheSharedTable_(p->cache_shared_table),
        allSharedTable_(p->all_shared_table),
        allowUpgrade_(p->allow_upgrade),
        weightBits_(p->feature_weight_bits),
        counterInit_(p->counter_init_value),
        weightInit_(p->feature_weight_init),
        prefThreshold_(p->prefetch_threshold),
        missTrainStep_(p->miss_training_step),
        hitTrainStep_(p->hit_training_step),
        uselessPrefStep_(p->useless_prefetch_training_step) {
    // 初始化预取器激进度调整周期
    lastDegreeUpdateTime_ = 0;

    // L1I和L1D使用相同的参数，实际上可能并不会使用
    CHECK_ARGS_EXIT(weightInit_.size(), "Init weight must not be empty");
    weightInit_.insert(weightInit_.begin(), weightInit_.front());
    
    CHECK_ARGS_EXIT(prefThreshold_.size(),
            "Prefetch threshold must not be empty");
    prefThreshold_.insert(prefThreshold_.begin(), prefThreshold_.front());
    
    CHECK_ARGS_EXIT(hitTrainStep_.size(),
            "Train step for hit event must not be empty");
    hitTrainStep_.insert(hitTrainStep_.begin(), hitTrainStep_.front());
    
    CHECK_ARGS_EXIT(missTrainStep_.size(),
            "Train step for miss event  must not be empty");
    missTrainStep_.insert(missTrainStep_.begin(), missTrainStep_.front());

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
    CHECK_WARN((tempFlag = LLCTable.prefetchTable_.init(
            p->prefetch_table_tag_bits,
            p->prefetch_table_size, p->prefetch_table_assoc,
            BasePrefetchFilter::cacheLineOffsetBits_)) >= 0,
            "Failed to initiate prefetch table for init");
    initFailFlag_ |= tempFlag < 0;
    
    CHECK_WARN((tempFlag = LLCTable.rejectTable_.init(
            p->reject_table_tag_bits,
            p->reject_table_size, p->reject_table_assoc,
            BasePrefetchFilter::cacheLineOffsetBits_)) >= 0,
            "Failed to initiate reject table for init");
    initFailFlag_ |= tempFlag < 0;
    
    LLCTable.featureTable_.resize(featureList_.size());
    for (int j = 0; j < featureList_.size(); j++) {
        CHECK_WARN((tempFlag = LLCTable.featureTable_[j].init(
                int8_t(-1), featureList_[j].getSize(), 1, 0,
                SaturatedCounter(weightBits_, 0), true)) >= 0,
                "Failed to initiate feature %s \"%s\" %s",
                "table for", featureList_[j].name_.c_str(), "in init");
        initFailFlag_ |= tempFlag < 0;
    }
    
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

PerceptronPrefetchFilter* PerceptronPrefetchFilter::create(
        const PerceptronPrefetchFilterParams *p) {
    // 生成hash数值
    CHECK_RET_EXIT(BasePrefetchFilter::genHash("PerceptronPrefetchFilter"),
            "Failed to generate hash for a specific prefetch filter");
    if (!onlyInstance_) {
        onlyInstance_ = new PerceptronPrefetchFilter(p);
    }
    return dynamic_cast<PerceptronPrefetchFilter*>(onlyInstance_);
}

int PerceptronPrefetchFilter::notifyCacheHit(BaseCache* cache,
        const PacketPtr& pkt, const uint64_t& hitAddr,
        const DataTypeInfo& info) {
    DEBUG_PF(0, "[Tick: %lu]", curTick());
    DEBUG_PF(0, "Event Hit From %s by packet[%p : %s]",
            BaseCache::levelName_[cache->cacheLevel_].c_str(),
            pkt, pkt->cmd.toString().c_str());
    DEBUG_PF(1, "%s @0x%lx Hit %s @0x%lx",
            getDataTypeString(info.source).c_str(), pkt->getAddr(),
            getDataTypeString(info.target).c_str(), hitAddr);
    CHECK_ARGS_EXIT(info.source != NullType, "Unexpected source data type");
    BasePrefetchFilter::notifyCacheHit(cache, pkt, hitAddr, info);
    
    const uint64_t hitBlkAddr = hitAddr & cacheLineAddrMask_;
    const uint8_t cacheLevel = cache->cacheLevel_;
    if (info.source == Dmd && info.target == Pref) {
        std::set<BaseCache*> srcCaches;
        CHECK_RET_EXIT(usefulTable_[cache].findSrcCaches(hitBlkAddr,
                &srcCaches), "Failed to found prefetch recorded in cache");
        // 如果没有反馈Cache，说明被无效化了，但是删除操作依旧需要处理
        for (BaseCache* srcCache : srcCaches) {
            // 不对ICache进行训练
            if (!srcCache->cacheLevel_) continue;
            Tables& workTable = getTable(srcCache);
            // 一个Demand Request命中了预取数据，因此训练奖励
            CHECK_RET_EXIT(train(workTable, hitBlkAddr, cacheLevel, GoodPref),
                    "Failed to update training data when dmd hit pref");
        }
        if (!srcCaches.empty()) {
            // 更新预取有害性统计数据（必须不是无效化预取）
            CHECK_RET_EXIT(prefUsefulTable_[cache].updateHit(hitBlkAddr),
                    "Failed to update pref hit to prefetch useful table");
        } else {
            CHECK_ARGS_EXIT(!usefulTable_[cache].isPrefValid(hitBlkAddr),
                    "Find prefetch not invalid in useful table");
        }
        // 由于Demand命中，需要强制删除记录
        CHECK_RET_EXIT(removePrefetch(cache, hitBlkAddr, true),
                "Failed to remove prefetch record when hit by dmd");
    } else if (info.source == Dmd && info.target == PendingPref) {
        // 该情况实际上源自于Demand Miss pendingPref导致的PostProcess
        std::set<BaseCache*> srcCaches;
        CHECK_RET_EXIT(usefulTable_[cache].findSrcCaches(hitBlkAddr,
                &srcCaches), "Failed to found prefetch recorded in cache");
        // 如果没有反馈Cache，说明被无效化了，但是删除操作依旧需要处理
        for (BaseCache* srcCache : srcCaches) {
            // 不对ICache进行训练
            if (!srcCache->cacheLevel_) continue;
            Tables& workTable = getTable(srcCache);
            // 一个Demand Request命中了预取数据，因此训练奖励
            CHECK_RET_EXIT(train(workTable, hitBlkAddr, cacheLevel, GoodPref),
                    "Failed to update training data when dmd hit pref");
        }
        if (!srcCaches.empty()) {
            // 更新预取有害性统计数据（必须不是无效化预取）
            CHECK_RET_EXIT(prefUsefulTable_[cache].updateHit(hitBlkAddr),
                    "Failed to update pref hit to prefetch useful table");
        } else {
            CHECK_ARGS_EXIT(!usefulTable_[cache].isPrefValid(hitBlkAddr),
                    "Find prefetch not invalid in useful table");
        }
        // 该情况只会更新命中，但是并不会进行删除
    } else if (info.source == Pref && info.target == Dmd) {
        // 如果一个降级预取命中了目标等级，则属于一个无用预取
        // 如果目标是预取，则暂时不训练无用预取
        if (pkt->targetCacheLevel_ > pkt->srcCacheLevel_ &&
                pkt->targetCacheLevel_ == cache->cacheLevel_) {
            CHECK_ARGS(pkt->caches_.size() == 1,
                    "Unexpected source cache count for a level-down prefetch");
            Tables& workTable = getTable(*(pkt->caches_.begin()));
            // 一个Demand Request命中了预取数据，因此训练奖励
            CHECK_RET_EXIT(train(workTable, hitBlkAddr, cacheLevel,
                    UselessPref), "Failed to update training data when "
                    "level-down pref just hit dmd in target cache level");
        
        }
    }
    // TODO 预取命中预取暂时不做处理
    DEBUG_PF_PLINE();
    return 0;
}

int PerceptronPrefetchFilter::notifyCacheMiss(BaseCache* cache,
        PacketPtr& pkt, const PacketPtr& combinedPkt,
        const DataTypeInfo& info) {
    DEBUG_PF(0, "[Tick: %lu]", curTick());
    DEBUG_PF(0, "Event Miss From %s by packet[%p : %s]",
            BaseCache::levelName_[cache->cacheLevel_].c_str(),
            pkt, pkt->cmd.toString().c_str());
    DEBUG_PF(1, "%s @0x%lx Miss %s @0x%lx",
            getDataTypeString(info.source).c_str(), pkt->getAddr(),
            getDataTypeString(info.target).c_str(),
            combinedPkt ? combinedPkt->getAddr() : invalidBlkAddr_);
    CHECK_ARGS_EXIT(info.source != NullType, "Unexpected source data type");
    BasePrefetchFilter::notifyCacheMiss(cache, pkt, combinedPkt, info);
    uint8_t cacheLevel = cache->cacheLevel_;
    
    PacketPtr uselessPkt = nullptr;
    std::string errorInfo;

    if (info.source == Dmd) {
        if (info.target == Pref) {
            // 如果一个预取Miss被Demand Request覆盖了，预取是无用的
            uselessPkt = combinedPkt;
            errorInfo = "Failed to update training when dmd miss added "
                    "to prefetch miss";
        } else if (info.target == PendingPref) {
            // 如果Demand合并了一个PendingPref，不会进行操作
            // 依赖于PostProcess处理即可
        }
        if (info.target == Dmd || info.target == NullType) {
            if (usePref_[cache->cacheLevel_]) {
                // 同时尝试训练一个已经被拒绝的预取
                Tables& workTable = getTable(cache);
                // 一个Demand Request命中了预取数据，因此训练奖励
                CHECK_RET_EXIT(train(workTable,
                        pkt->getAddr() & cacheLineAddrMask_,
                        cacheLevel, DemandMiss),
                        "Failed to update training data for demand miss");
            }
            // 对于一个Miss的Demand Request需要确定其有害性
            // 生成新的MSHR或者合并到Prefetch的MSHR都可以
            CHECK_RET_EXIT(prefUsefulTable_[cache].updateMiss(pkt->getAddr() &
                    cacheLineAddrMask_),
                    "Failed to update dmd miss in prefetch useful table");
        }
    } else if (info.target == Pref || info.target == PendingPref) {
        // 如果一个预取Miss被预取覆盖了，则被覆盖的预取是无用的
        uselessPkt = combinedPkt;
        CHECK_ARGS_EXIT(uselessPkt, "No combined packet provided");
        errorInfo = "Failed to update training when prefetch miss added "
                "to (pending) prefetch miss";
    } else if (info.target == NullType && *(pkt->caches_.begin()) == cache) {
        // 根据最后的结果更新统计数据
        for (auto cpuId : cache->cpuIds_) {
            (*prefTarget_[cacheLevel][pkt->targetCacheLevel_])[cpuId]++;
        }
    }

    if (uselessPkt) {
        CHECK_ARGS_EXIT(uselessPkt->caches_.size() == 1,
                "Prefetch packet should have only one source cache");
        // 如果一个预取Miss并合并到了Demand Request Miss，属于无用预取
        for (BaseCache* srcCache : uselessPkt->caches_) {
            // 不对ICache进行训练
            if (!srcCache->cacheLevel_) continue;
            Tables& workTable = getTable(srcCache);
            CHECK_RET_EXIT(train(workTable, uselessPkt->getAddr() &
                    cacheLineAddrMask_, cacheLevel, UselessPref),
                    errorInfo.c_str());
        }
    }

    DEBUG_PF_PLINE();
    return 0;
}

int PerceptronPrefetchFilter::notifyCacheFill(BaseCache* cache,
        const PacketPtr &pkt, const uint64_t& evictedAddr,
        const DataTypeInfo& info) {
    DEBUG_PF(0, "[Tick: %lu]", curTick());
    DEBUG_PF(0, "Event Fill From %s by packet[%p : %s]",
            BaseCache::levelName_[cache->cacheLevel_].c_str(),
            pkt, pkt->cmd.toString().c_str());
    DEBUG_PF(1, "%s @0x%lx replace %s @0x%lx",
            getDataTypeString(info.source).c_str(), pkt->getAddr(),
            getDataTypeString(info.target).c_str(), evictedAddr);
    CHECK_ARGS_EXIT(info.source != NullType, "Unexpected source data type");
    BasePrefetchFilter::notifyCacheFill(cache, pkt, evictedAddr, info);
    
    const uint64_t evictedBlkAddr = evictedAddr & cacheLineAddrMask_;
    if (info.target == Pref) {
        // 如果产生替换的也是预取，需要新增一个记录
        if (info.source == Pref) {
            uint64_t oldReplacedAddr;
            CHECK_RET_EXIT(prefUsefulTable_[cache].getReplacedAddr(
                    evictedBlkAddr, &oldReplacedAddr),
                    "Failed to get replaced addr for replaced prefetch");
        
            // 由于替换了预取，需要强制删除记录
            CHECK_RET_EXIT(removePrefetch(cache, evictedBlkAddr, false),
                    "Failed to remove prefetch record when hit by dmd");

            // 这里我们可以选择使用被替换的Prefetch作为Victim
            // 也可以选择原本预取替换的Demand地址作为Victim
            // (只有原来的预取替换了Demand，才会选择)
            std::map<uint64_t, uint8_t> conflictPref;
            CHECK_RET_EXIT(prefUsefulTable_[cache].addPref(pkt->getAddr() &
                    cacheLineAddrMask_,
                    (enableRecursiveReplace_ &&
                    oldReplacedAddr != invalidBlkAddr_) ?
                    oldReplacedAddr : evictedBlkAddr,
                    info.target, &conflictPref),
                    "Failed to add new pref info when pref replaced pref");
            // 处理因为冲突导致替换的预取有害情况进行训练
            CHECK_RET_EXIT(trainConflictPref(cache, conflictPref),
                    "Failed to train prefetch evicted due to conflict");
        } else {
            // 由于替换了预取，需要强制删除记录
            CHECK_RET_EXIT(removePrefetch(cache, evictedBlkAddr, false),
                    "Failed to remove prefetch record when hit by dmd");
        }
    } else if (info.source == Pref) {
        std::map<uint64_t, uint8_t> conflictPref;
        // 如果一个Demand Request数据被预取替换了需要新增一个记录
        CHECK_RET_EXIT(prefUsefulTable_[cache].addPref(pkt->getAddr() &
                cacheLineAddrMask_, evictedBlkAddr, info.target,
                &conflictPref),
                "Failed to add new pref info when pref replaced dmd");
        // 处理因为冲突导致替换的预取有害情况进行训练
        CHECK_RET_EXIT(trainConflictPref(cache, conflictPref),
                "Failed to train prefetch evicted due to conflict");
    }
    
    DEBUG_PF_PLINE();
    return 0;
}

int PerceptronPrefetchFilter::filterPrefetch(BaseCache* cache,
        const uint64_t& prefAddr, const PrefetchInfo& info) {
    DEBUG_PF(0, "[Tick: %lu]", curTick());
    DEBUG_PF(0, "Event Filter From %s",
            BaseCache::levelName_[cache->cacheLevel_].c_str());
    if (!enableFilter_) {
        return BasePrefetchFilter::filterPrefetch(cache, prefAddr, info);
    }

    const int featureCount = featureList_.size();
    const uint8_t cacheLevel = cache->cacheLevel_;
    // 不会对L1ICache的预取进行处理
    if (!cacheLevel) {
        DEBUG_PF_PLINE();
        return 0;
    }
    
    Tables& workTable = getTable(cache);

    // 计算权重加和
    uint16_t weightSum = 0;
    // 说明一下，如果我们标记某一个Index不可用，那么这个数值会被设置为
    // uint16_t的最大数值
    std::vector<uint16_t> indexes;
    int validFeatureCount = 0;
    DEBUG_PF(1, "Filter Prefetch @0x%lx", prefAddr);
    for (int i = 0; i < featureCount; i++) {
        uint16_t featureIndex;
        CHECK_RET_EXIT(featureList_[i].getIndex(info, &featureIndex),
                "Failed to get index for a feature");
        indexes.push_back(featureIndex);
        if (featureIndex != 0xffff) {
            validFeatureCount++;
            SaturatedCounter* weight;
            CHECK_RET_EXIT(workTable.featureTable_[i].read(
                    featureIndex, &weight, true) == 1,
                    "Failed to read weight for feature \"%s\" with index %d",
                    featureList_[i].name_.c_str(), featureIndex);
            CHECK_ARGS_EXIT(weight, "Feature index [%d] illegal for \"%s\"",
                    featureIndex, featureList_[i].name_.c_str());
            DEBUG_PF(2, "Weight @%s[%d]: %u",
                    featureList_[i].name_.c_str(), featureIndex,
                    uint32_t(*weight));
            weightSum = *weight + weightSum;
        }
    }

    // 取Weight均值，而非总和来避免可用Feature数量的影响
    // 该特性仅仅适用于软件，硬件设计无需要按照均值处理
    weightSum = std::round(double(weightSum) / validFeatureCount);
    
    // 依据权重加和进行预取更改和过滤
    uint8_t targetCacheLevel = 0;
    for (auto threshold : prefThreshold_) {
        if (weightSum >= threshold) {
            // 不会预取到L1Icache的
            if (targetCacheLevel) {
                break;
            }
        }
        targetCacheLevel++;
    }
    
    // 依据是否允许预取提升等级进行处理
    if (targetCacheLevel < cacheLevel && !allowUpgrade_) {
        targetCacheLevel = cacheLevel;
    }
    
    if (targetCacheLevel > maxCacheLevel_) {
        for (auto cpuId : cache->cpuIds_) {
            (*prefTarget_[cacheLevel][targetCacheLevel])[cpuId]++;
        }
    }
    DEBUG_PF(2, "Filter From %s To %s with %d Weights[%d]",
            BaseCache::levelName_[cache->cacheLevel_].c_str(),
            BaseCache::levelName_[targetCacheLevel].c_str(),
            validFeatureCount, weightSum);
    
    // 依据最后的结果更新Prefetch Table和Reject Table
    CHECK_RET_EXIT(updateTable(workTable, prefAddr & cacheLineAddrMask_,
            targetCacheLevel, cache, indexes),
            "Failed to update prefetch table & reject table");
    DEBUG_PF_PLINE();
    return targetCacheLevel;
}

int PerceptronPrefetchFilter::invalidatePrefetch(BaseCache* cache,
        const uint64_t& prefAddr) {
    DEBUG_PF(0, "[Tick: %lu]", curTick());
    DEBUG_PF(0, "PPF Event Invalidate From %s",
            BaseCache::levelName_[cache->cacheLevel_].c_str());
    
    CHECK_RET_EXIT(removePrefetch(cache, prefAddr, false),
            "Failed to remove prefetch record when invalidated");
    DEBUG_PF_PLINE();
    return 0;
}

int PerceptronPrefetchFilter::notifyCacheReqSentFailed(BaseCache* cache,
        const int totalEntries, const int waitingDemands,
        const uint8_t originalDegree, uint8_t* newDegree) {
    // 没有预取器的层级不做处理
    if (!usePref_[cache->cacheLevel_] || !originalDegree) {
        return 0;
    }
    
    DEBUG_PF(0, "[Tick: %lu]", curTick());
    DEBUG_PF(0, "PPF Event Cache Request Sent Failure[%d] From %s",
            waitingDemands, BaseCache::levelName_[cache->cacheLevel_].c_str());
    
    CHECK_RET_EXIT(BasePrefetchFilter::notifyCacheReqSentFailed(cache,
            totalEntries, waitingDemands, originalDegree, newDegree),
            "Failed to deal with cache request sent failure");
    // 更新计数
    auto statsIter = blockingHarm_.find(cache);
    CHECK_ARGS_EXIT(statsIter != blockingHarm_.end(),
            "Stats for cache not found for prefetcher degree "
            "controller update");
    statsIter->second += waitingDemands;

    // 如果到达统计周期结尾，进行结果总结和预取激进度更新计算
    if (lastDegreeUpdateTime_ + degreeUpdatePeriod_ <=
            prefetch_filter::tickNow_) {
        uint64_t avgWaitingDemands = static_cast<double>(statsIter->second) /
                (prefetch_filter::tickNow_ - lastDegreeUpdateTime_) *
                clockPeriod_;
        if (avgWaitingDemands == 0) {
            *newDegree = originalDegree;
        } else {
            *newDegree = static_cast<double>(originalDegree) * (totalEntries -
                    avgWaitingDemands) / totalEntries;
        }
        // 不会直接将预取完全关闭
        if (!*newDegree) {
            *newDegree = 1;
        }
        CHECK_ARGS_EXIT(*newDegree > 0 && *newDegree <= originalDegree,
                "Unexpected degree setting");
        DEBUG_PF(1, "PPF prefetcher new degree[%u/%u] with "
                "average waiting demands[%lu/%d]",
                *newDegree, originalDegree, avgWaitingDemands, totalEntries);
        lastDegreeUpdateTime_ = prefetch_filter::tickNow_;
        statsIter->second = 0;
    }
    return 0;
}

int PerceptronPrefetchFilter::initThis() {
    CHECK_ARGS(!initFailFlag_, "Failed when trying to initiate PPF");
    uint8_t cacheCount = caches_.size();
    // 初始化预取器激进度调整所需要的统计变量
    for (auto& level : caches_) {
        if (usePref_[level.front()->cacheLevel_]) {
            for (auto cache : level) {
                blockingHarm_[cache] = 0;
            }
        }
    }

    // 重新初始化训练幅度
    CHECK_WARN(missTrainStep_.size() <= cacheCount,
            "Too many train steps are given.");
    missTrainStep_.resize(cacheCount, missTrainStep_.back());
    
    CHECK_WARN(hitTrainStep_.size() <= cacheCount,
            "Too many train steps are given.");
    hitTrainStep_.resize(cacheCount, hitTrainStep_.back());
    
    // 重新初始化权重初始化数值
    CHECK_WARN(weightInit_.size() <= cacheCount,
            "Too many weight init value are given");
    uint8_t lastWeightInit = weightInit_.empty() ?
            uint8_t(1) << (weightBits_ - 1) : weightInit_.back();
    weightInit_.resize(cacheCount, uint8_t(1) << (weightBits_ - 1));

    // 重新初始化预取阈值设置
    CHECK_WARN(prefThreshold_.size() <= cacheCount,
            "Too many threshold value are given");
    // 我们选择权重初始化量中的最后一个数值，一般也是最小值，作为阈值
    prefThreshold_.resize(cacheCount, lastWeightInit);

    // 初始化PPF表格
    std::set<uint8_t> fullCpuIds;
    for (int i = 0; i < numCpus_; i++) {
        fullCpuIds.insert(i);
    }
    if (!allSharedTable_ && maxCacheLevel_ > 1) {
        const Tables sample = workTables_[0][0];
        workTables_.clear();
        
        // 依据共享情况初始化表格大小
        int cpuSize = cpuSharedTable_ ? 1 : numCpus_;
        if (cacheSharedTable_) {
            // 针对Cache共享PPF表格的情况
            // （由于是共享状态，只要有一个预取，就需要开启）
            if (havePref_[maxCacheLevel_ - 1]) {
                workTables_.push_back(std::vector<Tables>(cpuSize, sample));
                for (int i = 0; i < cpuSize; i++) {
                    workTables_[0][i].name_ = cpuSharedTable_ ?
                            "cpu_cache_shared" : "cache_shared";
                    workTables_[0][i].cpuIds_ = cpuSharedTable_ ?
                            fullCpuIds :
                            std::set<uint8_t> {static_cast<uint8_t>(i)};
                    // 对于部分共享，权重初始化为平均值
                    for (auto& table : workTables_[0][i].featureTable_) {
                        table.writeAll(SaturatedCounter(weightBits_,
                                (weightInit_[0] + weightInit_.back()) >> 1));
                    }
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
                        workTables_.back()[j].name_ = BaseCache::levelName_[i];
                        workTables_.back()[j].cpuIds_ = cpuSharedTable_ ?
                                fullCpuIds :
                                std::set<uint8_t> {static_cast<uint8_t>(j)};
                        for (auto& table :
                                workTables_.back()[j].featureTable_) {
                            table.writeAll(SaturatedCounter(weightBits_,
                                    weightInit_[i]));
                        }
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
            workTables_.back()[0].cpuIds_ = fullCpuIds;
            for (auto& table : workTables_.back()[0].featureTable_) {
                table.writeAll(SaturatedCounter(weightBits_,
                        weightInit_[maxCacheLevel_]));
            }
        } else {
            workTables_.push_back(std::vector<Tables>());
        }
    } else if (allSharedTable_) {
        // 如果没有任何预取开启，则不生成表格
        if (havePref_[maxCacheLevel_]) {
            workTables_.clear();
        } else {
            workTables_[0][0].name_ = "all_shared";
            workTables_[0][0].cpuIds_ = fullCpuIds;
            // 全局共享的PPF表格权重会被初始化为中间值
            for (auto& table : workTables_[0][0].featureTable_) {
                table.writeAll(SaturatedCounter(weightBits_,
                        uint8_t(1) << (weightBits_ - 1)));
            }
        }
    }
    
    // 初始化有害性统计表格
    CHECK_RET(initStats(), "Failed to initiate stats variables");

    // 修正BasePrefetchFilter的设置
    if (allowUpgrade_ || havePref_[maxCacheLevel_]) {
        for (auto& tablePair : usefulTable_) {
            tablePair.second.setValidBit(true);
        }
    }
    return 0;
}

int PerceptronPrefetchFilter::initStats() {
    uint8_t cacheCount = caches_.size();
    const uint8_t counterBits = prefUsefulTable_[nullptr].counterBits_;
    const int8_t CCTagBits = prefUsefulTable_[nullptr].counterCacheTagBits_;
    const uint32_t CCSize = prefUsefulTable_[nullptr].counterCacheSize_;
    const uint8_t CCAssoc = prefUsefulTable_[nullptr].counterCacheAssoc_;
    const int8_t VCTagBits = prefUsefulTable_[nullptr].victimCacheTagBits_;
    const uint32_t VCSize = prefUsefulTable_[nullptr].victimCacheSize_;
    const uint8_t VCAssoc = prefUsefulTable_[nullptr].victimCacheAssoc_;
    prefUsefulTable_.clear();
    for (auto& level : caches_) {
        for (BaseCache* cache : level) {
            prefUsefulTable_[cache] = PrefetchUsefulTable();
            if (havePref_[cache->cacheLevel_] || allowUpgrade_) {
                CHECK_RET(prefUsefulTable_[cache].init(counterBits, CCSize,
                        CCAssoc, VCSize, VCAssoc, counterInit_, cache,
                        CCTagBits, VCTagBits),
                        "Failed to init prefetch useful table");
            }
        }
    }

    // 初始化部分统计变量
    uint8_t levelCount = 0;
    for (auto& tables : workTables_) {
        for (auto& table : tables) {
            table.statsLevel_ = levelCount;
        }
        levelCount++;
    }
    prefAccepted_.resize(levelCount);
    prefRejected_.resize(levelCount);
    prefThreshing_.resize(levelCount);
    prefNotTrained_.resize(levelCount);
    dmdNotTrained_.resize(levelCount);
    prefTrainingType_.resize(levelCount, 
            std::vector<Stats::Vector*>(TrainTypeCount));
    
    CHECK_ARGS_EXIT(weightBits_ > 0, "Bit number of the feature weight must%s",
            " be greater than zero");
    int weightNum = 1 << weightBits_;
    featureWeightFrequency_.resize(levelCount,
            std::vector<std::vector<Stats::Vector*>>(featureList_.size(),
            std::vector<Stats::Vector*>(weightNum)));
    
    for (int i = 0; i < cacheCount; i++) {
        prefTarget_.push_back(usePref_[i] ?
                std::vector<Stats::Vector*>(cacheCount + 1) :
                std::vector<Stats::Vector*>());
    }
    return 0;
}

PerceptronPrefetchFilter::Tables& PerceptronPrefetchFilter::getTable(
        BaseCache* cache) {
    if (cache->cacheLevel_ == maxCacheLevel_) {
        CHECK_ARGS_EXIT(!workTables_.back().empty(),
                "Trying to get a worktable for cache without prefetcher");
        return workTables_.back()[0];
    } else {
        uint8_t cpuIndex = cpuSharedTable_ ? 0 : *(cache->cpuIds_.begin());
        CHECK_ARGS_EXIT(cache->cacheLevel_ != 0,
                "ICache work table not supportted");
        uint8_t cacheIndex = cacheSharedTable_ ? 0 : cache->cacheLevel_ - 1;
        CHECK_ARGS_EXIT(!workTables_[cacheIndex].empty(),
                "Trying to get a worktable for cache without prefetcher");
        return workTables_[cacheIndex][cpuIndex];
    }
}

int PerceptronPrefetchFilter::helpInvalidatePref(BaseCache* cache,
        const std::set<uint64_t>& addrs) {
    for (auto addr : addrs) {
        uint8_t counter;
        CHECK_RET(prefUsefulTable_[cache].invalidatePref(addr, &counter),
                "Failed to invalidate prefetch");
        
        // 首先查找相关的源Cache
        std::set<BaseCache*> srcCaches;
        CHECK_RET(usefulTable_[cache].findInvalidatedCaches(addr,
                &srcCaches), "Failed to find invalidated prefetch sources");
    
        // 基于被替换的预取有害情况进行训练，只有开启统计表格才会进行训练
        if (counter != 255 && prefUsefulTable_[cache].valid_ &&
                counter < counterInit_) {
            for (BaseCache* srcCache : srcCaches) {
                // 不对ICache进行训练，不训练有用/无用预取
                if (!srcCache->cacheLevel_) {
                    continue;
                }
                Tables& workTable = getTable(srcCache);
                CHECK_RET(train(workTable, addr, cache->cacheLevel_, BadPref),
                        "Failed to update training when pref evicted"); 
            }
        }
    }
    CHECK_RET(BasePrefetchFilter::helpInvalidatePref(cache, addrs),
            "Failed to invalidate prefetch from BPF")
    return 0;
}

int PerceptronPrefetchFilter::helpCorrectPref(
        const std::map<BaseCache*, std::set<uint64_t>>& correctionList) {
    // 针对校正处理的预取，不会进行训练
    // 因为不确定这些预取在此时进行训练，是否会产生错误的训练效果
    for (auto& correction : correctionList) {
        BaseCache* cache = correction.first;
        for (auto addr : correction.second) {
            uint8_t counter;
            CHECK_RET(prefUsefulTable_[cache].evictPref(addr, &counter),
                    "Failed to invalidate prefetch");
        }
    }    
    CHECK_RET(BasePrefetchFilter::helpCorrectPref(correctionList),
            "Failed to correct prefetch from BPF")
    return 0;
}

int PerceptronPrefetchFilter::train(Tables& workTable,
        const uint64_t& prefAddr, const uint8_t cacheLevel,
        const TrainType type) {
    DEBUG_PF(1, "Training prefetch @0x%lx for %s with type %s by %s",
            prefAddr, workTable.getName().c_str(),
            getTrainTypeStr(type).c_str(), 
            BaseCache::levelName_[cacheLevel].c_str());
    // 查询Prefetch Table
    std::vector<uint16_t>* prefIndexPtr = nullptr;
    CHECK_RET(workTable.prefetchTable_.read(prefAddr, &prefIndexPtr, false),
            "Failed to find prefetch in prefetch table");
    
    // 查询Reject Table
    std::vector<uint16_t>* rejectIndexPtr = nullptr;
    CHECK_RET(workTable.rejectTable_.read(prefAddr, &rejectIndexPtr, false),
            "Failed to find prefetch in prefetch table");
    
    // 如果均不存在，则更新相关计数
    if (!(prefIndexPtr || rejectIndexPtr)) {
        std::vector<Stats::Vector*>* notTrained_ = type == DemandMiss ?
                &dmdNotTrained_ : &prefNotTrained_;
        for (auto cpuId : workTable.cpuIds_) {
            (*(*notTrained_)[workTable.statsLevel_])[cpuId]++;
        }
        return 0;
    }
    
    // 更新训练类型计数
    for (auto cpuId : workTable.cpuIds_) {
        (*prefTrainingType_[workTable.statsLevel_][type])[cpuId]++;
    }
    
    CHECK_ARGS((prefIndexPtr != nullptr) ^ (rejectIndexPtr != nullptr),
            "One prefetch should not exist %s",
            "both in prefetch table and reject table");
    // 对于DemandMiss只会训练拒绝的预取
    if (type == DemandMiss && !rejectIndexPtr) {
        return 0;
    }
    std::vector<uint16_t>* indexPtr = prefIndexPtr ?
            prefIndexPtr : rejectIndexPtr;
    
    int8_t step = 0;
    switch (type) {
    case DemandMiss: step = missTrainStep_[cacheLevel]; break;
    case GoodPref: step = hitTrainStep_[cacheLevel]; break;
    case BadPref: step = -missTrainStep_[cacheLevel]; break;
    case UselessPref: step = -uselessPrefStep_; break;
    default: CHECK_ARGS(false, "Unexpected training type");
    }

    // 进行权重更新
    SaturatedCounter* weight;
    for (int i = 0; i < featureList_.size(); i++) {
        /// 对于无效的Index不会进行训练
        if ((*indexPtr)[i] != 0xffff) {
            CHECK_ARGS(workTable.featureTable_[i].read(
                    (*indexPtr)[i], &weight, true) == 1,
                    "Failed to read weight for a feature");
            *weight += step;
            DEBUG_PF(2, "Weight @%s[%d]: %u(add with %d)",
                    featureList_[i].name_.c_str(), (*indexPtr)[i],
                    uint32_t(*weight), step);
        }
    }
    return 0;
}

int PerceptronPrefetchFilter::removePrefetch(BaseCache* cache,
        const uint64_t& prefAddr, const bool isHit) {
    const uint64_t prefBlkAddr = prefAddr & cacheLineAddrMask_;
    
    // 首先查找相关的源Cache
    std::set<BaseCache*> srcCaches;
    CHECK_RET(usefulTable_[cache].findSrcCaches(prefBlkAddr,
            &srcCaches), "Failed to find prefetch recorded in cache");
    
    uint8_t counter;
    CHECK_RET(prefUsefulTable_[cache].evictPref(prefBlkAddr,
            &counter), "Failed to remove old pref info when replaced");
    
    if (counter != 255 && prefUsefulTable_[cache].valid_ &&
            counter < counterInit_) {
        // 基于被替换的预取有害情况进行训练，如果开启统计表格只会训练有害预取
        for (BaseCache* srcCache : srcCaches) {
            // 不对ICache进行训练
            if (!srcCache->cacheLevel_) continue;
            Tables& workTable = getTable(srcCache);
            CHECK_RET(train(workTable, prefBlkAddr, cache->cacheLevel_,
                    BadPref), "Failed to update training when pref evicted"); 
        }
        if (srcCaches.empty()) {
            CHECK_ARGS(!usefulTable_[cache].isPrefValid(prefAddr),
                    "Find prefetch not invalid in useful table");
        }
    } else if (!isHit) {
        // 如果没有训练有害预取，并且是因为替换导致remove
        // 则训练一个无用预取
        for (BaseCache* srcCache : srcCaches) {
            // 不对ICache进行训练
            if (!srcCache->cacheLevel_) continue;
            Tables& workTable = getTable(srcCache);
            CHECK_RET(train(workTable, prefBlkAddr,
                    cache->cacheLevel_, UselessPref),
                    "Failed to update training when pref evicted"); 
        }
        if (srcCaches.empty()) {
            CHECK_ARGS(!usefulTable_[cache].isPrefValid(prefAddr),
                    "Find prefetch not invalid in useful table");
        }
    }
    
    CHECK_RET(BasePrefetchFilter::removePrefetch(cache, prefAddr, isHit),
            "Failed to remove prefetch record from Base Prefetch Filter");
    
    return 0;
}

int PerceptronPrefetchFilter::trainConflictPref(BaseCache* cache,
        const std::map<uint64_t, uint8_t>& conflictPref) {
    // 只有开启统计表格才会进行训练, 有害/无用预取一定会被训练
    if (!prefUsefulTable_[cache].valid_) {
        return 0;
    }
    for (auto prefIter : conflictPref) {
        const uint64_t prefAddr = prefIter.first;
        const uint8_t counter = prefIter.second;
        // 建议不对冲突替换的无用预取进行训练
        if (counter != 255 && counter < counterInit_) {
            // 首先查找相关的源Cache
            std::set<BaseCache*> srcCaches;
            CHECK_RET(usefulTable_[cache].findSrcCaches(prefAddr, &srcCaches),
                    "Failed to find prefetch recorded in cache");
    
            if (srcCaches.empty()) {
                CHECK_ARGS(!usefulTable_[cache].isPrefValid(prefAddr),
                        "Find prefetch not invalid in useful table");
            }
        
            for (BaseCache* srcCache : srcCaches) {
                // 不对ICache进行训练
                if (!srcCache->cacheLevel_) continue;
                Tables& workTable = getTable(srcCache);
                CHECK_RET(train(workTable, prefAddr,
                        cache->cacheLevel_, BadPref),
                        "Failed to update training when pref evicted"); 
            }
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
                false, &evictedAddr, &replacedIndexes),
                "Failed to add new prefetch into prefetch table");
        
        // 如果对应的预取写入未命中，则生成新的表项包括替换，需要记录
        if (result) {
            auto prefIter = workTable.localPrefTable_.find(prefAddr);
            if (prefIter != workTable.localPrefTable_.end()) {
                prefIter->second.timesPT_++;
            } else {
                workTable.localPrefTable_[prefAddr] =
                        Tables::PrefInfoEntry(1, 0);
            }
        }

        // 如果替换的预取在另一个表格也不存在，删除记录表项
        if (result > 2 && workTable.rejectTable_.touch(
                evictedAddr, true) == 0) {
            // 依据表格出现次数更新统计信息
            CHECK_RET(updateWorkTableStats(workTable, evictedAddr),
                    "Failed to update work table statistics");
            // 预取被删除后更新权重统计
            CHECK_RET(updateFreqStats(workTable, replacedIndexes),
                    "Failed to update weight frequency stats");
        }
        CHECK_RET(workTable.rejectTable_.invalidate(prefAddr, false),
                "Failed to remove old prefetch from reject table");
    } else {
        // 如果预取被过滤，则预取会被记录到Reject Table
        // 并从Prefetch Table删除
        std::vector<uint16_t> replacedIndexes;
        CHECK_RET(workTable.rejectTable_.write(prefAddr, indexes,
                false, &evictedAddr, &replacedIndexes),
                "Failed to add new prefetch into reject table");
        
        // 如果对应的预取写入未命中，则生成新的表项包括替换，需要记录
        if (result) {
            auto prefIter = workTable.localPrefTable_.find(prefAddr);
            if (prefIter != workTable.localPrefTable_.end()) {
                prefIter->second.timesRT_++;
            } else {
                std::set<BaseCache*> caches {srcCache};
                workTable.localPrefTable_[prefAddr] =
                        Tables::PrefInfoEntry(0, 1);
            }
        }
        
        // 如果替换的预取在另一个表格也不存在，删除记录表项
        if (result == 2 && workTable.prefetchTable_.touch(
                evictedAddr, true) == 0) {
            // 依据表格出现次数更新统计信息
            CHECK_RET(updateWorkTableStats(workTable, evictedAddr),
                    "Failed to update work table statistics");
            // 预取被删除后更新权重统计
            CHECK_RET(updateFreqStats(workTable, replacedIndexes),
                    "Failed to update weight frequency stats");
        }
        CHECK_RET(workTable.prefetchTable_.invalidate(prefAddr, false),
                "Failed to remove old prefetch from prefetch table");
    }
    return 0;
}

int PerceptronPrefetchFilter::updateFreqStats(Tables& workTable,
        const std::vector<uint16_t>& indexes) {
    // 进行权重查询和统计更新
    SaturatedCounter* weight;
    for (int i = 0; i < featureList_.size(); i++) {
        // 对于无效的权重不会尝试更新
        if (indexes[i] != 0xffff) {
            CHECK_ARGS(workTable.featureTable_[i].read(
                    indexes[i], &weight, true) == 1,
                    "Failed to read weight for a feature when updating stats");
            for (auto cpuId : workTable.cpuIds_) {
                (*featureWeightFrequency_[workTable.statsLevel_][i][*weight])
                        [cpuId]++;
            }
        }
    }
    return 0;
}

int PerceptronPrefetchFilter::updateWorkTableStats(Tables& workTable,
        const uint64_t& prefAddr) {
    DEBUG_PF(2, "PPF update deleted prefetch @0x%lx stats", prefAddr);
    // 首先更新在不同表格的出现次数
    auto prefIter = workTable.localPrefTable_.find(prefAddr);
    CHECK_ARGS(prefIter != workTable.localPrefTable_.end(),
            "Can not find prefetch record in local prefetch table");
    uint8_t PTTime = prefIter->second.timesPT_;
    uint8_t RTTime = prefIter->second.timesRT_;
    
    for (auto cpuId : workTable.cpuIds_) {
        if (PTTime == 0) {
            (*prefRejected_[workTable.statsLevel_])[cpuId]++;
        } else if (RTTime == 0) {
            (*prefAccepted_[workTable.statsLevel_])[cpuId]++;
        } else {
            (*prefThreshing_[workTable.statsLevel_])[cpuId]++;
        }
    }
    
    // 更新完统计计数之后删除表项
    workTable.localPrefTable_.erase(prefIter);
    return 0;
}

void PerceptronPrefetchFilter::memCheck() {
    DEBUG_PF(0, "PPF memory check");
    DEBUG_PF(1, "Size: %lu", prefAccepted_.size());
    DEBUG_PF(1, "Size: %lu", prefRejected_.size());
    DEBUG_PF(1, "Size: %lu", prefThreshing_.size());
    DEBUG_PF(1, "Size: %lu", prefNotTrained_.size());
    DEBUG_PF(1, "Size: %lu", dmdNotTrained_.size());
    for (auto& vec : prefTrainingType_) {
        DEBUG_PF(1, "Size: %lu", vec.size());
    }
    for (auto& vec : prefTarget_) {
        DEBUG_PF(1, "Size: %lu", vec.size());
    }
    for (auto& vec : featureWeightFrequency_) {
        for (auto& vec2 : vec) {
            DEBUG_PF(1, "Size: %lu", vec2.size());
        }
    }
    DEBUG_PF(1, "Size: %lu", weightInit_.size());
    DEBUG_PF(1, "Size: %lu", prefThreshold_.size());
    DEBUG_PF(1, "Size: %lu", missTrainStep_.size());
    DEBUG_PF(1, "Size: %lu", hitTrainStep_.size());
    DEBUG_PF(1, "Size: %lu", featureList_.size());
    for (auto& iter : prefUsefulTable_) {
        iter.second.memCheck();
    }
    for (auto& vec : workTables_) {
        for (auto& item : vec) {
            item.prefetchTable_.memCheck();
            item.rejectTable_.memCheck();
            for (auto& item2 : item.featureTable_) {
                item2.memCheck();
            }
            DEBUG_PF(1, "Size: %lu", item.localPrefTable_.size());
        }
    }
    BasePrefetchFilter::memCheck();
}

void PerceptronPrefetchFilter::init() {
    if (initFlag_) {
        return;
    }
    BasePrefetchFilter::init();
    // 初始化表格
    CHECK_RET_EXIT(initThis(), "Failed to initiate PPF.");
    initFlag_ = true;
}

void PerceptronPrefetchFilter::regStats() {
    // 如果已经注册则不再注册
    if (regFlag_) {
        return;
    }

    // 执行父类的统计数据初始化操作
    BasePrefetchFilter::regStats();
   
    uint8_t cacheCount = caches_.size();
    for (int i = 0; i < cacheCount; i++) {
        const std::string srcCacheName = BaseCache::levelName_[i];
        if (usePref_[i] && enableFilter_) {
            for (int j = 0; j < prefTarget_[i].size(); j++) {
                const std::string tgtCacheName = BaseCache::levelName_[j];
                prefTarget_[i][j] = new Stats::Vector();
                prefTarget_[i][j]->init(numCpus_)
                        .name(name() + ".ppf_prefetch_sent_from_" +
                                srcCacheName +"_to_" + tgtCacheName)
                        .desc(std::string(
                                "Number of prefetch requests sent to ")
                                + tgtCacheName + " from " + srcCacheName)
                        .flags(total);
            }

            // 按照CPU初始化
            for (int j = 0; j < numCpus_; j++) {
                for (int k = 0; k < prefTarget_[i].size(); k++) {
                    prefTarget_[i][k]->subname(j, std::string("cpu") +
                            std::to_string(j));
                }
            }
        } else {
            for (int j = 0; j < prefTarget_[i].size(); j++) {
                prefTarget_[i][j] = emptyStatsVar_;
            }
        }
    }
    
    int weightNum = 1 << weightBits_;
    // 为不同的WorkTable设置相关统计变量
    for (auto& tables : workTables_) {
        if (tables.empty()) {
            continue;
        }
        Tables& workTable = tables.front();
        int i = workTable.statsLevel_;
        if (enableFilter_) {
            prefAccepted_[i] = new Stats::Vector();
            prefAccepted_[i]->init(numCpus_)
                    .name(name() + ".ppf_prefetch_accepted_from_" +
                            workTable.name_)
                    .desc(std::string("Number of prefetch requests "
                            "accepted from ") + workTable.name_)
                    .flags(total);
        
            prefRejected_[i] = new Stats::Vector();
            prefRejected_[i]->init(numCpus_)
                    .name(name() + ".ppf_prefetch_rejected_from_" +
                            workTable.name_)
                    .desc(std::string("Number of prefetch rejected "
                            "requests from ") + workTable.name_)
                    .flags(total);
        
            prefThreshing_[i] = new Stats::Vector();
            prefThreshing_[i]->init(numCpus_)
                    .name(name() + ".ppf_prefetch_threshing_from_" +
                            workTable.name_)
                    .desc(std::string("Number of threshing prefetch "
                            "requests from ") + workTable.name_)
                    .flags(total);
            
            prefNotTrained_[i] = new Stats::Vector();
            prefNotTrained_[i]->init(numCpus_)
                    .name(name() + ".ppf_prefetch_untrained_pref_from_" +
                            workTable.name_)
                    .desc(std::string("Number of untrained prefetch "
                            "requests from ") + workTable.name_)
                    .flags(total);
            
            dmdNotTrained_[i] = new Stats::Vector();
            dmdNotTrained_[i]->init(numCpus_)
                    .name(name() + ".ppf_prefetch_untrained_dmd_from_" +
                            workTable.name_)
                    .desc(std::string("Number of untrained demand "
                            "requests from ") + workTable.name_)
                    .flags(total);
            
            for (int j = 0; j < TrainTypeCount; j++) {
                const std::string typeStr = getTrainTypeStr(TrainType(j));
                prefTrainingType_[i][j] = new Stats::Vector();
                prefTrainingType_[i][j]->init(numCpus_)
                        .name(name() + ".ppf_training_type_" + typeStr +
                                "_for_" + workTable.name_)
                        .desc(std::string("Times of training with type ") +
                                typeStr + " from " + workTable.name_)
                        .flags(total);
            }
            
            for (int j = 0; j < featureList_.size(); j++) {
                for (int k = 0; k < weightNum; k++) {
                    featureWeightFrequency_[i][j][k] = new Stats::Vector();
                    featureWeightFrequency_[i][j][k]->init(numCpus_)
                            .name(name() + ".feature_" +
                                    featureList_[j].name_ + 
                                    "_weight_" + std::to_string(k) +
                                    "_in_" + workTable.name_)
                            .desc("Time of appearence of a specific"
                                    " weight for a feature.")
                            .flags(total);
                }
            }

            for (int n = 0; n < numCpus_; n++) {
                const std::string cpuIdStr = std::string("cpu") +
                        std::to_string(n);
                prefAccepted_[i]->subname(n, cpuIdStr);
                prefRejected_[i]->subname(n, cpuIdStr);
                prefThreshing_[i]->subname(n, cpuIdStr);
                prefNotTrained_[i]->subname(n, cpuIdStr);
                dmdNotTrained_[i]->subname(n, cpuIdStr);
                for (int j = 0; j < TrainTypeCount; j++) {
                    prefTrainingType_[i][j]->subname(n, cpuIdStr);
                }
                for (int j = 0; j < featureList_.size(); j++) {
                    for (int k = 0; k < weightNum; k++) {
                        featureWeightFrequency_[i][j][k]->subname(n, cpuIdStr);
                    }
                }
            }
        } else {
            prefAccepted_[i] = emptyStatsVar_;
            prefRejected_[i] = emptyStatsVar_;
            prefThreshing_[i] = emptyStatsVar_;
            prefNotTrained_[i] = emptyStatsVar_;
            dmdNotTrained_[i] = emptyStatsVar_;
            for (int j = 0; j < TrainTypeCount; j++) {
                prefTrainingType_[i][j] = emptyStatsVar_;
            }
            for (int j = 0; j < featureList_.size(); j++) {
                for (int k = 0; k < weightNum; k++) {
                    featureWeightFrequency_[i][j][k] = emptyStatsVar_;
                }
            }
        }
    }

    // 标记已注册状态
    regFlag_ = true;
}

std::string PerceptronPrefetchFilter::getTrainTypeStr(const TrainType type) {
    switch (type) {
    case GoodPref: return "GoodPref";
    case BadPref: return "BadPref";
    case UselessPref: return "UselessPref";
    case DemandMiss: return "DemandMiss";
    default: return "Unknown";
    }
}

PerceptronPrefetchFilter* PerceptronPrefetchFilterParams::create() {
    return PerceptronPrefetchFilter::create(this);
}
