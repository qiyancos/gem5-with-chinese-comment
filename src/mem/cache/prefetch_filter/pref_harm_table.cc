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

#include "mem/cache/base.hh"
#include "mem/cache/cache.hh"
#include "mem/cache/prefetch_filter/base.hh"
#include "mem/cache/prefetch_filter/pref_harm_table.hh"
#include "mem/cache/prefetch_filter/debug_flag.hh"

namespace prefetch_filter {

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
    if (!valid_) {
        *replacedAddr = invalidBlkAddr_;
        return 0;
    }
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
