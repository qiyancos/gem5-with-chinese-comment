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

#include "mem/packet.hh"
#include "mem/cache/base.hh"
#include "mem/cache/cache.hh"
#include "mem/cache/prefetch_filter/saturated_counter.hh"
#include "mem/cache/prefetch_filter/debug_flag.hh"
#include "mem/cache/prefetch_filter/table.hh"
#include "mem/cache/prefetch_filter/base.hh"
#include "mem/cache/prefetch_filter/ppf.hh"

namespace prefetch_filter {

template<typename T>
int CacheTable<T>::init(const uint32_t size, const uint8_t assoc,
        const bool valid) {
    return init(-1, size, assoc, valid);
}

template<typename T>
int CacheTable<T>::init(const int8_t tagBits, const uint32_t size,
        const uint8_t assoc, const bool valid) {
    CHECK_ARGS(size >= assoc && size % assoc == 0,
            "Illegal size and associativity combination");
    size_ = size;
    assoc_ = assoc;
    sets_ = size / assoc;
    
    // 生成对应的Mask
    // 生成对应的Mask
    tagMask_ = 0xffffffffffffffffLLU;
    setMask_ = tagMask_;
    uint8_t setBits = 0;
    while (sets_ >> ++setBits) {}
    setBits--;
    uint8_t totalBits = setBits + BasePrefetchFilter::cacheLineOffsetBits_;
    setMask_ = setMask_ << (64 - totalBits) >> (64 - setBits)
            << BasePrefetchFilter::cacheLineOffsetBits_;
    uint8_t tagBitsTemp = tagBits;
    if (tagBits == -1) {
        tagBitsTemp = 64 - totalBits;
        totalBits = 0;
    } else {
        totalBits = 64 - setBits + tagBits +
                BasePrefetchFilter::cacheLineOffsetBits_;
    }
    CHECK_ARGS(totalBits <= 64, "Bits of tag is too big for 64-bit addr");
    tagMask_ = tagMask_ << totalBits >> (64 - tagBitsTemp) <<
            (BasePrefetchFilter::cacheLineOffsetBits_ + setBits);

    // 初始化表格
    data_.clear();
    data_.resize(sets_, Set(assoc, CacheEntry {0, 0, valid}));
    return 0;
}

template<typename T>
int CacheTable<T>::init(const uint32_t size, const uint8_t assoc,
        const T& data, const bool valid) {
    return init(-1, size, assoc, data);
}

template<typename T>
int CacheTable<T>::init(const int8_t tagBits, const uint32_t size,
        const uint8_t assoc, const T& data, const bool valid) {
    CHECK_ARGS(size >= assoc && size % assoc == 0,
            "Illegal size and associativity combination");
    size_ = size;
    assoc_ = assoc;
    sets_ = size / assoc;
    
    // 生成对应的Mask
    tagMask_ = 0xffffffffffffffffLLU;
    setMask_ = tagMask_;
    uint8_t setBits = 0;
    while (sets_ >> ++setBits) {}
    setBits--;
    uint8_t totalBits = setBits + BasePrefetchFilter::cacheLineOffsetBits_;
    setMask_ = setMask_ << (64 - totalBits) >> (64 - setBits)
            << BasePrefetchFilter::cacheLineOffsetBits_;
    uint8_t tagBitsTemp = tagBits;
    if (tagBits == -1) {
        tagBitsTemp = 64 - totalBits;
        totalBits = 0;
    } else {
        totalBits = 64 - setBits + tagBits +
                BasePrefetchFilter::cacheLineOffsetBits_;
    }
    CHECK_ARGS(totalBits <= 64, "Bits of tag is too big for 64-bit addr");
    tagMask_ = tagMask_ << totalBits >> (64 - tagBitsTemp) <<
            (BasePrefetchFilter::cacheLineOffsetBits_ + setBits);

    // 初始化表格
    data_.clear();
    data_.resize(sets_, Set(assoc, CacheEntry {0, 0, valid, data}));
    return 0;
}

template<typename T>
int CacheTable<T>::touch(const uint64_t& addr) {
    uint32_t setIndex = ((addr & setMask_) >>
            BasePrefetchFilter::cacheLineOffsetBits_) % sets_;
    uint64_t tag = addr & tagMask_;
    Set& set = data_[setIndex];
    for (auto entry = set.begin(); entry != set.end(); entry++) {
        if (entry->tag_ == tag && entry->valid_) {
            return 1;
        }
    }
    return 0;
}

template<typename T>
int CacheTable<T>::read(const uint64_t& addr, T** data) {
    uint32_t setIndex = ((addr & setMask_) >>
            BasePrefetchFilter::cacheLineOffsetBits_) % sets_;
    uint64_t tag = addr & tagMask_;
    DEBUG_PF(3, "Read: Addr @0x%lx; Set %u; tag @0x%lx",
            addr, setIndex, tag)
    Set& set = data_[setIndex];
    for (auto entry = set.begin(); entry != set.end(); entry++) {
        if (entry->tag_ == tag && entry->valid_) {
            *data = &(entry->data_);
            CacheEntry temp = *entry;
            set.erase(entry);
            set.push_front(temp);
            return 1;
        }
    }
    *data = nullptr;
    return 0;
}

template<typename T>
int CacheTable<T>::write(const uint64_t& addr, const T& data,
        uint64_t* replacedAddr, T* replacedData) {
    uint32_t setIndex = ((addr & setMask_) >>
            BasePrefetchFilter::cacheLineOffsetBits_) % sets_;
    uint64_t tag = addr & tagMask_;
    DEBUG_PF(3, "Wrire: Addr @0x%lx; Set %u; tag @0x%lx",
            addr, setIndex, tag)
    Set& set = data_[setIndex];
    for (auto entry = set.begin(); entry != set.end(); entry++) {
        if (entry->tag_ == tag) {
            // 如果写入找到了表项，则写入并返回
            entry->data_ = data;
            CacheEntry temp = *entry;
            set.erase(entry);
            set.push_front(temp);
            return 0;
        }
    }

    // 没有命中则会进行替换
    CacheEntry& replacedEntry = set.back();
    if (replacedAddr) {
        *replacedAddr = replacedEntry.addr_;
    }
    if (replacedData) {
        *replacedData = replacedEntry.data_;
    }
    bool valid = replacedEntry.valid_;
    set.push_front(CacheEntry {tag, addr, true, data});
    set.pop_back();
    return 1 + valid;
}
    
template<typename T>
int CacheTable<T>::invalidate(const uint64_t& addr) {
    uint32_t setIndex = ((addr & setMask_) >>
            BasePrefetchFilter::cacheLineOffsetBits_) % sets_;
    uint64_t tag = addr & tagMask_;
    DEBUG_PF(3, "Invalidate: Addr @0x%lx; Set %u; tag @0x%lx",
            addr, setIndex, tag)
    Set& set = data_[setIndex];
    for (auto entry = set.begin(); entry != set.end(); entry++) {
        if (entry->tag_ == tag) {
            entry->valid_ = false;
            CacheEntry temp = *entry;
            set.erase(entry);
            set.push_back(temp);
            return 1;
        }
    }
    return 0;
}

template<typename T>
int CacheTable<T>::writeAll(const T& data, const bool valid) {
    for (auto& set : data_) {
        for (auto& entry : set) {
            entry.valid_ = valid;
            entry.data_ = data;
        }
    }
    return 0;
}

// 显示实例化
// Victim Cache
template class CacheTable<uint64_t>;
// Prefetch/Reject Table
template class CacheTable<std::vector<uint16_t>>;
// Weight Table
template class CacheTable<SaturatedCounter>;
// Counter Cache
template class CacheTable<PrefetchUsefulTable::CounterEntry>;
// Prefetch Information for PPF work table
template class CacheTable<PerceptronPrefetchFilter::Tables::PrefInfoEntry>;

std::map<uint64_t, PrefetchUsefulInfo> IdealPrefetchUsefulTable::infoList_;

IdealPrefetchUsefulTable::IdealPrefetchUsefulTable(BaseCache* cache) :
        valid_(true), cache_(cache) {}

IdealPrefetchUsefulTable::IdealPrefetchUsefulTable(BaseCache* cache,
        const bool valid) : valid_(valid), cache_(cache) {}

int IdealPrefetchUsefulTable::newPref(const PacketPtr& prefPkt) {
    if (!valid_) {
        return 0;
    }
    CHECK_ARGS(prefPkt->caches_.size() == 1,
            "New pref added to useful table must be combined");
    std::vector<uint64_t> index;
    CHECK_RET(genIndex(prefPkt, &index),
            "Failed to generate index for prefetch");
    if (infoList_.find(index[0]) == infoList_.end()) {
        DEBUG_PF(1, "New Prefetch @0x%lx Record[0x%lx] in %s",
                prefPkt->getAddr(), index[0],
                BaseCache::levelName_[cache_->cacheLevel_].c_str());
        infoList_.insert(std::pair<uint64_t, PrefetchUsefulInfo>(index[0],
                PrefetchUsefulInfo(*(prefPkt->caches_.begin()), index[0])));
    } else {
        DEBUG_PF(1, "New prefetch @0x%lx already exists[0x%lx] in %s",
                prefPkt->getAddr(), index[0],
                BaseCache::levelName_[cache_->cacheLevel_].c_str());
    }
    return 0;
}

int IdealPrefetchUsefulTable::addPref(const PacketPtr& prefPkt,
        const uint64_t& replacedAddr, const DataType type) {
    if (!valid_) {
        return 0;
    }
    std::vector<uint64_t> indexes;
    CHECK_RET(genIndex(prefPkt, &indexes),
            "Failed to generate index for prefetch");
    std::set<PrefetchUsefulInfo*> newInfoList;
    for (const uint64_t& index : indexes) {
        DEBUG_PF(1, "%s @0x%lx Record [0x%lx] Replace @0x%lx",
                "Add prefetch", prefPkt->getAddr(), index, replacedAddr);
        CHECK_ARGS(infoList_.find(index) != infoList_.end(),
                "Prefetch record can not be found when adding new prefetch");
        infoList_[index].addReplacedAddr(cache_, replacedAddr);
        newInfoList.insert(&(infoList_[index])); 
    }
    const uint64_t prefAddr = prefPkt->getAddr() &
            BasePrefetchFilter::cacheLineAddrMask_;
    CHECK_ARGS(prefMap_.find(prefAddr) == prefMap_.end(),
            "Trying to add a prefetch already exists in the prefetch map");
    prefMap_[prefAddr] = newInfoList;
    if (type != NullType) {
        CHECK_ARGS(evictMap_.find(replacedAddr) == evictMap_.end(),
                "Trying to add a prefetch already exists in the prefetch map");
        evictMap_[replacedAddr] = newInfoList;
    }
    return 0;
}

int IdealPrefetchUsefulTable::isPrefHit(const uint64_t& addr) {
    if (!valid_) {
        return 0;
    }
    auto infoList = prefMap_.find(addr);
    CHECK_RET(infoList != prefMap_.end(),
            "Prefetch not exists in prefetch map");
    return (*infoList->second.begin())->info_.demandHit_;
}

int IdealPrefetchUsefulTable::findSrcCaches(const uint64_t& addr,
        std::set<BaseCache*>* caches) {
    if (!valid_) {
        return 0;
    }
    
    DEBUG_PF(1, "Find Source Caches for Prefetch @0x%lx", addr);
    auto iter = prefMap_.find(addr);
    const std::set<PrefetchUsefulInfo*>* infoList;
    if (iter == prefMap_.end()) {
        auto iterTemp = evictMap_.find(addr);
        CHECK_ARGS(iterTemp != evictMap_.end(),
                "Prefetch exists in neither prefetch map nor evict map");
        infoList = &(iterTemp->second);
    } else {
        infoList = &(iter->second);
    }
    caches->clear();
    for (auto& info : *infoList) {
        caches->insert(info->srcCache_);
    }
    return 0;
}

int IdealPrefetchUsefulTable::findAllSrcCaches(BaseCache* cache,
        const PacketPtr& pkt, std::set<BaseCache*>* caches) {
    if (!valid_) {
        return 0;
    }
    const uint64_t addr = pkt->getAddr();
    
    DEBUG_PF(1, "Find Source Caches for Prefetch @0x%lx", addr);
    
    auto iter = prefMap_.find(addr);
    const std::set<PrefetchUsefulInfo*>* infoList;
    if (iter == prefMap_.end()) {
        auto iterTemp = evictMap_.find(addr);
        if (iterTemp == evictMap_.end()) {
            auto info = infoList_.find(genIndex(cache,
                    pkt->targetCacheLevel_, addr));
            CHECK_ARGS(info != infoList_.end(),
                    "Can not prefetch in anywhere");
            caches->insert(info->second.srcCache_);
            return 0;
        } else {
            infoList = &(iterTemp->second);
        }
    } else {
        infoList = &(iter->second);
    }
    caches->clear();
    for (auto& info : *infoList) {
        caches->insert(info->srcCache_);
    }
    return 0;
}

int IdealPrefetchUsefulTable::updateHit(const PacketPtr& srcPkt,
        const uint64_t& hitAddr, const DataType srcType) {
    if (!valid_) {
        return 0;
    }
    std::set<uint8_t> cpuIds;
    CHECK_ARGS(prefMap_.find(hitAddr) != prefMap_.end(),
            "Failed to find hit prefetch address in the prefetch map");
    // 计算所有使用到数据的相关CPU
    if (srcType == Pref) {
        // 如果源操作是一个预取，需要对源预取有用进行判断
        // 足够有用的话可以当作Demand命中Prefetch处理
        std::vector<uint64_t> indexes;
        CHECK_RET(genIndex(srcPkt, &indexes),
                "Fialed to generate index for source prefetch");
        for (const uint64_t& index : indexes) {
            auto info = infoList_.find(index);
            if (info != infoList_.end() && info->second.isUseful()) {
                cpuIds.insert(info->second.srcCache_->cpuIds_.begin(),
                        info->second.srcCache_->cpuIds_.end());
            }
        }
    } else {
        // 如果源操作是一个Demand，直接将相关CPU合并即可
        for (BaseCache* cache : srcPkt->caches_) {
            DEBUG_PF(1, "Update Prefetch @0x%lx Hit From %s", hitAddr,
                    BaseCache::levelName_[cache->cacheLevel_].c_str());
            cpuIds.insert(cache->cpuIds_.begin(), cache->cpuIds_.end());
        }
    }

    // 对每一个预取数据相关的预取进行更新
    for (auto& info : prefMap_[hitAddr]) {
        CHECK_RET(info->updateUse(cpuIds),
                "Failed to update use in prefetch useful info");
        info->info_.demandHit_ = srcType == Dmd;
    }
    return 0;
}

int IdealPrefetchUsefulTable::combinePref(const PacketPtr& srcPkt,
        const uint64_t& hitAddr) {
    if (!valid_) {
        return 0;
    }
    
    CHECK_ARGS(prefMap_.find(hitAddr) != prefMap_.end(),
            "Failed to find hit prefetch address in the prefetch map");
    auto& prefInfoList = prefMap_[hitAddr];
    uint64_t replacedAddr;
    CHECK_RET((*prefInfoList.begin())->getReplacedAddr(cache_,
            &replacedAddr), "Failed to get replaced addr for old prefetch");
    
    std::vector<uint64_t> indexes;
    CHECK_RET(genIndex(srcPkt, &indexes),
            "Failed to generate index for prefetch");
    std::set<PrefetchUsefulInfo*> newInfoList;
    for (const uint64_t& index : indexes) {
        DEBUG_PF(1, "%s @0x%lx Record [0x%lx] to @0x%lx",
                "Combine prefetch", srcPkt->getAddr(), index, hitAddr);
        CHECK_ARGS(infoList_.find(index) != infoList_.end(),
                "Prefetch record can not be found when combining prefetch");
        infoList_[index].addReplacedAddr(cache_, replacedAddr);
        newInfoList.insert(&(infoList_[index])); 
    }
    
    prefInfoList.insert(newInfoList.begin(), newInfoList.end());
    if (replacedAddr) {
        CHECK_ARGS(evictMap_.find(replacedAddr) == evictMap_.end(),
                "Trying to add a prefetch already exists in the prefetch map");
        evictMap_[replacedAddr].insert(newInfoList.begin(), newInfoList.end());
    }
    return 0;
}

int IdealPrefetchUsefulTable::updateMiss(const PacketPtr& srcPkt) {
    if (!valid_) {
        return 0;
    }
    const uint64_t& missAddr = srcPkt->getAddr() &
            BasePrefetchFilter::cacheLineAddrMask_;
    std::set<uint8_t> cpuIds;
    CHECK_ARGS(evictMap_.find(missAddr) != prefMap_.end(),
            "Failed to find miss demand address in the evict map");
    // 计算所有使用到数据的相关CPU
    for (BaseCache* cache : srcPkt->caches_) {
        cpuIds.insert(cache->cpuIds_.begin(), cache->cpuIds_.end());
    }

    // 对每一个预取数据相关的预取进行更新
    for (auto& info : evictMap_[missAddr]) {
        CHECK_RET(info->updateHarm(cpuIds),
                "Failed to update demand miss in prefetch useful info");
    }
    return 0;
}

int IdealPrefetchUsefulTable::updateEvict(const uint64_t& addr) {
    if (!valid_) {
        return 0;
    }
    auto oldInfoList = prefMap_.find(addr);
    CHECK_ARGS(oldInfoList != prefMap_.end(),
            "Failed to find info list for old prefetch request");
    uint64_t replacedAddr;
    CHECK_RET((*oldInfoList->second.begin())->getReplacedAddr(
            cache_, &replacedAddr),
            "Failed to find replaced addr for prefetch in cache");
    if (replacedAddr) {
        CHECK_RET(evictMap_.find(replacedAddr) != evictMap_.end(),
                "Failed to find replaced record in evicted map");
        evictMap_.erase(replacedAddr);
    }
    for (auto& info : oldInfoList->second) {
        evictedPref_.push_back(*info);
        infoList_.erase(info->index_);
    }
    prefMap_.erase(addr);
    return 0;
}
    
int IdealPrefetchUsefulTable::replaceEvict(const PacketPtr& newPrefPkt,
        const uint64_t& oldPrefAddr) {
    if (!valid_) {
        return 0;
    }
    auto oldInfoList = prefMap_.find(oldPrefAddr);
    CHECK_ARGS(oldInfoList != prefMap_.end(),
            "Failed to find info list for old prefetch request");
    uint64_t replacedAddr;
    CHECK_RET((*oldInfoList->second.begin())->getReplacedAddr(
            cache_, &replacedAddr),
            "Failed to find replaced addr for prefetch in cache");
    // 首先删除旧的记录
    CHECK_RET(updateEvict(oldPrefAddr),
            "Failed to evict old prefetch when runing replace-evict");
    // 然后更新新的记录，并继承被替换的地址信息
    CHECK_RET(addPref(newPrefPkt, replacedAddr, Pref),
            "Failed to evict old prefetch when runing replace-evict");
    return 0;
}

int IdealPrefetchUsefulTable::updatePrefTiming(
        std::vector<std::vector<Stats::Vector*>>& totalUsefulValue,
        std::vector<std::vector<std::vector<Stats::Vector*>>>& usefulDegree,
        std::vector<std::vector<Stats::Vector*>>& usefulType,
        std::vector<std::vector<std::vector<std::vector<uint32_t>>>*>
        timingDegree) {
    if (!valid_) {
        return 0;
    }
    DEBUG_PF(2, "%s[%p] update timing for %d prefetches",
                BaseCache::levelName_[cache_->cacheLevel_].c_str(),
                cache_, evictedPref_.size());
    for (auto& info : evictedPref_) {
        // 更新Total Useful Value
        const std::set<uint8_t>& cpuIds = info.srcCache_->cpuIds_;
        const uint8_t cacheLevel = info.srcCache_->cacheLevel_;
        CHECK_ARGS(totalUsefulValue[cacheLevel].size() == 2,
                "Trying to update prefetch from cache without prefetcher");
        int singleCoreCount = info.info_.singleCoreUsefulCount_ -
                    info.info_.singleCoreHarmCount_;
        int crossCoreCount = info.info_.crossCoreUsefulCount_ -
                    info.info_.crossCoreHarmCount_;
        
        // 计算单核心Useful Degree
        int singleCoreDegree;
        if (singleCoreCount < PREF_DEGREE_1) {
            singleCoreDegree = 0;
        } else if (singleCoreCount < PREF_DEGREE_2) {
            singleCoreDegree = 1;
        } else if (singleCoreCount < PREF_DEGREE_3) {
            singleCoreDegree = 2;
        } else {
            singleCoreDegree = 3;
        }
        
        // 更新多核Useful Degree
        int crossCoreDegree;
        if (crossCoreCount < PREF_DEGREE_1) {
            crossCoreDegree = 0;
        } else if (crossCoreCount < PREF_DEGREE_2) {
            crossCoreDegree = 1;
        } else if (crossCoreCount < PREF_DEGREE_3) {
            crossCoreDegree = 2;
        } else {
            crossCoreDegree = 3;
        }

        // 获取当前预取类型的索引
        int typeIndex = info.getTypeIndex();

        // 判断对应的类型
        for (auto cpuId : cpuIds) {
            (*totalUsefulValue[cacheLevel][0])[cpuId] += singleCoreCount;
            (*totalUsefulValue[cacheLevel][1])[cpuId] += crossCoreCount;
            (*usefulDegree[cacheLevel][0][singleCoreDegree])[cpuId]++;
            (*usefulDegree[cacheLevel][1][crossCoreDegree])[cpuId]++;
            (*timingDegree[cpuId])[cacheLevel][0][singleCoreDegree]++;
            (*timingDegree[cpuId])[cacheLevel][1][crossCoreDegree]++;
            (*usefulType[cacheLevel][typeIndex])[cpuId]++;
        }
    }
    evictedPref_.clear();
    return 0;
}

int IdealPrefetchUsefulTable::setValidBit(const bool valid) {
    valid_ = valid;
    return 0;
}

uint64_t IdealPrefetchUsefulTable::genIndex(BaseCache* cache,
        const uint8_t targetCacheLevel, const uint64_t& addr) {
    return addr ^ (uint64_t(cache->prefetcherId_) << 48) ^
            (uint64_t(targetCacheLevel) << 56);
}

int IdealPrefetchUsefulTable::genIndex(const PacketPtr& prefPkt,
        std::vector<uint64_t>* indexes) {
    indexes->clear();
    const uint64_t addr = prefPkt->getAddr() &
            BasePrefetchFilter::cacheLineAddrMask_;
    for (BaseCache* cache : prefPkt->caches_) {
        indexes->push_back(genIndex(cache, prefPkt->targetCacheLevel_, addr));
    }
    return 0;
}

} // namespace prefetch_filter
