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
#include "mem/cache/prefetch_filter/program_helper.hh"
#include "mem/cache/prefetch_filter/table.hh"
#include "mem/cache/prefetch_filter/base.hh"
#include "mem/cache/prefetch_filter/ppf.hh"

namespace prefetch_filter {

template<typename T>
int CacheTable::init(const uint32_t size, const uint8_t assoc, 
        const int8_t tagBits) {
    CHECK_ARGS(size >= assoc && size % assoc == 0,
            "Illegal size and associativity combination");
    size_ = size;
    assoc_ = assoc;
    sets_ = size / assoc;
    
    // 生成对应的Mask
    tagMask_ = 0xffffffffffffffffLLU;
    setMask_ = tagMask_;
    uint8_t totalBits = 0;
    while (sets_ >> ++totalBits) {}
    setMask_ = setMask_ << (64 - totalBits -
            BasePrefetch::cacheLineOffsetBits_) >> (64 - totalBits);
    if (tagBits != -1) {
        totalBits += (tagBits + BasePrefetchFilter::cacheLineOffsetBits_);
        CHECK_ARGS(totalBits <= 64, "Bits of tag is too big for 64-bit addr");
        totalBits = 64 - totalBits;
        tagMask_ = tagMask_ << totalBits >> (64 - tagBits);
    }

    // 初始化表格
    data_.clear();
    data_.resize(sets_, Set(assoc, CacheEntry {0, 0, false}));
    return 0;
}

template<typename T>
int CacheTable::init(const uint32_t size, const uint8_t assoc,
        const int8_t tagBits, const T& data) {
    CHECK_ARGS(size >= assoc && size % assoc == 0,
            "Illegal size and associativity combination");
    size_ = size;
    assoc_ = assoc;
    sets_ = size / assoc;
    
    // 生成对应的Mask
    tagMask_ = 0xffffffffffffffffLLU;
    setMask_ = tagMask_;
    uint8_t totalBits = 0;
    while (sets_ >> ++totalBits) {}
    setMask_ = setMask_ << (64 - totalBits -
            BasePrefetch::cacheLineOffsetBits_) >> (64 - totalBits);
    if (tagBits != -1) {
        totalBits += (tagBits + BasePrefetchFilter::cacheLineOffsetBits_);
        CHECK_ARGS(totalBits <= 64, "Bits of tag is too big for 64-bit addr");
        totalBits = 64 - totalBits;
        tagMask_ = tagMask_ << totalBits >> (64 - tagBits);
    }

    // 初始化表格
    data_.clear();
    data_.resize(sets_, Set(assoc, CacheEntry {0, 0, false, data}));
    return 0;
}

template<typename T>
int CacheTable::touch(const uint64_t& addr) {
    uint32_t setIndex = (addr & setMask_ >>
            BasePrefetchFilter::cacheLineOffsetBits_) % sets_;
    uint64_t tag = addr & tagMask_;
    Set& set = data_[setIndex];
    for (auto entry = set.begin(); entry != set.end(); entry++) {
        if (entry->tag_ == tag) {
            return 1;
        }
    }
    return 0;
}

template<typename T>
int CacheTable::read(const uint64_t& addr, T** data) {
    uint32_t setIndex = (addr & setMask_ >>
            BasePrefetchFilter::cacheLineOffsetBits_) % sets_;
    uint64_t tag = addr & tagMask_;
    Set& set = data_[setIndex];
    for (auto entry = set.begin(); entry != set.end(); entry++) {
        if (entry->tag_ == tag) {
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
int CacheTable::write(const uint64_t& addr, const T& data,
        uint64_t* replacedAddr = nullptr, T* replacedData) {
    uint32_t setIndex = (addr & setMask_ >>
            BasePrefetchFilter::cacheLineOffsetBits_) % sets_;
    uint64_t tag = addr & tagMask_;
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
int CacheTable::invalidate(const uint64_t& addr) {
    uint32_t setIndex = (addr & setMask_ >>
            BasePrefetchFilter::cacheLineOffsetBits_) % sets_;
    uint64_t tag = addr & tagMask_;
    Set& set = data_[setIndex];
    for (auto entry = set.begin(); entry != set.end(); entry++) {
        if (entry->tag_ == tag) {
            entry->valid = false;
            CacheEntry temp = *entry;
            set.erase(entry);
            set.push_back(temp);
            return 1;
        }
    }
    return 0;
}

// 显示实例化
template class CacheTable<std::vector<SaturatedCounter>>;
template class CacheTable<int8_t>;
template class CacheTable<BaseCache*>;
template class CacheTable<PrefetchUsefulTable::CounterEntry>;

IdealPrefetchUsefulTable::IdealPrefetchUsefulTable(BaseCache* cache) :
        cache_(cache), valid_(true) {}

IdealPrefetchUsefulTable::IdealPrefetchUsefulTable(BaseCache* cache,
        const bool valid) : cache_(cache), valid_(valid) {}

int IdealPrefetchUsefulTable::newPref(const PacketPtr& prefPkt) {
    CHECK_ARGS(valid_, "Should never call function in a invalid table");
    CHECK_ARGS(pkt->caches_.size() == 1,
            "New pref added to useful table must be combined");
    std::vector<uint64_t> index;
    CHECK_RET(genIndex(prefPkt, &index),
            "Failed to generate index for prefetch");
    CHECK_ARGS(infoList_.find(index[0]) == infoList_.end(),
            "Newly added prefetch already exists in list");
    infoList_[index[0]] = PrefetchUsefulInfo(prefPkt->caches_, index[0]);
    return 0;
}

int IdealPrefetchUsefulTable::addPref(const PacketPtr& prefPkt,
        const uint64_t& replacedAddr) {
    CHECK_ARGS(valid_, "Should never call function in a invalid table");
    std::vector<uint64_t> indexes;
    CHECK_RET(genIndex(prefPkt, &index),
            "Failed to generate index for prefetch");
    std::vector<PrefetchUsefulInfo*> newInfoList;
    for (const uint64_t& index : indexes) {
        CHECK_ARGS(infoList_.find(index) == infoList_.end(),
                "Prefetch record can not be found when adding new prefetch");
        infoList_[index].addReplacedAddr(cache_, replacedAddr);
        newInfoList.push_back(infoList_[index]); 
    }
    const uint64_t prefAddr = prefPkt->addr &
            BasePrefetchFilter::cacheLineAddrMask_;
    CHECK_ARGS(prefMap_.find(prefAddr) == prefMap_.end(),
            "Trying to add a prefetch already exists in the prefetch map")
    prefMap_[prefAddr] = newInfoList;
    CHECK_ARGS(evictMap_.find(replacedAddr) == evictMap_.end(),
            "Trying to add a prefetch already exists in the prefetch map")
    evictMap_[prefAddr] = newInfoList;
    return 0;
}

int IdealPrefetchUsefulTable::isPrefHit(const uint64_t& addr) {
    CHECK_ARGS(valid_, "Should never call function in a invalid table");
    auto infoList = prefMap_.find(addr);
    CHECK_RET(infoList != prefMap_.end(),
            "Prefetch not exists in prefetch map");
    return infoList->front()->info_.demandHit_;
}

int IdealPrefetchUsefulTable::findSrcCaches(const uint64_t& addr,
        std::vector<BaseCache*>* caches) {
    CHECK_ARGS(valid_, "Should never call function in a invalid table");
    auto iter = prefMap_.find(addr);
    const std::vector<PrefetchUsefulInfo*>* infoList;
    if (infoList != prefMap_.end()) {
        auto iterTemp = evictMap_.find(addr);
        CHECK_ARGS(iterTemp != evictMap_.end(),
                "Prefetch exists neither in prefetch map or evict map");
        infoList = &(iterTemp->second);
    } else {
        infoList = &(iter->second);
    }
    caches->clear();
    for (auto& info : *infoList) {
        caches->push_back(info->srcCache_);
    }
    return 0;
}

int IdealPrefetchUsefulTable::updateHit(const PacketPtr& srcPkt,
        const uint64_t& hitAddr, const DataType type,
        std::vector<Stats::Vector*>& cacheStats) {
    CHECK_ARGS(valid_, "Should never call function in a invalid table");
    std::set<uint8_t> cpuIds;
    CHECK_ARGS(prefMap_.find(hitAddr) != prefMap_.end(),
            "Failed to find hit prefetch address in the prefetch map");
    // 计算所有使用到数据的相关CPU
    if (type == Pref) {
        std::vector<uint64_t> indexes;
        CHECK_RET(genIndex(srcPkt, &indexes),
                "Fialed to generate index for source prefetch");
        for (const uint64_t& index : indexes) {
            auto info = infoList_.find(index);
            if (info != infoList_.end() && info->isUseful()) {
                cpuIds.insert(info->srcCache_->cpuIds.begin(),
                        info->srcCache_->cpuIds.end());
            }
        }
    } else {
        // 如果源操作是一个Demand，直接将相关CPU合并即可
        for (BaseCache* cache : srcPkt->caches_) {
            cpuIds.insert(cache->cpuIds_.begin(), cache->cpuIds_.end());
        }
    }

    // 对每一个预取数据相关的预取进行更新
    for (auto& info : prefMap_[hitAddr]) {
        CHECK_RET(info->updateUse(cpuIds, cacheStats),
                "Failed to update use in prefetch useful info");
        info->info_.demandHit_ = type == Dmd;
    }
    return 0;
}

int IdealPrefetchUsefulTable::updateMiss(const PacketPtr& srcPkt) {
    CHECK_ARGS(valid_, "Should never call function in a invalid table");
    const uint64_t& missAddr = srcPkt->addr &
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
    CHECK_ARGS(valid_, "Should never call function in a invalid table");
    auto oldInfoList = prefMap_.find(oldPrefAddr);
    CHECK_ARGS(oldInfoList != prefMap_.end(),
            "Failed to find info list for old prefetch request");
    uint64_t replacedAddr;
    CHECK_RET(oldInfoList->first().getReplacedAddr(cache_, &replacedAddr),
            "Failed to find replaced addr for prefetch in cache");
    CHECK_RET(evictMap_.find(replacedAddr) != evictMap_.end(),
            "Failed to find replaced record in evicted map");
    for (auto& info : oldInfoList->second) {
        evictedPref_.push_back(*info);
        infoList_.erase(info->index_);
    }
    prefMap_.erase(addr);
    evictMap_.erase(replacedAddr);
    return 0;
}
    
int IdealPrefetchUsefulTable::replaceEvict(const PacketPtr& newPrefPkt,
        const uint64_t& oldPrefAddr)
    CHECK_ARGS(valid_, "Should never call function in a invalid table");
    auto oldInfoList = prefMap_.find(oldPrefAddr);
    CHECK_ARGS(oldInfoList != prefMap_.end(),
            "Failed to find info list for old prefetch request");
    uint64_t replacedAddr;
    CHECK_RET(oldInfoList->first().getReplacedAddr(cache_, &replacedAddr),
            "Failed to find replaced addr for prefetch in cache");
    // 首先删除旧的记录
    CHECK_RET(updateEvict(oldPrefAddr),
            "Failed to evict old prefetch when runing replace-evict");
    // 然后更新新的记录，并继承被替换的地址信息
    CHECK_RET(addPref(newPrefPkt, replacedAddr),
            "Failed to evict old prefetch when runing replace-evict");
}

int IdealPrefetchUsefulTable::updatePrefTiming(
        Stats::Vector* totalUsefulValue[][],
        Stats::Vector* usefulDegree[][][],
        std::vector<Stats::Vector*>* usefulType[],
        uint32_t timingDegree[][][]) {
    CHECK_ARGS(valid_, "Should never call function in a invalid table");
    for (auto& info : evictedPref_) {
        // 更新Total Useful Value
        const std::set<uint8_t>& cpuIds = info.srcCache_->cpuIds_;
        const uint8_t cacheLevel = info.srcCache_->cacheLevel_ - 1;
        int singleCoreCount = info.info_.singleCoreUsefulCount_ -
                    info.info_.singleCoreHarmCount_;
        int crossCoreCount = info.info_.crossCoreUsefulCount_ -
                    info.info_.crossCoreHarmCount_;
        
        // 计算单核心Useful Degree
        int singleCoreDegree;
        if (singleCoreCount < NEG_DEGREE_DIVIDE) {
            singleCoreDegree = 0;
        } else if (singleCoreCount < 0) {
            singleCoreDegree = 1;
        } else if (singleCoreCount == 0) {
            singleCoreDegree = 2;
        } else if (singleCoreCount <= POS_DEGREE_DIVIDE) {
            singleCoreDegree = 3;
        } else {
            singleCoreDegree = 4;
        }
        
        // 更新多核Useful Degree
        if (crossCoreCount < NEG_DEGREE_DIVIDE) {
            crossCoreDegree = 0;
        } else if (crossCoreCount < 0) {
            crossCoreDegree = 1;
        } else if (crossCoreCount == 0) {
            crossCoreDegree = 2;
        } else if (crossCoreCount <= POS_DEGREE_DIVIDE) {
            crossCoreDegree = 3;
        } else {
            crossCoreDegree = 4;
        }

        // 获取当前预取类型的索引
        int typeIndex = info.getTypeIndex();

        // 判断对应的类型
        for (auto cpuId : cpuIds) {
            (*totalUsefulValue[cacheLevel][0])[cpuId] += singleCoreCount;
            (*totalUsefulValue[cacheLevel][1])[cpuId] += crossCoreCount;
            (*usefulDegree[cacheLevel][0][singleCoreDegree])[cpuId]++;
            (*usefulDegree[cacheLevel][1][crossCoreDegree])[cpuId]++;
            timingDegree[cacheLevel][0][singleCoreDegree]++;
            timingDegree[cacheLevel][1][crossCoreDegree]++;
            (*usefulType[cacheLevel][typeIndex])[cpuId]++;
        }
    }
    evictedPref_.clear();
    return 0;
}

int IdealPrefetchUsefulTable::genIndex(const PacketPtr& prefPkt,
        std::vector<uint64_t>* indexes) {
    indexes->clear();
    for (BaseCache* cache : pkt->caches_) {
        uint64_t index = pkt->addr & BasePrefetchFilter::cacheLineAddrMask_ ^
                (uint64_t(cache->cacheLevel_) << 54);
        for (uint8_t cpuId : cache->cpuIds_) {
            indexes.push_back(index ^ (uint64_t(cpuId) << 56);
        }
    }
    return 0;
}

} // namespace prefetch_filter

#endif // __MEM_CACHE_PREFETCH_FILTER_TABLE_HH__
