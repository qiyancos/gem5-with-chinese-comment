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
#include "mem/cache/cache_blk.hh"
#include "mem/cache/prefetch_filter/saturated_counter.hh"
#include "mem/cache/prefetch_filter/debug_flag.hh"
#include "mem/cache/prefetch_filter/table.hh"
#include "mem/cache/prefetch_filter/base.hh"
#include "mem/cache/prefetch_filter/ppf.hh"

namespace prefetch_filter {

template<typename T>
int CacheTable<T>::init(const uint32_t size, const uint8_t assoc,
        const uint8_t rightShiftBits, const bool valid) {
    return init(-1, size, assoc, rightShiftBits, valid);
}

template<typename T>
int CacheTable<T>::init(const int8_t tagBits, const uint32_t size,
        const uint8_t assoc, const uint8_t rightShiftBits, const bool valid) {
    CHECK_ARGS(size >= assoc && size % assoc == 0,
            "Illegal size and associativity combination");
    size_ = size;
    assoc_ = assoc;
    sets_ = size / assoc;
    rightShiftBits_ = rightShiftBits;
    
    // 生成对应的Mask
    // 生成对应的Mask
    tagMask_ = 0xffffffffffffffffLLU;
    fullTagMask_ = tagMask_;
    setMask_ = tagMask_;
    uint8_t setBits = 0;
    while (sets_ >> ++setBits) {}
    setBits--;
    uint8_t totalBits = setBits + rightShiftBits_;
    setMask_ = setMask_ << (64 - totalBits) >> (64 - setBits)
            << rightShiftBits_;
    uint8_t tagBitsTemp = tagBits;
    if (tagBits == -1) {
        tagBitsTemp = 64 - totalBits;
        totalBits = 0;
    } else {
        totalBits = 64 - setBits - tagBits - rightShiftBits_;
    }
    CHECK_ARGS(totalBits <= 64, "Bits of tag is too big for 64-bit addr");
    tagMask_ = tagMask_ << totalBits >> (64 - tagBitsTemp) <<
            (rightShiftBits_ + setBits);
    
    tagBitsTemp = 64 - setBits - rightShiftBits_;
    totalBits = 0;
    fullTagMask_ = fullTagMask_ << totalBits >> (64 - tagBitsTemp) <<
            (rightShiftBits_ + setBits);

    // 初始化表格
    data_.clear();
    data_.resize(sets_, Set(assoc, CacheEntry {0, invalidBlkAddr_, valid}));
    validAddr_.resize(sets_);
    return 0;
}

template<typename T>
int CacheTable<T>::init(const uint32_t size, const uint8_t assoc,
        const uint8_t rightShiftBits, const T& data, const bool valid) {
    return init(-1, size, assoc, rightShiftBits, data, valid);
}

template<typename T>
int CacheTable<T>::init(const int8_t tagBits, const uint32_t size,
        const uint8_t assoc, const uint8_t rightShiftBits, const T& data,
        const bool valid) {
    CHECK_ARGS(size >= assoc && size % assoc == 0,
            "Illegal size and associativity combination");
    size_ = size;
    assoc_ = assoc;
    sets_ = size / assoc;
    rightShiftBits_ = rightShiftBits;
    
    // 生成对应的Mask
    tagMask_ = 0xffffffffffffffffLLU;
    fullTagMask_ = tagMask_;
    setMask_ = tagMask_;
    uint8_t setBits = 0;
    while (sets_ >> ++setBits) {}
    setBits--;
    uint8_t totalBits = setBits + rightShiftBits_;
    setMask_ = setMask_ << (64 - totalBits) >> (64 - setBits)
            << rightShiftBits_;
    uint8_t tagBitsTemp = tagBits;
    if (tagBits == -1) {
        tagBitsTemp = 64 - totalBits;
        totalBits = 0;
    } else {
        totalBits = 64 - setBits - tagBits - rightShiftBits_;
    }
    CHECK_ARGS(totalBits <= 64, "Bits of tag is too big for 64-bit addr");
    tagMask_ = tagMask_ << totalBits >> (64 - tagBitsTemp) <<
            (rightShiftBits_ + setBits);
    
    tagBitsTemp = 64 - setBits - rightShiftBits_;
    totalBits = 0;
    fullTagMask_ = fullTagMask_ << totalBits >> (64 - tagBitsTemp) <<
            (rightShiftBits_ + setBits);

    // 初始化表格
    data_.clear();
    data_.resize(sets_, Set(assoc,
            CacheEntry {0, invalidBlkAddr_, valid, data}));
    validAddr_.resize(sets_);
    return 0;
}

template<typename T>
int CacheTable<T>::touch(const uint64_t& addr, const bool withFullTag) {
    uint32_t setIndex = ((addr & setMask_) >> rightShiftBits_) % sets_;
    uint64_t tagMask = withFullTag ? fullTagMask_ : tagMask_;
    Set& set = data_[setIndex];
    for (auto entry = set.begin(); entry != set.end(); entry++) {
        if ((entry->tag_ & tagMask) == (addr & tagMask) && entry->valid_) {
            return withFullTag ? 1 : (entry->addr_ == addr ? 1 : 2);
        }
    }
    return 0;
}

template<typename T>
int CacheTable<T>::read(const uint64_t& addr, T** data,
        const bool withFullTag) {
    uint32_t setIndex = ((addr & setMask_) >> rightShiftBits_) % sets_;
    uint64_t tagMask = withFullTag ? fullTagMask_ : tagMask_;
    uint64_t tag = addr & tagMask;
    Set& set = data_[setIndex];
    for (auto entry = set.begin(); entry != set.end(); entry++) {
        if ((entry->tag_ & tagMask) == tag && entry->valid_) {
            *data = &(entry->data_);
            CacheEntry temp = *entry;
            set.erase(entry);
            set.push_front(temp);
            DEBUG_PF(3, "Read hit: Addr @0x%lx; Set %u; tag @0x%lx",
                    entry->addr_, setIndex, tag);
            return withFullTag ? 1 : (entry->addr_ == addr ? 1 : 2);
        }
    }
    DEBUG_PF(3, "Read miss: Addr @0x%lx; Set %u; tag @0x%lx",
            addr, setIndex, tag);
    *data = nullptr;
    return 0;
}

template<typename T>
int CacheTable<T>::write(const uint64_t& addr, const T& data,
        const bool withFullTag, uint64_t* replacedAddr, T* oldData) {
    uint32_t setIndex = ((addr & setMask_) >> rightShiftBits_) % sets_;
    uint64_t tagMask = withFullTag ? fullTagMask_ : tagMask_;
    uint64_t tag = addr & tagMask;
    Set& set = data_[setIndex];
    for (auto entry = set.begin(); entry != set.end(); entry++) {
        if ((entry->tag_ & tagMask) == tag && entry->valid_) {
            // 如果写入找到了表项，则写入并返回
            uint64_t oldAddr = entry->addr_;
            if (replacedAddr) {
                *replacedAddr = oldAddr;
            }
            if (oldData) {
                *oldData = entry->data_;
            }
            entry->addr_ = addr;
            entry->tag_ = addr & fullTagMask_;
            entry->data_ = data;
            CacheEntry temp = *entry;
            set.erase(entry);
            set.push_front(temp);
            DEBUG_PF(3, "Write hit: Addr @0x%lx; Set %u; tag @0x%lx",
                    oldAddr, setIndex, tag);
            return withFullTag ? 0 : (oldAddr == addr ? 0 : 3);
        }
    }
    // 没有命中则会进行替换
    CacheEntry& replacedEntry = set.back();
    if (replacedAddr) {
        *replacedAddr = replacedEntry.addr_;
    }
    if (oldData) {
        *oldData = replacedEntry.data_;
    }
    bool valid = replacedEntry.valid_;
    
    if (valid) {
        DEBUG_PF(3, "Write replace: Addr @0x%lx; Set %u; tag @0x%lx",
                replacedEntry.addr_, setIndex, tag);
        validAddr_[setIndex].erase(replacedEntry.addr_);
    } else {
        DEBUG_PF(3, "Write new: Addr @0x%lx; Set %u; tag @0x%lx",
                addr, setIndex, tag);
    }

    set.pop_back();
    set.push_front(CacheEntry {addr & fullTagMask_, addr, true, data});
    validAddr_[setIndex].insert(addr);
    return 1 + valid;
}
    
template<typename T>
int CacheTable<T>::invalidate(const uint64_t& addr, const bool withFullTag) {
    uint32_t setIndex = ((addr & setMask_) >> rightShiftBits_) % sets_;
    uint64_t tagMask = withFullTag ? fullTagMask_ : tagMask_;
    uint64_t tag = addr & tagMask;
    Set& set = data_[setIndex];
    for (auto entry = set.begin(); entry != set.end(); entry++) {
        if ((entry->tag_ & tagMask) == tag && entry->valid_) {
            DEBUG_PF(3, "Invalidate hit: Addr @0x%lx; Set %u; tag @0x%lx",
                    entry->addr_, setIndex, tag)
            entry->valid_ = false;
            CacheEntry temp = *entry;
            validAddr_[setIndex].erase(entry->addr_);
            set.erase(entry);
            set.push_back(temp);
            return withFullTag ? 1 : (entry->addr_ == addr ? 1 : 2);
        }
    }
    DEBUG_PF(3, "Invalidate miss: Addr @0x%lx; Set %u; tag @0x%lx",
            addr, setIndex, tag)
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

template<typename T>
void CacheTable<T>::memCheck() {
    size_t total = 0;
    for (auto& vec : data_) {
        total += vec.size();
    }
    DEBUG_PF(1, "Size: %lu", total);
    total = 0;
    for (auto& vec : validAddr_) {
        total += vec.size();
    }
    DEBUG_PF(1, "Size: %lu", total);
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
// Training Target Table
template class CacheTable<std::set<BaseCache*>>;

std::vector<PrefetchUsefulInfo> IdealPrefetchUsefulTable::deletedPref_;
std::map<uint64_t, PrefetchUsefulInfo> IdealPrefetchUsefulTable::infoList_;

IdealPrefetchUsefulTable::IdealPrefetchUsefulTable(BaseCache* cache) :
        valid_(true), cache_(cache) {}

IdealPrefetchUsefulTable::IdealPrefetchUsefulTable(BaseCache* cache,
        const bool valid) : valid_(valid), cache_(cache) {}

int IdealPrefetchUsefulTable::checkDataType(const uint64_t& addr,
        const std::set<uint64_t>& prefIndexes,
        const DataType type, const bool isMiss) {
    if (type == NullType || !valid_) return 0;
    
    if (type == Pref) {
        CHECK_ARGS(addr != invalidBlkAddr_,
                "Prefetch packet with invalid addres");
        // 如果是Miss，那么MSHR中的预取应该只有info，还未添加记录
        if (isMiss) {
            CHECK_ARGS(prefIndexes.size() != 0,
                    "Found a prefetch packet without generated index");
            for (auto prefIndex : prefIndexes) {
                CHECK_ARGS(infoList_.find(prefIndex) != infoList_.end(),
                        "Can not find record for prefetch in MSHR");
            }
            return 0;
        }
        // 如果是不是Miss，那么一定存在本地记录
        if (prefMap_.find(addr) != prefMap_.end()) {
            return 0;
        } else if (invalidatingPref_.find(addr) != invalidatingPref_.end()) {
            return 0;
        } else {
            CHECK_ARGS(invalidatedMap_.find(addr) != invalidatedMap_.end(),
                    "Can not find prefetch record in BPF");
        }
    } else if (type == Dmd) {
        CHECK_ARGS(addr != invalidBlkAddr_,
                "Demand packet with invalid addres");
        if (isMiss) {
            CHECK_ARGS(prefIndexes.empty(),
                    "None prefetch packet should not have prefetch index");
            return 0;
        } else {
            CHECK_ARGS(prefMap_.find(addr) == prefMap_.end() &&
                    invalidatingPref_.find(addr) == invalidatingPref_.end() &&
                    invalidatedMap_.find(addr) == invalidatedMap_.end(),
                    "None prefetch should not have prefetch record");
            return 0;
        }
    } else {
        CHECK_ARGS(addr != invalidBlkAddr_,
                "PendingPref with invalid address");
        // 对于一个PendingPref，一定会有Info记录
        CHECK_ARGS(isMiss, "Only check pending prefetch when miss");
        CHECK_ARGS(prefIndexes.size() != 0,
                "Found a prefetch packet without generated index");
        for (auto prefIndex : prefIndexes) {
            CHECK_ARGS(infoList_.find(prefIndex) != infoList_.end(),
                    "Can not find record for prefetch in MSHR");
        }
        return 0;
    }
    return 0;
}

int IdealPrefetchUsefulTable::isPrefValid(const uint64_t& addr) {
    if (!valid_) {
        return 0;
    }
    return prefMap_.find(addr) != prefMap_.end();
}

int IdealPrefetchUsefulTable::newPref(PacketPtr& prefPkt) {
    if (!valid_) {
        return 0;
    }
    CHECK_ARGS(prefPkt->packetType_ == Pref,
            "Packet is not for a prefetch request");
    CHECK_ARGS(prefPkt->caches_.size() == 1,
            "New pref added to useful table must not be combined");
    CHECK_ARGS(prefPkt->prefIndexes_.empty(),
            "New prefetch should not have global index");
    uint64_t index = generatePrefIndex(prefPkt);
    prefPkt->prefIndexes_.insert(index);
    if (infoList_.find(index) == infoList_.end()) {
        DEBUG_PF(1, "BPF new prefetch @0x%lx Record[0x%lx] in %s",
                prefPkt->getAddr(), index,
                BaseCache::levelName_[cache_->cacheLevel_].c_str());
        PrefetchUsefulInfo info(*(prefPkt->caches_.begin()),
                index, prefPkt->getAddr());
        info.isLevelDown_ =
                prefPkt->targetCacheLevel_ > prefPkt->srcCacheLevel_;
        info.regTime_ = prefPkt->req->time();
        infoList_.emplace(index, info);
        // 确保总共的可用预取数目不会超过LLC的大小
        CHECK_ARGS(infoList_.size() <=
                BasePrefetchFilter::caches_.back()[0]->tags->numBlocks,
                "It seems some prefetches are not deleted normally");
    } else {
        DEBUG_PF(1, "BPF new prefetch @0x%lx already exists[0x%lx] in %s",
                prefPkt->getAddr(), index,
                BaseCache::levelName_[cache_->cacheLevel_].c_str());
    }
    return 0;
}

int IdealPrefetchUsefulTable::deletePref(const PacketPtr& prefPkt) {
    if (!valid_) {
        return 0;
    }
    CHECK_ARGS(prefPkt->packetType_ == Pref,
            "Packet is not for a prefetch request");
    CHECK_ARGS(prefPkt->prefIndexes_.size() == 1,
            "Too many or no valid prefetch index for a prefetch packet");
    uint64_t addr = prefPkt->getAddr();
    auto infoIter = infoList_.find(*(prefPkt->prefIndexes_.begin()));
    CHECK_ARGS(infoIter != infoList_.end(),
            "Failed to find prefetch info when trying delete prefetch");
    if (infoIter->second.canDelete()) {
        deletedPref_.push_back(infoIter->second);
        infoList_.erase(infoIter);
        DEBUG_PF(2, "BPF directly delete prefetch @0x%lx Record[0x%lx] in %s",
                addr, *(prefPkt->prefIndexes_.begin()),
                BaseCache::levelName_[cache_->cacheLevel_].c_str());
    }
    return 0;
}

int IdealPrefetchUsefulTable::addPref(const PacketPtr& prefPkt,
        const uint64_t& replacedAddr, const DataType type) {
    if (!valid_) {
        return 0;
    }
    
    std::set<PrefetchUsefulInfo*> newInfoList;
    for (auto index : prefPkt->prefIndexes_) {
        DEBUG_PF(2, "%s @0x%lx record [0x%lx] replace @0x%lx",
                "BPF add prefetch", prefPkt->getAddr(), index, replacedAddr);
        auto infoIter = infoList_.find(index);
        CHECK_ARGS(infoIter != infoList_.end(),
                "Prefetch record can not be found when adding new prefetch");
        infoIter->second.addReplacedAddr(cache_, replacedAddr);
        newInfoList.insert(&(infoIter->second)); 
    }

    const uint64_t prefAddr = prefPkt->getAddr() &
            BasePrefetchFilter::cacheLineAddrMask_;
    CHECK_ARGS(prefMap_.find(prefAddr) == prefMap_.end(),
            "Trying to add a prefetch already exists in the prefetch map");
    prefMap_[prefAddr] = newInfoList;
    // 确保本级别记录的预取数目不会超过当前Cache的容量
    CHECK_ARGS(prefMap_.size() + invalidatingPref_.size() +
            invalidatedMap_.size() <= cache_->tags->numBlocks,
            "It seems that some prefetches are not invalidated properly");
 
    if (type == Dmd) {
        // 如果预取替换目标是一个Demand
        // 存在一种情况导致两个预取替换同一个地址，A预取替换了D
        // 但是之后D又被读取进来，然后B预取再次替换了D
        auto evictIter = evictMap_.find(replacedAddr);
        if (evictIter == evictMap_.end()) {
            DEBUG_PF(3, "BPF adds new victim record for prefetch");
            evictMap_[replacedAddr] = newInfoList;
        } else {
            DEBUG_PF(3, "BPF combines victim record for prefetch");
            evictIter->second.insert(newInfoList.begin(), newInfoList.end());
            CHECK_ARGS(evictIter->second.size() <= infoList_.size(),
                    "It seems that some info in evict map are not "
                    "released normally");
        }
    } else if (type == Pref && replacedAddr != invalidBlkAddr_) {
        auto evictIter = evictMap_.find(replacedAddr);
        if (evictIter != evictMap_.end()) {
            // 如果替换目标是一个预取，则不会检查是否存在
            const std::set<PrefetchUsefulInfo*>& oldInfoList =
                    evictIter->second;
            for (auto info : oldInfoList) {
                CHECK_RET(info->resetReplacedAddr(cache_, invalidBlkAddr_),
                        "Failed to reset replaced address for a "
                        "replaced prefetch");
            }
            DEBUG_PF(3, "BPF replaces old victim record for prefetch");
            evictIter->second = newInfoList;
        } else {
            DEBUG_PF(3, "BPF adds new victim record for prefetch");
            evictMap_[replacedAddr] = newInfoList;
        }
    } else {
        CHECK_ARGS(replacedAddr == invalidBlkAddr_,
                "NullType can not have valid address");
    }
    CHECK_ARGS(evictMap_.size() <= cache_->tags->numBlocks,
            "It seems that some evictions are not erased properly");
    return 0;
}

int IdealPrefetchUsefulTable::combinePref(const uint64_t& addr,
        const std::set<uint64_t>& indexes,
        std::set<BaseCache*>* correlatedCaches) {
    if (!valid_) {
        return 0;
    }
    
    auto prefInfoIter = prefMap_.find(addr);
    CHECK_ARGS(prefInfoIter != prefMap_.end(),
            "Failed to find hit prefetch address in the prefetch map");
    auto& prefInfoList = prefInfoIter->second;
    
    for (auto index : indexes) {
        auto infoIter = infoList_.find(index);
        CHECK_ARGS(infoIter != infoList_.end(),
                "Prefetch record can not be found when combining prefetch");
        PrefetchUsefulInfo* newInfo = &(infoIter->second); 
        // 如果已经存在，则不做处理
        if (prefInfoList.find(newInfo) != prefInfoList.end()) {
            DEBUG_PF(2, "%s @0x%lx with record[0x%lx] but already exists",
                    "BPF combine prefetch", addr, index);
            return 0;
        }
 
        DEBUG_PF(2, "%s @0x%lx with record[0x%lx] with old prefetch",
                "BPF combine prefetch", addr, index);
        uint64_t replacedAddr;
        CHECK_RET((*prefInfoList.begin())->getReplacedAddr(cache_,
                &replacedAddr),
                "Failed to get replaced addr for old prefetch");
        CHECK_RET((*prefInfoList.begin())->getCorrelatedCaches(cache_,
                correlatedCaches), "Failed to get correlated caches for "
                "combined prefetch");
        newInfo->addReplacedAddr(cache_, replacedAddr);
        
        prefInfoList.insert(newInfo);
        if (replacedAddr != invalidBlkAddr_) {
            CHECK_ARGS(evictMap_.find(replacedAddr) != evictMap_.end(),
                    "Record for replaced address not exist in evict map");
            evictMap_[replacedAddr].insert(newInfo);
        }
    }
    return 0;
}

int IdealPrefetchUsefulTable::evictPref(const uint64_t& addr,
        std::set<BaseCache*>* invalidatingCaches) {
    if (!valid_) {
        return 0;
    }

    bool alreadyInvalidated = false;
    std::set<PrefetchUsefulInfo*> infoList; 
    auto invalidatedInfoIter = invalidatedMap_.find(addr);
    if (invalidatedInfoIter != invalidatedMap_.end()) {
        CHECK_ARGS(prefMap_.find(addr) == prefMap_.end(),
                "Prefetch invalidated must not be in the prefetch table");
        // 如果已经被无效化过了，那么不会再次清除训练表格记录
        infoList = invalidatedInfoIter->second.infoList_;
        invalidatedMap_.erase(invalidatedInfoIter);
        alreadyInvalidated = true;
    } else {
        // 如果被删除的预取正在被无效化，会被记录下来
        auto invalidatingInfoIter = invalidatingPref_.find(addr);
        if (invalidatingInfoIter != invalidatingPref_.end()) {
            invalidatingPref_.erase(invalidatingInfoIter);
            CHECK_ARGS(forceDeletedPref_.find(addr) == forceDeletedPref_.end(),
                    "Interrupted invalidation recorded more than once");
            forceDeletedPref_.insert(addr);
        }
        
        // 进行训练表格删除操作
        auto prefInfoIter = prefMap_.find(addr);
        CHECK_ARGS(prefInfoIter != prefMap_.end(),
                "Failed to find record for prefetch ready to be deleted");
        infoList = prefInfoIter->second;
        prefMap_.erase(prefInfoIter);
        uint64_t replacedAddr;
        CHECK_RET((*infoList.begin())->getReplacedAddr(cache_, &replacedAddr),
                "Failed to find replaced addr for prefetch in cache");
        if (replacedAddr != invalidBlkAddr_) {
            auto evictInfoIter = evictMap_.find(replacedAddr);
            CHECK_ARGS(evictInfoIter != evictMap_.end(),
                    "Failed to find replaced record in evicted map");
            
            // 鉴于多个预取可以共享一个Victim，因此这里可能不会完全删除
            auto& evictInfoList = evictInfoIter->second;
            for (auto evictInfo : evictInfoList) {
                if (evictInfo->addr_ == addr) {
                    evictInfoList.erase(evictInfo);
                }
            }
            if (evictInfoList.empty()) {
                evictMap_.erase(evictInfoIter);
            }
        }
    } 
    
    // 删除对应信息的中Cache记录
    for (auto info : infoList) {
        CHECK_RET(info->rmReplacedAddr(cache_, invalidatingCaches),
                "Failed to remove replaced address record in info");
        // 判断预取是否完全删除，从而可以放入统计槽中
        if (!alreadyInvalidated) {
            DEBUG_PF(2, "BPF invalidate prefetch @0x%lx Record[0x%lx] in %s",
                    addr, info->index_,
                    BaseCache::levelName_[cache_->cacheLevel_].c_str());
        }
        if (info->canDelete()) {
            DEBUG_PF(2, "BPF delete prefetch @0x%lx Record[0x%lx] in %s",
                    addr, info->index_,
                    BaseCache::levelName_[cache_->cacheLevel_].c_str());
            deletedPref_.push_back(*info);
            infoList_.erase(info->index_);
            DEBUG_PF(2, "BPF after delete %lu info left", infoList_.size());
        }
    }
    return 0;
}
    
int IdealPrefetchUsefulTable::findSrcCaches(const uint64_t& addr,
        std::set<BaseCache*>* caches) {
    if (!valid_) {
        return 0;
    }
    
    auto prefInfoIter = prefMap_.find(addr);
    if (prefInfoIter == prefMap_.end()) {
        // 如果一个预取已经被提前无效化，那么不会返回任何信息
        return 0;
    }
    const std::set<PrefetchUsefulInfo*>& infoList = prefInfoIter->second;
    caches->clear();
    for (auto& info : infoList) {
        caches->insert(info->srcCache_);
    }
    return 0;
}

int IdealPrefetchUsefulTable::findInvalidatedCaches(const uint64_t& addr,
        std::set<BaseCache*>* caches) {
    if (!valid_) {
        return 0;
    }
    
    CHECK_ARGS(prefMap_.find(addr) == prefMap_.end(),
            "Prefetch should be already invalidated");
    auto invalidatedIter = invalidatedMap_.find(addr);
    CHECK_ARGS(invalidatedIter != invalidatedMap_.end(),
            "Can not find the prefetch already invalidated");
    const std::set<PrefetchUsefulInfo*>& infoList =
            invalidatedIter->second.infoList_;
    caches->clear();
    for (auto& info : infoList) {
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
    
    auto prefInfoIter = prefMap_.find(addr);
    const std::set<PrefetchUsefulInfo*>* infoList;
    if (prefInfoIter == prefMap_.end()) {
        CHECK_ARGS(pkt->prefIndexes_.size() != 0,
                "Prefetch pakcet must have at least one valid index");
        for (auto index : pkt->prefIndexes_) {
            auto infoIter = infoList_.find(index);
            CHECK_ARGS(infoIter != infoList_.end(),
                    "Can not prefetch in anywhere");
            caches->insert(infoIter->second.srcCache_);
        }
        return 0;
    } else {
        infoList = &(prefInfoIter->second);
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
        for (auto index : srcPkt->prefIndexes_) {
            auto infoIter = infoList_.find(index);
            if (infoIter != infoList_.end() && infoIter->second.isUseful()) {
                cpuIds.insert(infoIter->second.srcCache_->cpuIds_.begin(),
                        infoIter->second.srcCache_->cpuIds_.end());
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
        DEBUG_PF(2, "BPF update hit for prefetch @0x%lx Record[0x%lx]",
                hitAddr, info->index_);
        CHECK_RET(info->updateUse(cpuIds),
                "Failed to update use in prefetch useful info");
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
    if (evictMap_.find(missAddr) == evictMap_.end()) {
        // 如果没有找到替换数据的信息则跳过
        return 0;
    }
    // 计算所有使用到数据的相关CPU
    for (BaseCache* cache : srcPkt->caches_) {
        cpuIds.insert(cache->cpuIds_.begin(), cache->cpuIds_.end());
    }

    // 对每一个预取数据相关的预取进行更新
    for (auto& info : evictMap_[missAddr]) {
        DEBUG_PF(2, "BPF update demand miss for prefetch @0x%lx Record[0x%lx]",
                info->addr_, info->index_);
        CHECK_RET(info->updateHarm(cpuIds),
                "Failed to update demand miss in prefetch useful info");
    }
    return 0;
}

int IdealPrefetchUsefulTable::getReplacedAddr(const uint64_t& addr,
        uint64_t* replacedAddr) {
    if (!valid_) {
        return 0;
    }
    auto infoIter = prefMap_.find(addr);
    if (infoIter == prefMap_.end()) {
        CHECK_ARGS(invalidatingPref_.find(addr) != invalidatingPref_.end() ||
                invalidatedMap_.find(addr) != invalidatedMap_.end(),
                "Can not find prefetch record anywhere when trying to get"
                " replaced addr");
        *replacedAddr = invalidBlkAddr_;
        return 0;
    }
    CHECK_RET((*infoIter->second.begin())->getReplacedAddr(
            cache_, replacedAddr),
            "Failed to find replaced addr for prefetch in cache");
    return 0;
}

int IdealPrefetchUsefulTable::addPrefInvalidation(BaseCache* cache,
        const Tick& completeTick, const uint64_t& addr) {
    if (!valid_) {
        return 0;
    }
    
    // 由于对于共享的Cache可能会受到多个无效化请求，因此可能不存在
    // CHECK_ARGS(invalidatingPref_.find(addr) == invalidatingPref_.end(),
    //         "Trying to invalidate prefetch being invalidated");
    
    auto invalidatedInfoIter = invalidatedMap_.find(addr);
    if (invalidatedInfoIter != invalidatedMap_.end()) {
        // 如果希望无效化已经被无效化的内容，则应该不是同一个Cache触发的
        CHECK_ARGS(invalidatedInfoIter->second.srcCache_ != cache,
                "Trying to invalidate prefetch already invalidated "
                "by the same cache");
        return 0;
    }

    // 记录需要无效化的信息
    CHECK_ARGS(prefMap_.find(addr) != prefMap_.end(),
            "No trainable record for prefetch ready for invalidation");
    invalidatingTick_.push(Invalidation {completeTick, addr, cache});
    invalidatingPref_.insert(addr);
    const std::set<PrefetchUsefulInfo*>& infoList = prefMap_[addr];
    for (auto info : infoList) {
        DEBUG_PF(2, "%s @0x%lx Record[0x%lx] with end tick[%lu] in %s",
                "BPF start invalidating prefetch",
                addr, info->index_, completeTick,
                BaseCache::levelName_[cache_->cacheLevel_].c_str());
    }
    return 0;
}
    
int IdealPrefetchUsefulTable::addPrefCombination(const Tick& completeTick,
        const uint64_t& addr, const std::set<uint64_t>& indexes) {
    if (!valid_) {
        return 0;
    }
    
    // 记录需要无效化的信息
    CHECK_ARGS(prefMap_.find(addr) != prefMap_.end(),
            "No trainable record for prefetch ready for invalidation");
    combiningTick_.push(Combination {completeTick, addr, indexes});
    for (auto index : indexes) {
        DEBUG_PF(2, "%s @0x%lx with new record[0x%lx] and end tick[%lu] in %s",
                "BPF start ticking combination for prefetch",
                addr, index, completeTick,
                BaseCache::levelName_[cache_->cacheLevel_].c_str());
    }
    return 0;
}

int IdealPrefetchUsefulTable::updateInvalidation(const Tick& tickNow,
        std::set<uint64_t>* invalidatedPref) {
    if (!valid_) {
        return 0;
    }
    if (invalidatingTick_.empty()) {
        CHECK_ARGS(invalidatingPref_.empty(),
                "Left invalidating prefetch record without tick record");
        return 0;
    }

    Invalidation nextInvalidatedPref = invalidatingTick_.top();
    while (tickNow >= nextInvalidatedPref.completeTick_) {
        const uint64_t addr = nextInvalidatedPref.addr_;
        BaseCache* srcCache = nextInvalidatedPref.cache_;
        invalidatingTick_.pop();
        auto invalidatingIter = invalidatingPref_.find(addr);
        if (invalidatingIter == invalidatingPref_.end()) {
            auto invalidatedIter = invalidatedMap_.find(addr);
            if (invalidatedIter != invalidatedMap_.end()) {
                // 如果无效化操作被其他Cache触发的无效化抢先完成
                CHECK_ARGS(invalidatedIter->second.srcCache_ != srcCache,
                        "Finish invalidation by the same cache for "
                        "more than once");
            } else if (cache_->cpuIds_.size() == 1) {
                // 如果是一个非共享Cache，无效化期间被删除一定会有记录
                CHECK_ARGS(forceDeletedPref_.find(addr) !=
                        forceDeletedPref_.end(), "Can not find force-deleted "
                        "pref record for invalidation");
            }
            forceDeletedPref_.erase(addr);
            if (invalidatingTick_.empty()) {
                CHECK_ARGS(invalidatingPref_.empty(), "Left invalidating "
                        "prefetch record without tick record");
                return 0;
            } else {
                nextInvalidatedPref = invalidatingTick_.top();
                continue;
            }
        }

        // 无效化完成删除记录
        auto prefInfoIter = prefMap_.find(addr);
        CHECK_ARGS(prefInfoIter != prefMap_.end(),
                "Can not find prefetch record in prefetch map table");
        const std::set<PrefetchUsefulInfo*> infoList = prefInfoIter->second;
        prefMap_.erase(prefInfoIter);
        uint64_t replacedAddr;
        CHECK_RET((*infoList.begin())->getReplacedAddr(cache_, &replacedAddr),
                "Failed to find replaced addr for prefetch in cache");
        if (replacedAddr != invalidBlkAddr_) {
            auto evictInfoIter = evictMap_.find(replacedAddr);
            CHECK_RET(evictInfoIter != evictMap_.end(),
                    "Failed to find replaced record in evicted map");
            // 鉴于多个预取可以共享一个Victim，因此这里可能不会完全删除
            auto& evictInfoList = evictInfoIter->second;
            for (auto evictInfo : evictInfoList) {
                if (evictInfo->addr_ == addr) {
                    evictInfoList.erase(evictInfo);
                }
            }
            if (evictInfoList.empty()) {
                evictMap_.erase(evictInfoIter);
            }
        }
    
        for (auto info : infoList) {
            DEBUG_PF(2, "%s @0x%lx Record[0x%lx] with end tick[%lu] in %s",
                    "BPF finish invalidating prefetch", addr, info->index_,
                    nextInvalidatedPref.completeTick_,
                    BaseCache::levelName_[cache_->cacheLevel_].c_str());
        }
        
        // 记录无效化完成信息
        CHECK_ARGS(invalidatedMap_.find(addr) == invalidatedMap_.end(),
                "Invalidating prefetch already invalidated");
        invalidatedMap_[addr] = InvalidatedRecord {srcCache, infoList};
        invalidatingPref_.erase(invalidatingIter);
        invalidatedPref->insert(addr);
        if (invalidatingTick_.empty()) {
            CHECK_ARGS(invalidatingPref_.empty(),
                    "Left invalidating prefetch record without tick record");
            return 0;
        } else {
            nextInvalidatedPref = invalidatingTick_.top();
        }
    }
    return 0;
}

int IdealPrefetchUsefulTable::updateCombination(const Tick& tickNow) {
    if (!valid_) {
        return 0;
    }
    
    if (combiningTick_.empty()) {
        return 0;
    }
    Combination nextCombination = combiningTick_.top();
    while (tickNow >= nextCombination.completeTick_) {
        combiningTick_.pop();
        CHECK_RET(combinePref(nextCombination.addr_,
                nextCombination.indexes_),
                "Failed to combine prefetch to old one's record");
        for (auto index : nextCombination.indexes_) {
            DEBUG_PF(2, "%s @0x%lx record[0x%lx] with end tick[%lu] in %s",
                    "BPF finish combining prefetch",
                    nextCombination.addr_, index,
                    nextCombination.completeTick_,
                    BaseCache::levelName_[cache_->cacheLevel_].c_str());
        }
        if (combiningTick_.empty()) {
            return 0;
        } else {
            nextCombination = combiningTick_.top();
        }
    }
    return 0;
}
int IdealPrefetchUsefulTable::updatePrefTiming(
        std::vector<std::vector<Stats::Vector*>>& totalUsefulValue,
        std::vector<std::vector<std::vector<Stats::Vector*>>>& usefulDegree,
        std::vector<std::vector<Stats::Vector*>>& usefulType,
        std::vector<std::vector<std::vector<std::vector<uint32_t>>>*>
        timingDegree) {
    if (deletedPref_.empty()) {
        return 0;
    }
    DEBUG_PF(2, "BPF update timing for %d prefetches",
                deletedPref_.size());
    for (auto& info : deletedPref_) {
        // 更新Total Useful Value
        const std::set<uint8_t>& cpuIds = info.srcCache_->cpuIds_;
        const uint8_t cacheLevel = info.srcCache_->cacheLevel_;
        CHECK_ARGS(totalUsefulValue[cacheLevel].size() == 2,
                "Trying to update prefetch from cache without prefetcher");
        int singleCoreCount =
                static_cast<int>(info.info_.singleCoreUsefulCount_) -
                info.info_.singleCoreHarmCount_;
        int crossCoreCount =
                static_cast<int>(info.info_.crossCoreUsefulCount_) -
                info.info_.crossCoreHarmCount_;
        
        // 计算单核心Useful Degree
        int singleCoreDegree;
        if (singleCoreCount < PREF_DEGREE_HARM) {
            singleCoreDegree = 0;
        } else if (singleCoreCount < PREF_DEGREE_USELESS) {
            singleCoreDegree = 1;
        } else if (singleCoreCount == PREF_DEGREE_USELESS) {
            singleCoreDegree = 2;
        } else if (singleCoreCount < PREF_DEGREE_USEFUL) {
            singleCoreDegree = 3;
        } else {
            singleCoreDegree = 4;
        }
        
        // 更新多核Useful Degree
        int crossCoreDegree;
        if (crossCoreCount < PREF_DEGREE_HARM) {
            crossCoreDegree = 0;
        } else if (crossCoreCount < PREF_DEGREE_USELESS) {
            crossCoreDegree = 1;
        } else if (crossCoreCount == PREF_DEGREE_USELESS) {
            crossCoreDegree = 2;
        } else if (crossCoreCount < PREF_DEGREE_USEFUL) {
            crossCoreDegree = 3;
        } else {
            crossCoreDegree = 4;
        }

        // 获取当前预取类型的索引
        int typeIndex = info.getTypeIndex();
        if (typeIndex < 0) {
            CHECK_ARGS(false, "Failed to get a prorper type index for "
                    "prefetch @0x%lx with info[%d, %d, %d, %d]",
                    info.addr_, info.info_.singleCoreUsefulCount_,
                    info.info_.singleCoreHarmCount_,
                    info.info_.crossCoreUsefulCount_,
                    info.info_.crossCoreHarmCount_);
        }

        // 判断对应的类型
        for (auto cpuId : cpuIds) {
            (*totalUsefulValue[cacheLevel][0])[cpuId] += singleCoreCount;
            (*totalUsefulValue[cacheLevel][1])[cpuId] += crossCoreCount;
            (*usefulDegree[cacheLevel][0][singleCoreDegree])[cpuId]++;
            (*usefulDegree[cacheLevel][1][crossCoreDegree])[cpuId]++;
            (*timingDegree[cpuId])[cacheLevel][0][singleCoreDegree]++;
            (*timingDegree[cpuId])[cacheLevel][1][crossCoreDegree]++;
            (*usefulType[cacheLevel][typeIndex])[cpuId]++;
            (*BasePrefetchFilter::totalStatsPref_[cacheLevel])[cpuId]++;
        }
    }
    deletedPref_.clear();
    return 0;
}

int IdealPrefetchUsefulTable::setValidBit(const bool valid) {
    valid_ = valid;
    return 0;
}

void IdealPrefetchUsefulTable::memCheck() {
    size_t total = 0;
    for (auto& iter : prefMap_) {
        total += iter.second.size();
    }
    DEBUG_PF(1, "Size: %lu", total);
    total = 0;
    for (auto& iter : evictMap_) {
        total += iter.second.size();
    }
    DEBUG_PF(1, "Size: %lu", total);
    DEBUG_PF(1, "Size: %lu", invalidatingTick_.size());
    DEBUG_PF(1, "Size: %lu", invalidatingPref_.size());
    DEBUG_PF(1, "Size: %lu", forceDeletedPref_.size());
    total = 0;
    for (auto& iter : invalidatedMap_) {
        total += iter.second.infoList_.size();
    }
    DEBUG_PF(1, "Size: %lu", total);
    DEBUG_PF(1, "Size: %lu", combiningTick_.size());
    DEBUG_PF(1, "Size: %lu", deletedPref_.size());
    DEBUG_PF(1, "Size: %lu", infoList_.size());
}

int IdealPrefetchUsefulTable::findCorrection(
        std::map<BaseCache*, std::set<uint64_t>>* correctionList) {
    DEBUG_PF(2, "BPF start correction for prefetches");
    const BasePrefetchFilter::DoublePref& skipPref =
            BasePrefetchFilter::skipCorrectPref_;
    int total = 0;
    
    for (auto& level : BasePrefetchFilter::caches_) {
        for (auto cache : level) {
            correctionList->emplace(cache, std::set<uint64_t>());
        }
    }

    for (auto& infoIter : infoList_) {
        PrefetchUsefulInfo& info = infoIter.second;
        if (info.canDelete()) {
            if (info.srcCache_ == skipPref.first && (
                    info.addr_ == skipPref.second.first ||
                    info.addr_ == skipPref.second.second)) {
                continue;
            }
            if (info.srcCache_->mshrQueue.haveMatch(info.addr_)) {
                continue;
            } else if (info.isLevelDown_) {
                CHECK_ARGS(tickNow_ - info.regTime_ < maxResponseGap_,
                        "Found a level-down prefetch @0x%lx has exceeded "
                        "reponse-time limit[start: %lu]",
                        info.addr_, info.regTime_);
                continue;
            } else {
                CHECK_ARGS(false, "Can not find match MSHR for prefetch "
                        "which can be deleted when correcting prefetch");
            }
            continue;
        }
        std::set<BaseCache*> caches;
        CHECK_RET(info.getLocatedCaches(&caches),
                "Failed to get located caches for a prefetch");
        for (auto cache : caches) {
            if (cache == skipPref.first &&
                    (info.addr_ == skipPref.second.first ||
                    info.addr_ == skipPref.second.second)) {
                continue;
            }
            CacheBlk* blk = cache->tags->findBlock(info.addr_);
            // 如果没有对应的CacheBlock或者预取属性已经消失
            if (!blk || !blk->wasPrefetched()) {
                DEBUG_PF(3, "Prefetch @0x%lx in %s needs to be corrected",
                        info.addr_, cache->getName().c_str());
                (*correctionList)[cache].insert(info.addr_);
            }
        }
    }
    for (auto& level : BasePrefetchFilter::caches_) {
        uint8_t cacheLevel = level.front()->cacheLevel_;
        for (auto cache : level) {
            auto listIter = correctionList->find(cache);
            if (listIter->second.empty()) {
                correctionList->erase(listIter);
            } else {
                total += listIter->second.size();
                DEBUG_PF(2, "%lu prefetches will be corrected in %s[%p]",
                        listIter->second.size(),
                        BaseCache::levelName_[cacheLevel].c_str(), cache);
            }
        }
    }
    DEBUG_PF(2, "Total %lu prefetches will be corrected", total);
    return 0;
}

} // namespace prefetch_filter
