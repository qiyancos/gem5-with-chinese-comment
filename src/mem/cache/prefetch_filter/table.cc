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
        totalBits = 64 - setBits + tagBits + rightShiftBits_;
    }
    CHECK_ARGS(totalBits <= 64, "Bits of tag is too big for 64-bit addr");
    tagMask_ = tagMask_ << totalBits >> (64 - tagBitsTemp) <<
            (rightShiftBits_ + setBits);

    // 初始化表格
    data_.clear();
    data_.resize(sets_, Set(assoc, CacheEntry {0, 0, valid}));
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
        totalBits = 64 - setBits + tagBits + rightShiftBits_;
    }
    CHECK_ARGS(totalBits <= 64, "Bits of tag is too big for 64-bit addr");
    tagMask_ = tagMask_ << totalBits >> (64 - tagBitsTemp) <<
            (rightShiftBits_ + setBits);

    // 初始化表格
    data_.clear();
    data_.resize(sets_, Set(assoc, CacheEntry {0, 0, valid, data}));
    return 0;
}

template<typename T>
int CacheTable<T>::touch(const uint64_t& addr) {
    uint32_t setIndex = ((addr & setMask_) >> rightShiftBits_) % sets_;
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
    uint32_t setIndex = ((addr & setMask_) >> rightShiftBits_) % sets_;
    uint64_t tag = addr & tagMask_;
    Set& set = data_[setIndex];
    for (auto entry = set.begin(); entry != set.end(); entry++) {
        if (entry->tag_ == tag && entry->valid_) {
            *data = &(entry->data_);
            CacheEntry temp = *entry;
            set.erase(entry);
            set.push_front(temp);
            DEBUG_PF(3, "Read hit: Addr @0x%lx; Set %u; tag @0x%lx",
                    addr, setIndex, tag);
            return 1;
        }
    }
    DEBUG_PF(3, "Read miss: Addr @0x%lx; Set %u; tag @0x%lx",
            addr, setIndex, tag);
    *data = nullptr;
    return 0;
}

template<typename T>
int CacheTable<T>::write(const uint64_t& addr, const T& data,
        uint64_t* replacedAddr, T* replacedData) {
    uint32_t setIndex = ((addr & setMask_) >> rightShiftBits_) % sets_;
    uint64_t tag = addr & tagMask_;
    Set& set = data_[setIndex];
    for (auto entry = set.begin(); entry != set.end(); entry++) {
        if (entry->tag_ == tag) {
            // 如果写入找到了表项，则写入并返回
            entry->data_ = data;
            CacheEntry temp = *entry;
            set.erase(entry);
            set.push_front(temp);
            DEBUG_PF(3, "Write hit: Addr @0x%lx; Set %u; tag @0x%lx",
                    addr, setIndex, tag);
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
    if (valid) {
        DEBUG_PF(3, "Write replace: Addr @0x%lx; Set %u; tag @0x%lx",
                addr, setIndex, tag);
    } else {
        DEBUG_PF(3, "Write new: Addr @0x%lx; Set %u; tag @0x%lx",
                addr, setIndex, tag);
    }
    return 1 + valid;
}
    
template<typename T>
int CacheTable<T>::invalidate(const uint64_t& addr) {
    uint32_t setIndex = ((addr & setMask_) >> rightShiftBits_) % sets_;
    uint64_t tag = addr & tagMask_;
    Set& set = data_[setIndex];
    for (auto entry = set.begin(); entry != set.end(); entry++) {
        if (entry->tag_ == tag) {
            DEBUG_PF(3, "Invalidate: Addr @0x%lx; Set %u; tag @0x%lx",
                    addr, setIndex, tag)
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

std::vector<PrefetchUsefulInfo> IdealPrefetchUsefulTable::deletedPref_;
std::map<uint64_t, PrefetchUsefulInfo> IdealPrefetchUsefulTable::infoList_;

IdealPrefetchUsefulTable::IdealPrefetchUsefulTable(BaseCache* cache) :
        valid_(true), cache_(cache) {}

IdealPrefetchUsefulTable::IdealPrefetchUsefulTable(BaseCache* cache,
        const bool valid) : valid_(valid), cache_(cache) {}

int IdealPrefetchUsefulTable::isPrefValid(const uint64_t& addr) {
    if (!valid_) {
        return 0;
    }
    auto infoList = prefMap_.find(addr);
    return infoList != prefMap_.end();
}

int IdealPrefetchUsefulTable::newPref(PacketPtr& prefPkt) {
    if (!valid_) {
        return 0;
    }
    CHECK_ARGS(prefPkt->packetType_ == Pref,
            "Packet is not for a prefetch request");
    CHECK_ARGS(prefPkt->caches_.size() == 1,
            "New pref added to useful table must not be combined");
    CHECK_ARGS(prefPkt->indexes_.empty(), "New pref should no global index");
    uint64_t index = generatePrefIndex(prefPkt);
    prefPkt->indexes_.insert(index);
    if (infoList_.find(index) == infoList_.end()) {
        DEBUG_PF(1, "BPF new prefetch @0x%lx Record[0x%lx] in %s",
                prefPkt->getAddr(), index,
                BaseCache::levelName_[cache_->cacheLevel_].c_str());
        infoList_.insert(std::pair<uint64_t, PrefetchUsefulInfo>(index,
                PrefetchUsefulInfo(*(prefPkt->caches_.begin()), index)));
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
    CHECK_ARGS(!prefPkt->indexes_.empty(),
            "No invalid index for a prefetch packet");
    uint64_t addr = prefPkt->getAddr();
    for (auto index : prefPkt->indexes_) {
        CHECK_ARGS(infoList_.find(index) != infoList_.end(),
                "Failed to find prefetch info when trying delete prefetch");
        if (infoList_[index].canDelete()) {
            infoList_.erase(index);
            DEBUG_PF(2, "BPF delete prefetch @0x%lx Record[0x%lx] in %s", addr,
                    index, BaseCache::levelName_[cache_->cacheLevel_].c_str());
        }
    }
    return 0;
}

int IdealPrefetchUsefulTable::addPref(const PacketPtr& prefPkt,
        const uint64_t& replacedAddr, const DataType type) {
    if (!valid_) {
        return 0;
    }
    std::set<PrefetchUsefulInfo*> newInfoList;
    for (const uint64_t& index : prefPkt->indexes_) {
        DEBUG_PF(2, "%s @0x%lx Record [0x%lx] Replace @0x%lx",
                "BPF add prefetch", prefPkt->getAddr(), index, replacedAddr);
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
    
    std::set<PrefetchUsefulInfo*> newInfoList;
    for (const uint64_t& index : srcPkt->indexes_) {
        DEBUG_PF(2, "%s @0x%lx Record [0x%lx] to @0x%lx",
                "BPF combine prefetch", srcPkt->getAddr(), index, hitAddr);
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

int IdealPrefetchUsefulTable::evictPref(const uint64_t& addr,
        std::set<BaseCache*>* invalidatingCaches) {
    if (!valid_) {
        return 0;
    }

    std::set<PrefetchUsefulInfo*> infoList; 
    auto invalidatedInfoList = invalidatedMap_.find(addr);
    if (invalidatedInfoList != invalidatedMap_.end()) {
        CHECK_ARGS(prefMap_.find(addr) == prefMap_.end(),
                "Prefetch invalidated must not be in the prefetch table");
        // 如果已经被无效化过了，那么不会再次清除训练表格记录
        infoList = invalidatedInfoList->second.infoList_;
        invalidatedMap_.erase(addr);
    } else {
        // 如果被删除的预取正在被无效化，会被记录下来
        auto invalidatingInfoList = invalidatingPref_.find(addr);
        if (invalidatingInfoList == invalidatingPref_.end()) {
            invalidatingPref_.erase(addr);
            CHECK_ARGS(forceDeletedPref_.find(addr) == forceDeletedPref_.end(),
                    "Interrupted invalidation recorded more than once");
            forceDeletedPref_.insert(addr);
        }
        
        // 进行训练表格删除操作
        auto prefInfoList = prefMap_.find(addr);
        CHECK_ARGS(prefInfoList != prefMap_.end(),
                "Failed to find record for prefetch ready to be deleted");
        infoList = prefInfoList->second;
        prefMap_.erase(addr);
        uint64_t replacedAddr;
        CHECK_RET((*infoList.begin())->getReplacedAddr(cache_, &replacedAddr),
                "Failed to find replaced addr for prefetch in cache");
        if (replacedAddr) {
            CHECK_RET(evictMap_.find(replacedAddr) != evictMap_.end(),
                    "Failed to find replaced record in evicted map");
            evictMap_.erase(replacedAddr);
        }
    } 
    
    // 删除对应信息的中Cache记录
    for (auto info : infoList) {
        CHECK_RET(info->rmReplacedAddr(cache_, invalidatingCaches),
                "Failed to remove replaced address record in info");
    }

    // 判断预取是否完全删除，从而可以放入统计槽中
    for (auto info : infoList) {
        DEBUG_PF(2, "BPF invalidate prefetch @0x%lx Record[0x%lx] in %s",
                addr, info->index_,
                BaseCache::levelName_[cache_->cacheLevel_].c_str());
        if (info->canDelete()) {
            DEBUG_PF(2, "BPF delete prefetch @0x%lx Record[0x%lx] in %s",
                    addr, info->index_,
                    BaseCache::levelName_[cache_->cacheLevel_].c_str());
            deletedPref_.push_back(*info);
            infoList_.erase(info->index_);
        }
    }
    return 0;
}
    
int IdealPrefetchUsefulTable::findSrcCaches(const uint64_t& addr,
        std::set<BaseCache*>* caches) {
    if (!valid_) {
        return 0;
    }
    
    auto prefMapIter = prefMap_.find(addr);
    if (prefMapIter == prefMap_.end()) {
        // 如果一个预取已经被提前无效化，那么不会返回任何信息
        return 0;
    }
    const std::set<PrefetchUsefulInfo*>& infoList = prefMapIter->second;
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
    
    auto prefMapIter = prefMap_.find(addr);
    const std::set<PrefetchUsefulInfo*>* infoList;
    if (prefMapIter == prefMap_.end()) {
        auto info = infoList_.find(generatePrefIndex(pkt));
        CHECK_ARGS(info != infoList_.end(),
                "Can not prefetch in anywhere");
        caches->insert(info->second.srcCache_);
        return 0;
    } else {
        infoList = &(prefMapIter->second);
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
        for (const uint64_t& index : srcPkt->indexes_) {
            auto info = infoList_.find(index);
            if (info != infoList_.end() && info->second.isUseful()) {
                cpuIds.insert(info->second.srcCache_->cpuIds_.begin(),
                        info->second.srcCache_->cpuIds_.end());
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
    CHECK_RET(evictPref(oldPrefAddr),
            "Failed to evict old prefetch when runing replace-evict");
    // 然后更新新的记录，并继承被替换的地址信息
    CHECK_RET(addPref(newPrefPkt, replacedAddr, Pref),
            "Failed to evict old prefetch when runing replace-evict");
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
    
    if (invalidatedMap_.find(addr) != invalidatedMap_.end()) {
        // 如果希望无效化已经被无效化的内容，则应该不是同一个Cache触发的
        CHECK_ARGS(invalidatedMap_[addr].srcCache_ != cache,
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
            nextInvalidatedPref = invalidatingTick_.top();
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
            continue;
        }

        // 无效化完成删除记录
        auto prefInfoList = prefMap_.find(addr);
        CHECK_ARGS(prefInfoList != prefMap_.end(),
                "Can not find prefetch record in prefetch map table");
        const std::set<PrefetchUsefulInfo*>& infoList = prefInfoList->second;
        prefMap_.erase(addr);
        uint64_t replacedAddr;
        CHECK_RET((*infoList.begin())->getReplacedAddr(cache_, &replacedAddr),
                "Failed to find replaced addr for prefetch in cache");
        if (replacedAddr) {
            CHECK_RET(evictMap_.find(replacedAddr) != evictMap_.end(),
                    "Failed to find replaced record in evicted map");
            evictMap_.erase(replacedAddr);
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
        invalidatingPref_.erase(addr);
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

int IdealPrefetchUsefulTable::updatePrefTiming(
        std::vector<std::vector<Stats::Vector*>>& totalUsefulValue,
        std::vector<std::vector<std::vector<Stats::Vector*>>>& usefulDegree,
        std::vector<std::vector<Stats::Vector*>>& usefulType,
        std::vector<std::vector<std::vector<std::vector<uint32_t>>>*>
        timingDegree) {
    if (!valid_) {
        return 0;
    }
    DEBUG_PF(2, "BPF update timing for %d prefetches in %s",
                deletedPref_.size(),
                BaseCache::levelName_[cache_->cacheLevel_].c_str());
    for (auto& info : deletedPref_) {
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
    deletedPref_.clear();
    return 0;
}

int IdealPrefetchUsefulTable::setValidBit(const bool valid) {
    valid_ = valid;
    return 0;
}

} // namespace prefetch_filter
