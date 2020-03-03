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

namespace prefetch_filter {

// 显示实例化
template class CacheTable<std::vector<SaturatedCounter>>;
template class CacheTable<int8_t>;
template class CacheTable<BaseCache*>;

template<typename T>
int CacheTable::Set::init(const uint8_t assoc) {
    assoc_ = assoc;
    data_.resize(assoc, TableEntry {0, false});
    replaceIndex_ = 0;
    return 0;
}

template<typename T>
int CacheTable::Set::init(const uint8_t assoc, const T& data) {
    assoc_ = assoc;
    data_.resize(assoc, TableEntry {0, false, data});
    replaceIndex_ = 0;
    return 0;
}

template<typename T>
int CacheTable::Set::touch(const uint64_t& tag) {
    for (auto entry = data_.begin(); entry != data_.end(); entry++) {
        if (entry->tag_ == tag) {
            return 1;
        }
    }
    return 0;
}
        
template<typename T>
int CacheTable::Set::read(const uint64_t& tag, T** data) {
    for (auto entry = data_.begin(); entry != data_.end(); entry++) {
        if (entry->tag_ == tag) {
            *data = &(entry->data_);
            TableEntry temp = *entry;
            data_.erase(entry);
            data_.push_front(temp);
            return 1;
        }
    }
    *data = nullptr;
    return 0;
}
        
template<typename T>
int CacheTable::Set::write(const uint64_t& tag, const T& data,
        uint64_t* oldTag, T* replacedValue) {
    for (auto entry = data_.begin(); entry != data_.end(); entry++) {
        if (entry->tag_ == tag) {
            // 如果写入找到了表项，则写入并返回
            entry->data_ = data;
            TableEntry temp = *entry;
            data_.erase(entry);
            data_.push_front(temp);
            return 0;
        }
    }

    // 没有命中则会进行替换
    TableEntry& replacedEntry = data_.back();
    if (oldTag) {
        *oldTag = replacedEntry.tag_;
    }
    if (replacedValue) {
        *replacedValue = replacedEntry.data_;
    }
    bool valid = replacedEntry.valid_;
    data.push_front(TableEntry {tag, true, data});
    data.pop_back();
    return 1 + valid;
}
    
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
    data_.resize(sets_);
    for (Set& set : data_) {
        CHECK_RET(set.init(assoc), "Failed to initiate set");
    }
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
    data_.resize(sets_);
    for (Set& set : data_) {
        CHECK_RET(set.init(assoc, data), "Failed to initiate set");
    }
    return 0;
}

template<typename T>
int CacheTable::touch(const uint64_t& addr) {
    uint32_t setIndex = (addr & setMask_ >>
            BasePrefetchFilter::cacheLineOffsetBits_) % sets_;
    uint64_t tag = addr & tagMask_;
    return data_[setIndex].touch(tag);
}

template<typename T>
int CacheTable::read(const uint64_t& addr, T**) {
}

template<typename T>
int CacheTable::write(const uint64_t& addr, const T& data,
        uint64_t* replacedAddr = nullptr) {
}
    
template<typename T>
int CacheTable::invalidate(const uint64_t& addr) {
}

int IdealPrefetchUsefulTable::~IdealPrefetchUsefulTable() {
}

int IdealPrefetchUsefulTable::init(const uint8_t numCpus) {
}

int IdealPrefetchUsefulTable::updateHit(const uint64_t& addr,
        const DataType type, std::vector<Stats::Vector*>& cacheStats);
    
int IdealPrefetchUsefulTable::replaceEvict(const PacketPtr& newPrefPkt,
        const uint64_t& oldPrefAddr,
        const Stats::Vector* totalUsefulValue[][],
        Stats::Vector* usefulDegree[][][],
        std::vector<Stats::Vector*>* usefulType[],
        uint32_t timingStatus[][][]) {
}

int IdealPrefetchUsefulTable::updateEvict(const uint64_t& addr,
        Stats::Vector* totalUsefulValue[][],
        Stats::Vector* usefulDegree[][][],
        std::vector<Stats::Vector*>* usefulType[],
        uint32_t timingStatus[][][]) {
}
    
int IdealPrefetchUsefulTable::updatePrefTiming(uint32_t timingStatus[][][]) {
}

int IdealPrefetchUsefulTable::addPref(const PacketPtr& prefPkt,
        const uint64_t& evictAddr) {
}

int IdealPrefetchUsefulTable::isPrefHit(const uint64_t& addr) {
}

} // namespace prefetch_filter

#endif // __MEM_CACHE_PREFETCH_FILTER_TABLE_HH__
