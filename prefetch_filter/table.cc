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

template<typename T>
CacheTable::Set::Set(const uint8_t assoc) {
}


template<typename T>
T& CacheTable::Set::read(const uint64_t& tag) {
}
        
template<typename T>
int CacheTable::Set::write(const uint64_t& tag, const T& value) {
}

    
template<typename T>
int CacheTable::init(const uint32_t size, const uint8_t assoc) {
}

template<typename T>
int CacheTable::init(const uint32_t size, const uint8_t assoc,
        const T& initVal) {
}

template<typename T>
int CacheTable::touch(const uint64_t& addr) {
}

template<typename T>
int CacheTable::read(const uint64_t& addr, T**) {
}

template<typename T>
int CacheTable::write(const uint64_t& addr, const T& value,
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
