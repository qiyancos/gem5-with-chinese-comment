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

#include "mem/cache/base.hh"
#include "mem/cache/cache.hh"
#include "mem/cache/prefetch_filter/base.hh"
#include "mem/cache/prefetch_filter/debug_flag.hh"

namespace prefetch_filter {

Tick debugStartTick_ = 1000000000000LLU; //920000000LLU;
Tick maxResponseGap_ = 1000000000LLU;
Tick timerPrintGap_ = 100000000LLU;
Tick tickNow_ = 0;

uint64_t generateCoreIDMap(const std::set<BaseCache*>& caches) {
    std::set<uint8_t> cpuIds;
    for (auto cache : caches) {
        cpuIds.insert(cache->cpuIds_.begin(), cache->cpuIds_.end());
    }
    uint64_t result = 0;
    for (auto id : cpuIds) {
        CHECK_ARGS_EXIT(id < 64, "CPU ID out of mapping bound");
        result |= uint64_t(1) << id;
    }
    return result;
}

uint64_t generatePrefIndex(const PacketPtr pkt) {
    // 只对新的预取生成Index
    CHECK_ARGS_EXIT(pkt->caches_.size() == 1,
            "Only generate index for newly generated prefetch");
    CHECK_ARGS_EXIT(pkt->prefIndex_ == 0,
            "Only generate index for newly generated prefetch");
    return pkt->getAddr() ^
            ((pkt->req->time() / BasePrefetchFilter::clockPeriod_) << 32) ^
            (uint64_t((*pkt->caches_.begin())->prefetcherId_) << 48) ^
            (uint64_t(pkt->targetCacheLevel_) << 56);
}

std::string getDataTypeString(const DataType type) {
    switch(type) {
    case NullType: return "NullType";
    case Dmd: return "Demand";
    case Pref: return "Prefetch";
    case PendingPref: return "Pending Prefetch";
    }
    return "Unknown";
}

std::map<std::string, IndexInfo> PrefInfoIndexMap;

int addNewInfo(const std::string& name, const std::string& varName,
        const uint8_t bits) {
    const uint8_t index = static_cast<uint8_t>(PrefInfoIndexMap.size());
    PrefInfoIndexMap[name] = IndexInfo(index, bits, varName);
    return index;
}

std::vector<PrefUsefulType> PrefUsefulTypeList;

int addNewPrefUsefulType(const std::string& name,
        std::function<bool(const uint64_t&, const uint64_t&, const uint64_t&,
        const uint64_t&)> judgeFunc) {
    const int index = static_cast<uint8_t>(PrefUsefulTypeList.size());
    PrefUsefulTypeList.push_back(PrefUsefulType(index, name, judgeFunc));
    return index;
}

int PrefetchInfo::setInfo(const uint8_t index, const uint32_t value) {
    CHECK_ARGS(index < 64,
            "Prefetch information can only hold up to 64 info entries");
    if (info_.size() != PrefInfoIndexMap.size()) {
        info_.resize(PrefInfoIndexMap.size(), 0);
    }
    CHECK_ARGS(index < info_.size(),
            "Prefetch information index %u %s %d",
            index, "out of bound", info_.size());
    info_[index] = value;
    valid_ |= uint64_t(1) << index;
    return 0;
}

int PrefetchInfo::setInfo(const std::string& name, const uint32_t value) {
    auto infoIter = PrefInfoIndexMap.find(name);
    CHECK_RET_EXIT(infoIter != PrefInfoIndexMap.end(),
            "Can not find info named as \"%s\"", name);
    CHECK_RET_EXIT(setInfo(infoIter->second.index_, value),
            "Failed to set info with index");
    return 0;
}

int PrefetchInfo::getInfo(const uint8_t index, uint32_t* value) const {
    CHECK_RET(info_.size() == PrefInfoIndexMap.size(),
            "Can not get value from uninitiated prefetch info");
    CHECK_ARGS(index < info_.size(), "Prefetch information index %u %s %d",
            index, "out of bound", info_.size());
    if (valid_ & uint64_t(1) << index) {
        *value = info_[index];
        return 1;
    }
    return 0;
}

int PrefetchInfo::getInfo(const std::string& name, uint32_t* value) const {
    auto infoIter = PrefInfoIndexMap.find(name);
    CHECK_RET(infoIter != PrefInfoIndexMap.end(),
            "Can not find info named as \"%s\"", name);
    CHECK_RET(getInfo(infoIter->second.index_, value),
            "Failed to get info with index");
    return 0;
}

// Original
// 触发预取的指令PC, 12 bits
DEF_INFO(PC1, PC1, 12)
// 触发预取指令之前触发指令的PC(右移1bit), 12 bits
DEF_INFO(PC2_1, PC2>>1, 12)
// 触发预取指令之前的之前触发指令的PC(右移2bit), 12 bits
DEF_INFO(PC3_2, PC3>>2, 12)
// 当前预取的可信度, 4 bits
DEF_INFO(Confidence, Confidence, 3)
// 当前预取的目标地址, 32 bits
DEF_INFO(Address, Address, 32)
// 预取地址的物理页号低12bit, 12 bits
DEF_INFO(PageAddress, PageAddress, 12)
// 预取的深度, 8 bits 
DEF_INFO(Depth, Depth, 8)
// 预取的地址间隔, 7 bits
DEF_INFO(Delta, Delta, 7)
// SPP使用的签名, 12 bits
DEF_INFO(Signature, Signature, 12) 

// Added
// 触发预取时最近的一个分支PC, 12 bits
DEF_INFO(BPC1, BPC1, 12)
// 触发预取时最近分支之前的分支PC(右移1bit), 12 bits
DEF_INFO(BPC2_1, BPC2>>1, 12)
// 触发预取时最近分支之前的之前分支PC(右移2bit), 12 bits
DEF_INFO(BPC3_2, BPC3>>2, 12)
// 预取的有害度信息, 4 bits
DEF_INFO(PrefHarm, PrefHarm, 4)
// 相关核心的BitMap, 最多支持8核心, 8 bits
DEF_INFO(CoreIDMap, CoreIDMap, 8)
// 相关核心的BitMap, 最多支持8核心, 3 bits
DEF_INFO(CoreID, CoreID, 3)
// 发射预取的PrefetcherID, 最多支持16核心, 6 bits
DEF_INFO(PrefetcherID, PrefetcherID, 6)

// 避免出现unused-variable错误提示
std::vector<int> PrefInfoIndexes {PC1, PC2_1, PC3_2, Confidence, Address,
        PageAddress, Depth, Delta, Signature, BPC1, BPC2_1, BPC3_2,
        PrefHarm, CoreIDMap, CoreID, PrefetcherID};

// 多核有用的预取（他核心有用多于单核心有用）
int CrossCoreUseful = addNewPrefUsefulType("cross_core_useful",
        [] (const uint64_t& singleCoreUseful, const uint64_t& singleCoreHarm,
        const uint64_t& crossCoreUseful, const uint64_t& crossCoreHarm)
        -> bool {
                return (singleCoreUseful >= singleCoreHarm) &&
                (crossCoreUseful > crossCoreHarm) &&
                (singleCoreUseful - singleCoreHarm) < 
                (crossCoreUseful - crossCoreHarm);
        });

// 单核有用的预取（单核心有用多于多他核心有用）
int SingleCoreUseful = addNewPrefUsefulType("single_core_useful",
        [] (const uint64_t& singleCoreUseful, const uint64_t& singleCoreHarm,
        const uint64_t& crossCoreUseful, const uint64_t& crossCoreHarm)
        -> bool {
                return (singleCoreUseful > singleCoreHarm) &&
                (crossCoreUseful >= crossCoreHarm) &&
                (singleCoreUseful - singleCoreHarm) >=
                (crossCoreUseful - crossCoreHarm);
        });

// 自私的预取（单核心有用，他核心有害）
int Selfish = addNewPrefUsefulType("selfish",
        [] (const uint64_t& singleCoreUseful, const uint64_t& singleCoreHarm,
        const uint64_t& crossCoreUseful, const uint64_t& crossCoreHarm)
        -> bool {
                return (singleCoreUseful > singleCoreHarm) &&
                (crossCoreUseful < crossCoreHarm);
        });

// 无私的预取（单核心有害，他核心有用）
int Selfless = addNewPrefUsefulType("selfless",
        [] (const uint64_t& singleCoreUseful, const uint64_t& singleCoreHarm,
        const uint64_t& crossCoreUseful, const uint64_t& crossCoreHarm)
        -> bool {
                return (singleCoreUseful < singleCoreHarm) &&
                (crossCoreUseful > crossCoreHarm);
        });

// 无用预取（单核心无用，他核心无用）
int Useless = addNewPrefUsefulType("useless",
        [] (const uint64_t& singleCoreUseful, const uint64_t& singleCoreHarm,
        const uint64_t& crossCoreUseful, const uint64_t& crossCoreHarm)
        -> bool {
                return (singleCoreUseful == singleCoreHarm) &&
                (crossCoreUseful == crossCoreHarm);
        });

// 单核有害的预取（单核心有害多于多他核心有害）
int SingleCoreHarmful = addNewPrefUsefulType("single_core_harmful",
        [] (const uint64_t& singleCoreUseful, const uint64_t& singleCoreHarm,
        const uint64_t& crossCoreUseful, const uint64_t& crossCoreHarm)
        -> bool {
                return (singleCoreUseful < singleCoreHarm) &&
                (crossCoreUseful <= crossCoreHarm) &&
                (singleCoreHarm - singleCoreUseful) >
                (crossCoreHarm - crossCoreUseful);
        });

// 多核有害的预取（他核心有害多于多单核心有害）
int CrossCoreHarmful = addNewPrefUsefulType("cross_core_harmful",
        [] (const uint64_t& singleCoreUseful, const uint64_t& singleCoreHarm,
        const uint64_t& crossCoreUseful, const uint64_t& crossCoreHarm)
        -> bool {
                return (singleCoreUseful <= singleCoreHarm) &&
                (crossCoreUseful < crossCoreHarm) &&
                (singleCoreHarm - singleCoreUseful) <=
                (crossCoreHarm - crossCoreUseful);
        });

PrefetchUsefulInfo::PrefetchUsefulInfo(BaseCache* srcCache,
        const uint64_t& index, const uint64_t& addr) :
        srcCache_(srcCache), index_(index), addr_(addr) {}

int PrefetchUsefulInfo::updateUse(const std::set<uint8_t>& cpuIds) {
    int singleCoreValue;
    int crossCoreValue;
    CHECK_RET(getUpdateValue(cpuIds, cpuIds, &singleCoreValue,
            &crossCoreValue), "Failed to get update value");
    info_.singleCoreUsefulCount_ += singleCoreValue;
    info_.crossCoreUsefulCount_ += crossCoreValue;
    return 0;
}

int PrefetchUsefulInfo::updateHarm(const std::set<uint8_t>& cpuIds) {
    int singleCoreValue;
    int crossCoreValue;
    CHECK_RET(getUpdateValue(cpuIds, cpuIds, &singleCoreValue,
            &crossCoreValue), "Failed to get update value");
    info_.singleCoreHarmCount_ += singleCoreValue;
    info_.crossCoreHarmCount_ += crossCoreValue;
    return 0;
}

int PrefetchUsefulInfo::addReplacedAddr(BaseCache* cache,
        const uint64_t& replacedAddr) {
    CHECK_ARGS(replacedAddress_.find(cache) == replacedAddress_.end(),
            "Replaced address already exists before add one");
    replacedAddress_[cache] = replacedAddr;
    return 0;
}

int PrefetchUsefulInfo::resetReplacedAddr(BaseCache* cache,
        const uint64_t& replacedAddr) {
    auto replacedIter = replacedAddress_.find(cache);
    CHECK_ARGS(replacedIter != replacedAddress_.end(),
            "Replaced address which does not exist can not be reset");
    replacedIter->second = replacedAddr;
    return 0;
}

int PrefetchUsefulInfo::rmReplacedAddr(BaseCache* cache,
        std::set<BaseCache*>* correlatedCaches) {
    auto replacedIter = replacedAddress_.find(cache);
    CHECK_ARGS(replacedIter != replacedAddress_.end(),
            "Replaced address does not exist");
    replacedAddress_.erase(replacedIter);
    getCorrelatedCaches(cache, correlatedCaches);
    return 0;
}

int PrefetchUsefulInfo::getReplacedAddr(BaseCache* cache,
        uint64_t* replacedAddr) {
    auto replacedIter = replacedAddress_.find(cache);
    CHECK_ARGS(replacedIter != replacedAddress_.end(),
            "Replaced address not exists");
    *replacedAddr = replacedIter->second;
    return 0;
}

int PrefetchUsefulInfo::getCorrelatedCaches(BaseCache* cache,
        std::set<BaseCache*>* correlatedCaches) {
    if (correlatedCaches) {
        uint64_t srcCoreIDMap =
                generateCoreIDMap(std::set<BaseCache*> {cache});
        for (auto mapPair : replacedAddress_) {
            BaseCache* otherCache = mapPair.first;
            if (otherCache->cacheLevel_ > cache->cacheLevel_) {
                uint64_t coreIDMap = generateCoreIDMap(
                        std::set<BaseCache*> {otherCache});
                if ((coreIDMap & srcCoreIDMap) == srcCoreIDMap) {
                    correlatedCaches->insert(otherCache);
                }
            }
        }
    }
    correlatedCaches->erase(cache);
    return 0;
}

int PrefetchUsefulInfo::getLocatedCaches(std::set<BaseCache*>* caches) {
    for (auto mapPair : replacedAddress_) {
        caches->insert(mapPair.first);
    }
    return 0;
}

int PrefetchUsefulInfo::isUseful() {
    // 只有有用程度达到一定标准才是有用预取
    if (info_.singleCoreUsefulCount_ - info_.singleCoreHarmCount_ >
            PREF_DEGREE_USELESS && info_.crossCoreUsefulCount_ >=
            info_.crossCoreHarmCount_) {
        return 1;
    } else if (info_.crossCoreUsefulCount_ - info_.crossCoreHarmCount_ >
            PREF_DEGREE_USELESS && info_.singleCoreUsefulCount_ >=
            info_.singleCoreHarmCount_) {
        return 1;
    }
    return 0;
}

int PrefetchUsefulInfo::canDelete() {
    return replacedAddress_.empty();
}

int PrefetchUsefulInfo::getTypeIndex() {
    for (PrefUsefulType& type : PrefUsefulTypeList) {
        if (type.isType(info_.singleCoreUsefulCount_,
                info_.singleCoreHarmCount_, info_.crossCoreUsefulCount_,
                info_.crossCoreHarmCount_)) {
            return type.index_;
        }
    }
    return -1;
}

int PrefetchUsefulInfo::getUpdateValue(const std::set<uint8_t>& srcCpuIds,
        const std::set<uint8_t>& targetCpuIds, int* singleCoreUpdate,
        int* crossCoreUpdate) {
    int SCUpdate = 0;
    int CCUpdate = 0;
    for (uint8_t srcCpuId : srcCpuIds) {
        if (targetCpuIds.find(srcCpuId) != targetCpuIds.end()) {
            SCUpdate++;
        } else {
            CCUpdate++;
        }
    }
    *singleCoreUpdate = SCUpdate;
    *crossCoreUpdate = CCUpdate;
    return 0;
}

} // namespace prefetch_filter
