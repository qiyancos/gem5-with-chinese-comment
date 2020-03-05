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

namespace prefetch_filter {

std::map<std::string, IndexInfo> IndexMap;

int addNewInfo(const std::string& name, const uint8_t bits) {
    const uint8_t index = static_cast<uint8_t>(IndexMap.size());
    IndexMap[name] = IndexInfo(index, bits);
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
    if (info_.size() != IndexMap.size()) {
        info_.resize(IndexMap.size(), 0);
    }
    CHECK_ARGS(index < info_.size(), "Prefetch information index %u %s %d",
            index, "out of bound", info_.size());
    info_[index] = value;
    return 0;
}

uint32_t PrefetchInfo::getInfo(const uint8_t index) {
    CHECK_RET(info_.size == Indexmap.size(), "Can not get value from %s",
            "uninitiated prefetch info");
    CHECK_ARGS(index < info_.size(), "Prefetch information index %u %s %d",
            index, "out of bound", info_.size());
    return info_[index];
}

// Original
DEF_INFO(PC1, 12) // 触发预取的指令PC, 12 bits
DEF_INFO(PC2_1, 12) // 触发预取指令之前触发指令的PC(右移1bit), 12 bits
DEF_INFO(PC3_2, 12) // 触发预取指令之前的之前触发指令的PC(右移2bit), 12 bits
DEF_INFO(Confidence, 4) // 当前预取的可信度, 4 bits
DEF_INFO(Address, 32) // 当前预取的目标地址, 32 bits
DEF_INFO(PageAddress, 12) // 预取地址的物理页号低12bit, 12 bits
DEF_INFO(Depth, 8) // 预取的深度, 8 bits 
DEF_INFO(Delta, 7) // 预取的地址间隔, 7 bits
DEF_INFO(Signature, 12) // SPP使用的签名, 12 bits
// Added
DEF_INFO(BPC1, 12) // 触发预取时最近的一个分支PC, 12 bits
DEF_INFO(BPC2_1, 12) // 触发预取时最近分支之前的分支PC(右移1bit), 12 bits
DEF_INFO(BPC2_2, 12) // 触发预取时最近分支之前的之前分支PC(右移2bit), 12 bits
DEF_INFO(PrefHarm, 4) // 预取的有害度信息, 4 bits
DEF_INFO(CoreIDMap, 8) // 相关核心的BitMap, 最多支持8核心, 8 bits
DEF_INFO(CoreID, 6) // 相关核心的BitMap, 最多支持64核心, 6 bits


// 多核有用的预取（他核心有用多于单核心有用）
int CrossCoreUseful = addNewPrefUsefulType("cross_core_useful",
        [] (const uint64_t& singleCoreUseful, const uint64_t& singleCoreHarm,
        const uint64_t& corssCoreUseful, const uint64_t& crossCoreHarm)
        -> bool {
                return (singleCoreUseful >= singleCoreHarm) &&
                (crossCoreUseful > crossCoreHarm) &&
                (singleCoreUseful - singleCoreHarm) < 
                (crossCoreUseful - crossCoreHarm);
        });

// 单核有用的预取（单核心有用多于多他核心有用）
int SingleCoreUseful = addNewPrefUsefulType("single_core_useful",
        [] (const uint64_t& singleCoreUseful, const uint64_t& singleCoreHarm,
        const uint64_t& corssCoreUseful, const uint64_t& crossCoreHarm)
        -> bool {
                return (singleCoreUseful > singleCoreHarm) &&
                (crossCoreUseful >= crossCoreHarm) &&
                (singleCoreUseful - singleCoreHarm) >=
                (crossCoreUseful - crossCoreHarm);
        });

// 自私的预取（单核心有用，他核心有害）
int Selfish = addNewPrefUsefulType("selfish",
        [] (const uint64_t& singleCoreUseful, const uint64_t& singleCoreHarm,
        const uint64_t& corssCoreUseful, const uint64_t& crossCoreHarm)
        -> bool {
                return (singleCoreUseful > singleCoreHarm) &&
                (crossCoreUseful < crossCoreHarm);
        });

// 无私的预取（单核心有害，他核心有用）
int Selfless = addNewPrefUsefulType("selfless",
        [] (const uint64_t& singleCoreUseful, const uint64_t& singleCoreHarm,
        const uint64_t& corssCoreUseful, const uint64_t& crossCoreHarm)
        -> bool {
                return (singleCoreUseful < singleCoreHarm) &&
                (crossCoreUseful > crossCoreHarm);
        });

// 无用预取（单核心无用，他核心无用）
int Useless = addNewPrefUsefulType("useless",
        [] (const uint64_t& singleCoreUseful, const uint64_t& singleCoreHarm,
        const uint64_t& corssCoreUseful, const uint64_t& crossCoreHarm)
        -> bool {
                return (singleCoreUseful == singleCoreHarm) &&
                (crossCoreUseful == crossCoreHarm);
        });

// 单核有害的预取（单核心有害多于多他核心有害）
int SingleCoreHarmful = addNewPrefUsefulType("single_core_harmful",
        [] (const uint64_t& singleCoreUseful, const uint64_t& singleCoreHarm,
        const uint64_t& corssCoreUseful, const uint64_t& crossCoreHarm)
        -> bool {
                return (singleCoreUseful < singleCoreHarm) &&
                (crossCoreUseful < crossCoreHarm) &&
                (singleCoreHarm - singleCoreUseful) >
                (crossCoreHarm - crossCoreUseful);
        });

// 多核有害的预取（他核心有害多于多单核心有害）
int CrossCoreHarmful = addNewPrefUsefulType("cross_core_harmful",
        [] (const uint64_t& singleCoreUseful, const uint64_t& singleCoreHarm,
        const uint64_t& corssCoreUseful, const uint64_t& crossCoreHarm)
        -> bool {
                return (singleCoreUseful < singleCoreHarm) &&
                (crossCoreUseful < crossCoreHarm) &&
                (singleCoreHarm - singleCoreUseful) <=
                (crossCoreHarm - crossCoreUseful);
        });

PrefetchUsefulInfo::PrefetchUsefulInfo(BaseCache* srcCache,
        const uint64_t& index) :
        srcCache_(srcCache), index_(index) {}

int PrefetchUsefulInfo::updateUse(const std::set<uint8_t>& cpuIds,
        std::vector<Stats::Vector*>& cacheStats) {
    uint8_t cacheLevel = srcCache->cacheLevel_ - 1;
    if (cacheLevel < 2) {
        for (uint8_t& cpuId : srcCache_->cpuIds_) {
            (*cacheStats[cacheLevel])[cpuId]++;
        }
    }
    int singleCoreValue;
    int crossCoreValue;
    CHECK_RET(getUpdateValue(cpuIds, srcCache->cpuIds, &singleCoreValue,
            &crossCoreValue), "Failed to get update value");
    info_.singleCoreUsefulCount_ += singleCoreValue;
    info_.crossCoreUsefulCount_ += crossCoreValue;
    return 0;
}

int PrefetchUsefulInfo::updateHarm(const std::set<uint8_t>& cpuIds) {
    int singleCoreValue;
    int crossCoreValue;
    CHECK_RET(getUpdateValue(cpuIds, srcCache->cpuIds, &singleCoreValue,
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

int PrefetchUsefulInfo::getReplacedAddr(BaseCache* cache,
        uint64_t* replacedAddr) {
    CHECK_ARGS(replacedAddress_.find(cache) != replacedAddress_.end(),
            "Replaced address not exists");
    *replacedAddr = replacedAddress_[cache];
    return 0;
}

int PrefetchUsefulInfo::isUseful() {
    // 只有有用程度达到一定标准才是有用预取
    if (info_.singleCoreUsefulCount_ - info_.singleCoreHarmCount_ >
            POS_DEGREE_DIVIDE && info_.crossCoreUsefulCount >=
            info_.crossCoreHarmCount) {
        return 1;
    } else if (info_.crossCoreUsefulCount_ - info_.crossCoreHarmCount_ >
            POS_DEGREE_DIVIDE && info_.singleCoreUsefulCount >=
            info_.singleCoreHarmCount) {
        return 1;
    }
    return 0;
}

int PrefetchUsefulInfo::getTypeIndex() {
    int typeIndex;
    for (PrefUsefulType& type : PrefUsefulTypeList) {
        if (type.isType(info_.singleCoreUsefulCount_,
                info_.singleCoreHarmCount_, info_.corssCoreUsefulCount_,
                info_.crossCoreHarmCount_)) {
            return type.index_;
        }
    }
    return -1;
}

int PrefetchUsefulInfo::getUpdateValue(const std::set<uint8_t>& srcCpuIds,
        const std::set<uint8_t>& targetCpuIds, int* singleCoreUpdate,
        int* crossCoreUpdate) {
    *singleCoreUpdate = 0;
    *crossCoreUpdate = 0;
    for (uint8_t srcCpuId : srcCpuIds) {
        if (targetCpuIds.find(srcCpuId) != targetCpuIds.end()) {
            *singleCoreUpdate++;
        } else {
            *crossCoreUpdate++;
        }
    }
    return 0;
}

} // namespace prefetch_filter
