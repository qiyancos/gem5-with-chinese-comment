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
    const uint8_t index = static_cast<uint8_t>(PrefUsefulTypeList.size());
    PrefUsefulTypeList.push_back(PrefUsefulType(name, judgeFunc));
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

PrefetchUsefulInfo::PrefetchUsefulInfo(const uint8_t numCpus) {
    stats_.resize(numCpus);
}

int PrefetchUsefulInfo::updateUse(const uint64_t& coreBitMap,
        std::vector<Stats::Vector*> cacheStats) {
}

int PrefetchUsefulInfo::updateHarm(const uint64_t& coreBitMap) {
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

} // namespace prefetch_filter
