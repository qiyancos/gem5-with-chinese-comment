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

#include "base/stats/info.hh"
#include "mem/packet.hh"
#include "params/BasePrefetchFilter.hh"
#include "mem/cache/base.hh"
#include "mem/cache/cache.hh"
#include "mem/cache/prefetch_filter/base.hh"
#include "mem/cache/prefetch_filter/debug_flag.hh"

class BaseCache;

BasePrefetchFilter* BasePrefetchFilter::onlyInstance_ = nullptr;
int64_t BasePrefetchFilter::typeHash_ = 0;
bool BasePrefetchFilter::initFlag_ = false;
bool BasePrefetchFilter::regFlag_ = false;
uint64_t BasePrefetchFilter::cacheLineAddrMask_ = 0;
uint8_t BasePrefetchFilter::cacheLineOffsetBits_ = 0;
std::vector<Stats::Vector*> BasePrefetchFilter::totalStatsPref_;
std::vector<Stats::Vector*> BasePrefetchFilter::dismissedLevelUpPrefNoWB_;
std::vector<Stats::Vector*> BasePrefetchFilter::dismissedLevelUpPrefLate_;
std::vector<Stats::Vector*> BasePrefetchFilter::dismissedLevelDownPref_;
Tick BasePrefetchFilter::clockPeriod_ = 0;
std::vector<std::vector<BaseCache*>> BasePrefetchFilter::caches_;
uint8_t BasePrefetchFilter::maxCacheLevel_ = 0;
BasePrefetchFilter::DoublePref BasePrefetchFilter::skipCorrectPref_;

using namespace prefetch_filter;
using namespace Stats;

BasePrefetchFilter::BasePrefetchFilter(const BasePrefetchFilterParams *p) :
        ClockedObject(p),
        statsPeriod_(p->stats_period),
        nextPeriodTick_(p->stats_period),
        enableStats_(p->enable_stats),
        enableFilter_(p->enable_filter),
        enableRecursiveReplace_(p->enable_recursive_replace) {
    nextCorrectionTick_ = correctionPeriod_;
    cacheLineAddrMask_ = ~(p->block_size - 1);
    int blockSize = p->block_size;
    cacheLineOffsetBits_ = 0;
    while (blockSize >> ++cacheLineOffsetBits_) {}
    cacheLineOffsetBits_--;
    clockPeriod_ = clockPeriod();
}

BasePrefetchFilter* BasePrefetchFilter::create(
        const BasePrefetchFilterParams *p) {
    if (!onlyInstance_) {
        onlyInstance_ = new BasePrefetchFilter(p);
    }
    return onlyInstance_;
}

int BasePrefetchFilter::getCpuSideConnectedPortId(PacketPtr pkt,
        std::vector<int16_t>* portIds) {
    portIds->clear();
    std::set<BaseCache*> connectedCaches;
    CHECK_RET_EXIT(getCpuSideConnectedCache(pkt, &connectedCaches),
            "Failed to get cpu-side connected caches");
    CHECK_ARGS_EXIT(connectedCaches.size() == 1,
            "Too many or no connected cpu-side caches found");
    for (auto upLevelCache : connectedCaches) {
       portIds->push_back(upLevelCache->memSidePort._baseSlavePort->id);
    }
    return 0;
}

int BasePrefetchFilter::getCpuSideConnectedCache(PacketPtr pkt,
        std::set<BaseCache*>* caches) {
    caches->clear();
    BaseCache* cache = pkt->recentCache_;
    CHECK_ARGS_EXIT(cache, "No recent processing cache provided");
    uint8_t cacheLevel = cache->cacheLevel_;
    CHECK_ARGS_EXIT(cacheLevel > 1, "Can not find cpu port for a cache");
    
    uint64_t cacheCoreIDMap = generateCoreIDMap(std::set<BaseCache*> {cache});
    BaseCache* connectedCache = nullptr;
    uint8_t targetCacheLevel = cacheLevel == 2 ? !pkt->req->isInstFetch() :
            cacheLevel - 1;
    for (auto upLevelCache : caches_[targetCacheLevel]) {
        uint64_t coreIDMap = generateCoreIDMap(
                std::set<BaseCache*> {upLevelCache});
        if ((coreIDMap & cacheCoreIDMap) == coreIDMap) {
            // 我们现在不支持对共享Cache的向上查询
            CHECK_ARGS_EXIT(connectedCache == nullptr,
                    "Not supportted for shared cache");
            connectedCache = upLevelCache;
            caches->insert(connectedCache);
        }
    }
    return 0;
}

int BasePrefetchFilter::addCache(BaseCache* newCache) {
    int level = newCache->cacheLevel_;
    if (level >= caches_.size()) {
        caches_.resize(level + 1);
    }
    caches_[level].push_back(newCache);
    return 0;
}

int BasePrefetchFilter::notifyCacheHit(BaseCache* cache,
        const PacketPtr& pkt, const uint64_t& hitAddr,
        const DataTypeInfo& info) {
    // 时间维度的更新
    CHECK_ARGS_EXIT(info.source != NullType && info.target != NullType,
            "Unexpected source data type");
    CHECK_RET_EXIT(checkUpdateTimingAhead(), "Failed update timing processes");

    const uint64_t hitBlkAddr = hitAddr & cacheLineAddrMask_; 
    if (info.source == Dmd) {
        uint8_t cacheLevel = cache->cacheLevel_;
        for (const uint8_t cpuId : cache->cpuIds_) {
            timingStats_[cpuId].demandHit_[cacheLevel]++;
        }
        if (cacheLevel == maxCacheLevel_) {
            CHECK_ARGS_EXIT(pkt->recentCache_,
                    "Every demand packet snet to llc  must have recent cache");
            for (const uint8_t cpuId : pkt->recentCache_->cpuIds_) {
                (*demandReqHitCount_[cacheLevel])[cpuId]++;
                (*demandReqHitTotal_)[cpuId]++;
            }
        } else {
            for (const uint8_t cpuId : cache->cpuIds_) {
                (*demandReqHitCount_[cacheLevel])[cpuId]++;
                (*demandReqHitTotal_)[cpuId]++;
            }
        }
    }
    
    // 检查目标的正确性
    CHECK_RET_EXIT(usefulTable_[cache].checkDataType(hitBlkAddr,
            std::set<uint64_t>(), info.target, false),
            "Data type not match with the record in BPF");

    // 依据源数据和目标数据类型进行统计数据更新
    if (info.source == Dmd) {
        if (info.target == Pref) {
            if (usefulTable_[cache].isPrefValid(hitBlkAddr)) {
                // Demand Request命中了预取数据，只有当前预取还是预取才会处理
                CHECK_RET_EXIT(usefulTable_[cache].updateHit(pkt, hitBlkAddr,
                        Dmd), "Failed to update when dmd hit pref data");
            }
            // 如果没有子类，则删除，否则由子类进行删除
            if (!typeHash_) {
                CHECK_RET_EXIT(removePrefetch(cache, hitBlkAddr, true),
                        "Failed to remove prefetch record when hit by dmd");
            }
        }
    } else if (info.source == Pref) {
        // 更新预取请求的命中情况
        CHECK_ARGS(pkt->caches_.size() == 1,
                "Prefetch pakcet should have only one source cache");
        for (BaseCache* srcCache : pkt->caches_) {
            int cacheLevel = srcCache->cacheLevel_;
            uint8_t targetCacheOffset = cacheLevel ?
                    cache->cacheLevel_ - cacheLevel - 1 :
                    cache->cacheLevel_ - 2;
            for (const uint8_t cpuId : srcCache->cpuIds_) {
                (*prefHitCount_[cacheLevel][targetCacheOffset])[cpuId]++;
            }
        }
       
        // 预取命中了预取数据
        if (info.target == Pref &&
                usefulTable_[cache].isPrefValid(hitBlkAddr)) {
            // 只有目标预取确实是一个预取，才会更新
            CHECK_RET_EXIT(usefulTable_[cache].updateHit(pkt, hitBlkAddr,
                    Pref), "Failed to update when useful pref hit "
                    "pref data");
            std::set<BaseCache*> correlatedCaches;
            CHECK_RET_EXIT(usefulTable_[cache].combinePref(hitBlkAddr,
                    pkt->prefIndexes_, &correlatedCaches),
                    "Failed to combine two prefetch when pref hit pref data");
            
            // 开始传递合并预取信息
            if (!correlatedCaches.empty()) {
                uint8_t cacheLevel = cache->cacheLevel_ ?
                        cache->cacheLevel_ : 1;
                Tick completedTick = curTick() +
                        cache->forwardLatency * clockPeriod();
                while (++cacheLevel <= maxCacheLevel_) {
                    for (auto otherCache : correlatedCaches) {
                        if (otherCache->cacheLevel_ == cacheLevel) {
                            CHECK_RET(usefulTable_[otherCache].
                                    addPrefCombination(completedTick,
                                    hitBlkAddr, pkt->prefIndexes_),
                                    "Failed to forward prefetch combination");
                        }
                    }
                    completedTick += caches_[cacheLevel].front()
                            ->forwardLatency * clockPeriod();
                }
            }
        } else if (info.target == Dmd) {
            // 如果一个降级预取正好命中了目标等级的Cache，则删除该预取记录
            if (pkt->targetCacheLevel_ > pkt->srcCacheLevel_ &&
                    pkt->targetCacheLevel_ == cache->cacheLevel_) {
                CHECK_RET_EXIT(usefulTable_[*(pkt->caches_.begin())].
                        deletePref(pkt),
                        "Failed to delete prefetch when a level-down "
                        "prefetch just hit target cache level");
            }
        }
    }

    skipCorrectPref_ = DoublePref(cache,
            std::pair<uint64_t, uint64_t>(info.source == Pref ?
            hitBlkAddr : 0, 0));
    // 时间维度的检查与更新
    CHECK_RET_EXIT(checkUpdateTimingPost(),
            "Failed check & update timing stats information");
    return 0;
}

int BasePrefetchFilter::notifyCacheMiss(BaseCache* cache,
        PacketPtr& pkt, const PacketPtr& combinedPkt,
        const DataTypeInfo& info) {
    // 时间维度的更新
    CHECK_ARGS_EXIT(info.source != NullType, "Unexpected source data type");
    
    // 检查目标的正确性
    CHECK_RET_EXIT(usefulTable_[cache].checkDataType(combinedPkt ?
            combinedPkt->getAddr() & cacheLineAddrMask_ : 0,
            combinedPkt ? combinedPkt->prefIndexes_ : std::set<uint64_t>(),
            info.target, true),
            "Data type not match with the record in BPF");
    
    CHECK_RET_EXIT(checkUpdateTimingAhead(), "Failed update timing processes");
    
    if (info.source == Dmd) {
        uint8_t cacheLevel = cache->cacheLevel_;
        for (const uint8_t cpuId : cache->cpuIds_) {
            timingStats_[cpuId].demandMiss_[cacheLevel]++;
        }
        if (cacheLevel == maxCacheLevel_) {
            CHECK_ARGS_EXIT(pkt->recentCache_,
                    "Every demand packet snet to llc  must have recent cache");
            for (const uint8_t cpuId : pkt->recentCache_->cpuIds_) {
                (*demandReqMissCount_[cacheLevel])[cpuId]++;
            }
        } else {
            for (const uint8_t cpuId : cache->cpuIds_) {
                (*demandReqMissCount_[cacheLevel])[cpuId]++;
            }
        }
    }
    
    // 这里的info记录的source是新的Miss属性
    // 其中的target表示mshr中已存在的内容属性
    // prefetch->nulltype表示prefetch获得了一个新的mshr表项
    // 而非和一个demand类型的mshr合并，但是我们并不关心
    CHECK_ARGS_EXIT(info.source != Pref || info.target != Dmd,
            "Prefetch will not combined with demand in mshr.");
    if (info.source == Dmd) {
        if (info.target == Pref) {
            CHECK_ARGS_EXIT(combinedPkt->caches_.size() == 1,
                    "Prefetch pakcet should have only one source cache");
            for (BaseCache* srcCache : combinedPkt->caches_) {
                for (const uint8_t cpuId : srcCache->cpuIds_) {
                    (*shadowedPrefCount_[srcCache->cacheLevel_])[cpuId]++;
                }
            }
            // 当一个Demand合并了一个Prefetch，则会导致预取失效
            CHECK_RET_EXIT(usefulTable_[*(combinedPkt->caches_.begin())].
                    deletePref(combinedPkt),
                    "Failed to delete prefetch info after being shadowed");
        }
        if (info.target == Dmd || info.target == NullType) {
            // 一个新的Dmd请求发生了Miss
            CHECK_RET_EXIT(usefulTable_[cache].updateMiss(pkt),
                    "Failed to update when demand request miss");
        }
    } else if (info.target == Pref || info.target == PendingPref) {
        // 删除的一定是合并的Target
        CHECK_ARGS_EXIT(combinedPkt, "No combined packet provided");
        CHECK_ARGS_EXIT(combinedPkt->caches_.size() == 1,
                "Prefetch pakcet should have only one source cache");
        for (BaseCache* srcCache : combinedPkt->caches_) {
            for (const uint8_t cpuId : srcCache->cpuIds_) {
                (*squeezedPrefCount_[srcCache->cacheLevel_])[cpuId]++;
            }
        }
        CHECK_RET_EXIT(usefulTable_[*(combinedPkt->caches_.begin())].
                deletePref(combinedPkt),
                "Failed to delete prefetch info after being squeezed");
    } else if (info.target == NullType && *(pkt->caches_.begin()) == cache) {
        // 更新相关计数
        uint8_t level = cache->cacheLevel_;
        for (auto cpuId : cache->cpuIds_) {
            (*prefIssuedCount_[level])[cpuId]++;
            Tick waitingTime = pkt->getTimeStamp(level, Packet::WhenRecv);
            CHECK_ARGS_EXIT(pkt->req->time() <= waitingTime,
                    "It seems that a prefetch is generated later than sent");
            waitingTime -= pkt->req->time();
            (*prefWaitingCycles_[level])[cpuId] += (waitingTime + 20) /
                    clockPeriod_;
        }
        // 一个新生成的没有合并的预取，需要添加对应信息进行追踪
        CHECK_RET_EXIT(usefulTable_[cache].newPref(pkt),
                "Failed to add new prefetch info to useful table");
    }
    
    skipCorrectPref_ = DoublePref(cache,
            std::pair<uint64_t, uint64_t>(info.source == Pref ?
            pkt->getAddr() & cacheLineAddrMask_ : 0, 0));
    // 时间维度的检查与更新
    CHECK_RET_EXIT(checkUpdateTimingPost(),
            "Failed check & update timing stats information");
    return 0;
}

int BasePrefetchFilter::notifyCacheFill(BaseCache* cache,
        const PacketPtr &pkt, const uint64_t& evictedAddr,
        const DataTypeInfo& info) {
    // 时间维度的更新
    CHECK_ARGS_EXIT(info.source != NullType, "Unexpected source data type");
    const uint64_t evictedBlkAddr = evictedAddr & cacheLineAddrMask_;
    
    // 检查目标的正确性
    CHECK_RET_EXIT(usefulTable_[cache].checkDataType(evictedBlkAddr,
            std::set<uint64_t>(), info.target, false),
            "Data type not match with the record in BPF");
    CHECK_RET_EXIT(checkUpdateTimingAhead(), "Failed update timing processes");
    
    if (info.source == Dmd && info.target == Pref) {
        // 如果一个Demand Request替换了一个预取请求的数据
        // 则会将对应预取从当前Cache记录表中剔除
        if (!typeHash_) {
            CHECK_RET_EXIT(removePrefetch(cache, evictedBlkAddr, false),
                    "Failed to remove prefetch record when hit by dmd");
        }
    } else if (info.source == Pref) {
        // 更新时间戳信息
        for (auto cpuId : (*pkt->caches_.begin())->cpuIds_) {
            (*prefFillCount_[pkt->srcCacheLevel_][
                    cache->cacheLevel_ ? cache->cacheLevel_ : 1])[cpuId]++;
            Tick processTime = pkt->getProcessTime(cache->cacheLevel_ + 1);
            // 这里加上20是为了保证不会出现499 / 500 = 0的情况出现
            (*prefProcessCycles_[pkt->srcCacheLevel_][
                    cache->cacheLevel_ ? cache->cacheLevel_ : 1])[cpuId] +=
                    (processTime + 20) / clockPeriod_;
            DEBUG_PF(2, "Add %s process time %lu for prefetch @0x%lx",
                    BaseCache::levelName_[cache->cacheLevel_ + 1].c_str(),
                    processTime, pkt->getAddr());
            // 如果当前等级恰好是目标等级，则额外更新一个处理时间
            if (pkt->targetCacheLevel_ == cache->cacheLevel_ &&
                    pkt->srcCacheLevel_ <= cache->cacheLevel_) {
                (*prefFillCount_[pkt->srcCacheLevel_][
                        cache->cacheLevel_ ?
                        cache->cacheLevel_ - 1 : 0])[cpuId]++;
                processTime = pkt->getProcessTime(cache->cacheLevel_);
                (*prefProcessCycles_[pkt->srcCacheLevel_][
                        cache->cacheLevel_ ?
                        cache->cacheLevel_ - 1 : 0])[cpuId] +=
                        (processTime + 20) / clockPeriod_;
                DEBUG_PF(2, "Add %s process time %lu for prefetch @0x%lx",
                        BaseCache::levelName_[cache->cacheLevel_].c_str(),
                        processTime, pkt->getAddr());
            }
        }
        // 预取替换了一个Demand Request请求的数据
        // 就需要对相关的替换信息进行记录
        if (info.target == Dmd) {
            CHECK_RET_EXIT(usefulTable_[cache].addPref(pkt, evictedBlkAddr,
                    Dmd), "Failed to add pref replacing dmd in the table");
        } else if (info.target == Pref) {
            if (!usefulTable_[cache].isPrefValid(evictedBlkAddr)) {
                // 如果被替换的预取被Demand Request覆盖，那么它将会被看作是
                // 一个Demand Request，成为一个有害情况的备选项
                CHECK_RET_EXIT(usefulTable_[cache].addPref(pkt, evictedBlkAddr,
                        Dmd), "Failed to add pref replacing %s",
                        "used pref in the table");
            } else {
                // 如果一个预取替换了一个没有被Demand Request命中过的无用预取
                // 则原预取将会被剔除，并进行记录
                uint64_t oldReplacedAddr;
                CHECK_RET_EXIT(usefulTable_[cache].getReplacedAddr(
                        evictedBlkAddr, &oldReplacedAddr),
                        "Failed to get replaced addr for replaced prefetch");
                if (!typeHash_) {
                    CHECK_RET_EXIT(removePrefetch(cache, evictedBlkAddr,
                            false), "Failed to remove prefetch record "
                            "when hit by dmd");
                }
                // 这里我们可以选择使用被替换的Prefetch作为Victim
                // 也可以选择原本预取替换的Demand地址作为Victim
                // (只有原来的预取替换了Demand，才会选择)
                CHECK_RET_EXIT(usefulTable_[cache].addPref(pkt,
                        enableRecursiveReplace_ && oldReplacedAddr?
                        oldReplacedAddr : evictedBlkAddr, Pref),
                        "Failed to replace old pref with new pref");
            }
        } else {
            CHECK_RET_EXIT(usefulTable_[cache].addPref(pkt, evictedBlkAddr,
                    NullType), "Failed to add pref replacing %s",
                    "used pref in the table");
        }
    }
    
    skipCorrectPref_ = DoublePref(cache,
            std::pair<uint64_t, uint64_t>(
            info.source == Pref ? pkt->getAddr() & cacheLineAddrMask_ : 0,
            info.target == Pref ? evictedBlkAddr : 0));
    // 时间维度的检查与更新
    CHECK_RET_EXIT(checkUpdateTimingPost(),
            "Failed check & update timing stats information");
    return 0;
}

int BasePrefetchFilter::filterPrefetch(BaseCache* cache,
        const uint64_t& prefAddr, const PrefetchInfo& info) {
    // 时间维度的更新
    CHECK_RET_EXIT(checkUpdateTimingAhead(), "Failed update timing processes");
    
    // 时间维度的检查与更新
    CHECK_RET_EXIT(checkUpdateTimingPost(),
            "Failed check & update timing stats information");
    // 默认的过滤器没有实现，因此不会改变预取数据的存放位置
    return cache->cacheLevel_;
}

int BasePrefetchFilter::invalidatePrefetch(BaseCache* cache,
        const uint64_t& prefAddr){
    // 时间维度的更新
    CHECK_RET_EXIT(checkUpdateTimingAhead(), "Failed update timing processes");
    
    const uint64_t prefBlkAddr = prefAddr & cacheLineAddrMask_;
    // 删除被替换掉的预取
    CHECK_RET_EXIT(removePrefetch(cache, prefBlkAddr, false),
            "Failed to remove prefetch record when invalidated");
    
    skipCorrectPref_ = DoublePref(cache,
            std::pair<uint64_t, uint64_t>(prefAddr, 0));
    // 时间维度的检查与更新
    CHECK_RET_EXIT(checkUpdateTimingPost(),
            "Failed check & update timing stats information");
    return 0;
}

int BasePrefetchFilter::notifyCacheReqSentFailed(BaseCache* cache,
        const int totalEntries, const int waitingDemands,
        const uint8_t originalDegree, uint8_t* newDegree) {
    // 时间维度的更新
    CHECK_RET_EXIT(checkUpdateTimingAhead(), "Failed update timing processes");
   
    // 默认的过滤取不对预取器的预取深度进行调节
    *newDegree = originalDegree;
    
    // 时间维度的检查与更新
    CHECK_RET_EXIT(checkUpdateTimingPost(),
            "Failed check & update timing stats information");
    return 0;
}

int BasePrefetchFilter::genHash(const std::string& name) {   
    register int64_t hash = 0;
    int index = 0;
    while (index < name.length()) {
        hash = hash * 131 + name[index++];
    }
    hash = abs(hash);
    if (typeHash_ == 0) {
        typeHash_ = hash;
    } else {
        CHECK_RET(typeHash_ == hash,
                "Trying to initiate different types of prefetch filter");
    }
    return 0;
}

int BasePrefetchFilter::helpInvalidatePref(BaseCache* cache,
        const std::set<uint64_t>& addrs) {
    return 0;
}

int BasePrefetchFilter::helpCorrectPref(
        const std::map<BaseCache*, std::set<uint64_t>>& correctionList) {
    std::set<BaseCache*> correlatedCaches;
    for (auto& correction : correctionList) {
        BaseCache* cache = correction.first;
        for (auto addr : correction.second) {
            CHECK_RET(usefulTable_[cache].evictPref(addr, &correlatedCaches),
                    "Failed to remove prefetch from useful table when doing"
                    " correction");
        }
    }
    return 0;
}

int BasePrefetchFilter::removePrefetch(BaseCache* cache,
        const uint64_t& prefAddr, const bool isHit){
    const uint64_t prefBlkAddr = prefAddr & cacheLineAddrMask_;
    std::set<BaseCache*> correlatedCaches;
    // 删除被替换掉的预取
    CHECK_RET(usefulTable_[cache].evictPref(prefBlkAddr, &correlatedCaches),
            "Failed to remove pref from the table");
    
    // 对无效化传递进行设置
    if (isHit) {
        uint8_t cacheLevel = cache->cacheLevel_ ? cache->cacheLevel_ : 1;
        Tick completedTick = curTick() + cache->forwardLatency * clockPeriod_;
        while (++cacheLevel <= maxCacheLevel_) {
            for (auto otherCache : correlatedCaches) {
                if (otherCache->cacheLevel_ == cacheLevel) {
                    CHECK_RET(usefulTable_[otherCache].addPrefInvalidation(
                            cache, completedTick, prefAddr),
                            "Failed to forward invalidate prefetch");
                }
            }
            completedTick += caches_[cacheLevel].front()->forwardLatency *
                    clockPeriod_;
        }
    }
    return 0;
}

void BasePrefetchFilter::memCheck() {
    DEBUG_PF(0, "BPF memory check:");
    DEBUG_PF(1, "Size: %lu", demandReqHitCount_.size());
    DEBUG_PF(1, "Size: %lu", demandReqMissCount_.size());
    for (auto& vec : prefHitCount_) {
        DEBUG_PF(1, "Size: %lu", vec.size());
    }
    for (auto& vec : prefFillCount_) {
        DEBUG_PF(1, "Size: %lu", vec.size());
    }
    for (auto& vec : prefProcessCycles_) {
        DEBUG_PF(1, "Size: %lu", vec.size());
    }
    for (auto& vec : prefAvgProcessCycles_) {
        DEBUG_PF(1, "Size: %lu", vec.size());
    }
    DEBUG_PF(1, "Size: %lu", prefIssuedCount_.size());
    DEBUG_PF(1, "Size: %lu", prefWaitingCycles_.size());
    DEBUG_PF(1, "Size: %lu", prefAvgWaitingCycles_.size());
    DEBUG_PF(1, "Size: %lu", shadowedPrefCount_.size());
    DEBUG_PF(1, "Size: %lu", totalStatsPref_.size());
    DEBUG_PF(1, "Size: %lu", dismissedLevelUpPrefNoWB_.size());
    DEBUG_PF(1, "Size: %lu", dismissedLevelUpPrefLate_.size());
    DEBUG_PF(1, "Size: %lu", dismissedLevelDownPref_.size());
    DEBUG_PF(1, "Size: %lu", squeezedPrefCount_.size());
    for (auto& vec : prefTotalUsefulValue_) {
        DEBUG_PF(1, "Size: %lu", vec.size());
    }
    for (auto& vec : prefUsefulDegree_) {
        for (auto& vec2 : vec) {
            DEBUG_PF(1, "Size: %lu", vec2.size());
        }
    }
    for (auto& vec : prefUsefulType_) {
        DEBUG_PF(1, "Size: %lu", vec.size());
    }
    for (auto& item : timingStats_) {
        DEBUG_PF(1, "Size: %lu", item.demandHit_.size());
        DEBUG_PF(1, "Size: %lu", item.demandMiss_.size());
        for (auto& vec : item.prefDegreeCount_) {
            for (auto& vec2 : vec) {
                DEBUG_PF(1, "Size: %lu", vec2.size());
            }
        }
    }
    DEBUG_PF(1, "Size: %lu", timingDegree_.size());
    for (auto& vec : caches_) {
        DEBUG_PF(1, "Size: %lu", vec.size());
    }
    for (auto& iter : usefulTable_) {
        iter.second.memCheck();
    }
    DEBUG_PF(1, "Size: %lu", usePref_.size());
    DEBUG_PF(1, "Size: %lu", havePref_.size());
}

int BasePrefetchFilter::initThis() {   
    maxCacheLevel_ = caches_.size() - 1;
    havePref_.resize(caches_.size());
    usePref_.resize(caches_.size());
    numCpus_ = caches_[0].size();
    prefetch_filter::numCpus_ = numCpus_;

    for (int i = 0; i <= maxCacheLevel_; i++) {
        usePref_[i] = caches_[i][0]->prefetcher != NULL;
        if (i > 1) {
            havePref_[i] = usePref_[i] | havePref_[i - 1];
        } else {
            havePref_[i] = usePref_[i];
        }
    }
    
    for (auto& level : caches_) {
        uint8_t cacheLevel = level.front()->cacheLevel_;
        // 初始化统计变量
        if (havePref_[cacheLevel] && cacheLevel != maxCacheLevel_) {
            prefHitCount_.push_back(std::vector<Stats::Vector*>(
                    cacheLevel < 2 ? maxCacheLevel_ - 1 :
                    maxCacheLevel_ - cacheLevel));
            prefFillCount_.push_back(std::vector<Stats::Vector*>(
                    maxCacheLevel_ + 1));
            prefProcessCycles_.push_back(std::vector<Stats::Vector*>(
                    maxCacheLevel_ + 1));
            prefAvgProcessCycles_.push_back(std::vector<Stats::Formula*>(
                    maxCacheLevel_ + 1));
        } else {
            prefHitCount_.push_back(std::vector<Stats::Vector*>());
            prefFillCount_.push_back(std::vector<Stats::Vector*>());
            prefProcessCycles_.push_back(std::vector<Stats::Vector*>());
            prefAvgProcessCycles_.push_back(std::vector<Stats::Formula*>());
        }
        
        if (usePref_[cacheLevel]) {
            prefTotalUsefulValue_.push_back(std::vector<Stats::Vector*>(2));
            prefUsefulDegree_.push_back(
                    std::vector<std::vector<Stats::Vector*>>(2,
                    std::vector<Stats::Vector*>(5)));
            prefUsefulType_.push_back(std::vector<Stats::Vector*>(
                    PrefUsefulTypeList.size()));
        } else {
            prefTotalUsefulValue_.push_back(std::vector<Stats::Vector*>());
            prefUsefulDegree_.push_back(
                    std::vector<std::vector<Stats::Vector*>>());
            prefUsefulType_.push_back(std::vector<Stats::Vector*>());
        }

        // 初始化有用/有害信息表格（如果存在提升级别的预取，那么需要修正）
        for (auto cache : level) {
            usefulTable_.insert(std::pair<BaseCache*, IdealPrefetchUsefulTable>
                    (cache, IdealPrefetchUsefulTable(cache,
                    havePref_[cacheLevel])));
        }
    }
    
    // 初始化统计相关的变量
    uint8_t cacheCount = caches_.size();
    demandReqHitCount_.resize(cacheCount);
    demandReqMissCount_.resize(cacheCount);
    prefIssuedCount_.resize(cacheCount);
    prefWaitingCycles_.resize(cacheCount);
    prefAvgWaitingCycles_.resize(cacheCount);
    shadowedPrefCount_.resize(cacheCount);
    squeezedPrefCount_.resize(cacheCount);
    totalStatsPref_.resize(cacheCount);
    dismissedLevelUpPrefNoWB_.resize(cacheCount);
    dismissedLevelUpPrefLate_.resize(cacheCount);
    dismissedLevelDownPref_.resize(cacheCount);

    // 初始化时间维度的统计变量
    TimingStats sample;
    sample.demandHit_.resize(cacheCount);
    sample.demandMiss_.resize(cacheCount);
    sample.prefDegreeCount_.resize(cacheCount,
            std::vector<std::vector<uint32_t>>(2, std::vector<uint32_t>(5)));
    timingStats_.resize(numCpus_, sample);
    for (int i = 0; i < numCpus_; i++) {
        timingDegree_.push_back(&(timingStats_[i].prefDegreeCount_));
    }
    return 0;
}

int BasePrefetchFilter::checkUpdateTimingAhead() {
    Tick tickNow = curTick();
    // 更新全局Tick数，用于Debug
    prefetch_filter::tickNow_ = tickNow;
    
    // 尝试更新无效化和预取合并下传
    for (auto& tablePair : usefulTable_) {
        std::set<uint64_t> invalidatedPref;
        CHECK_RET(tablePair.second.updateInvalidation(tickNow,
                &invalidatedPref),
                "Failed to update invalidation for a cache");
        if (!invalidatedPref.empty()) {
            CHECK_RET(helpInvalidatePref(tablePair.first, invalidatedPref),
                    "Failed to forward invalidation info to child class");
        }
        CHECK_RET(tablePair.second.updateCombination(tickNow),
                "Failed to update combination for a cache");
    }
    return 0;
}

int BasePrefetchFilter::checkUpdateTimingPost() {
    // 打印Tick位置信息
    if (prefetch_filter::tickNow_ > nextTimerPrintTick_) {
        DPRINTF(PrefetchFilterTimer, "BasePrefetchFilter.Timer\n");
        nextTimerPrintTick_ += prefetch_filter::timerPrintGap_;
    }
    
    // 如果到达了校正位置，进行全局数据校正
    if (prefetch_filter::tickNow_ > nextCorrectionTick_) {
        std::map<BaseCache*, std::set<uint64_t>> correctionList;
        CHECK_RET(IdealPrefetchUsefulTable::findCorrection(&correctionList),
                "Failed to correct prefetch record timingly");
        nextCorrectionTick_ += correctionPeriod_;
        if (correctionList.size()) {
            CHECK_RET(helpCorrectPref(correctionList),
                    "Failed to forward correction to child class");
        }
        // 调用内存检查
        // memCheck();
    }
    
    if (enableStats_ && curTick() > nextPeriodTick_) {
        timingStatsPeriodCount_++;

        // 更新时间维度的统计数据
        CHECK_RET_EXIT(IdealPrefetchUsefulTable::updatePrefTiming(
                prefTotalUsefulValue_, prefUsefulDegree_, prefUsefulType_,
                timingDegree_), "Failed to update stats timingly");
        
        // 打印时间维度的信息
        DPRINTF(PrefetchFilter, "Stats Period Id: %d\n",
                timingStatsPeriodCount_);
        int cacheCount = caches_.size();
        for (int i = 0; i < numCpus_; i++) {
            DPRINTF(PrefetchFilter, "    CPU Id: %d\n", i);
            std::stringstream outputInfo;
            outputInfo << "    Demand Hit Count: ";
            for (int j = 0; j < cacheCount; j++) {
                outputInfo << timingStats_[i].demandHit_[j] << ", ";
            }
            outputInfo << std::endl;
            outputInfo << "    Demand Miss Count: ";
            for (int j = 0; j < cacheCount; j++) {
                outputInfo << timingStats_[i].demandMiss_[j] << ", ";
            }
            outputInfo << "    Pref Type Count: \n";
            for (int j = 0; j < cacheCount; j++) {
                for (int k = 0; k < 2; k++) {
                    outputInfo << "        " << BaseCache::levelName_[j] << "-";
                    std::string usefulTypeStr = k ? "cross_core" :
                            "single_core";
                    outputInfo << usefulTypeStr << ": ";
                    for (int n = 0; n < TOTAL_DEGREE; n++) {
                        outputInfo <<
                                timingStats_[i].prefDegreeCount_[j][k][n] <<
                                ", ";
                    }
                    outputInfo << std::endl;
                }
            }
            DPRINTF(PrefetchFilter, "%s", outputInfo.str());
        }
        // 方案1：该方案在全局固定的分割间隔内进行采样，采样范围是对齐的
        /*
        while (nextPeriodTick_ - curTick() < statsPeriod_) {
            nextPeriodTick_ += statsPeriod_;
        }
        */
        // 方案2：动态的采样，每次都从当前的位置开启，采样范围不对齐
        nextPeriodTick_ = curTick() + statsPeriod_;
        
        // 重置时间维度的统计数据
        for (auto& stats : timingStats_) {
            stats.reset();
        }
    }
    // 重置空统计变量防止溢出
    for (int i = 0; i < numCpus_; i++) {
        (*emptyStatsVar_)[i] = 0;
    }
    return 0;
}

void BasePrefetchFilter::init() {
    if (initFlag_) {
        return;
    }
    ClockedObject::init();
    // 进行初始化操作
    CHECK_RET_EXIT(initThis(), "Failed to initiate base prefetch filter");
    initFlag_ = true;
}

void BasePrefetchFilter::regStats() {
    // 如果已经初始化则不再初始化
    if (regFlag_) {
        return;
    }

    ClockedObject::regStats();

    int i, j, k;
    int cacheCount = caches_.size();
    
    emptyStatsVar_ = new Stats::Vector();
    emptyStatsVar_->init(numCpus_)
            .name(name() + ".empty_stats")
            .desc("Empty stats variable for deleted stats");
    emptyStatsVar_->info()->flags.clear(display);

    for (j = 0; j < numCpus_; j++) {
        emptyStatsVar_->subname(j, std::string("cpu") +
                std::to_string(j));
    }

    if (enableStats_) {
        demandReqHitTotal_ = new Stats::Vector();
        demandReqHitTotal_->init(numCpus_)
                .name(name() + ".demand_total_hit")
                .desc("Number of demand requests")
                .flags(total);
        for (j = 0; j < numCpus_; j++) {
            demandReqHitTotal_->subname(j, std::string("cpu") +
                    std::to_string(j));
        }
        
        for (i = 0; i < cacheCount; i++) {
            std::string srcCacheName = BaseCache::levelName_[i];
            demandReqHitCount_[i] = new Stats::Vector();
            demandReqHitCount_[i]->init(numCpus_)
                    .name(name() + ".demand_requests_hit_" + srcCacheName)
                    .desc(std::string("Number of demand requests hit in ") +
                            srcCacheName)
                    .flags(total);
            
            demandReqMissCount_[i] = new Stats::Vector();
            demandReqMissCount_[i]->init(numCpus_)
                    .name(name() + ".demand_requests_miss_" + srcCacheName)
                    .desc(std::string("Number of demand requests miss in ") +
                            srcCacheName)
                    .flags(total);
        
            if (usePref_[i]) {
                prefIssuedCount_[i] = new Stats::Vector();
                prefIssuedCount_[i]->init(numCpus_)
                        .name(name() + ".prefetch_issued_from_" + srcCacheName)
                        .desc(std::string("Number of prefetches issued from ")
                                + srcCacheName);
                prefIssuedCount_[i]->info()->flags.clear(display);
                
                prefWaitingCycles_[i] = new Stats::Vector();
                prefWaitingCycles_[i]->init(numCpus_)
                        .name(name() + ".prefetch_total_waiting_cycles_from_" +
                                srcCacheName)
                        .desc(std::string("Total waiting cycles of prefetches "
                                "issued from ") + srcCacheName);
                prefWaitingCycles_[i]->info()->flags.clear(display);
                
                prefAvgWaitingCycles_[i] = new Stats::Formula();
                prefAvgWaitingCycles_[i]
                        ->name(name() + ".prefetch_avg_waiting_cycle_from_" +
                                srcCacheName)
                        .desc(std::string("Average waiting cycles for "
                                "prefetches from ") + srcCacheName);
                
                shadowedPrefCount_[i] = new Stats::Vector();
                shadowedPrefCount_[i]->init(numCpus_)
                        .name(name() + ".prefetch_shadowed_" + srcCacheName)
                        .desc(std::string("Number of prefetches shadowed by "
                                "demand requests from ") + srcCacheName)
                        .flags(total);
                
                squeezedPrefCount_[i] = new Stats::Vector();
                squeezedPrefCount_[i]->init(numCpus_)
                        .name(name() + ".prefetch_squeezed_" + srcCacheName)
                        .desc(std::string("Number of prefetches squeezed by "
                                "prefetch requests from ") + srcCacheName)
                        .flags(total);
                
                totalStatsPref_[i] = new Stats::Vector();
                totalStatsPref_[i]->init(numCpus_)
                        .name(name() + ".prefetch_stats_count_from_" +
                                srcCacheName)
                        .desc(std::string("Number of prefetches "
                                "have been counted from ") + srcCacheName)
                        .flags(total);
                
                dismissedLevelUpPrefNoWB_[i] = new Stats::Vector();
                dismissedLevelUpPrefNoWB_[i]->init(numCpus_)
                        .name(name() + ".prefetch_dismissed_level_up_nowb_" +
                                srcCacheName)
                        .desc(std::string("Number of level-up prefetches "
                                "dismissed from ") + srcCacheName +
                                " due to lack of write buffer")
                        .flags(total);
                
                dismissedLevelUpPrefLate_[i] = new Stats::Vector();
                dismissedLevelUpPrefLate_[i]->init(numCpus_)
                        .name(name() + ".prefetch_dismissed_level_up_late_" +
                                srcCacheName)
                        .desc(std::string("Number of level-up prefetches "
                                "dismissed from ") + srcCacheName +
                                " due to late")
                        .flags(total);
                
                dismissedLevelDownPref_[i] = new Stats::Vector();
                dismissedLevelDownPref_[i]->init(numCpus_)
                        .name(name() + ".prefetch_dismissed_level_down_" +
                                srcCacheName)
                        .desc(std::string("Number of level-down prefetches "
                                "dismissed from ") + srcCacheName +
                                " to  prevent level-down threshing")
                        .flags(total);
            } else {
                prefIssuedCount_[i] = emptyStatsVar_;
                prefWaitingCycles_[i] = emptyStatsVar_;
                shadowedPrefCount_[i] = emptyStatsVar_;
                squeezedPrefCount_[i] = emptyStatsVar_;
                totalStatsPref_[i] = emptyStatsVar_;
                dismissedLevelUpPrefNoWB_[i] = emptyStatsVar_;
                dismissedLevelUpPrefLate_[i] = emptyStatsVar_;
                dismissedLevelDownPref_[i] = emptyStatsVar_;
            }

            for (j = 0; j < prefHitCount_[i].size(); j++) {
                std::string tgtCacheName = BaseCache::levelName_[i ?
                        j + i + 1 : j + 2];
                prefHitCount_[i][j] = new Stats::Vector();
                prefHitCount_[i][j]->init(numCpus_)
                        .name(name() + ".prefetch_from_" + srcCacheName +
                                "_hit_" + tgtCacheName)
                        .desc(std::string("Number of prefetches request from ")
                                + srcCacheName + " hit in " + tgtCacheName)
                        .flags(total);
            }
            
            for (j = 0; j < prefFillCount_[i].size(); j++) {
                std::string tgtCacheName = j ?
                        BaseCache::levelName_[j] : "cpu";
                prefFillCount_[i][j] = new Stats::Vector();
                prefFillCount_[i][j]->init(numCpus_)
                        .name(name() + ".prefetch_from_" + srcCacheName +
                                "_fill_in_" + tgtCacheName)
                        .desc(std::string("Number of prefetches request from ")
                                + srcCacheName + " filled in " + tgtCacheName)
                        .flags(total);
                if (j == 0) {
                    prefFillCount_[i][j]->info()->flags.clear(total);
                    prefFillCount_[i][j]->info()->flags.clear(display);
                }
            }
            
            for (j = 0; j < prefProcessCycles_[i].size(); j++) {
                std::string tgtCacheName = BaseCache::levelName_[j + 1];
                prefProcessCycles_[i][j] = new Stats::Vector();
                prefProcessCycles_[i][j]->init(numCpus_)
                        .name(name() + ".prefetch_total_process_cycle_from_" +
                                srcCacheName + "_in_" + tgtCacheName)
                        .desc(std::string("Cycles of process for prefetches "
                                "request from ") + srcCacheName +
                                " in " + tgtCacheName);
                prefProcessCycles_[i][j]->info()->flags.clear(display);
            }
            
            for (j = 0; j < prefAvgProcessCycles_[i].size(); j++) {
                std::string tgtCacheName = BaseCache::levelName_[j + 1];
                prefAvgProcessCycles_[i][j] = new Stats::Formula();
                prefAvgProcessCycles_[i][j]
                        ->name(name() + ".prefetch_avg_process_cycle_from_" +
                                srcCacheName + "_in_" + tgtCacheName)
                        .desc(std::string("Average cycles of process for "
                                "prefetches from ") + srcCacheName +
                                " in " + tgtCacheName);
            }
            
            std::string usefulTypeStr;
            for (j = 0; j < prefTotalUsefulValue_[i].size(); j++) {
                usefulTypeStr = j ? "cross_core" : "single_core";
                prefTotalUsefulValue_[i][j] = new Stats::Vector();
                prefTotalUsefulValue_[i][j]->init(numCpus_)
                        .name(name() + ".prefetch_total_useful_value_from_" +
                                srcCacheName +"_type_" + usefulTypeStr)
                        .desc(std::string("Total") + usefulTypeStr +
                                "useful value for of prefetches from" +
                                srcCacheName)
                        .flags(total);
            }
            
            for (j = 0; j < prefUsefulDegree_[i].size(); j++) {
                usefulTypeStr = j ? "cross_core" : "single_core";
                for (k = 0; k < TOTAL_DEGREE; k++) {
                    int realDegree = k - (TOTAL_DEGREE >> 1);
                    std::string degreeStr = realDegree < 0 ?
                            std::string("_") + std::to_string(-realDegree) :
                            std::to_string(realDegree);
                    prefUsefulDegree_[i][j][k] = new Stats::Vector();
                    prefUsefulDegree_[i][j][k]->init(numCpus_)
                            .name(name() + ".prefetch_useful_degree_from_" +
                                    srcCacheName + "_type_" + usefulTypeStr +
                                    "_degree_" + degreeStr)
                            .desc(std::string("Number of prefetches with ") +
                                    usefulTypeStr + " useful degree " +
                                    degreeStr + " from " + srcCacheName)
                            .flags(total);
                }
            }
            
            for (j = 0; j < prefUsefulType_[i].size(); j++) {
                const PrefUsefulType& type = PrefUsefulTypeList[j];
                prefUsefulType_[i][j] = new Stats::Vector();
                prefUsefulType_[i][j]->init(numCpus_)
                        .name(name() + ".prefetch_useful_type_from_" +
                                srcCacheName + "_" + type.name_)
                        .desc(std::string("Number of ") + type.name_ +
                                " prefetches from" + srcCacheName)
                        .flags(total);
            }

            // 对不同CPU进行初始化
            for (k = 0; k < numCpus_; k++) {
                std::string cpuIdStr = std::string("cpu") + std::to_string(k);
                demandReqHitCount_[i]->subname(k, cpuIdStr);
                demandReqMissCount_[i]->subname(k, cpuIdStr);
                if (usePref_[i]) {
                    prefIssuedCount_[i]->subname(k, cpuIdStr);
                    prefWaitingCycles_[i]->subname(k, cpuIdStr);
                    prefAvgWaitingCycles_[i]->subname(k, cpuIdStr);
                    if (k == numCpus_ - 1) {
                        *prefAvgWaitingCycles_[i] = *prefWaitingCycles_[i] /
                                *prefIssuedCount_[i];
                    }
                    shadowedPrefCount_[i]->subname(k, cpuIdStr);
                    squeezedPrefCount_[i]->subname(k, cpuIdStr);
                    totalStatsPref_[i]->subname(k, cpuIdStr);
                    dismissedLevelUpPrefNoWB_[i]->subname(k, cpuIdStr);
                    dismissedLevelUpPrefLate_[i]->subname(k, cpuIdStr);
                    dismissedLevelDownPref_[i]->subname(k, cpuIdStr);
                }
                for (j = 0; j < prefTotalUsefulValue_[i].size(); j++) {
                    prefTotalUsefulValue_[i][j]->subname(k, cpuIdStr);
                }
                for (j = 0; j < prefHitCount_[i].size(); j++) {
                    prefHitCount_[i][j]->subname(k, cpuIdStr);
                }
                for (j = 0; j < prefFillCount_[i].size(); j++) {
                    prefFillCount_[i][j]->subname(k, cpuIdStr);
                }
                for (j = 0; j < prefProcessCycles_[i].size(); j++) {
                    prefProcessCycles_[i][j]->subname(k, cpuIdStr);
                }
                for (j = 0; j < prefAvgProcessCycles_[i].size(); j++) {
                    prefAvgProcessCycles_[i][j]->subname(k, cpuIdStr);
                    if (k == numCpus_ - 1) {
                        *prefAvgProcessCycles_[i][j] =
                                *prefProcessCycles_[i][j] /
                                *prefFillCount_[i][j];
                    }
                }
                for (j = 0; j < prefUsefulDegree_[i].size(); j++) {
                    for (int n = 0; n < TOTAL_DEGREE; n++) {
                        prefUsefulDegree_[i][j][n]->subname(k, cpuIdStr);
                    }
                }
                for (j = 0; j < prefUsefulType_[i].size(); j++) {
                    prefUsefulType_[i][j]->subname(k, cpuIdStr);
                }
            }
        }
    } else {
        demandReqHitTotal_ = emptyStatsVar_;
        for (i = 0; i < cacheCount; i++) {
            demandReqHitCount_[i] = emptyStatsVar_;
            demandReqMissCount_[i] = emptyStatsVar_;
            prefIssuedCount_[i] = emptyStatsVar_;
            prefWaitingCycles_[i] = emptyStatsVar_;
            shadowedPrefCount_[i] = emptyStatsVar_;
            squeezedPrefCount_[i] = emptyStatsVar_;
            totalStatsPref_[i] = emptyStatsVar_;
            dismissedLevelUpPrefNoWB_[i] = emptyStatsVar_;
            dismissedLevelUpPrefLate_[i] = emptyStatsVar_;
            dismissedLevelDownPref_[i] = emptyStatsVar_;
            for (j = 0; j < prefHitCount_[j].size(); j++) {
                prefHitCount_[i][j] = emptyStatsVar_;
            }
            for (j = 0; j < prefFillCount_[j].size(); j++) {
                prefFillCount_[i][j] = emptyStatsVar_;
            }
            for (j = 0; j < prefProcessCycles_[j].size(); j++) {
                prefProcessCycles_[i][j] = emptyStatsVar_;
            }
            for (j = 0; j < prefTotalUsefulValue_[i].size(); j++) {
                prefTotalUsefulValue_[i][j] = emptyStatsVar_;
            }
            for (j = 0; j < prefUsefulDegree_[i].size(); j++) {
                for (k = 0; k < TOTAL_DEGREE; k++) {
                    prefUsefulDegree_[i][j][k] = emptyStatsVar_;
                }
            }
            for (j = 0; j < prefUsefulType_[i].size(); j++) {
                prefUsefulType_[i][j] = emptyStatsVar_;
            }
        }
    }

    // 标记已经进行了注册
    regFlag_ = true;
}

void BasePrefetchFilter::TimingStats::reset() {
    memset(demandHit_.data(), 0, sizeof(uint32_t) * demandHit_.size());
    memset(demandMiss_.data(), 0, sizeof(uint32_t) * demandMiss_.size());
    for (auto& vec1 : prefDegreeCount_) {
        for (auto& vec2 : vec1) {
            memset(vec2.data(), 0, sizeof(uint32_t) * vec2.size());
        }
    }
}

BasePrefetchFilter* BasePrefetchFilterParams::create() {
    return BasePrefetchFilter::create(this);
}
