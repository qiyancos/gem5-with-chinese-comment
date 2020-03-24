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
int64_t BasePrefetchFilter::typeHash_ = -1;
bool BasePrefetchFilter::initFlag_ = false;
bool BasePrefetchFilter::regFlag_ = false;
uint64_t BasePrefetchFilter::cacheLineAddrMask_ = 0;
uint8_t BasePrefetchFilter::cacheLineOffsetBits_ = 0;
std::vector<std::vector<BaseCache*>> BasePrefetchFilter::caches_;

using namespace prefetch_filter;
using namespace Stats;

BasePrefetchFilter::BasePrefetchFilter(const BasePrefetchFilterParams *p) :
        ClockedObject(p),
        statsPeriod_(p->stats_period),
        nextPeriodTick_(p->stats_period),
        enableStats_(p->enable_stats),
        enableFilter_(p->enable_filter) {
    cacheLineAddrMask_ = ~(p->block_size - 1);
    int blockSize = p->block_size;
    cacheLineOffsetBits_ = 0;
    while (blockSize >> ++cacheLineOffsetBits_) {}
    cacheLineOffsetBits_--;
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
            CHECK_ARGS_EXIT(connectedCache == nullptr,
                    "Not supportted for shared cache");
            connectedCache = upLevelCache;
            portIds->clear();
            portIds->push_back(connectedCache->memSidePort._baseSlavePort->id);
            break;
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
    CHECK_RET_EXIT(checkUpdateTiming(), "Failed update timing processes");
    
    const uint64_t hitBlkAddr = hitAddr & cacheLineAddrMask_; 
    if (info.source == Dmd) {
        for (BaseCache* cache : pkt->caches_) {
            for (const uint8_t cpuId : cache->cpuIds_) {
                timingStats_[cpuId].demandHit_[cache->cacheLevel_]++;
                (*demandReqHitCount_[cache->cacheLevel_])[cpuId]++;
                (*demandReqHitTotal_)[cpuId]++;
            }
        }
    }
    
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
    } else {
        // 更新预取请求的命中情况
        for (BaseCache* srcCache : pkt->caches_) {
            int level = srcCache->cacheLevel_;
            uint8_t targetCacheOffset = level ?
                    cache->cacheLevel_ - level - 1 : cache->cacheLevel_ - 2;
            for (const uint8_t cpuId : srcCache->cpuIds_) {
                (*prefHitCount_[level][targetCacheOffset])[cpuId]++;
            }
        }
       
        // 预取命中了预取数据
        if (info.target == Pref &&
                usefulTable_[cache].isPrefValid(hitBlkAddr)) {
            // 只有目标预取确实是一个预取，才会更新
            CHECK_RET_EXIT(usefulTable_[cache].updateHit(pkt, hitBlkAddr,
                    Pref), "Failed to update when useful pref hit "
                    "pref data");
            CHECK_RET_EXIT(usefulTable_[cache].combinePref(pkt, hitBlkAddr),
                    "Failed to combine two prefetch when pref hit pref data");
        }
    }

    // 时间维度的检查与更新
    CHECK_RET_EXIT(checkUpdateTimingStats(),
            "Failed check & update timing stats information");
    return 0;
}

int BasePrefetchFilter::notifyCacheMiss(BaseCache* cache,
        PacketPtr& pkt, const PacketPtr& combinedPkt,
        const DataTypeInfo& info) {
    // 时间维度的更新
    CHECK_RET_EXIT(checkUpdateTiming(), "Failed update timing processes");
    
    if (info.source == Dmd) {
        for (BaseCache* cache : pkt->caches_) {
            for (const uint8_t cpuId : cache->cpuIds_) {
                timingStats_[cpuId].demandMiss_[cache->cacheLevel_]++;
                (*demandReqMissCount_[cache->cacheLevel_])[cpuId]++;
            }
        }
    }
    
    // const uint64_t combinedBlkAddr &= cacheLineAddrMask_;
    // 这里的info记录的source是新的Miss属性
    // 其中的target表示mshr中已存在的内容属性
    // prefetch->nulltype表示prefetch获得了一个新的mshr表项
    // 而非和一个demand类型的mshr合并，但是我们并不关心
    CHECK_ARGS_EXIT(info.source != Pref || info.target != Dmd,
            "Prefetch will not combined with demand in mshr.");
    if (info.source == Dmd) {
        if (info.target == Pref) {
            for (BaseCache* cache : pkt->caches_) {
                for (const uint8_t cpuId : cache->cpuIds_) {
                    (*shadowedPrefCount_[cache->cacheLevel_])[cpuId]++;
                }
            }
            // 一个新生成的没有合并的预取，需要添加对应信息进行追踪
            CHECK_RET_EXIT(usefulTable_[cache].deletePref(combinedPkt),
                    "Failed to add new prefetch info to useful table");
        } else {
            // 一个新的Dmd请求发生了Miss
            CHECK_RET_EXIT(usefulTable_[cache].updateMiss(pkt),
                    "Failed to update when demand request miss");
        }
    } else if (info.target == NullType && pkt->caches_.size() == 1 &&
            *(pkt->caches_.begin()) == cache) {
        // 一个新生成的没有合并的预取，需要添加对应信息进行追踪
        CHECK_RET_EXIT(usefulTable_[cache].newPref(pkt),
                "Failed to add new prefetch info to useful table");
    }
    
    // 时间维度的检查与更新
    CHECK_RET_EXIT(checkUpdateTimingStats(),
            "Failed check & update timing stats information");
    return 0;
}

int BasePrefetchFilter::notifyCacheFill(BaseCache* cache,
        const PacketPtr &pkt, const uint64_t& evictedAddr,
        const DataTypeInfo& info) {
    // 时间维度的更新
    CHECK_RET_EXIT(checkUpdateTiming(), "Failed update timing processes");
    
    const uint64_t evictedBlkAddr = evictedAddr & cacheLineAddrMask_;
    if (info.source == Dmd && info.target == Pref) {
        // 如果一个Demand Request替换了一个预取请求的数据
        // 则会将对应预取从当前Cache记录表中剔除
        if (!typeHash_) {
            CHECK_RET_EXIT(removePrefetch(cache, evictedBlkAddr, false),
                    "Failed to remove prefetch record when hit by dmd");
        }
    } else if (info.source == Pref) {
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
                        Pref), "Failed to add pref replacing %s",
                        "used pref in the table");
            } else {
                // 如果一个预取替换了一个没有被Demand Request命中过的无用预取
                // 则原预取将会被剔除，并进行记录，同时新的预取替换对象会继承
                // 之前替换对象的地址
                CHECK_RET_EXIT(usefulTable_[cache].replaceEvict(pkt,
                        evictedBlkAddr),
                        "Failed to replace old pref with new pref");
            }
        } else {
            CHECK_RET_EXIT(usefulTable_[cache].addPref(pkt, evictedBlkAddr,
                    NullType), "Failed to add pref replacing %s",
                    "used pref in the table");
        }
    }
    
    // 时间维度的检查与更新
    CHECK_RET_EXIT(checkUpdateTimingStats(),
            "Failed check & update timing stats information");
    return 0;
}

int BasePrefetchFilter::filterPrefetch(BaseCache* cache,
        const uint64_t& prefAddr, const PrefetchInfo& info) {
    // 时间维度的更新
    CHECK_RET_EXIT(checkUpdateTiming(), "Failed update timing processes");
    
    // 默认的过滤器没有实现，因此不会改变预取数据的存放位置
    return cache->cacheLevel_;
}

int BasePrefetchFilter::invalidatePrefetch(BaseCache* cache,
        const uint64_t& prefAddr){
    // 时间维度的更新
    CHECK_RET_EXIT(checkUpdateTiming(), "Failed update timing processes");
    
    const uint64_t prefBlkAddr = prefAddr & cacheLineAddrMask_;
    // 删除被替换掉的预取
    CHECK_RET_EXIT(removePrefetch(cache, prefBlkAddr, false),
            "Failed to remove prefetch record when invalidated");
    return 0;
}

int BasePrefetchFilter::genHash(const std::string& name) {   
    register int64_t hash = 0;
    int index = 0;
    while (index < name.length()) {
        hash = hash * 131 + name[index++];
    }
    hash = abs(hash);
    if (typeHash_ == -1) {
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
        Tick completedTick = curTick() + cache->forwardLatency * clockPeriod();
        while (++cacheLevel <= maxCacheLevel_) {
            for (auto otherCache : correlatedCaches) {
                if (otherCache->cacheLevel_ == cacheLevel) {
                    CHECK_RET(usefulTable_[otherCache].addPrefInvalidation(
                            cache, completedTick, prefAddr),
                            "Failed to forward invalidate prefetch");
                }
            }
            completedTick += caches_[cacheLevel].front()->forwardLatency *
                    clockPeriod();
        }
    }
    return 0;
}

int BasePrefetchFilter::initThis() {   
    maxCacheLevel_ = caches_.size() - 1;
    havePref_.resize(caches_.size());
    usePref_.resize(caches_.size());
    numCpus_ = caches_[0].size();
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
        } else {
            prefHitCount_.push_back(std::vector<Stats::Vector*>());
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
    shadowedPrefCount_.resize(cacheCount);
    
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

int BasePrefetchFilter::checkUpdateTiming() {
    for (auto& tablePair : usefulTable_) {
        std::set<uint64_t> invalidatedPref;
        CHECK_RET(tablePair.second.updateInvalidation(curTick(),
                &invalidatedPref),
                "Failed to update invalidation for a cache");
        if (!invalidatedPref.empty()) {
            CHECK_RET(helpInvalidatePref(tablePair.first, invalidatedPref),
                    "Failed to forward invalidation info to child class");
        }
    }
    return 0;
}

int BasePrefetchFilter::checkUpdateTimingStats() {
    if (enableStats_ && curTick() > nextPeriodTick_) {
        timingStatsPeriodCount_++;

        // 更新时间维度的统计数据
        for (auto& mapPair : usefulTable_) {
            CHECK_RET_EXIT(mapPair.second.updatePrefTiming(
                    prefTotalUsefulValue_, prefUsefulDegree_, prefUsefulType_,
                    timingDegree_), "Failed to update stats timingly");
        }
        
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
            .desc("Empty stats variable for deleted stats")
            .flags(total);
    for (j = 0; j < numCpus_; j++) {
        emptyStatsVar_->subname(j, std::string("cpu") +
                std::to_string(j));
    }

    if (enableStats_) {
        demandReqHitTotal_ = new Stats::Vector();
        demandReqHitTotal_->init(numCpus_)
                .name(name() + ".total_demand_requests_hit")
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
                shadowedPrefCount_[i] = new Stats::Vector();
                shadowedPrefCount_[i]->init(numCpus_)
                        .name(name() + "." + srcCacheName +
                                "_shadowed_prefetches")
                        .desc(std::string("Number of prefetches shadowed by "
                                "demand requests from ") + srcCacheName)
                        .flags(total);
            } else {
                shadowedPrefCount_[i] = emptyStatsVar_;
            }

            for (j = 0; j < prefHitCount_[i].size(); j++) {
                std::string tgtCacheName = BaseCache::levelName_[i ?
                        j + i + 1 : j + 2];
                prefHitCount_[i][j] = new Stats::Vector();
                prefHitCount_[i][j]->init(numCpus_)
                        .name(name() + "." + srcCacheName +
                                "_prefetch_requests_hit_" + tgtCacheName)
                        .desc(std::string("Number of prefetches request from ")
                                + srcCacheName + " hit in " + tgtCacheName)
                        .flags(total);
            }
            
            std::string usefulTypeStr;
            for (j = 0; j < prefTotalUsefulValue_[i].size(); j++) {
                usefulTypeStr = j ? "cross_core" : "single_core";
                prefTotalUsefulValue_[i][j] = new Stats::Vector();
                prefTotalUsefulValue_[i][j]->init(numCpus_)
                        .name(name() + "." + srcCacheName +
                                "_prefetch_" + usefulTypeStr +
                                "_useful_value")
                        .desc(std::string("Total") + usefulTypeStr +
                                "useful value for of prefetches from" +
                                srcCacheName)
                        .flags(total);
            }
            
            for (j = 0; j < prefUsefulDegree_[i].size(); j++) {
                usefulTypeStr = j ? "cross_core" : "single_core";
                for (k = 0; k < TOTAL_DEGREE; k++) {
                    std::string degreeStr = std::to_string(k);
                    prefUsefulDegree_[i][j][k] = new Stats::Vector();
                    prefUsefulDegree_[i][j][k]->init(numCpus_)
                            .name(name() + "." + srcCacheName +
                                    "_prefetch_" + usefulTypeStr +
                                    "_useful_degree_" + degreeStr)
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
                        .name(name() + "." + srcCacheName +
                                + "_" + type.name_ + "_prefetches")
                        .desc(std::string("Number of ") + type.name_ +
                                " prefetches from" + srcCacheName)
                        .flags(total);
            }

            // 对不同CPU进行初始化
            for (k = 0; k < numCpus_; k++) {
                std::string cpuId = std::to_string(k);
                demandReqHitCount_[i]->subname(k, std::string("cpu") + cpuId);
                demandReqMissCount_[i]->subname(k, std::string("cpu") + cpuId);
                if (usePref_[i]) {
                    shadowedPrefCount_[i]->subname(k, std::string("cpu") +
                            cpuId);
                }
                for (j = 0; j < prefTotalUsefulValue_[i].size(); j++) {
                    prefTotalUsefulValue_[i][j]->subname(k,
                            std::string("cpu") + cpuId);
                }
                for (j = 0; j < prefHitCount_[i].size(); j++) {
                    prefHitCount_[i][j]->subname(k, std::string("cpu") +
                            std::to_string(k));
                }
                for (j = 0; j < prefUsefulDegree_[i].size(); j++) {
                    for (int n = 0; n < TOTAL_DEGREE; n++) {
                        prefUsefulDegree_[i][j][n]->subname(k,
                                std::string("cpu") + cpuId);
                    }
                }
                for (j = 0; j < prefUsefulType_[i].size(); j++) {
                    prefUsefulType_[i][j]->subname(k, std::string("cpu") +
                            cpuId);
                }
            }
        }
    } else {
        demandReqHitTotal_ = emptyStatsVar_;
        for (i = 0; i < cacheCount; i++) {
            demandReqHitCount_[i] = emptyStatsVar_;
            demandReqMissCount_[i] = emptyStatsVar_;
            shadowedPrefCount_[i] = emptyStatsVar_;
            for (j = 0; j < prefHitCount_[j].size(); j++) {
                prefHitCount_[i][j] = emptyStatsVar_;
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
