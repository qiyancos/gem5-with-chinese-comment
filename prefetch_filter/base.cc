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

#include "mem/cache/prefetch_filter/base.hh"

#include "base/compiler.hh"
#include "base/logging.hh"
#include "debug/Cache.hh"
#include "debug/CachePort.hh"
#include "debug/CacheRepl.hh"
#include "debug/CacheVerbose.hh"
#include "mem/cache/mshr.hh"
#include "mem/cache/prefetch/base.hh"
#include "mem/cache/queue_entry.hh"
#include "params/BaseCache.hh"
#include "params/WriteAllocator.hh"
#include "sim/core.hh"

class BaseCache;
struct BasePrefetchFilterParams;

namespace prefetch_filter {

BasePrefetchFilter::BasePrefetchFilter(const BasePrefetchFilterParams *p) :
        statsPeriod_(p->stats_period),
        nextPeroidTick_(p->stats_period),
        enableStats_(p->enable_stats),
        enableFilter_(p->enable_filter) {}

int BasePrefetchFilter::addCache(BaseCache* newCache) {
    int level = newCache->cacheLevel;
    if (level > caches_.size()) {
        caches_.resize(level);
    }
    caches_[level].push_back(newCache);
    return 0;
}

int BasePrefetchFilter::notifyCacheHit(BaseCache* cache,
        const PacketPtr& pkt, const DataTypeInfo& info) {
    // 暂时不处理ICache的相关内容
    if (!cache->cacheLevel) {
        if (info.source = Dmd) {
            (*demandReqHitTotal_)[srcCpu]++;
            (*demandHitCount_[cacheIndex])[srcCpu]++;
            demandHit_[cache->cacheLevel_]++;
        }
        return 0;
    }

    uint8_t srcCpu = pkt->srcCpuId_;
    uint8_t srcCacheLevel = pkt->srcCacheLevel_;
    CHECK_ARGS_EXIT(srcCpu < numCpus_, "Source cpu id exceeds number of cpus");
    CHECK_ARGS_EXIT(srcCacheLevel <= maxCacheLevel_,
            "Source cache level exceeds max cache level");
    uint8_t cacheIndex = cache->cacheLevel_ - 1;
    
    std::vector<Stats::Vector*> cacheStats =
            {l1PrefHitCount_[cache->cacheLevel_]};
    if (cache->cacheLevel_ == 3) {
        cacheStats.push_back(l2PrefHitCount_[3]);
    }
    
    // 依据源数据和目标数据类型进行统计数据更新
    if (info.source == Dmd) {
        if (info.target == Pref) {
            // Demand Request命中了预取数据
            CHECK_RET_EXIT(usefulTable_[cacheIndex].updateHit(pkt->addr, Dmd,
                    cacheStats), "Failed to update when dmd hit pref data");
        }
        (*demandReqHitTotal_)[srcCpu]++;
        (*demandHitCount_[cacheIndex])[srcCpu]++;
        demandHit_[cache->cacheLevel_]++;
    } else if (info.target == Pref) {
        // 预取命中了预取数据
        CHECK_RET_EXIT(usefulTable_[cacheIndex].updateHit(pkt->addr, Pref,
                cacheStats), "Failed to update when useful %s",
                "pref hit pref data");
    }

    // 时间维度的检查与更新
    CHECK_RET_EXIT(checkUpdateTimingStats(),
            "Failed check & update timing stats information");
    return 0;
}

// 出现Miss情况下
int BasePrefetchFilter::notifyCacheMiss(BaseCache* cache,
        const PacketPtr& pkt, const DataTypeInfo& info) {
    // 这里的info记录的source是新的Miss属性
    // 其中的target表示mshr中已存在的内容属性
    // prefetch->demand表示prefetch获得了一个新的mshr表项
    // 而非和一个demand类型的mshr合并，但是我们并不关心
    CHECK_ARGS_EXIT(info.target != Pref || info.source != Pref,
            "Prefetch will not cimbined with prefetch in mshr.");
    if (info.source == Dmd) {
        demandMiss_[cache->cacheLevel_]++;
        (*demandReqMissCount_[cache->cacheLevel_])++;
        if (info.target == Pref && cache->cacheLevel_) {
            (*shadowedPrefCount_[cache->cacheLevel_ - 1])++
        }
    }
    
    // 时间维度的检查与更新
    CHECK_RET_EXIT(checkUpdateTimingStats(),
            "Failed check & update timing stats information");
    return 0;
}

int BasePrefetchFilter::notifyCacheFill(BaseCache* cache,
        const PacketPtr &pkt, const DataTypeInfo& info, uint64_t evictedAddr) {
    if (!cache->cacheLevel_) {
        return 0;
    }
    
    uint8_t cacheIndex = cache->cacheLevel_ - 1;
    if (info.source == Dmd && info.target == Pref) {
        // 如果一个Demand Request替换了一个预取请求的数据
        // 则会将对应预取从当前Cache记录表中剔除
        CHECK_RET_EXIT(usefulTable_[cacheIndex].updateEvict(evictedAddr,
                prefTotalUsefulValue_, prefUsefulDegree_, prefUsefulType_,
                prefTypeCount_, "Failed to remove pref from the table");
    } else if (info.source == Pref) {
        // 预取替换了一个Demand Request请求的数据
        // 就需要对相关的替换信息进行记录
        if (info.target == Dmd) {
            CHECK_RET_EXIT(usefulTable_[cacheIndex].addPref(pkt, evictedAddr),
                    "Failed to add pref replacing dmd in the table");
        } else {
            int result = isPrefHit(pkt->addr);
            CHECK_RET_EXIT(result, "Pref not exist in the table");
            if (result) {
                // 如果一个预取被Demand Request覆盖，那么它将会被看作是
                // 一个Demand Request，成为一个有害情况的备选项
                CHECK_RET_EXIT(usefulTable_[cacheIndex].addPref(pkt,
                        evictedAddr), "Failed to add pref replacing %s",
                        "used pref in the table");
            } else {
                // 如果一个预取替换了一个没有被Demand Request命中过的无用预取
                // 则原预取将会被剔除，并进行记录，同时新的预取替换对象会继承
                // 之前替换对象的地址

            }
        }
    }
    
    // 时间维度的检查与更新
    CHECK_RET_EXIT(checkUpdateTimingStats(),
            "Failed check & update timing stats information");
    return 0;
}

int BasePrefetchFilter::filterPrefetch(BaseCache* cache,
        const PacketPtr &pkt, const PrefetchInfo& info) {
    // 默认的过滤器没有实现，因此不会改变预取数据的存放位置
    return cache->cacheLevel_;
}

void BasePrefetchFilter::init() {
    maxCacheLevel_ = caches_.size();
    numCpus_ = caches_[1].size();
    usefulTable.resize(maxCacheLevel_);
    for (int i = 0; i <= maxCacheLevel_; i++) {
        usePref_[level] = caches_[i][0]->prefetcher != NULL;
    }
}

int BasePrefetchFilter::checkUpdateTimingStats() {
    memset(demandHit_, 0, sizeof(demandHit_);
    memset(demandMiss_, 0, sizeof(demandMiss_);
    memset(prefTypeCount_, 0, sizeof(prefTypeCount_);
}

void BasePrefetchFilter::regStats() {
    int i, j, k;
    
    // 进行初始化操作
    init();

    emptyStatsVar_
            .name(name() + "empty_stats")
            .desc("Empty stats variable for deleted stats")
            .flag(nozero);
    for (j = 0; j < numCpus_; j++) {
        emptyStatsVar_[i]->subname(j, std::string("cpu") +
                std::to_string(j));
    }

    if (enableStats_) {
        demandReqHitTotal_ = new Stats::Vector();
                .name(name() + "total_demand_requests_hit")
                .desc("Number of demand requests")
                .flag(total);
        for (j = 0; j < numCpus_; j++) {
            demandReqHitTotal_[i]->subname(j, std::string("cpu") +
                    std::to_string(j));
        }
    }

    for (i = 0; i < 4; i++) {
        if (i <= maxCacheLevel_ && enableStats_) {
            demandReqHitCount_[i] = new Stats::Vector();
            demandReqHitCount_[i]
                    ->name(name() + "demand_requests_hit_" +
                            BaseCache::levelName[i])
                    .desc(std::string("Number of demand requests hit in ") +
                            BaseCache::levelName[i])
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                demandReqHitCount_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            demandReqHitCount_[i] = &emptyStatsVar_;
        }
    }
    
    for (i = 0; i < 4; i++) {
        if (i <= maxCacheLevel_ && enableStats_) {
            demandReqMissCount_[i] = new Stats::Vector();
            demandReqMissCount_[i]
                    ->name(name() + "demand_requests_miss_" +
                            BaseCache::levelName[i])
                    .desc(std::string("Number of demand requests miss in ") +
                            BaseCache::levelName[i])
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                demandReqMissCount_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            demandReqMissCount_[i] = &emptyStatsVar_;
        }
    }
    
    if (usePref_[1] && enableStats_) {
        for (i = 0; i < 3; i++) {
            l1PrefHitCount_[i] = new Stats::Vector();
            l1PrefHitCount_[i]
                    ->name(name() + "L1_prefetch_requests_hit_" +
                            BaseCache::levelName[i + 2])
                    .desc(std::string("Number of prefetches request from L1 " +
                            "hit in " + BaseCache::levelName[i + 1])
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                l1PrefHitCount_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        }
    } else {
        for (i = 0; i < 3; i++) {
            l1PrefHitCount_[i] = &emptyStatsVar_;
        }
    }

    if (usePref_[2] && enableStats_) {
        for (i = 0; i < 2; i++) {
            l2PrefHitCount_[i] = new Stats::Vector();
            l2PrefHitCount_[i]
                    ->name(name() + "L2_prefetch_requests_hit_" +
                            BaseCache::levelName[i + 3])
                    .desc(std::string("Number of prefetches request from") +
                            " L2 hit in " + BaseCache::levelName[i + 3])
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                l2PrefHitCount_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        }
    } else {
        for (i = 0; i < 2; i++) {
            l2PrefHitCount_[i] = &emptyStatsVar_;
        }
    }

    for (i = 0; i < 3; i++) {
        if (usePref_[i + 1] && enableStats_) {
            shadowedPrefCount_[i] = new Stats::Vector();
            shadowedPrefCount_[i]
                    ->name(name() + BaseCache::levelName[i + 1] +
                            "_shadowed_prefetches")
                    .desc(std::string("Number of prefetches shadowed by demand "
                            "requests from " + BaseCache::levelName[i + 1])
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                shadowedPrefHitCount_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            shadowedPrefCount_[i] = &emptyStatsVar_;
        }
    }

    for (i = 0; i < 3; i++) {
        if (usePref_[i + 1] && enableStats_) {
            std::string usefulTypeStr;
            for (j = 0; j < 2; j++) {
                if (j == 0) {
                    usefulTypeStr = "single_core";
                } else {
                    usefulTypeStr = "cross_core";
                }
                prefTotalUsefulValue_[i][j] = new Stats::Vector();
                prefTotalUsefulValue_[i][j]
                        ->name(name() + BaseCache::levelName[i + 1] +
                                "_prefetch_" + usefulTypeStr + "_useful_value")
                        .desc(std::string("Total") + usefulTypeStr +
                                "useful value for of prefetches from" +
                                BaseCache::levelName[i + 1])
                        .flag(total);
                for (k = 0; k < numCpus_; k++) {
                    shadowedPrefHitCount_[i][j]->subname(k, std::string("cpu") +
                            std::to_string(k));
               }
            }
        } else {
            for (j = 0; j < 2; j++) {
                prefTotalUsefulValue_[i][j] = &emptyStatsVar_;
            }
        }
    }

    for (i = 0; i < 3; i++) {
        if (usePref_[i + 1] && enableStats_) {
            std::string usefulTypeStr;
            for (j = 0; j < 2; j++) {
                if (j == 0) {
                    usefulTypeStr = "single_core";
                } else {
                    usefulTypeStr = "cross_core";
                }
                for (k = 0; k < 5; k++) {
                    std::string degreeStr = std::to_string(k);
                    prefUsefulDegree_[i][j][k] = new Stats::Vector();
                    prefUsefulDegree_[i][j][k]
                            ->name(name() + BaseCache::levelName[i + 1] +
                                    "_prefetch_" + usefulTypeStr +
                                    "_useful_degree_" + degreeStr)
                            .desc(std::string("Number of prefetches with " +
                                    usefulType + " useful degree " +
                                    degreeStr + " from " +
                                    BaseCache::levelName[i + 1])
                        .flag(total);
                for (int n = 0; n < numCpus_; n++) {
                    prefUsefulDegreei_[i][j][k]->subname(n, std::string("cpu") +
                            std::to_string(n));
                }
            }
        } else {
            for (j = 0; j < 2; j++) {
                for (k = 0; k < 5; k++) {
                    prefUsefulDegree_[i][j][k] = &emptyStatsVar_;
                }
            }
        }
    }
    
    for (PrefUsefulType& type : PrefUsefulTypeList) {
        for (i = 0; i < 3; i++) {
            if (usePref_[i + 1] && enableStats_) {
                prefUsefulType_[i].push_back(new Stats::Vector());
                prefUsefulType_[i].back()
                        ->name(name() + BaseCache::levelName[i + 1] +
                                + "_" + type.name + "_prefetches")
                        .desc(std::string("Number of ") + type.name +
                                " prefetches from" +
                                BaseCache::levelName[i + 1])
                        .flag(total);
                for (j = 0; j < numCpus_; j++) {
                    prefUsefulType_[i].back()->subname(j, std::string("cpu") +
                            std::to_string(j));
              }
            } else {
                prefUsefulType_[i].push_back(&emptyStatsVar_);
            }
        }
    }
}

} // namespace prefetch_filter
