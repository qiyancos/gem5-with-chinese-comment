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
        return 0;
    }

    uint8_t srcCpu = pkt->srcCpuId_;
    uint8_t srcCacheLevel = pkt->srcCacheLevel_;
    CHECK_ARGS_EXIT(srcCpu < numCpus_, "Source cpu id exceeds number of cpus");
    CHECK_ARGS_EXIT(srcCacheLevel <= maxCacheLevel_,
            "Source cache level exceeds max cache level");
    uint8_t cacheIndex = cache->cacheLevel - 1;
    // 依据源数据和目标数据类型进行统计数据更新
    if (info.source == Dmd) {
        if (info.target == Pref) {
            
            // Demand Request命中了预取数据
            CHECK_RET_EXIT(usefulTable_[cacheIndex].updateHit(pkt->addr, Dmd),
                    "Failed to update when dmd hit pref data");
            switch (srcCacheLevel) {
            case 1: (*l1PrefHitCount_[cache->cacheLeveli])[]
            case 2:
            case 3:
            default: break;
            }
        }
        (*demandReqHitTotal_)[srcCpu]++;
        (*demandHitCount_[cacheIndex])[srcCpu]++;
        demandHit_++;
    } else if (info.target == Pref) {
            // 预取命中了预取数据
            int result = usefulTable_[cacheIndex].updateHit(pkt->addr, Pref);
            CHECK_RET_EXIT(result, "Failed to update when pref hit pref data");
            if (result > 0) {
                // 表明预取请求被预测为有用预取
            } else {
                // 表明预取请求被预测为无用预取
            }
        } else {
        }
    }

    // 统计周期的检查和计算
}

int BasePrefetchFilter::notifyCacheMiss(BaseCache* cache,
        const PacketPtr& pkt, const DataTypeInfo& info) {
}

int BasePrefetchFilter::notifyCacheFill(BaseCache* cache,
        const PacketPtr &pkt, const DataTypeInfo& info) {
}

int BasePrefetchFilter::filterPrefetch(BaseCache* cache,
        const PacketPtr &pkt, const PrefetchInfo& info) {
}

void BasePrefetchFilter::init() {
    maxCacheLevel_ = caches_.size();
    numCpus_ = caches_[1].size();
    usefulTable.resize(maxCacheLevel_);
    for (int i = 0; i <= maxCacheLevel_; i++) {
        usePref_[level] = caches_[i][0]->prefetcher != NULL;
    }
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

    for (i = 0; i < 3; i++) {
        if (i < maxCacheLevel_ && enableStats_) {
            demandReqHitCount_[i] = new Stats::Vector();
            demandReqHitCount_[i]
                    ->name(name() + "demand_requests_hit_" +
                            BaseCache::levelName[i + 1])
                    .desc("Number of demand requests hit in " +
                            BaseCache::levelName[i + 1])
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                demandReqHitCount_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            demandReqHitCount_[i] = &emptyStatsVar_;
        }
    }
    
    if (usePref_[1] && enableStats_) {
        for (i = 0; i < 3; i++) {
            l1PrefHitCount_[i] = new Stats::Vector();
            l1PrefHitCount_[i]
                    ->name(name() + "L1_prefetch_requests_hit_" +
                            BaseCache::levelName[i + 2])
                    .desc("Number of prefetches request from L1 hit in "
                            BaseCache::levelName[i + 1])
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
                    .desc("Number of prefetches request from L2 hit in "
                            BaseCache::levelName[i + 3])
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
    
    for (i = 0; i < 3; i++) {
        if (usePref_[i + 1] && enableStats_) {
            prefCrossCoreUseful_[i] = new Stats::Vector();
            prefCrossCoreUseful_[i]
                    ->name(name() + BaseCache::levelName[i + 1] +
                            "_cross_core_useful_prefetches")
                    .desc("Number of cross_core useful prefetches from" +
                            BaseCache::levelName[i + 1])
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefCrossCoreUseful_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefCrossCoreUseful_[i] = &emptyStatsVar_;
        }
    }

    for (i = 0; i < 3; i++) {
        if (usePref_[i + 1] && enableStats_) {
            prefSingleCoreUseful_[i] = new Stats::Vector();
            prefSingleCoreUseful_[i]
                    ->name(name() + BaseCache::levelName[i + 1] +
                            "_single_core_useful_prefetches")
                   .desc("Number of single_core useful prefetches from" +
                            BaseCache::levelName[i + 1])
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefSingleCoreUseful_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefSingleCoreUseful_[i] = &emptyStatsVar_;
        }
    }

    for (i = 0; i < 3; i++) {
        if (usePref_[i + 1] && enableStats_) {
            prefSelfish_[i] = new Stats::Vector();
            prefSelfish_[i]
                    ->name(name() + BaseCache::levelName[i + 1] +
                            "_selfish_prefetches")
                    .desc("Number of selfish prefetches from" +
                            BaseCache::levelName[i + 1])
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefSelfish_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefSelfish_[i] = &emptyStatsVar_;
        }
    }

    for (i = 0; i < 3; i++) {
        if (usePref_[i + 1] && enableStats_) {
            prefSelfless_[i] = new Stats::Vector();
            prefSelfless_[i]
                    ->name(name() + BaseCache::levelName[i + 1] +
                            "_selfless_prefetches")
                    .desc("Number of selfless prefetches from" +
                            BaseCache::levelName[i + 1])
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefSelfless_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefSelfless_[i] = &emptyStatsVar_;
        }
    }

    for (i = 0; i < 3; i++) {
        if (usePref_[i + 1] && enableStats_) {
            prefUseless_[i] = new Stats::Vector();
            prefUseless_[i]
                    ->name(name() + BaseCache::levelName[i + 1] +
                            "_useless_prefetches")
                    .desc("Number of useless prefetches from" +
                            BaseCache::levelName[i + 1])
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefSelfless[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefUseless_[i] = &emptyStatsVar_;
        }
    }

    for (i = 0; i < 3; i++) {
        if (usePref_[i + 1] && enableStats_) {
            prefSingleCoreHarmful_[i] = new Stats::Vector();
            prefSingleCoreHarmful_[i]
                    ->name(name() + BaseCache::levelName[i + 1] +
                            "_single_core_harmful_prefetches")
                    .desc("Number of single_core harmful prefetches from" +
                            BaseCache::levelName[i + 1])
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefSingleCoreHarmful_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefSingleCoreHarmful_[i] = &emptyStatsVar_;
        }
    }

    for (i = 0; i < 3; i++) {
        if (usePref_[i + 1] && enableStats_) {
            prefCrossCoreHarmful_[i] = new Stats::Vector();
            prefCrossCoreHarmful_[i]
                    ->name(name() + BaseCache::levelName[i + 1] +
                            "_corss_core_harmful_prefetches")
                    .desc("Number of cross_core harmful prefetches from" +
                            BaseCache::levelName[i + 1])
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefCrossCoreHarmful_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefCrossCoreHarmful_[i] = &emptyStatsVar_;
        }
    }
}

} // namespace prefetch_filter
