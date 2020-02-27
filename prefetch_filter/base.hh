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

#ifndef __MEM_CACHE_PREFETCH_FILTER_BASE_HH__
#define __MEM_CACHE_PREFETCH_FILTER_BASE_HH__

#include <cstdint>

#include "arch/isa_traits.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "sim/byteswap.hh"
#include "sim/clocked_object.hh"
#include "sim/probe/probe.hh"

class BaseCache;
struct BasePrefetchFilterParams;

namespace prefetch_filter {

// 过滤器基类，仅仅添加了统计功能，不进行实际过滤
class BasePrefetchFilter : public ClockedObject {
public:
    // 构造函数 
    BasePrefetchFilter(const BasePrefetchFilterParams *p);
    // 析构函数
    virtual ~BasePrefetchFilter() {}

    // 添加一个Cache到Filter中来
    int addCache(BaseCache* newCache);

    // 通知发生了Hit事件
    int notifyCacheHit(BaseCache* cache, const PacketPtr& pkt,
            const DataTypeInfo& info);
    
    // 通知发生了Miss事件
    int notifyCacheMiss(BaseCache* cache, const PacketPtr& pkt,
            const DataTypeInfo& info);
    
    // 通知发生了Fill事件
    int notifyCacheFill(BaseCache* cache, const PacketPtr &pkt,
            const DataTypeInfo& info, uint64_t evictedAddr);

    // 对一个预取进行过滤，返回发送的Cache Level或者不预取
    int filterPrefetch(BaseCache* cache, const PacketPtr &pkt,
            const PrefetchInfo& info);
   
    // 注册统计变量
    void regStats() override;

private:
    // 进行基本结构的初始化
    void init();

    // 该函数会对时间维度信息进行统计和打印
    int checkUpdateTimingStats();

public:
    // 用于处理未启用的统计变量
    Stats::Vector emptyStatsVar_;
    
    // Demand Request总个数
    Stats::Vector* demandReqHitTotal_;
    
    // Demand Request命中不同层级, 区分不同核心和命中层级
    Stats::Vector* demandReqHitCount_[4];

    // Demand Request发生Miss不同层级, 区分不同核心和命中层级
    Stats::Vector* demandReqMissCount_[4];

    // L1预取器预取命中不同层级的个数，区分不同核心和命中层级
    Stats::Vector* l1PrefHitCount_[3];

    // L2预取器预取命中不同层级的个数，区分不同核心和命中层级（分析重点）
    Stats::Vector* l2PrefHitCount_[2];

    // L1预取器中被DemandReq覆盖的预取请求个数，区分不同核心和缓存等级
    Stats::Vector* shadowedPrefCount_[3];
    
    // L1预取器发出预取的有益分数和，第一维度对应缓存等级，第二维度对应单多核
    Stats::Vector* prefTotalUsefulValue_[3][2];
    
    // L1预取器发出预取的不同有益/有害程度的预取个数
    // 第一维度对应缓存等级，第二维度对应单多核，第三维度对应有益/有害水平
    Stats::Vector* prefUsefulDegree_[3][2][5];
   
    // 不同分类的预取个数
    std::vector<Stats::Vector*> prefUsefulType_[3];

private:
    // 基于时间维度的统计
    const uint64_t statsPeriod_;

    // 下一个统计周期开始的Tick位置
    uint64_t nextPeriodTick_;

    // Demand请求的命中数量
    uint32_t demandHit_[4] = {0};

    // Demand请求的缺失数量
    uint32_t demandMiss_[4] = {0};

    // 统计周期内的不同类型的预取个数
    uint32_t prefTypeCount_[3][2][5] = {0};

    // 用于记录预取有害信息的结构，每一级缓存都会有一个
    std::vector<IdealPrefetchUsefulTable> usefulTable_;

private:
    // 当前Filter负责监视的所有Cache指针
    std::vector<std::vector<BaseCache*>> caches_;
    
    // 当前某一级缓存是否开启了预取
    bool usePref_[4];

    // 当前系统下的最高缓存等级
    uint8_t maxCacheLevel_;

    // 当前系统下CPU的个数
    uint8_t numCpus_;

    // 是否开启统计操作，如果不做统计，则所有统计函数将会无效
    const bool enableStats_;
    
    // 是否开启过滤器，如果不开启，则所有预取都不会改变
    const bool enableFilter_;
};

} // namespace prefetch_filter

#endif // __MEM_CACHE_PREFETCH_FILTER_BASE_HH__
