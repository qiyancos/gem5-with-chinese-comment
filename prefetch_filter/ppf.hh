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
struct PerceptronPrefetchFilterParams;

namespace prefetch_filter {

// 过滤器基类，仅仅添加了统计功能，不进行实际过滤
class PerceptronPrefetchFilter : public ClockedObject {
public:
    // 构造函数 
    PerceptronPrefetchFilter(const PerceptronPrefetchFilterParams *p);
    
    // 析构函数
    ~PerceptronPrefetchFilter() {}

    // 通知发生了Hit事件
    int notifyCacheHit(BaseCache* senderCache, const PacketPtr& pkt,
            const DataTypeInfo& info);
    
    // 通知发生了Miss事件
    int notifyCacheMiss(BaseCache* senderCache, const PacketPtr& pkt,
            const DataTypeInfo& info);
    
    // 通知发生了Fill事件
    int notifyCacheFill(BaseCache* senderCache, const PacketPtr &pkt,
            const DataTypeInfo& info);

    // 对一个预取进行过滤，返回发送的Cache Level或者不预取
    int filterPrefetch(BaseCache* senderCache, const PacketPtr &pkt,
            const PrefetchInfo& info);
    
    // 注册统计变量
    void regStats() override;

public:
    // 多核有害的预取（他核心有害多于多单核心有害）
    Stats::Vector prefCrossCoreHarmful;

private:
    // 将预取设置为到L1的可信度阈值
    const uint16_t l1PrefThreshold_;

    // 将预取设置为到L2的可信度阈值
    const uint16_t l2PrefThreshold_;

    // 将预取设置为到L3的可信度阈值
    const uint16_t l3PrefThreshold_;
    
    // 没有被过滤的预取索引表格
    FeatureIndexTable* prefetchTablePtr_;
    
    // 被过滤的预取索引表格
    FeatureIndexTable* rejectTablePtr_;
    
    // 存放Feature权重的表格
    std::vector<FeatureWeightTable*> featureTable_;

    // 存放Feature的向量
    std::vector<Feature> featureList_;
};

} // namespace prefetch_filter

#endif // __MEM_CACHE_PREFETCH_FILTER_BASE_HH__
