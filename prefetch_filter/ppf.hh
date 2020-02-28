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

#ifndef __MEM_CACHE_PREFETCH_FILTER_PPF_HH__
#define __MEM_CACHE_PREFETCH_FILTER_PPF_HH__

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

class Feature {
public:
    // 依据字符串初始化Feature
    int init(const std::string& feature);
    
    // 获取一个预取信息的索引
    uint16_t getIndex(const PrefetchInfo& info);

    // 获取当前Feature的表格大小
    uint16_t getSize();

private:
    // 当前Feature所使用的预取信息索引
    std::vector<uint8_t> infoIndexList_;

    // Feature索引对应结果的起始位次
    uint8_t startBits_;

    // Feature索引对应结果的位数
    uint8_t bits_;
};

// 过滤器基类，仅仅添加了统计功能，不进行实际过滤
class PerceptronPrefetchFilter : public BasePrefetchFilter {

typedef CacheTable<std::vector<uint16_t>> FeatureIndexTable;
typedef CacheTable<int8_t> FeatureWeightTable;

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

private:
    // 用于初始化数据的函数
    int init();

public:
    // 一直都未被过滤的预取请求个数，区分核心，编号区分缓存
    Stats::Vector* prefAccepted[3];
    
    // 一直都被过滤的预取请求个数，区分核心，编号区分缓存
    Stats::Vector* prefRejected[3];

    // 在Prefetch Table和Reject Table之间存在切换的预取
    Stats::Vector* prefThreshing[3];
    
    // 最终被处理成预取至L1的请求个数
    Stats::Vector* prefToL1;

    // 最终被处理成预取至L2的请求个数，0对应没有降级的预取，1对应降1级的预取
    Stats::Vector* prefToL2[2];

    // 最终被处理成预取至L3的请求个数，编号对应降级的数值
    Stats::Vector* prefToL3[3];

    // 不同Feature不同权重的数值出现次数
    std::vector<std::vector<Stats::Vector*>> featureWeightFrequency;

private:
    // 初始化成功与否的标志
    bool initFailFlag;

    // 是否在不同CPU之间共享表格
    const bool sharedTable_;

    // 将预取设置为到L1的可信度阈值
    const uint16_t l1PrefThreshold_;

    // 将预取设置为到L2的可信度阈值
    const uint16_t l2PrefThreshold_;

    // 将预取设置为到L3的可信度阈值
    const uint16_t l3PrefThreshold_;
    
    // Feature权重的bit数
    const uint8_t weightBits_;

    // 不同层级缓存相关反馈对应的训练幅度
    const uint8_t trainStep_[3];

    // 没有被过滤的预取索引表格
    std::vector<FeatureIndexTable> prefetchTable_;
    
    // 被过滤的预取索引表格
    std::vector<FeatureIndexTable> rejectTablePtr_;
    
    // 存放Feature权重的表格
    std::vector<std::vector<FeatureWeightTable>> featureTable_;

    // 存放Feature的向量
    std::vector<Feature> featureList_;
};

} // namespace prefetch_filter

#endif // __MEM_CACHE_PREFETCH_FILTER_PPF_HH__
