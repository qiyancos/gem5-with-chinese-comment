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
#include <string>
#include <cmath>
#include <vector>
#include <list>
#include <set>
#include <map>

#include <memory.h>

#include "base/statistics.hh"
#include "base/types.hh"
#include "sim/clocked_object.hh"

#include "mem/cache/prefetch_filter/table.hh"
#include "mem/cache/prefetch_filter/pref_info.hh"

#include "debug/PrefetchFilter.hh"
#include "debug/PrefetchFilterTimer.hh"

class BaseCache;
class Packet;
struct BasePrefetchFilterParams;

// 过滤器基类，仅仅添加了统计功能，不进行实际过滤
class BasePrefetchFilter : public ClockedObject {
protected:
    typedef Packet *PacketPtr;
    typedef prefetch_filter::DataTypeInfo DataTypeInfo;
    typedef prefetch_filter::PrefetchInfo PrefetchInfo;

private:
    typedef prefetch_filter::IdealPrefetchUsefulTable IdealPrefetchUsefulTable;

public:
    // 构造函数 
    BasePrefetchFilter(const BasePrefetchFilterParams *p);
    // 析构函数
    virtual ~BasePrefetchFilter() {}

    // 创建全局唯一实例
    static BasePrefetchFilter* create(const BasePrefetchFilterParams *p);
    
    // 获取给定Cache连接的CPU侧连接的接口ID，用于发送提级预取
    static int getCpuSideConnectedPortId(PacketPtr pkt,
            std::vector<int16_t>* portIds);

    // 获取给定Cache连接的CPU侧对应的Cache，用于处理提级预取
    static int getCpuSideConnectedCache(PacketPtr pkt,
            std::set<BaseCache*>* caches);

    // 添加一个Cache到Filter中来
    int addCache(BaseCache* newCache);

    // 通知发生了Hit事件
    virtual int notifyCacheHit(BaseCache* cache, const PacketPtr& pkt,
            const uint64_t& hitAddr, const DataTypeInfo& info);
    
    // 通知发生了Miss事件
    virtual int notifyCacheMiss(BaseCache* cache, PacketPtr& pkt,
            const PacketPtr& combinedPkt, const DataTypeInfo& info);
    
    // 通知发生了Fill事件
    virtual int notifyCacheFill(BaseCache* cache, const PacketPtr &pkt,
            const uint64_t& evictedAddr, const DataTypeInfo& info);

    // 对一个预取进行过滤，返回发送的Cache Level或者不预取
    virtual int filterPrefetch(BaseCache* cache, const uint64_t& prefAddr,
            const PrefetchInfo& info);
   
    // 对一个预取到的数据进行Invalid的时候需要执行
    virtual int invalidatePrefetch(BaseCache* cache, const uint64_t& prefAddr);
    
    // 通知一个Cache发生了因为下层Block无法发送请求
    // 并设置预取激进度的调整数值
    virtual int notifyCacheReqSentFailed(BaseCache* cache,
            const int totalEntries, const int waitingDemands,
            const uint8_t originalDegree, uint8_t* newDegree);

    // 默认的初始化函数
    void init() override;

    // 注册统计变量
    void regStats() override;
    
protected:
    // 用于生成当前类型实例的hash数值
    static int genHash(const std::string& name);
   
    // 该函数被用于内存泄漏检查
    virtual void memCheck();

    // 该函数被用于处理子类的预取无效化操作，减轻子类压力
    virtual int helpInvalidatePref(BaseCache* cache,
            const std::set<uint64_t>& addrs);

    // 该函数被用于矫正数据后对子类的更新
    virtual int helpCorrectPref(
            const std::map<BaseCache*, std::set<uint64_t>>& correctionList);

    // 删除和某一个预取相关的记录
    int removePrefetch(BaseCache* cache, const uint64_t& prefAddr,
            const bool isHit);

private:
    // 进行基本结构的初始化
    int initThis();

    // 该函数会执行和时序相关的更新（在最开始执行）
    int checkUpdateTimingAhead();

    // 该函数会执行和时序相关的更新（在处理的最后执行）
    int checkUpdateTimingPost();

public:
    // 用于处理未启用的统计变量
    Stats::Vector* emptyStatsVar_;
    
    // Demand Request总个数
    Stats::Vector* demandReqHitTotal_;
    
    // Demand Request命中不同层级, 区分不同核心和命中层级
    std::vector<Stats::Vector*> demandReqHitCount_;

    // Demand Request发生Miss不同层级, 区分不同核心和命中层级
    std::vector<Stats::Vector*> demandReqMissCount_;

    // 预取器预取命中不同层级的个数，区分不同核心和命中层级
    std::vector<std::vector<Stats::Vector*>> prefHitCount_;

    // 在某一级Cache中填充的预取总个数（cpu, l1, l2, llc）
    std::vector<std::vector<Stats::Vector*>> prefFillCount_;

    // 计算一个预取请求在某一个层级Cache中的总处理时间（l1, l2, l3, dram）
    std::vector<std::vector<Stats::Vector*>> prefProcessCycles_;

    // 计算一个预取请求在某一个层级Cache中的平均处理时间（l1, l2, l3, dram）
    std::vector<std::vector<Stats::Formula*>> prefAvgProcessCycles_;

    // 预取器中被DemandReq覆盖的预取请求个数，区分不同核心和缓存等级
    std::vector<Stats::Vector*> shadowedPrefCount_;
    
    // 预取器中因为合并了Prefetch MSHR导致被删除的预取请求个数
    // 区分不同核心和缓存等级
    std::vector<Stats::Vector*> squeezedPrefCount_;
    
    // 预取器发出预取的有益分数和，第一维度对应缓存等级，第二维度对应单多核
    // 第二维度0表示单核心，1表示多核心
    std::vector<std::vector<Stats::Vector*>> prefTotalUsefulValue_;
    
    // 预取器发出预取的不同有益/有害程度的预取个数
    // 第一维度对应缓存等级，第二维度对应单多核，第三维度对应有益/有害水平
    // 其中2对应有害/有益水平为0，编号小则有害，大则有益，按序递增
    std::vector<std::vector<std::vector<Stats::Vector*>>> prefUsefulDegree_;
   
    // 不同分类的预取个数
    std::vector<std::vector<Stats::Vector*>> prefUsefulType_;

    // 总共进行统计的预取个数
    static std::vector<Stats::Vector*> totalStatsPref_;

    // 因为没有WriteBuffer阻塞无法上传的提级预取
    static std::vector<Stats::Vector*> dismissedLevelUpPrefNoWB_;

    // 因为预取不及时而被忽略的提级预取
    static std::vector<Stats::Vector*> dismissedLevelUpPrefLate_;

    // 为了避免造成带宽压力而删除的降级预取
    static std::vector<Stats::Vector*> dismissedLevelDownPref_;

    // 时间维度的统计计数数据结构
    class TimingStats {
    public:
        // 重置数据
        void reset();
        
    public:
        // Demand请求的命中数量
        std::vector<uint32_t> demandHit_;

        // Demand请求的缺失数量
        std::vector<uint32_t> demandMiss_;

        // 统计周期内的不同类型的预取个数，第一维度表示Cahce等级
        // 第二个维度表示单核/多核，第三个维度表示Degee分类
        std::vector<std::vector<std::vector<uint32_t>>> prefDegreeCount_;
    };

    // 一个指向时间维度统计数据中prefDegreeCount_的指针，方便计算
    std::vector<std::vector<std::vector<std::vector<uint32_t>>>*>
            timingDegree_;

    // 时间维度的统计计数数据，每一个CPU都有一个
    std::vector<TimingStats> timingStats_;

public:
    // 提取缓存行地址对应的Mask
    static uint64_t cacheLineAddrMask_;

    // 缓存行内偏移地址对应的Bits
    static uint8_t cacheLineOffsetBits_;
    
    // 当前单个Cycle对应的Tick大小
    static Tick clockPeriod_;

    // 当前Filter负责监视的所有Cache指针
    static std::vector<std::vector<BaseCache*>> caches_;
    
    // 当前系统下的最高缓存等级
    static uint8_t maxCacheLevel_;

    typedef std::pair<BaseCache*, std::pair<uint64_t, uint64_t>> DoublePref;

    // 该变量存放着当前处理的信息，以便避免被预取校正错误的处理
    static DoublePref skipCorrectPref_;

private:
    // 矫正数据的周期
    const Tick correctionPeriod_ = 5000000000LLU;

    // 下一个矫正周期执行的Tick位置
    Tick nextCorrectionTick_ = 0;
    
    // 基于时间维度的统计
    const Tick statsPeriod_;

    // 下一个统计周期开始的Tick位置
    Tick nextPeriodTick_;

    // 下一次打印Tick的时间信息
    Tick nextTimerPrintTick_ = 0;

    // 时间维度统计周期的计数
    uint64_t timingStatsPeriodCount_ = 0;

    // 是否开启统计操作，如果不做统计，则所有统计函数将会无效
    const bool enableStats_ = false;
    
protected:
    // 全局唯一的实例指针
    static BasePrefetchFilter* onlyInstance_;
    
    // 用于区分不同类型过滤器的哈希数值
    static int64_t typeHash_;

    // 用于判断是否进行了初始化
    static bool initFlag_;

    // 用于判断是否进行了注册
    static bool regFlag_;

    // 是否开启过滤器，如果不开启，则所有预取都不会改变
    const bool enableFilter_ = false;
    
    // 是否使用被替换的预取之前替换的地址作为新的预取替换的Victim地址
    const bool enableRecursiveReplace_ = false;
    
    // 用于记录预取有害信息的结构，每一级缓存都会有一个
    std::map<BaseCache*, IdealPrefetchUsefulTable> usefulTable_;
    
    // 当前某一级缓存是否开启了预取
    std::vector<bool> usePref_;

    // 当前级别是否会接收到预取
    std::vector<bool> havePref_;

    // 当前系统下CPU的个数
    uint8_t numCpus_ = 0;
};

#endif // __MEM_CACHE_PREFETCH_FILTER_BASE_HH__
