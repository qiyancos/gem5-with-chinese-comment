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

#ifndef __MEM_CACHE_PREFETCH_FILTER_PPF_TRAINER_HH__
#define __MEM_CACHE_PREFETCH_FILTER_PPF_TRAINER_HH__

#include <cstdint>
#include <queue>

#include "mem/cache/prefetch_filter/base.hh"
#include "mem/cache/prefetch_filter/table.hh"

class PerceptronPrefetchFilter;

namespace prefetch_filter {

// 针对不同类型预取的训练方法对应的enum类型
enum TrainingType {DemandMiss, GoodPref, BadPref, UselessPref, TrainTypeCount};

// 获取训练类型的字符串表示，用于Debug
std::string getTrainingTypeStr(const TrainingType type);

class TrainingEvent final {
public:
    // 默认初始化函数
    TrainingEvent() = default;
    
    // 默认初始化函数
    TrainingEvent(const uint64_t& addr, const TrainingType type,
            const uint8_t srcCacheLevel, BaseCache* cache) :
            addr_(addr), type_(type),
            srcCacheLevel_(srcCacheLevel), cache_(cache) {}
    
    // 设置训练目标
    int setTrainingTarget(BaseCache* cache);

    // 设置准备就绪的时间
    int setReadyTime(const Tick& readyTime);

    // 按照时间先后顺序排列，时间越小越靠前
    bool operator< (const TrainingEvent& b) const {
        return readyTime_ > b.readyTime_;
    }

public:
    // 训练预取地址
    uint64_t addr_ = invalidBlkAddr_;
    
    // 训练类型
    TrainingType type_ = DemandMiss;
    
    // 生成训练事件的Cache层级
    uint8_t srcCacheLevel_ = 255;

    // 训练目标Cache
    BaseCache* cache_ = nullptr;
    
    // 准备就绪的时间
    Tick readyTime_ = 0;
};

typedef std::priority_queue<TrainingEvent, std::vector<TrainingEvent>>
        ReadyEventQueue; 

class TrainingEventDistributor final {
public:
    // 初始化函数
    int init(const uint32_t queueSize);

    // 初始化函数
    int init(const std::set<BaseCache*>& caches);

    // 添加一个训练事件到队列中，0表示成功，1表示失败
    int addTrainingEvent(TrainingEvent event);
    
    // 获取所有当前周期准备就绪的训练事件
    int getReadyTrainingEvent(ReadyEventQueue* readyEvents); 

private:
    // EventQueue的大小
    int queueSize_;

    // 记录处理周期位置
    Tick workingTick_ = 0;

    // 判断给定的目标是否已经添加了训练事件
    std::set<BaseCache*> invalidTargets_;

    // 为每一个PPF准备的eventQueue
    std::map<BaseCache*, std::list<TrainingEvent>> eventQueues_;
};

typedef CacheTable<std::set<BaseCache*>> TrainingTargetTable;

class PPFTrainer final {
public:
    // 初始化函数
    int init(const std::set<BaseCache*>& caches, const int8_t tagBits,
            const uint32_t tagetTableSize, const uint8_t targetTableAssoc,
            const uint32_t eventQueueSize);
    
    // 初始化函数
    int init(const int8_t tagBits, const uint32_t tagetTableSize,
            const uint8_t targetTableAssoc, const uint32_t eventQueueSize);
    
    // 初始化函数
    int init(const std::set<BaseCache*>& caches);
    
    // 添加一个训练事件到队列中
    int addTrainingEvent(BaseCache* cache, const uint64_t& addr,
            const TrainingType type, std::set<BaseCache*>* fullTargets);

    // 更新本周期所有准备就绪的训练事件，返回本周期执行的训练数目
    int completeTrainingEvent(PerceptronPrefetchFilter* prefFilter);

    // 添加一个预取记录
    int addPref(BaseCache* cache, const uint64_t& addr,
            const std::set<BaseCache*>& targets);
    
    // 无效化一个给定的预取
    int invalidatePref(BaseCache* cache, const uint64_t& addr);
    
    // 删除一个给定的预取
    int deletePref(BaseCache* cache, const uint64_t& addr);

private:
    // 训练事件分发结构
    TrainingEventDistributor distributor_;

    // 训练目标表的映射结构
    std::map<BaseCache*, TrainingTargetTable> targetTables_;
};

}
#endif // __MEM_CACHE_PREFETCH_FILTER_PPF_TRAINER_HH__
