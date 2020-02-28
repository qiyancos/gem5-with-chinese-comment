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

#include "mem/cache/prefetch_filter/ppf.hh"

namespace prefetch_filter {

#define isNumber(String) { \
    return String == "0" || atoi(String.c_str()) != 0; \
}

int Feature::init(const std::string& feature) {
    // 将原始Feature依据空格进行拆分
    std::string word;
	std::vector<std::string> wordList;
    std::string::size_type start, end;
    start = 0;
    end = feature.find(' ');
    while (std::string::npos != end) {
        if (start != end) {
            word = feature.substr(start, end - start);
            wordList.push_back(word);
        }
        start = end + 1;
        end = feature.find(' ', start);
    }
    if (start != feature.length()) {
        word = str.substr(start);
        wordList.push_back(word);
    }
    // 进行解析
    CHECK_ARGS(wordList.size() > 2, "Not enough arguments for a feature");
    CHECK_ARGS(isNumber(wordList.back()) &&
            isNumber(wordList[wordList.size() - 2]),
            "Not enough arguments for a feature");
    for (int i = 0; i < wordList.size() - 2; i++) {
        auto infoPair = IndexMap.find(wordList[i]);
        CHECK_ARGS(infoPair != IndexMap.end(),
                "Unknow feature name \"%s\" in \"%s\"", wordList[i].c_str(),
                feature.c_str());
        infoIndexList_.push_back(infoPair.second);
    }
    startBits_ = atoi(wordList[wordList.size() - 2].c_str());
    bits_ = atoi(wordList.back().c_str());
    return 0;
}
    
uint16_t Feature::getIndex(const PrefetchInfo& info) {
    CHECK_ARGS(!infoIndexList.empty(),
            "Can not get index before feature not initiated");
    uint32_t index = info.getInfo(infoIndexList_[0]);
    for (int i = 1; i < infoIndexList_.size(); i++) {
        index ^= info.getInfo(infoIndexList_[i]);
    }
    uint32_t mask = -1;
    mask = mask >> startBits_ << (32 - bits) >> (32 - bits);
    return index & mask;
}

uint16_t Feature::getSize() {
    return static_cast<uint16_t>(1) << bits_;
}

PerceptronPrefetchFilter::PerceptronPrefetchFilter(
        const PerceptronPrefetchFilterParams *p) :
        BasePrefetchFilter(p),
        sharedTable_(p->share_table),
        l1PrefThreshold_(p->l1_threshold),
        l2PrefThreshold_(p->l1_threshold),
        l3PrefThreshold_(p->l1_threshold),
        weightBits_(p->feature_weight_bits) {
    // 初始化训练幅度
    if (p->find_grained_training) {
        trainStep_[0] = p->l1_training _step;
        trainStep_[1] = p->l2_training _step;
        trainStep_[2] = p->l3_training _step;
    } else {
        for (int i = 0; i < 3; i++) {
            trainStep_[i] = p->default_training_step;
        }
    }

    // 初始化Feature
    for (const std::string& featureStr : original_features) {
        featureList_.push_back(Feature());
        if (featureList_.back().init(featureStr) < 0) {
            initFailFlag = true;
        }
    }
    
    for (const std::string& featureStr : added_features) {
        featureList_.push_back(Feature());
        if (featureList_.back().init(featureStr) < 0) {
            initFailFlag = true;
        }
    }

    // 简单初始化信息表格
    prefetchTable_.push_back(FeatureIndexTable());
    prefetchTable_.back().init(p->prefetch_table_size,
            p->prefetch_table_assoc);
    
    rejectTable_.push_back(FeatureIndexTable());
    rejectTable_.back().init(p->reject_table_size, p->reject_table_assoc);
    
    featureTable_.push_back(std::vector<FeatureWeightTable> (
            featureList_.size()));
    for (int j = 0; j < featureList_.size(); j++) {
        featureTable_[0][j].init(featureList_[j].getSize(), 1);
    }
}

int PerceptronPrefetchFilter::init() {
    // 依据Feature信息初始化表格
    if (!sharedTable_) {
        prefetchTable_.resize(numCpus_);
        rejectTable_.resize(numCpus_);
        featureTable.resize(std::vector<FeatureWeightTable> (
                featureList_.size()));
        int prefSize = prefetchTable_[0].size_;
        int prefAssoc = prefetchTable_[0].assoc_;
        int rejectSize = rejectTable_[0].size_;
        int rejectAssoc = rejectTable_[0].assoc_;
        for (int i = 1; i < numCpus_; i++) {
            prefetchTable_[i].init(prefSize, prefAssoc);
            rejectTable_[i].init(rejectSize, rejectAssoc);
            for (int j = 0; j < featureList_.size(); j++) {
                featureTable_[i][j].init(featureList_[j].getSize(), 1);
            }
        }
    }
}

// 通知发生了Hit事件
int PerceptronPrefetchFilter::notifyCacheHit(BaseCache* cache,
        const PacketPtr& pkt, const DataTypeInfo& info) {
    // 对于指令Cache的预取不进行过滤和训练处理
    if (!cache->cacheLevel_) {
        return 0;
    }

    if (info.source == Dmd) {
        if (info.target == Pref) {
        }
    } else {
        if (info.target == Dmd) {
        } else {
        }
    }
}

// 通知发生了Miss事件
int PerceptronPrefetchFilter::notifyCacheMiss(BaseCache* cache,
        const PacketPtr& pkt, const DataTypeInfo& info) {
    
}

// 通知发生了Fill事件
int PerceptronPrefetchFilter::notifyCacheFill(BaseCache* cache,
        const PacketPtr &pkt, const DataTypeInfo& info) {
}

// 对一个预取进行过滤，返回发送的Cache Level或者不预取
int PerceptronPrefetchFilter::filterPrefetch(BaseCache* cache,
        const PacketPtr &pkt, const PrefetchInfo& info) {
}

void regStats() {
    // 执行父类的统计数据初始化操作
    BasePrefetcherFilter::regStats();
   
    // 初始化表格
    init();

    for (i = 0; i < 3; i++) {
        if (usePref_[i + 1]) {
            prefAccpeted_[i] = new Stats::Vector();
            prefAccpeted_[i]
                    ->name(name() + ".rejected_prefetch_from_" +
                            BaseCache::levelName_[i + 1])
                    .desc(std::string("Number of prefetch requests from " +
                            BaseCache::levelName_[i] + " rejected.")
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefAccpeted_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefAccepted_[i] = &emptyStatsVar_;
        }
    }
    
    for (i = 0; i < 3; i++) {
        if (usePref_[i + 1]) {
            prefRejected_[i] = new Stats::Vector();
            prefRejected_[i]
                    ->name(name() + ".accepted_prefetch_from_" +
                            BaseCache::levelName_[i + 1])
                    .desc(std::string("Number of prefetch requests from " +
                            BaseCache::levelName_[i] + " accepted.")
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefRejected_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefRejected_[i] = &emptyStatsVar_;
        }
    }
    
    for (i = 0; i < 3; i++) {
        if (usePref_[i + 1]) {
            prefThreshing_[i] = new Stats::Vector();
            prefThreshing_[i]
                    ->name(name() + ".rejected_prefetch_from_" +
                            BaseCache::levelName_[i + 1])
                    .desc(std::string("Number of prefetch requests from " +
                            BaseCache::levelName_[i] + " rejected.")
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefThreshing_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefThreshing_[i] = &emptyStatsVar_;
        }
    }
    
    for (i = 0; i < 3; i++) {
        if (usePref_[i + 1]) {
            prefThreshing_[i] = new Stats::Vector();
            prefThreshing_[i]
                    ->name(name() + ".threshing_prefetch_from_" +
                            BaseCache::levelName_[i + 1])
                    .desc(std::string("Number of threshing prefetch requests " +
                            "from " + BaseCache::levelName_[i] + ".")
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefThreshing_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefThreshing_[i] = &emptyStatsVar_;
        }
    }

    if (usePref_[1]) {
        prefToL1_[i] = new Stats::Vector();
        prefToL1_[i]
                ->name(name() + ".prefetch_sent_to_L1D")
                .desc("Number of prefetch requests sent to L1D.")
                .flag(total);
        for (j = 0; j < numCpus_; j++) {
            prefToL1_[i]->subname(j, std::string("cpu") +
                    std::to_string(j));
        }
    } else {
        prefToL1_[i] = &emptyStatsVar_;
    }
    
    for (i = 0; i < 2; i++) {
        if (usePref_[2 - i] && maxCacheLevel_ > 1) {
            prefToL2_[i] = new Stats::Vector();
            prefToL2_[i]
                    ->name(name() + ".prefetch_sent_to_L2_from" +
                            BaseCache::levelName_[2 - i])
                    .desc(std::string("Number of prefetch requests sent to " +
                            "L2 from " + BaseCache::levelName_[2 - i] + ".")
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefToL2_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefToL2_[i] = &emptyStatsVar_;
        }
    }

    for (i = 0; i < 3; i++) {
        if (usePref_[3 - i] && maxCacheLevel_ > 2) {
            prefToL3_[i] = new Stats::Vector();
            prefToL3_[i]
                    ->name(name() + ".prefetch_sent_to_L3_from" +
                            BaseCache::levelName_[3 - i])
                    .desc(std::string("Number of prefetch requests sent to " +
                            "L3 from " + BaseCache::levelName_[3 - i] + ".")
                    .flag(total);
            for (j = 0; j < numCpus_; j++) {
                prefToL3_[i]->subname(j, std::string("cpu") +
                        std::to_string(j));
            }
        } else {
            prefToL3_[i] = &emptyStatsVar_;
        }
    }
    
    CHECK_ARGS_EXIT(weightBits_ > 0, "Bit number of the feature weight must%s",
            " be greater than zero");
    int weightNum = 1 << (weightBits_ - 1);
    featureWeightFrequency_.resize(featureList.size(),
            std::vector<Stats::Vector*> (weightNum));
    for (int i = 0; i < featureList_.size(); i++) {
        for (int j = 0; j < weightNum; j++) {
            std::string weightStr = std::to_string(j);
            // 需要至少一个缓存层级开启了预取器
            if (usePref_[0] || usePref_[1] || usePref_[2] || usePref_[3]) {
                featureWeightFrequency_[i][j] = new Stats::Vector();
                featureWeightFrequency_[i][j]
                        ->name(name() + ".feature_" + featureList_[i].name_ + 
                                "_weight_" + weightStr)
                        .desc(std::string("Time of appearence of a specific") +
                                " weight for a feature.")
                        .flag(total);
                for (j = 0; j < numCpus_; j++) {
                    featureWeightFrequency[i]->subname(j, std::string("cpu") +
                            std::to_string(j));
                }
            } else {
                featureWeightFrequency_[i][j] = &emptyStatsVar_;
            }
        }
    }
}

} // namespace prefetch_filter
