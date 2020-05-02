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

#ifndef __MEM_CACHE_PREFETCH_FILTER_PREF_HARM_TABLE_HH__
#define __MEM_CACHE_PREFETCH_FILTER_PREF_HARM_TABLE_HH__

#include <cstdint>

#include "mem/cache/prefetch_filter/saturated_counter.hh"
#include "mem/cache/prefetch_filter/table.hh"

namespace prefetch_filter {

class PrefetchUsefulTable {
public:
    // 进行表格的初始化
    int init(const uint8_t counterBits, const uint32_t CCSize,
            const uint8_t CCAssoc, const uint32_t VCSize,
            const uint8_t VCAssoc, const int counterInit,
            BaseCache* cache = nullptr,
            const int8_t CCTagBits = -1, const int8_t VCTagBits = -1);

    // 判断一个预取是否可用（可以不可训练，但是必须存在，
    // 既没有被无效化或者删除）
    int isPrefValid(const uint64_t& addr);

    // 获取一个预取当前的替换地址
    int getReplacedAddr(const uint64_t& addr, uint64_t* replacedAddr);

    // 对命中Pref的情况进行处理
    int updateHit(const uint64_t& addr);
    
    // 对Miss的情况进行处理，不区分Pref还是替换数据地址
    int updateMiss(const uint64_t& addr);
    
    // 当新的Pref出现的时候，进行替换信息的更新
    int addPref(const uint64_t& prefAddr, const uint64_t evictedAddr,
            const DataType type, std::map<uint64_t, uint8_t>* conflictPref);
    
    // 当前Cache中的预取被替换了（如果需要训练返回1，否则返回0）
    int evictPref(const uint64_t& addr, uint8_t* counterPtr);
   
    // 将给定的预取无效化
    int invalidatePref(const uint64_t& addr, uint8_t* counterPtr);

    // 用于作内存检查
    void memCheck();

private:
    // 该函数负责完全删除一个预取的记录
    int deletePref(const uint64_t& addr, uint8_t* counterPtr);

public:
    // Counter的大小
    uint8_t counterBits_;

    // Counter的初始化数值
    int counterInit_;

    // Counter Cache的Tag大小
    int8_t counterCacheTagBits_;
    
    // Counter Cache的大小
    uint32_t counterCacheSize_;

    // Counter Cache的组相联毒
    uint8_t counterCacheAssoc_;

    // Counter Cache的Tag大小
    int8_t victimCacheTagBits_;
    
    // Victim Cache的大小
    uint32_t victimCacheSize_;

    // Victim Cache的组相联度
    uint8_t victimCacheAssoc_;

public:
    // Counter Cache的存储表项
    class CounterEntry {
    public:
        // 默认初始化函数
        CounterEntry() {}
        
        // 默认初始化函数
        CounterEntry(const uint8_t counterBits, const int counterInit) {
            counter_.init(counterBits, counterInit);
        }
        
        // 完整初始化函数
        CounterEntry(const uint8_t counterBits, const int counterInit,
                const uint64_t& prefAddr, const uint64_t& evictedAddr) :
                prefAddr_(prefAddr), evictedAddr_(evictedAddr) {
            counter_.init(counterBits, counterInit);
        }

    public:
        // 计数器大小
        SaturatedCounter counter_;
        
        // 对应的预取地址
        uint64_t prefAddr_;
        
        // 对应的被替换的地址
        uint64_t evictedAddr_;
    };

    // 是否开启了统计
    bool valid_ = false;

private:
    // 当前处于Cache中的预取地址，此处不做压缩映射，实际硬件设计
    
    // 预取的类型
    enum PrefType {
            // 当前预取可以训练，有替换数据和Counter
            Trainable,
            // 当前预取因为冲突而被剔除，导致无法训练
            NotTrainable,
            // 一个没有替换任何数据的预取，并且从未被Demand命中
            CleanPref,
            // 一个没有替换任何数据的预取，并且被Demand命中
            UsefulCleanPref};

    // Cache中存在压缩地址的相关记录结构，后者说明了预取的类型
    std::map<uint64_t, PrefType> prefInCache_;

    // 被替换数据到Counter Cache压缩地址的映射（实际上这里使用的是预取地址）
    CacheTable<uint64_t> victimCache_;

    // Counter Cache
    CacheTable<CounterEntry> counterCache_;

    // 最近被无效化但是尚未删除的预取地址（仅用于正确性检查）
    std::set<uint64_t> invalidatedPref_;
    
    // 所属Cache
    BaseCache* cache_;
};

} // namespace prefetch_filter

#endif // __MEM_CACHE_PREFETCH_FILTER_PREF_HARM_TABLE_HH__
