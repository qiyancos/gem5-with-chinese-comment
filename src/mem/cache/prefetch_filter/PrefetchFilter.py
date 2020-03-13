# Copyright (c) 2020 Peking University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Rock Lee

from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject

class BasePrefetchFilter(SimObject):
    type = 'PrefetchFilter'
    cxx_header = "mem/cache/prefetch_filter/base.hh"

    enable_filter = Param.Bool(True, "Whether to enable Filter Engine")
    enable_stats = Param.Bool(True, "Whether to enable Stats Engine")

    stats_period = Param.Uint64(?, "Period of statistics in time dimension")
    
    block_size = Param.Int(Parent.cache_line_size, "block size in bytes")

class PerceptronPrefetchFilter(BasePrefetchFilter):
    type = 'PrefetchFilter'
    cxx_header = "mem/cache/prefetch_filter/ppf.hh"
    
    # 结构设置
    cpu_shared_table = Param.Bool(True, "Whether to share table across "
            "differenct cores")
    cache_shared_table = Param.Bool(True, "Whether to share table across "
            "differenct caches")
    all_shared_table = Param.Bool(True, "Whether to share table across all "
            "caches")
    # 在共享表格方面，LLC一定共享，其他层级选择性共享
    # 该设计暂时不支持SW架构

    allow_upgrade = Param.Bool(True, "If a pref from low-level cache can be"
            " sent to a higher level cache")

    prefetch_table_size = Param.UInt32(1024, "Size of the prefetch table")
    prefetch_table_assoc = Param.UInt8(4, "Associativity of the prefetch "
            "table")
    
    reject_table_size = Param.UInt32(1024, "Size of the reject table")
    reject_table_assoc = Param.UInt8(4, "Associativity of the reject "
            "table")
    
    old_pref_table_size = Param.UInt32(1024, "Size of the old prefetch table")
    old_pref_table_assoc = Param.UInt8(4, "Associativity of the old prefetch "
            "table")
    
    # 过滤设置
    # 0-L1ICache，1-L1DCache，2-L2Cache...n-LnCache
    prefetch_threashold = VectorParam.UInt16([?, ?, ?],
            "Thresholds for prefetch to different caches")
    
    # PPF自带的特征
    feature_weight_bits = Param.UInt8(4, "Number of bits of weight for a "
            "feature")
    feature_weight_init = Param.UInt8(7, "Initiated value for weight")

    # Feature表示方式: "key1 key2 key3 x n" 表示从[key1]^[key2]^[key3]的结果中
    #          取第x位开始共n个bits(包括第x位)的数据作为Weight Table的索引
    original_features = VectorParam.String([
            "PC1 Confidence 0 ?", # PC ^ Confidence; ?
            "Address 0 6", # Address inside a cache block; 64(2^6)
            "Address 6 12", # Offset of a cache block within a page; 64(2^6)
            "PageAddress 0 ?", # Lower bits of physical page number; ?
            "Confidence 0 ?", # Confidence of the prefetch; ?
            "PC1 PC2>>1 PC3>>2 0 12", # Hash of last insts; 4096(2^12)
            "Signature Delta 0 ?", # Signature ^ Delta; ? 
            "PC1 Depth 0 ?", # PC ^ Depth, ?
            "PC1 Delta 0 ?", # PC ^ Delta, ?
            ], "List of original features used for PPF")
    
    # 新增的PPF特征
    added_features = VectorParam.String([
            "BPC1 BPC2>>1 BPC3>>2 0 12", # Hash of branch insts; 4096(2^12)
            "PrefHarm 0 ?", # Prefetch harmfulness; ?
            "PrefetcherID PC1 0 ?", # CoreID bitmap ^ PC; ?
            "PrefetcherID PageAddress 0 ?", # CoreID bitmap ^ Page address; ?
            "PrefetcherID PrefHarm 0 ?", # CoreID bitmap ^ Prefetch Harmfulness; ?
            ], "List of added features used for PPF")
    
    # 训练设置
    default_training_step = Param.UInt8(1,
            "Default training step for all caches");
    useless_prefetch_training_step = Param.UInt8(1, "Punish useless prefetch "
            "if this number is set nozero")
    training_step = VectorParam.UInt8([3, 2, 1],
            "Step for training of different cache feedback")
    
    # 有害性统计结构的设置
    counter_cache_size = Param.UInt32(?, "Size of the counter cache")
    counter_cache_assoc = Param.UInt8(?, "Associativity of the counter cache")
    counter_cache_tag_bits = Param.Int8(?, "Number of tag bits of the "
            "counter cache (-1 means deafult setting)")
    counter_bits = Param.UInt8(?, "Number of bits for a counter")
    victim_cache_size = Param.UInt32(?, "Size of the victim cache")
    victim_cache_assoc = Param.UInt8(?, "Associativity of the victim cache")