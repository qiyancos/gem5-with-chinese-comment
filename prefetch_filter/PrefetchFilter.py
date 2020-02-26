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

class PerceptronPrefetchFilter(BasePrefetchFilter):
    type = 'PrefetchFilter'
    cxx_header = "mem/cache/prefetch_filter/ppf.hh"
    
    # 结构设置
    share_table = Param.Bool(True, "If share table across differenct cores")
    
    prefetch_table_size = Param.UInt32(1024, "Size of the prefetch table")
    prefetch_table_assoc = Param.UInt8(4, "Associativity of the prefetch "
            "table")
    
    reject_table_size = Param.UInt32(1024, "Size of the reject table")
    reject_table_assoc = Param.UInt8(4, "Associativity of the reject "
            "table")
    
    # 过滤设置
    l1_threashold = Param.UInt16(?, "Threshold for prefetch to l1")
    l2_threashold = Param.UInt16(?, "Threshold for prefetch to l2")
    l3_threashold = Param.UInt16(?, "Threshold for prefetch to l3")
    
    # PPF自带的特征
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
            "CoreID PC1 0 ?", # CoreID bitmap ^ PC; ?
            "CoreID PageAddress 0 ?", # CoreID bitmap ^ Page address; ?
            "CoreID PrefHarm 0 ?", # CoreID bitmap ^ Prefetch Harmfulness; ?
            ], "List of added features used for PPF")
    
    # 训练设置
    fine_grained_training = Param.Bool(True, "Use differenct training speed "
            "for differentcache level")
    
    default_training_step = Param.Uint8(1, "Step for all feedback")

    l1_training_step = Param.Uint8(3, "Step for training of l1 feedback")
    l2_training_step = Param.Uint8(2, "Step for training of l1 feedback")
    l3_training_step = Param.Uint8(1, "Step for training of l1 feedback")
