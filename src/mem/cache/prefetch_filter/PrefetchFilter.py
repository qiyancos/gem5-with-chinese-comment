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

from m5.objects.ClockedObject import ClockedObject

class BasePrefetchFilter(ClockedObject):
    type = 'BasePrefetchFilter'
    cxx_header = "mem/cache/prefetch_filter/base.hh"
    sys = Param.System(Parent.any, "System this prefetcher belongs to")

    enable_filter = Param.Bool(True, "Whether to enable Filter Engine")
    enable_stats = Param.Bool(True, "Whether to enable Stats Engine")

    enable_recursive_replace = Param.Bool(True, "Whether to use old replaced"
            "addr of replaced prefetch instead of new reaplced prefetch")

    # print per 10000 cycles
    stats_period = Param.UInt64(5000000,
            "Period of statistics in time dimension")
    
    block_size = Param.Int(Parent.cache_line_size, "block size in bytes")

class PerceptronPrefetchFilter(BasePrefetchFilter):
    type = 'PerceptronPrefetchFilter'
    cxx_header = "mem/cache/prefetch_filter/ppf.hh"

    # Prefetcher degree controller setting
    degree_update_period = Param.UInt64(5000000, "Gap between two prefetcher "
            "degree update checking")

    # Structure Setting
    cpu_shared_table = Param.Bool(False, "Whether to share table across "
            "differenct cores")
    cache_shared_table = Param.Bool(False, "Whether to share table across "
            "differenct caches")
    all_shared_table = Param.Bool(False, "Whether to share table across all "
            "caches")
    # LLC must be shared, other level will be shared according to settings
    # SW not supported

    allow_upgrade = Param.Bool(True, "If a pref from low-level cache can be"
            " sent to a higher level cache")

    prefetch_table_size = Param.UInt32(1024, "Size of the prefetch table")
    prefetch_table_assoc = Param.UInt8(4, "Associativity of the prefetch "
            "table")
    prefetch_table_tag_bits = Param.Int8(6, "Tag bits of the prefetch "
            "table")
    
    reject_table_size = Param.UInt32(1024, "Size of the reject table")
    reject_table_assoc = Param.UInt8(4, "Associativity of the reject "
            "table")
    reject_table_tag_bits = Param.Int8(6, "Tag bits of the reject "
            "table")
    
    # Filter Setting
    # Feature Table Setting
    feature_weight_bits = Param.UInt8(5, "Number of bits of weight for a "
            "feature")
    feature_weight_init = VectorParam.UInt8([28, 20, 12], "Initiated value "
            "for weight in differenct cache")
    # 0-L1ICache, 1-L1DCache, 2-L2Cache...n-LnCache
    prefetch_threshold = VectorParam.UInt16([24, 16, 4],
            "Thresholds for prefetch to different caches")
    
    # Used to deal with death valley for prefetch training
    feature_weight_reset_times = Param.UInt32(4096, "The number of rejected "
            "prefetches we see when weight table need to be reset")

    # Feature: "key1 key2 key3 x n" means to extract n bits start from
    #          the xth bit (include xth bit) from '[key1]^[key2]^[key3]'
    #          as the feature index
    original_features = VectorParam.String([
            "PageAddress Confidence 0 10", # PageAddr ^ conf
            "Address 6 6", # Offset of a cache block within a page;
            "Address 0 12", # Lower bits of address;
            "PageAddress 0 10", # Lower bits of physical page num;
            "Confidence 0 3", # Confidence of the prefetch;
            "PC1 PC2>>1 PC3>>2 0 12", # Hash of last insts;
            "Signature Delta 0 12", # Signature ^ Delta;
            "PC1 Depth 0 12", # PC ^ Depth;
            "PC1 Delta 0 12", # PC ^ Delta;
            ], "List of original features used for PPF")
    
    # Newly added features
    added_features = VectorParam.String([
            #"PC1 Confidence 0 12", # PC ^ Confidence;
            "BPC1 BPC2>>1 BPC3>>2 0 12", # Hash of branch insts;
            #"PrefetcherID PC1 0 12", # CoreID bitmap ^ PC;
            #"PrefetcherID PageAddress 0 12", # CoreID bitmap ^ Page address;
            # "PrefHarm 0 4", # Prefetch Harm;
            # "PrefetcherID PrefHarm 0 8", # CoreID bitmap ^ Prefetch Harm;
            ], "List of added features used for PPF")
    
    # Training Setting
    default_training_step = Param.UInt8(1,
            "Default training step for all caches");
    useless_prefetch_training_step = Param.UInt8(1, "Punish useless prefetch "
            "if this number is set nozero")
    miss_training_step = VectorParam.UInt8([3, 2, 1],
            "Step for training of different cache feedback")
    hit_training_step = VectorParam.UInt8([1, 2, 3],
            "Step for training of different cache feedback")
    
    # Harmful Table Setting
    # Counter Cache Setting
    counter_cache_size = Param.UInt32(1024, "Size of the counter cache")
    counter_cache_assoc = Param.UInt8(4, "Associativity of the counter cache")
    counter_cache_tag_bits = Param.Int8(6, "Number of tag bits of the "
            "counter cache (-1 means using full-length tag)")
    
    counter_bits = Param.UInt8(3, "Number of bits for a counter")
    counter_init_value = Param.UInt8(3, "Initiated value for a counter")
    
    # Victim Cache Setting
    victim_cache_size = Param.UInt32(1024, "Size of the victim cache")
    victim_cache_assoc = Param.UInt8(4, "Associativity of the victim cache")
    victim_cache_tag_bits = Param.Int8(6, "Number of tag bits of the "
            "victim cache (-1 means using full-length tag)")
    
    # Trainer Setting
    target_table_size = Param.UInt32(1024, "Size of the training target table")
    target_table_assoc = Param.UInt8(4, "Associativity of the training "
            "target table")
    target_table_tag_bits = Param.Int8(6, "Number of tag bits of the "
            "training target table (-1 means using full-length tag)")
    
    training_delay = Param.UInt32(3, "Number of cycles needed for training");
    event_queue_size = Param.UInt32(2, "Size of training event queue in "
            "training event distributor")
