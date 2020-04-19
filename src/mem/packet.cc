/*
 * Copyright (c) 2011-2019 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2006 The Regents of The University of Michigan
 * Copyright (c) 2010,2015 Advanced Micro Devices, Inc.
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
 * Authors: Ali Saidi
 *          Steve Reinhardt
 */

/**
 * @file
 * Definition of the Packet Class, a packet is a transaction occuring
 * between a single level of the memory heirarchy (ie L1->L2).
 */

#include "mem/packet.hh"
#include "mem/cache/base.hh"
#include "mem/cache/mshr.hh"
#include "mem/cache/prefetch_filter/debug_flag.hh"
#include "mem/cache/prefetch_filter/base.hh"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

#include "base/cprintf.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "mem/packet_access.hh"

// The one downside to bitsets is that static initializers can get ugly.
#define SET1(a1)                     (1 << (a1))
#define SET2(a1, a2)                 (SET1(a1) | SET1(a2))
#define SET3(a1, a2, a3)             (SET2(a1, a2) | SET1(a3))
#define SET4(a1, a2, a3, a4)         (SET3(a1, a2, a3) | SET1(a4))
#define SET5(a1, a2, a3, a4, a5)     (SET4(a1, a2, a3, a4) | SET1(a5))
#define SET6(a1, a2, a3, a4, a5, a6) (SET5(a1, a2, a3, a4, a5) | SET1(a6))
#define SET7(a1, a2, a3, a4, a5, a6, a7) (SET6(a1, a2, a3, a4, a5, a6) | \
                                          SET1(a7))

const MemCmd::CommandInfo
MemCmd::commandInfo[] =
{
    /* InvalidCmd */
    { 0, InvalidCmd, "InvalidCmd" },
    /* ReadReq - Read issued by a non-caching agent such as a CPU or
     * device, with no restrictions on alignment. */
    { SET3(IsRead, IsRequest, NeedsResponse), ReadResp, "ReadReq" },
    /* ReadResp */
    { SET3(IsRead, IsResponse, HasData), InvalidCmd, "ReadResp" },
    /* ReadRespWithInvalidate */
    { SET4(IsRead, IsResponse, HasData, IsInvalidate),
            InvalidCmd, "ReadRespWithInvalidate" },
    /* WriteReq */
    { SET5(IsWrite, NeedsWritable, IsRequest, NeedsResponse, HasData),
            WriteResp, "WriteReq" },
    /* WriteResp */
    { SET2(IsWrite, IsResponse), InvalidCmd, "WriteResp" },
    /* WritebackDirty */
    { SET5(IsWrite, IsRequest, IsEviction, HasData, FromCache),
            InvalidCmd, "WritebackDirty" },
    /* WritebackClean - This allows the upstream cache to writeback a
     * line to the downstream cache without it being considered
     * dirty. */
    { SET5(IsWrite, IsRequest, IsEviction, HasData, FromCache),
            InvalidCmd, "WritebackClean" },
    /* WriteClean - This allows a cache to write a dirty block to a memory
       below without evicting its copy. */
    { SET4(IsWrite, IsRequest, HasData, FromCache), InvalidCmd, "WriteClean" },
    /* CleanEvict */
    { SET3(IsRequest, IsEviction, FromCache), InvalidCmd, "CleanEvict" },
    /* SoftPFReq */
    { SET4(IsRead, IsRequest, IsSWPrefetch, NeedsResponse),
            SoftPFResp, "SoftPFReq" },
    /* SoftPFExReq */
    { SET6(IsRead, NeedsWritable, IsInvalidate, IsRequest,
           IsSWPrefetch, NeedsResponse), SoftPFResp, "SoftPFExReq" },
    /* HardPFReq */
    { SET5(IsRead, IsRequest, IsHWPrefetch, NeedsResponse, FromCache),
            HardPFResp, "HardPFReq" },
    /* SoftPFResp */
    { SET4(IsRead, IsResponse, IsSWPrefetch, HasData),
            InvalidCmd, "SoftPFResp" },
    /* HardPFResp */
    { SET4(IsRead, IsResponse, IsHWPrefetch, HasData),
            InvalidCmd, "HardPFResp" },
    /* WriteLineReq */
    { SET5(IsWrite, NeedsWritable, IsRequest, NeedsResponse, HasData),
            WriteResp, "WriteLineReq" },
    /* UpgradeReq */
    { SET6(IsInvalidate, NeedsWritable, IsUpgrade, IsRequest, NeedsResponse,
            FromCache),
            UpgradeResp, "UpgradeReq" },
    /* SCUpgradeReq: response could be UpgradeResp or UpgradeFailResp */
    { SET7(IsInvalidate, NeedsWritable, IsUpgrade, IsLlsc,
           IsRequest, NeedsResponse, FromCache),
            UpgradeResp, "SCUpgradeReq" },
    /* UpgradeResp */
    { SET2(IsUpgrade, IsResponse),
            InvalidCmd, "UpgradeResp" },
    /* SCUpgradeFailReq: generates UpgradeFailResp but still gets the data */
    { SET7(IsRead, NeedsWritable, IsInvalidate,
           IsLlsc, IsRequest, NeedsResponse, FromCache),
            UpgradeFailResp, "SCUpgradeFailReq" },
    /* UpgradeFailResp - Behaves like a ReadExReq, but notifies an SC
     * that it has failed, acquires line as Dirty*/
    { SET3(IsRead, IsResponse, HasData),
            InvalidCmd, "UpgradeFailResp" },
    /* ReadExReq - Read issues by a cache, always cache-line aligned,
     * and the response is guaranteed to be writeable (exclusive or
     * even modified) */
    { SET6(IsRead, NeedsWritable, IsInvalidate, IsRequest, NeedsResponse,
            FromCache),
            ReadExResp, "ReadExReq" },
    /* ReadExResp - Response matching a read exclusive, as we check
     * the need for exclusive also on responses */
    { SET3(IsRead, IsResponse, HasData),
            InvalidCmd, "ReadExResp" },
    /* ReadCleanReq - Read issued by a cache, always cache-line
     * aligned, and the response is guaranteed to not contain dirty data
     * (exclusive or shared).*/
    { SET4(IsRead, IsRequest, NeedsResponse, FromCache),
            ReadResp, "ReadCleanReq" },
    /* ReadSharedReq - Read issued by a cache, always cache-line
     * aligned, response is shared, possibly exclusive, owned or even
     * modified. */
    { SET4(IsRead, IsRequest, NeedsResponse, FromCache),
            ReadResp, "ReadSharedReq" },
    /* LoadLockedReq: note that we use plain ReadResp as response, so that
     *                we can also use ReadRespWithInvalidate when needed */
    { SET4(IsRead, IsLlsc, IsRequest, NeedsResponse),
            ReadResp, "LoadLockedReq" },
    /* StoreCondReq */
    { SET6(IsWrite, NeedsWritable, IsLlsc,
           IsRequest, NeedsResponse, HasData),
            StoreCondResp, "StoreCondReq" },
    /* StoreCondFailReq: generates failing StoreCondResp */
    { SET6(IsWrite, NeedsWritable, IsLlsc,
           IsRequest, NeedsResponse, HasData),
            StoreCondResp, "StoreCondFailReq" },
    /* StoreCondResp */
    { SET3(IsWrite, IsLlsc, IsResponse),
            InvalidCmd, "StoreCondResp" },
    /* SwapReq -- for Swap ldstub type operations */
    { SET6(IsRead, IsWrite, NeedsWritable, IsRequest, HasData, NeedsResponse),
        SwapResp, "SwapReq" },
    /* SwapResp -- for Swap ldstub type operations */
    { SET4(IsRead, IsWrite, IsResponse, HasData),
            InvalidCmd, "SwapResp" },
    /* IntReq -- for interrupts */
    { SET4(IsWrite, IsRequest, NeedsResponse, HasData),
        MessageResp, "MessageReq" },
    /* IntResp -- for interrupts */
    { SET2(IsWrite, IsResponse), InvalidCmd, "MessageResp" },
    /* MemFenceReq -- for synchronization requests */
    {SET2(IsRequest, NeedsResponse), MemFenceResp, "MemFenceReq"},
    /* MemFenceResp -- for synchronization responses */
    {SET1(IsResponse), InvalidCmd, "MemFenceResp"},
    /* Cache Clean Request -- Update with the latest data all existing
       copies of the block down to the point indicated by the
       request */
    { SET4(IsRequest, IsClean, NeedsResponse, FromCache),
      CleanSharedResp, "CleanSharedReq" },
    /* Cache Clean Response - Indicates that all caches up to the
       specified point of reference have a up-to-date copy of the
       cache block or no copy at all */
    { SET2(IsResponse, IsClean), InvalidCmd, "CleanSharedResp" },
    /* Cache Clean and Invalidate Request -- Invalidate all existing
       copies down to the point indicated by the request */
    { SET5(IsRequest, IsInvalidate, IsClean, NeedsResponse, FromCache),
      CleanInvalidResp, "CleanInvalidReq" },
     /* Cache Clean and Invalidate Respose -- Indicates that no cache
        above the specified point holds the block and that the block
        was written to a memory below the specified point. */
    { SET3(IsResponse, IsInvalidate, IsClean),
      InvalidCmd, "CleanInvalidResp" },
    /* InvalidDestError  -- packet dest field invalid */
    { SET2(IsResponse, IsError), InvalidCmd, "InvalidDestError" },
    /* BadAddressError   -- memory address invalid */
    { SET2(IsResponse, IsError), InvalidCmd, "BadAddressError" },
    /* FunctionalReadError */
    { SET3(IsRead, IsResponse, IsError), InvalidCmd, "FunctionalReadError" },
    /* FunctionalWriteError */
    { SET3(IsWrite, IsResponse, IsError), InvalidCmd, "FunctionalWriteError" },
    /* PrintReq */
    { SET2(IsRequest, IsPrint), InvalidCmd, "PrintReq" },
    /* Flush Request */
    { SET3(IsRequest, IsFlush, NeedsWritable), InvalidCmd, "FlushReq" },
    /* Invalidation Request */
    { SET5(IsInvalidate, IsRequest, NeedsWritable, NeedsResponse, FromCache),
      InvalidateResp, "InvalidateReq" },
    /* Invalidation Response */
    { SET2(IsInvalidate, IsResponse),
      InvalidCmd, "InvalidateResp" }
};

/// 该函数用于设置给定处理的时间点
void
Packet::setTimeStamp(const uint8_t level, const TimeStampType type,
        const Tick& tick) {
    panic_if(level > BasePrefetchFilter::maxCacheLevel_,
            "Unexpected level when setting time stamp");
    if (packetType_ == prefetch_filter::Pref) {
        DEBUG_MEM("Set time stamp for prefetch[%p] @0x%lx [%s -> %s] "
                "with level %u, type %s and tick %lu", this, getAddr(),
                BaseCache::levelName_[srcCacheLevel_].c_str(),
                BaseCache::levelName_[targetCacheLevel_].c_str(), level,
                type == WhenRecv ? "recv" : type == WhenFill ? "fill" : "send",
                tick);
    }
    if (timeStamp_.size() < level + 1) {
        timeStamp_.resize(level + 1, std::vector<Tick>(WhenRecv, 0));
    }
    if (type == WhenRecv) {
        panic_if(level == 0 || level > BasePrefetchFilter::maxCacheLevel_
                || timeStamp_[level - 1][WhenSend],
                "Can not set recv time stamp for the given level");
        if (level > 1 && timeStamp_[level - 2][WhenSend]) {
            panic_if(tick < timeStamp_[level - 2][WhenSend],
                    "Illegal time stamp for send gap");
        }
        timeStamp_[level - 1][WhenSend] = tick;
    } else {
        if (type == WhenFill && level < BasePrefetchFilter::maxCacheLevel_ &&
                level < timeStamp_.size() - 1 &&
                timeStamp_[level + 1][WhenFill]) {
            panic_if(tick < timeStamp_[level + 1][WhenFill],
                    "Illegal time stamp for fill gap");
        } else if (type == WhenFill &&
                level == BasePrefetchFilter::maxCacheLevel_ &&
                timeStamp_[level][WhenSend]) {
            panic_if(tick < timeStamp_[level][WhenSend],
                    "Illegal time stamp for fill gap");
        } else if (type == WhenSend && level > 0 &&
                timeStamp_[level - 1][WhenSend]) {
            panic_if(tick < timeStamp_[level - 1][WhenSend],
                    "Illegal time stamp for fill gap");
        }
        timeStamp_[level][type] = tick;
    }
}

/// 获取一个给定的时间戳
Tick
Packet::getTimeStamp(const uint8_t level, const TimeStampType type) {
    panic_if(level >= timeStamp_.size(),
            "Unexpected level when trying to get time stamp");
    if (type == WhenRecv) {
        panic_if(level == 0, "Unexpected level for recv time stamp");
        return timeStamp_[level - 1][WhenSend];
    } else {
        return timeStamp_[level][type];
    }
}

/// 获取当前Packet在某一个层级Cache中的时间，这里的Level不允许出现0
Tick
Packet::getProcessTime(const uint8_t level) {
    panic_if(level > timeStamp_.size() || level == 0,
            "Can not get process time for the given level");
    Tick result = 0;
    if (level == BasePrefetchFilter::maxCacheLevel_ + 1) {
        /// 获取DRAM的处理时间
        result = timeStamp_[level - 1][WhenFill] &&
                timeStamp_[level - 1][WhenSend] ?
                timeStamp_[level - 1][WhenFill] -
                timeStamp_[level - 1][WhenSend] : 0;
    } else {
        /// 获取其他层级的处理时间
        Tick sendGap = timeStamp_[level][WhenSend] &&
                timeStamp_[level][WhenSend] ?
                (timeStamp_[level][WhenSend] -
                timeStamp_[level - 1][WhenSend]) : 0;
        Tick fillGap = timeStamp_[level - 1][WhenFill] &&
                timeStamp_[level][WhenFill] ?
                (timeStamp_[level - 1][WhenFill] -
                timeStamp_[level][WhenFill]) : 0;
        result = sendGap + fillGap;
    }
    /*
    if (packetType_ == prefetch_filter::Pref) {
        DEBUG_MEM("Get process time for prefetch[%p] @0x%lx [%s -> %s] "
                "with level %u and result %lu", this, getAddr(),
                BaseCache::levelName_[srcCacheLevel_].c_str(),
                BaseCache::levelName_[targetCacheLevel_].c_str(),
                level, result);
    }
    */
    return result;
}

/// 迁移到CC文件以便可以随意更改
Addr
Packet::getAddr() const {
    // assert(flags.isSet(VALID_ADDR));
    return flags.isSet(VALID_ADDR) ? addr : prefetch_filter::invalidBlkAddr_;
}

/// 迁移到cc以避免头文件互相包含问题
void Packet::initPref(BaseCache* srcCache, const uint8_t targetCacheLevel,
        const std::list<uint64_t>& recentBranchPC) {
    srcCacheLevel_ = srcCache->cacheLevel_;
    targetCacheLevel_ = targetCacheLevel;
    packetType_ = prefetch_filter::Pref;
    recentBranchPC_ = recentBranchPC;
    addSrcCache(srcCache);
    // 对于预取的发送时间改为预取分配MSHR的时候
    // setTimeStamp(srcCacheLevel_, WhenRecv, req->time());
}

/// 迁移到cc以避免头文件互相包含问题
void
Packet::deleteData() {
    DEBUG_MEM("Delete packet[%p] @0x%lx data[%p]", this, addr, data);
    if (flags.isSet(DYNAMIC_DATA))
        delete [] data;

    flags.clear(STATIC_DATA|DYNAMIC_DATA);
    data = NULL;
}

bool
Packet::trySatisfyFunctional(Printable *obj, Addr addr, bool is_secure, int size,
                        uint8_t *_data)
{
    const Addr func_start = getAddr();
    const Addr func_end   = getAddr() + getSize() - 1;
    const Addr val_start  = addr;
    const Addr val_end    = val_start + size - 1;

    if (is_secure != _isSecure || func_start > val_end ||
        val_start > func_end) {
        // no intersection
        return false;
    }

    // check print first since it doesn't require data
    if (isPrint()) {
        assert(!_data);
        safe_cast<PrintReqState*>(senderState)->printObj(obj);
        return false;
    }

    // we allow the caller to pass NULL to signify the other packet
    // has no data
    if (!_data) {
        return false;
    }

    const Addr val_offset = func_start > val_start ?
        func_start - val_start : 0;
    const Addr func_offset = func_start < val_start ?
        val_start - func_start : 0;
    const Addr overlap_size = std::min(val_end, func_end)+1 -
        std::max(val_start, func_start);

    if (isRead()) {
        std::memcpy(getPtr<uint8_t>() + func_offset,
               _data + val_offset,
               overlap_size);

        // initialise the tracking of valid bytes if we have not
        // used it already
        if (bytesValid.empty())
            bytesValid.resize(getSize(), false);

        // track if we are done filling the functional access
        bool all_bytes_valid = true;

        int i = 0;

        // check up to func_offset
        for (; all_bytes_valid && i < func_offset; ++i)
            all_bytes_valid &= bytesValid[i];

        // update the valid bytes
        for (i = func_offset; i < func_offset + overlap_size; ++i)
            bytesValid[i] = true;

        // check the bit after the update we just made
        for (; all_bytes_valid && i < getSize(); ++i)
            all_bytes_valid &= bytesValid[i];

        return all_bytes_valid;
    } else if (isWrite()) {
        std::memcpy(_data + val_offset,
               getConstPtr<uint8_t>() + func_offset,
               overlap_size);
    } else {
        panic("Don't know how to handle command %s\n", cmdString());
    }

    // keep going with request by default
    return false;
}

void
Packet::copyResponderFlags(const PacketPtr pkt)
{
    assert(isRequest());
    // If we have already found a responder, no other cache should
    // commit to responding
    assert(!pkt->cacheResponding() || !cacheResponding());
    flags.set(pkt->flags & RESPONDER_FLAGS);
}

void
Packet::pushSenderState(Packet::SenderState *sender_state)
{
    assert(sender_state != NULL);
    sender_state->predecessor = senderState;
    senderState = sender_state;
}

Packet::SenderState *
Packet::popSenderState()
{
    assert(senderState != NULL);
    SenderState *sender_state = senderState;
    senderState = sender_state->predecessor;
    sender_state->predecessor = NULL;
    return sender_state;
}

uint64_t
Packet::getUintX(ByteOrder endian) const
{
    switch(getSize()) {
      case 1:
        return (uint64_t)get<uint8_t>(endian);
      case 2:
        return (uint64_t)get<uint16_t>(endian);
      case 4:
        return (uint64_t)get<uint32_t>(endian);
      case 8:
        return (uint64_t)get<uint64_t>(endian);
      default:
        panic("%i isn't a supported word size.\n", getSize());
    }
}

void
Packet::setUintX(uint64_t w, ByteOrder endian)
{
    switch(getSize()) {
      case 1:
        set<uint8_t>((uint8_t)w, endian);
        break;
      case 2:
        set<uint16_t>((uint16_t)w, endian);
        break;
      case 4:
        set<uint32_t>((uint32_t)w, endian);
        break;
      case 8:
        set<uint64_t>((uint64_t)w, endian);
        break;
      default:
        panic("%i isn't a supported word size.\n", getSize());
    }

}

void
Packet::print(std::ostream &o, const int verbosity,
              const std::string &prefix) const
{
    ccprintf(o, "%s%s [%x:%x]%s%s%s%s%s%s", prefix, cmdString(),
             getAddr(), getAddr() + getSize() - 1,
             req->isSecure() ? " (s)" : "",
             req->isInstFetch() ? " IF" : "",
             req->isUncacheable() ? " UC" : "",
             isExpressSnoop() ? " ES" : "",
             req->isToPOC() ? " PoC" : "",
             req->isToPOU() ? " PoU" : "");
}

std::string
Packet::print() const {
    std::ostringstream str;
    print(str);
    return str.str();
}

Packet::PrintReqState::PrintReqState(std::ostream &_os, int _verbosity)
    : curPrefixPtr(new std::string("")), os(_os), verbosity(_verbosity)
{
    labelStack.push_back(LabelStackEntry("", curPrefixPtr));
}

Packet::PrintReqState::~PrintReqState()
{
    labelStack.pop_back();
    assert(labelStack.empty());
    delete curPrefixPtr;
}

Packet::PrintReqState::
LabelStackEntry::LabelStackEntry(const std::string &_label,
                                 std::string *_prefix)
    : label(_label), prefix(_prefix), labelPrinted(false)
{
}

void
Packet::PrintReqState::pushLabel(const std::string &lbl,
                                 const std::string &prefix)
{
    labelStack.push_back(LabelStackEntry(lbl, curPrefixPtr));
    curPrefixPtr = new std::string(*curPrefixPtr);
    *curPrefixPtr += prefix;
}

void
Packet::PrintReqState::popLabel()
{
    delete curPrefixPtr;
    curPrefixPtr = labelStack.back().prefix;
    labelStack.pop_back();
    assert(!labelStack.empty());
}

void
Packet::PrintReqState::printLabels()
{
    if (!labelStack.back().labelPrinted) {
        LabelStack::iterator i = labelStack.begin();
        LabelStack::iterator end = labelStack.end();
        while (i != end) {
            if (!i->labelPrinted) {
                ccprintf(os, "%s%s\n", *(i->prefix), i->label);
                i->labelPrinted = true;
            }
            i++;
        }
    }
}


void
Packet::PrintReqState::printObj(Printable *obj)
{
    printLabels();
    obj->print(os, verbosity, curPrefix());
}
