/*
 * Copyright (c) 2013-2017 ARM Limited
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
 * Authors: Stephan Diestelhorst
 */

/**
 * @file
 * Implementation of a snoop filter.
 */

#include "mem/snoop_filter.hh"
#include "mem/cache/base.hh"
#include "mem/cache/prefetch_filter/pref_info.hh"
#include "mem/cache/prefetch_filter/debug_flag.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/SnoopFilter.hh"
#include "sim/system.hh"

void
SnoopFilter::eraseIfNullEntry(SnoopFilterCache::iterator& sf_it,
        const bool forceDelete, const bool forceNotDelete)
{
    /// 不能同时强制保留和删除
    assert(!(forceDelete && forceNotDelete));
    SnoopItem& sf_item = sf_it->second;
    if ((!(sf_item.requested | sf_item.holder) || forceDelete) &&
            !forceNotDelete) {
        DEBUG_MEM("SnoopFilter[%p] erase entry @0x%lx", this, sf_it->first);
        cachedLocations.erase(sf_it);
        DPRINTF(SnoopFilter, "%s:   Removed SF entry.\n",
                __func__);
    }
}

std::pair<SnoopFilter::SnoopList, Cycles>
SnoopFilter::lookupRequest(const Packet* cpkt, const SlavePort& slave_port,
        bool* success)
{
    DPRINTF(SnoopFilter, "%s: src %s packet %s\n", __func__,
            slave_port.name(), cpkt->print());

    /// 标记当前处理的是否是一个提升级别的预取
    const bool isLevelUpPref = cpkt->packetType_ == prefetch_filter::Pref &&
            cpkt->recentCache_->cacheLevel_ <= cpkt->srcCacheLevel_;
    
    if (isLevelUpPref && cpkt->isResponse()) {
        panic_if(!success, "Success flag must be provided for level up "
                "prefetch process");
        *success = true;
    }
    // check if the packet came from a cache
    /// 如果是一个提升级别的预取，调用可以当作请求使用
    bool allocate = !cpkt->req->isUncacheable() && slave_port.isSnooping() &&
        (cpkt->fromCache() || isLevelUpPref);
    Addr line_addr = cpkt->getBlockAddr(linesize);
    if (cpkt->isSecure()) {
        line_addr |= LineSecure;
    }
    SnoopMask req_port = portToMask(slave_port);
    reqLookupResult = cachedLocations.find(line_addr);
    bool is_hit = (reqLookupResult != cachedLocations.end());

    // If the snoop filter has no entry, and we should not allocate,
    // do not create a new snoop filter entry, simply return a NULL
    // portlist.
    if (!is_hit && !allocate) {
        return snoopDown(lookupLatency);
    }

    // If no hit in snoop filter create a new element and update iterator
    if (!is_hit) {
        if (isLevelUpPref) {
            DEBUG_MEM("SnoopFilter[%p] add entry @0x%lx for level-up prefetch",
                    this, line_addr);
        } else {
            DEBUG_MEM("SnoopFilter[%p] add entry @0x%lx", this, line_addr);
        }
        reqLookupResult = cachedLocations.emplace(line_addr, SnoopItem()).first;
    } else {
        if (reqLookupResult->second.isLevelUpPref_) {
            /// 如果发现已经存在一个提级预取的记录，那么会删除旧记录
            DEBUG_MEM("SnoopFilter[%p] add new demand entry and replace "
                    "level-up prefetch @0x%lx ", this, line_addr);
            reqLookupResult = cachedLocations.emplace(line_addr,
                    SnoopItem()).first;
        } else {
            if (isLevelUpPref) {
                DEBUG_MEM("SnoopFilter[%p] cancle adding new entry for "
                        "level-up prefetch @0x%lx ", this, line_addr);
                /// 如果发现已经存在了一个其他记录，当前是一个提级预取
                /// 那么对应的Packet将不会上传
                *success = false;
                return std::pair<SnoopList, Cycles>();
            }
            DEBUG_MEM("SnoopFilter[%p] entry @0x%lx already exists",
                    this, line_addr);
        }
    }
    SnoopItem& sf_item = reqLookupResult->second;
    /// 如果是一个提升等级的预取调用该操作，那么设置标志
    sf_item.isLevelUpPref_ = isLevelUpPref;
    SnoopMask interested = sf_item.holder | sf_item.requested;

    // Store unmodified value of snoop filter item in temp storage in
    // case we need to revert because of a send retry in
    // updateRequest.
    retryItem = sf_item;

    totRequests++;
    if (is_hit) {
        // Single bit set -> value is a power of two
        if (isPow2(interested))
            hitSingleRequests++;
        else
            hitMultiRequests++;
    }

    DPRINTF(SnoopFilter, "%s:   SF value %x.%x\n",
            __func__, sf_item.requested, sf_item.holder);

    // If we are not allocating, we are done
    if (!allocate)
        return snoopSelected(maskToPortList(interested & ~req_port),
                             lookupLatency);
    
    /// 如果是一个提级预取，可以模仿一个正常的请求
    if (cpkt->needsResponse() || isLevelUpPref) {
        if (!cpkt->cacheResponding() || isLevelUpPref) {
            // Max one request per address per port
            panic_if(sf_item.requested & req_port, "double request :( " \
                     "SF value %x.%x\n", sf_item.requested, sf_item.holder);

            // Mark in-flight requests to distinguish later on
            sf_item.requested |= req_port;
            DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
                    __func__,  sf_item.requested, sf_item.holder);
        } else {
            // NOTE: The memInhibit might have been asserted by a cache closer
            // to the CPU, already -> the response will not be seen by this
            // filter -> we do not need to keep the in-flight request, but make
            // sure that we know that that cluster has a copy
            panic_if(!(sf_item.holder & req_port), "Need to hold the value!");
            DPRINTF(SnoopFilter,
                    "%s: not marking request. SF value %x.%x\n",
                    __func__,  sf_item.requested, sf_item.holder);
        }
    } else { // if (!cpkt->needsResponse())
        assert(cpkt->isEviction());
        // make sure that the sender actually had the line
        panic_if(!(sf_item.holder & req_port), "requester %x is not a " \
                 "holder :( SF value %x.%x\n", req_port,
                 sf_item.requested, sf_item.holder);
        // CleanEvicts and Writebacks -> the sender and all caches above
        // it may not have the line anymore.
        if (!cpkt->isBlockCached()) {
            sf_item.holder &= ~req_port;
            DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
                    __func__,  sf_item.requested, sf_item.holder);
        }
    }

    return snoopSelected(maskToPortList(interested & ~req_port), lookupLatency);
}

void
SnoopFilter::finishRequest(bool will_retry, Addr addr, bool is_secure,
        const bool forceDelete, const bool forceNotDelete)
{
    /// 不能同时强制保留和删除
    assert(!(forceDelete && forceNotDelete));
    if (reqLookupResult != cachedLocations.end()) {
        // since we rely on the caller, do a basic check to ensure
        // that finishRequest is being called following lookupRequest
        Addr line_addr = (addr & ~(Addr(linesize - 1)));
        if (is_secure) {
            line_addr |= LineSecure;
        }
        assert(reqLookupResult->first == line_addr);
        if (will_retry) {
            // Undo any changes made in lookupRequest to the snoop filter
            // entry if the request will come again. retryItem holds
            // the previous value of the snoopfilter entry.
            reqLookupResult->second = retryItem;

            DPRINTF(SnoopFilter, "%s:   restored SF value %x.%x\n",
                    __func__,  retryItem.requested, retryItem.holder);
        }

        eraseIfNullEntry(reqLookupResult, forceDelete, forceNotDelete);
    }
}

std::pair<SnoopFilter::SnoopList, Cycles>
SnoopFilter::lookupSnoop(const Packet* cpkt)
{
    DPRINTF(SnoopFilter, "%s: packet %s\n", __func__, cpkt->print());

    assert(cpkt->isRequest());

    Addr line_addr = cpkt->getBlockAddr(linesize);
    if (cpkt->isSecure()) {
        line_addr |= LineSecure;
    }
    auto sf_it = cachedLocations.find(line_addr);
    bool is_hit = (sf_it != cachedLocations.end());

    panic_if(!is_hit && (cachedLocations.size() >= maxEntryCount),
             "snoop filter exceeded capacity of %d cache blocks\n",
             maxEntryCount);

    // If the snoop filter has no entry, simply return a NULL
    // portlist, there is no point creating an entry only to remove it
    // later
    if (!is_hit)
        return snoopDown(lookupLatency);

    SnoopItem& sf_item = sf_it->second;

    DPRINTF(SnoopFilter, "%s:   old SF value %x.%x\n",
            __func__, sf_item.requested, sf_item.holder);

    SnoopMask interested = (sf_item.holder | sf_item.requested);

    totSnoops++;
    // Single bit set -> value is a power of two
    if (isPow2(interested))
        hitSingleSnoops++;
    else
        hitMultiSnoops++;

    // ReadEx and Writes require both invalidation and exlusivity, while reads
    // require neither. Writebacks on the other hand require exclusivity but
    // not the invalidation. Previously Writebacks did not generate upward
    // snoops so this was never an issue. Now that Writebacks generate snoops
    // we need a special case for Writebacks. Additionally cache maintenance
    // operations can generate snoops as they clean and/or invalidate all
    // caches down to the specified point of reference.
    assert(cpkt->isWriteback() || cpkt->req->isUncacheable() ||
           (cpkt->isInvalidate() == cpkt->needsWritable()) ||
           cpkt->req->isCacheMaintenance());
    if (cpkt->isInvalidate() && !sf_item.requested) {
        // Early clear of the holder, if no other request is currently going on
        // @todo: This should possibly be updated even though we do not filter
        // upward snoops
        sf_item.holder = 0;
    }

    eraseIfNullEntry(sf_it);
    DPRINTF(SnoopFilter, "%s:   new SF value %x.%x interest: %x \n",
            __func__, sf_item.requested, sf_item.holder, interested);

    return snoopSelected(maskToPortList(interested), lookupLatency);
}

void
SnoopFilter::updateSnoopResponse(const Packet* cpkt,
                                 const SlavePort& rsp_port,
                                 const SlavePort& req_port)
{
    DPRINTF(SnoopFilter, "%s: rsp %s req %s packet %s\n",
            __func__, rsp_port.name(), req_port.name(), cpkt->print());

    assert(cpkt->isResponse());
    assert(cpkt->cacheResponding());

    // if this snoop response is due to an uncacheable request, or is
    // being turned into a normal response, there is nothing more to
    // do
    if (cpkt->req->isUncacheable() || !req_port.isSnooping()) {
        return;
    }

    Addr line_addr = cpkt->getBlockAddr(linesize);
    if (cpkt->isSecure()) {
        line_addr |= LineSecure;
    }
    SnoopMask rsp_mask = portToMask(rsp_port);
    SnoopMask req_mask = portToMask(req_port);
    SnoopItem& sf_item = cachedLocations[line_addr];

    DPRINTF(SnoopFilter, "%s:   old SF value %x.%x\n",
            __func__,  sf_item.requested, sf_item.holder);

    // The source should have the line
    panic_if(!(sf_item.holder & rsp_mask), "SF value %x.%x does not have "\
             "the line\n", sf_item.requested, sf_item.holder);

    // The destination should have had a request in
    panic_if(!(sf_item.requested & req_mask), "SF value %x.%x missing "\
             "the original request\n",  sf_item.requested, sf_item.holder);

    // If the snoop response has no sharers the line is passed in
    // Modified state, and we know that there are no other copies, or
    // they will all be invalidated imminently
    if (!cpkt->hasSharers()) {
        DPRINTF(SnoopFilter,
                "%s: dropping %x because non-shared snoop "
                "response SF val: %x.%x\n", __func__,  rsp_mask,
                sf_item.requested, sf_item.holder);
        sf_item.holder = 0;
    }
    assert(!cpkt->isWriteback());
    // @todo Deal with invalidating responses
    sf_item.holder |=  req_mask;
    sf_item.requested &= ~req_mask;
    assert(sf_item.requested | sf_item.holder);
    DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
            __func__, sf_item.requested, sf_item.holder);
}

void
SnoopFilter::updateSnoopForward(const Packet* cpkt,
        const SlavePort& rsp_port, const MasterPort& req_port)
{
    DPRINTF(SnoopFilter, "%s: rsp %s req %s packet %s\n",
            __func__, rsp_port.name(), req_port.name(), cpkt->print());

    assert(cpkt->isResponse());
    assert(cpkt->cacheResponding());

    Addr line_addr = cpkt->getBlockAddr(linesize);
    if (cpkt->isSecure()) {
        line_addr |= LineSecure;
    }
    auto sf_it = cachedLocations.find(line_addr);
    bool is_hit = sf_it != cachedLocations.end();

    // Nothing to do if it is not a hit
    if (!is_hit)
        return;

    SnoopItem& sf_item = sf_it->second;

    DPRINTF(SnoopFilter, "%s:   old SF value %x.%x\n",
            __func__,  sf_item.requested, sf_item.holder);

    // If the snoop response has no sharers the line is passed in
    // Modified state, and we know that there are no other copies, or
    // they will all be invalidated imminently
    if (!cpkt->hasSharers()) {
        sf_item.holder = 0;
    }
    DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
            __func__, sf_item.requested, sf_item.holder);
    eraseIfNullEntry(sf_it);

}

void
SnoopFilter::updateResponse(const Packet* cpkt, const SlavePort& slave_port)
{
    DPRINTF(SnoopFilter, "%s: src %s packet %s\n",
            __func__, slave_port.name(), cpkt->print());

    assert(cpkt->isResponse());

    // we only allocate if the packet actually came from a cache, but
    // start by checking if the port is snooping
    if (cpkt->req->isUncacheable() || !slave_port.isSnooping())
        return;

    // next check if we actually allocated an entry
    Addr line_addr = cpkt->getBlockAddr(linesize);
    if (cpkt->isSecure()) {
        line_addr |= LineSecure;
    }
    auto sf_it = cachedLocations.find(line_addr);
    if (sf_it == cachedLocations.end())
        return;

    SnoopMask slave_mask = portToMask(slave_port);
    SnoopItem& sf_item = sf_it->second;

    DPRINTF(SnoopFilter, "%s:   old SF value %x.%x\n",
            __func__,  sf_item.requested, sf_item.holder);

    // Make sure we have seen the actual request, too
    panic_if(!(sf_item.requested & slave_mask), "SF value %x.%x missing "\
             "request bit\n", sf_item.requested, sf_item.holder);

    sf_item.requested &= ~slave_mask;
    // Update the residency of the cache line.

    if (cpkt->req->isCacheMaintenance()) {
        // A cache clean response does not carry any data so it
        // shouldn't change the holders, unless it is invalidating.
        if (cpkt->isInvalidate()) {
            sf_item.holder &= ~slave_mask;
        }
        eraseIfNullEntry(sf_it);
    } else {
        // Any other response implies that a cache above will have the
        // block.
        sf_item.holder |= slave_mask;
        assert(sf_item.holder | sf_item.requested);
    }
    DPRINTF(SnoopFilter, "%s:   new SF value %x.%x\n",
            __func__, sf_item.requested, sf_item.holder);
}

void
SnoopFilter::regStats()
{
    SimObject::regStats();

    totRequests
        .name(name() + ".tot_requests")
        .desc("Total number of requests made to the snoop filter.");

    hitSingleRequests
        .name(name() + ".hit_single_requests")
        .desc("Number of requests hitting in the snoop filter with a single "\
              "holder of the requested data.");

    hitMultiRequests
        .name(name() + ".hit_multi_requests")
        .desc("Number of requests hitting in the snoop filter with multiple "\
              "(>1) holders of the requested data.");

    totSnoops
        .name(name() + ".tot_snoops")
        .desc("Total number of snoops made to the snoop filter.");

    hitSingleSnoops
        .name(name() + ".hit_single_snoops")
        .desc("Number of snoops hitting in the snoop filter with a single "\
              "holder of the requested data.");

    hitMultiSnoops
        .name(name() + ".hit_multi_snoops")
        .desc("Number of snoops hitting in the snoop filter with multiple "\
              "(>1) holders of the requested data.");
}

SnoopFilter *
SnoopFilterParams::create()
{
    return new SnoopFilter(this);
}
