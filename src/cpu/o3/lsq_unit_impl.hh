
/*
 * Copyright (c) 2010-2014, 2017 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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
 * Authors: Kevin Lim
 *          Korey Sewell
 */

#ifndef __CPU_O3_LSQ_UNIT_IMPL_HH__
#define __CPU_O3_LSQ_UNIT_IMPL_HH__

#include "arch/generic/debugfaults.hh"
#include "arch/locked_mem.hh"
#include "base/str.hh"
#include "config/the_isa.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/o3/lsq.hh"
#include "cpu/o3/lsq_unit.hh"
#include "debug/Activity.hh"
#include "debug/IEW.hh"
#include "debug/LSQUnit.hh"
#include "debug/O3PipeView.hh"
#include "mem/packet.hh"
#include "mem/request.hh"

template<class Impl>
LSQUnit<Impl>::WritebackEvent::WritebackEvent(DynInstPtr &_inst, PacketPtr _pkt,
                                              LSQUnit *lsq_ptr)
    : Event(Default_Pri, AutoDelete),
      inst(_inst), pkt(_pkt), lsqPtr(lsq_ptr)
{
}

template<class Impl>
void
LSQUnit<Impl>::WritebackEvent::process()
{
    assert(!lsqPtr->cpu->switchedOut());

    lsqPtr->writeback(inst, pkt);

    if (pkt->senderState)
        delete pkt->senderState;

    delete pkt->req;
    delete pkt;
}

template<class Impl>
const char *
LSQUnit<Impl>::WritebackEvent::description() const
{
    return "Store writeback";
}

template<class Impl>
void
LSQUnit<Impl>::completeDataAccess(PacketPtr pkt)
{
    LSQSenderState *state = dynamic_cast<LSQSenderState *>(pkt->senderState);
    DynInstPtr inst = state->inst;
    DPRINTF(IEW, "Writeback event [sn:%lli].\n", inst->seqNum);
    DPRINTF(Activity, "Activity: Writeback event [sn:%lli].\n", inst->seqNum);

    if (state->cacheBlocked) {
        // This is the first half of a previous split load,
        // where the 2nd half blocked, ignore this response
        DPRINTF(IEW, "[sn:%lli]: Response from first half of earlier "
                "blocked split load recieved. Ignoring.\n", inst->seqNum);
        delete state;
        return;
		// 对于拆分后的访问，只完成了一半的情况，后半部分因为cache blocked
		// 失败的情况，直接忽视Packet请求返回的内容并返回
    }

    // If this is a split access, wait until all packets are received.
    if (TheISA::HasUnalignedMemAcc && !state->complete()) {
        return;
    }
	// 如果这是一个拆分的非对齐内存访问，那么此处可能完成的是第一个Packet
	// 或者第二个Packet，如果两个Packet没有都完成，此处就会直接返回

    assert(!cpu->switchedOut());
	
    if (!inst->isSquashed()) {
		// 只有发出Packet的指令没有被squash才会进行下面的处理
		if (!state->noWB) {
            // Only loads and store conditionals perform the writeback
            // after receving the response from the memory
            // 该标志位在Packet处理过程中设置，只有load和SC操作在处理Packet后
			// 会执行对寄存器的写回处理
			assert(inst->isLoad() || inst->isStoreConditional());
            if (!TheISA::HasUnalignedMemAcc || !state->isSplit ||
                !state->isLoad) {
                writeback(inst, pkt);
				// 如果是没有拆分的访存操作或者SC，直接将本Packet写回即可
				// 因为对于这些指令，参数pkt即main Packet
            } else {
                writeback(inst, state->mainPkt);
				// 而对于拆分访存操作，能够到达这里，说明两个minor Packet
				// 均已经完成并将结果汇总到了mainPacket中，这里直接
				// 写回main Packet的结果即可
            }
        }

        if (inst->isStore()) {
            completeStore(state->idx);
        }
		// 对于其他store指令直接将对应表项从SQ删除即可
    }

    if (TheISA::HasUnalignedMemAcc && state->isSplit && state->isLoad) {
        delete state->mainPkt->req;
        delete state->mainPkt;
    }

    pkt->req->setAccessLatency();
    cpu->ppDataAccessComplete->notify(std::make_pair(inst, pkt));
	// 主Packet会保留并且在设置访问延迟之后，记录在CPU的Probe point中
	
    delete state;
	// 删除LSQSenderState
}

template <class Impl>
LSQUnit<Impl>::LSQUnit()
    : loads(0), stores(0), storesToWB(0), cacheBlockMask(0), stalled(false),
      isStoreBlocked(false), storeInFlight(false), hasPendingPkt(false),
      pendingPkt(nullptr)
{
}

template<class Impl>
void
LSQUnit<Impl>::init(O3CPU *cpu_ptr, IEW *iew_ptr, DerivO3CPUParams *params,
        LSQ *lsq_ptr, unsigned maxLQEntries, unsigned maxSQEntries,
        unsigned id)
{
    cpu = cpu_ptr;
    iewStage = iew_ptr;
    lsq = lsq_ptr;
    lsqID = id;

    DPRINTF(LSQUnit, "Creating LSQUnit%i object.\n",id);

    // Add 1 for the sentinel entry (they are circular queues).
    LQEntries = maxLQEntries + 1;
    SQEntries = maxSQEntries + 1;

    //Due to uint8_t index in LSQSenderState
    assert(LQEntries <= 256);
    assert(SQEntries <= 256);
	// 因为索引的数据类型限制，LQ和SQ的大小都不得超过256

    loadQueue.resize(LQEntries);
    storeQueue.resize(SQEntries);

    depCheckShift = params->LSQDepCheckShift;
    checkLoads = params->LSQCheckLoads;
    cacheStorePorts = params->cacheStorePorts;
    needsTSO = params->needsTSO;

    resetState();
}


template<class Impl>
void
LSQUnit<Impl>::resetState()
{
    loads = stores = storesToWB = 0;

    loadHead = loadTail = 0;

    storeHead = storeWBIdx = storeTail = 0;

    usedStorePorts = 0;

    retryPkt = NULL;
    memDepViolator = NULL;

    stalled = false;

    cacheBlockMask = ~(cpu->cacheLineSize() - 1);
}

template<class Impl>
std::string
LSQUnit<Impl>::name() const
{
    if (Impl::MaxThreads == 1) {
        return iewStage->name() + ".lsq";
    } else {
        return iewStage->name() + ".lsq.thread" + std::to_string(lsqID);
    }
}

template<class Impl>
void
LSQUnit<Impl>::regStats()
{
    lsqForwLoads
        .name(name() + ".forwLoads")
        .desc("Number of loads that had data forwarded from stores");

    invAddrLoads
        .name(name() + ".invAddrLoads")
        .desc("Number of loads ignored due to an invalid address");

    lsqSquashedLoads
        .name(name() + ".squashedLoads")
        .desc("Number of loads squashed");

    lsqIgnoredResponses
        .name(name() + ".ignoredResponses")
        .desc("Number of memory responses ignored because the instruction is squashed");

    lsqMemOrderViolation
        .name(name() + ".memOrderViolation")
        .desc("Number of memory ordering violations");

    lsqSquashedStores
        .name(name() + ".squashedStores")
        .desc("Number of stores squashed");

    invAddrSwpfs
        .name(name() + ".invAddrSwpfs")
        .desc("Number of software prefetches ignored due to an invalid address");

    lsqBlockedLoads
        .name(name() + ".blockedLoads")
        .desc("Number of blocked loads due to partial load-store forwarding");

    lsqRescheduledLoads
        .name(name() + ".rescheduledLoads")
        .desc("Number of loads that were rescheduled");

    lsqCacheBlocked
        .name(name() + ".cacheBlocked")
        .desc("Number of times an access to memory failed due to the cache being blocked");
}

template<class Impl>
void
LSQUnit<Impl>::setDcachePort(MasterPort *dcache_port)
{
    dcachePort = dcache_port;
}

template<class Impl>
void
LSQUnit<Impl>::clearLQ()
{
    loadQueue.clear();
}

template<class Impl>
void
LSQUnit<Impl>::clearSQ()
{
    storeQueue.clear();
}

template<class Impl>
void
LSQUnit<Impl>::drainSanityCheck() const
{
    for (int i = 0; i < loadQueue.size(); ++i)
        assert(!loadQueue[i]);

    assert(storesToWB == 0);
    assert(!retryPkt);
}

template<class Impl>
void
LSQUnit<Impl>::takeOverFrom()
{
    resetState();
}

template<class Impl>
void
LSQUnit<Impl>::resizeLQ(unsigned size)
{
    unsigned size_plus_sentinel = size + 1;
    assert(size_plus_sentinel >= LQEntries);

    if (size_plus_sentinel > LQEntries) {
        while (size_plus_sentinel > loadQueue.size()) {
            DynInstPtr dummy;
            loadQueue.push_back(dummy);
            LQEntries++;
        }
    } else {
        LQEntries = size_plus_sentinel;
    }
	// 通过向原本的vector中push空指令，增加size
	// 最后更新LQEntries

    assert(LQEntries <= 256);
}

template<class Impl>
void
LSQUnit<Impl>::resizeSQ(unsigned size)
{
    unsigned size_plus_sentinel = size + 1;
    if (size_plus_sentinel > SQEntries) {
        while (size_plus_sentinel > storeQueue.size()) {
            SQEntry dummy;
            storeQueue.push_back(dummy);
            SQEntries++;
        }
    } else {
        SQEntries = size_plus_sentinel;
    }

    assert(SQEntries <= 256);
}

template <class Impl>
void
LSQUnit<Impl>::insert(DynInstPtr &inst)
{
    assert(inst->isMemRef());

    assert(inst->isLoad() || inst->isStore());

    if (inst->isLoad()) {
        insertLoad(inst);
    } else {
        insertStore(inst);
    }

    inst->setInLSQ();
}

template <class Impl>
void
LSQUnit<Impl>::insertLoad(DynInstPtr &load_inst)
{
    assert((loadTail + 1) % LQEntries != loadHead);
    assert(loads < LQEntries);

    DPRINTF(LSQUnit, "Inserting load PC %s, idx:%i [sn:%lli]\n",
            load_inst->pcState(), loadTail, load_inst->seqNum);

    load_inst->lqIdx = loadTail;

    if (stores == 0) {
        load_inst->sqIdx = -1;
    } else {
        load_inst->sqIdx = storeTail;
    }
	// 记录当前SQ尾部指令的索引记录到指令中，sqIdx变量用于判断前递
	// 关系，因为load不可能从在它之后进入SQ的store指令获取前递数据

    loadQueue[loadTail] = load_inst;

    incrLdIdx(loadTail);

    ++loads;
}

template <class Impl>
void
LSQUnit<Impl>::insertStore(DynInstPtr &store_inst)
{
    // Make sure it is not full before inserting an instruction.
    assert((storeTail + 1) % SQEntries != storeHead);
    assert(stores < SQEntries);

    DPRINTF(LSQUnit, "Inserting store PC %s, idx:%i [sn:%lli]\n",
            store_inst->pcState(), storeTail, store_inst->seqNum);

    store_inst->sqIdx = storeTail;
    store_inst->lqIdx = loadTail;

    storeQueue[storeTail] = SQEntry(store_inst);

    incrStIdx(storeTail);

    ++stores;
}

template <class Impl>
typename Impl::DynInstPtr
LSQUnit<Impl>::getMemDepViolator()
{
    DynInstPtr temp = memDepViolator;

    memDepViolator = NULL;

    return temp;
}

template <class Impl>
unsigned
LSQUnit<Impl>::numFreeLoadEntries()
{
        //LQ has an extra dummy entry to differentiate
        //empty/full conditions. Subtract 1 from the free entries.
        DPRINTF(LSQUnit, "LQ size: %d, #loads occupied: %d\n", LQEntries, loads);
        return LQEntries - loads - 1;
}

template <class Impl>
unsigned
LSQUnit<Impl>::numFreeStoreEntries()
{
        //SQ has an extra dummy entry to differentiate
        //empty/full conditions. Subtract 1 from the free entries.
        DPRINTF(LSQUnit, "SQ size: %d, #stores occupied: %d\n", SQEntries, stores);
        return SQEntries - stores - 1;

 }

template <class Impl>
void
LSQUnit<Impl>::checkSnoop(PacketPtr pkt)
{
    // Should only ever get invalidations in here
    assert(pkt->isInvalidate());

    int load_idx = loadHead;
    DPRINTF(LSQUnit, "Got snoop for address %#x\n", pkt->getAddr());

    // Only Invalidate packet calls checkSnoop
    assert(pkt->isInvalidate());
    for (int x = 0; x < cpu->numContexts(); x++) {
        ThreadContext *tc = cpu->getContext(x);
        bool no_squash = cpu->thread[x]->noSquashFromTC;
        cpu->thread[x]->noSquashFromTC = true;
		// 短暂设置noSquashFromTC为true，防止处理期间会被
		// 线程上下文的squash操作打断
        TheISA::handleLockedSnoop(tc, pkt, cacheBlockMask);
		// 执行snoop的上锁操作？？？
        cpu->thread[x]->noSquashFromTC = no_squash;
		// 恢复noSquashFromTC
    }

    Addr invalidate_addr = pkt->getAddr() & cacheBlockMask;
	// 确定需要invalidate数据所在的cache block索引，可以确定
	// 这里使用的snoop协议处理粒度也是一个cache block

    DynInstPtr ld_inst = loadQueue[load_idx];
	// 获取当前LQ中最老的load指令
	
    if (ld_inst) {
        Addr load_addr_low = ld_inst->physEffAddrLow & cacheBlockMask;
        Addr load_addr_high = ld_inst->physEffAddrHigh & cacheBlockMask;

        // Check that this snoop didn't just invalidate our lock flag
        if (ld_inst->effAddrValid() && (load_addr_low == invalidate_addr
                                        || load_addr_high == invalidate_addr)
            && ld_inst->memReqFlags & Request::LLSC)
            TheISA::handleLockedSnoopHit(ld_inst.get());
		// 如果LQ最新的指令处理的数据范围有效，并且处于被invlidate的
		// cache block中，且属于LLSC，那么处理该load指令，怎么处理呢？？？
    }
	// 为什么LQ最老的指令要进行特殊的处理呢？？？

    // If this is the only load in the LSQ we don't care
    if (load_idx == loadTail)
        return;
	// 如果该指令是LQ唯一的load指令，那么上面的处理就足够了，可以返回了

    incrLdIdx(load_idx);

    bool force_squash = false;
	// 循环遍历剩余的指令
    while (load_idx != loadTail) {
        DynInstPtr ld_inst = loadQueue[load_idx];

        if (!ld_inst->effAddrValid() || ld_inst->strictlyOrdered()) {
			// 如果该load指令的目标地址是无效的，或者该指令是一个严格
			// 按序执行指令（因为严格按序的话，现在不应该处理，到时候
			// 轮到该指令的时候对应内容也已经成功invalidate了），则跳
			// 过该指令
            incrLdIdx(load_idx);
            continue;
        }

        Addr load_addr_low = ld_inst->physEffAddrLow & cacheBlockMask;
        Addr load_addr_high = ld_inst->physEffAddrHigh & cacheBlockMask;

        DPRINTF(LSQUnit, "-- inst [sn:%lli] load_addr: %#x to pktAddr:%#x\n",
                    ld_inst->seqNum, load_addr_low, invalidate_addr);

        if ((load_addr_low == invalidate_addr
             || load_addr_high == invalidate_addr) || force_squash) {
			// 如果该load指令的目标地址位于invalidate的block内或者
			// 由于需要实现TSO系统（要求load按序执行），而之前已经有
			// load指令被squash了，那么比他更新的load指令都将被squash
            if (needsTSO) {
                // If we have a TSO system, as all loads must be ordered with
                // all other loads, this load as well as *all* subsequent loads
                // need to be squashed to prevent possible load reordering.
                force_squash = true;
				// 如果该ISA需要遵守TSO模型，那么该load被squash之后
				// 所有更新的load都需要squash，因此将force_squash
				// 设置为true
            }
            if (ld_inst->possibleLoadViolation() || force_squash) {
                DPRINTF(LSQUnit, "Conflicting load at addr %#x [sn:%lli]\n",
                        pkt->getAddr(), ld_inst->seqNum);

                // Mark the load for re-execution
                ld_inst->fault = std::make_shared<ReExec>();
				// 检查发现该load可能存在访存顺序violation，那么不会
				// 急于将该load指令squash，而是尝试将它重新执行
            } else {
                DPRINTF(LSQUnit, "HitExternal Snoop for addr %#x [sn:%lli]\n",
                        pkt->getAddr(), ld_inst->seqNum);

                // Make sure that we don't lose a snoop hitting a LOCKED
                // address since the LOCK* flags don't get updated until
                // commit.
                if (ld_inst->memReqFlags & Request::LLSC)
                    TheISA::handleLockedSnoopHit(ld_inst.get());
				// 如果该指令是一个LLSC，那么按照前面类似的方法处理

                // If an older load checks this and it's true
                // then we might have missed the snoop
                // in which case we need to invalidate to be sure
                ld_inst->hitExternalSnoop(true);
				// 这里会标记该load指令包含了snoop协议要求的invalidate操作
				// 同时存在指令和该指令操作区域一致，需要squash；这里仅作
				// 标记，实际的squash处理在checkViolation扫描时进行
            }
        }
        incrLdIdx(load_idx);
    }
    return;
}

template <class Impl>
Fault
LSQUnit<Impl>::checkViolations(int load_idx, DynInstPtr &inst)
{
    Addr inst_eff_addr1 = inst->effAddr >> depCheckShift;
    Addr inst_eff_addr2 = (inst->effAddr + inst->effSize - 1) >> depCheckShift;
	// 获取需要检查指令的处理数据范围
	
    /** @todo in theory you only need to check an instruction that has executed
     * however, there isn't a good way in the pipeline at the moment to check
     * all instructions that will execute before the store writes back. Thus,
     * like the implementation that came before it, we're overly conservative.
     */
	// 从理论上来说你只需要检查一个已经执行的指令，但是实际上并没有一个检查
	// 所有可能在一个store指令写回之前执行的指令的方法，因此正如前面的实现
	// 一样，这样做是过度保守的？？？
	
	// 遍历整个LQ中的指令，进行相关检查
    while (load_idx != loadTail) {
        DynInstPtr ld_inst = loadQueue[load_idx];
        if (!ld_inst->effAddrValid() || ld_inst->strictlyOrdered()) {
            incrLdIdx(load_idx);
			// 这里不检查目标地址无效或者严格按序执行的指令；严格按序不会出
			// 错是因为，该load指令只有在之前所有指令完成才会发射，所以该指
			// 令和load之间的顺序一定是不会导致violation
            continue;
        }

        Addr ld_eff_addr1 = ld_inst->effAddr >> depCheckShift;
        Addr ld_eff_addr2 =
            (ld_inst->effAddr + ld_inst->effSize - 1) >> depCheckShift;
		// 获得当前检查load指令的目标地址范围

        if (inst_eff_addr2 >= ld_eff_addr1 && inst_eff_addr1 <= ld_eff_addr2){
			// 进入这里说明两个指令的处理数据区域有交叠
            if (inst->isLoad()) {
                // If this load is to the same block as an external snoop
                // invalidate that we've observed then the load needs to be
                // squashed as it could have newer data
				
				// 一般来说两个load指令之间不应该有什么问题，但是此处指出的
				// 问题来自于snoopy协议中的invalidate操作，在执行checkSnoop
				// 检查的时候发现新的load尝试读取被invalidate区域的数据，那
				// 么出现invalidate的指令会被标记，错误的load指令在本函数才
				// 真正的被处理
				
                if (ld_inst->hitExternalSnoop()) {
                    if (!memDepViolator ||
                            ld_inst->seqNum < memDepViolator->seqNum) {
						// 如果当前发现尚未检查到目标指令和一个load指令的
						// violation，或者检查到的violation导致的squash操
						// 作并未波及到当前的load指令
                        DPRINTF(LSQUnit, "Detected fault with inst [sn:%lli] "
                                "and [sn:%lli] at address %#x\n",
                                inst->seqNum, ld_inst->seqNum, ld_eff_addr1);
                        memDepViolator = ld_inst;
						// 虽然这里是因为snoopy协议invalidate操作导致的
						// load指令squash操作，但是却被当作一个violation
						// 处理了，从实际效果来看没影响，但是对store-set
						// 的访存操作相关性预测是否有影响呢？？？

                        ++lsqMemOrderViolation;

                        return std::make_shared<GenericISA::M5PanicFault>(
                            "Detected fault with inst [sn:%lli] and "
                            "[sn:%lli] at address %#x\n",
                            inst->seqNum, ld_inst->seqNum, ld_eff_addr1);
						// 返回一个violation相关错误
                    }
                }				
				
                // Otherwise, mark the load has a possible load violation
                // and if we see a snoop before it's commited, we need to squash
                ld_inst->possibleLoadViolation(true);
				// 当前的虽然是load指令，但是目前没有找到需要处理的violation
				// 但是能够对一个load指令调用checkViolation，可以确定的是
				// 该load指令一定携带了snoopy协议的invalidate操作，因此还是
				// 需要执行一些squash操作。
				
                DPRINTF(LSQUnit, "Found possible load violation at addr: %#x"
                        " between instructions [sn:%lli] and [sn:%lli]\n",
                        inst_eff_addr1, inst->seqNum, ld_inst->seqNum);
            } else {
				// 如果目标指令是一个store指令，那么进行一般的violation检查
                // A load/store incorrectly passed this store.
                // Check if we already have a violator, or if it's newer
                // squash and refetch.
				
                if (memDepViolator && ld_inst->seqNum > memDepViolator->seqNum)
                    break;
				// 如果已经发现了产生violator的指令并且当前检查load已经处于
				// 其squash的指令范围中，那么剩余的load指令一定也处于squash
				// 范围内，无需继续检查。

                DPRINTF(LSQUnit, "Detected fault with inst [sn:%lli] and "
                        "[sn:%lli] at address %#x\n",
                        inst->seqNum, ld_inst->seqNum, ld_eff_addr1);
                memDepViolator = ld_inst;

                ++lsqMemOrderViolation;

                return std::make_shared<GenericISA::M5PanicFault>(
                    "Detected fault with "
                    "inst [sn:%lli] and [sn:%lli] at address %#x\n",
                    inst->seqNum, ld_inst->seqNum, ld_eff_addr1);
				// 如果这是第一个检测到violation的指令，那么记录对应指令
				// 无需继续检查直接返回。
            }
        }

        incrLdIdx(load_idx);
    }
    return NoFault;
	// 从checkViolation规则来看，如果checkViolation内部发现已检测
	// 到violation，那么一定是之前调用checkViolation发现的，而不是
	// 本次调用发现的，因为发现以后会立即返回！
}

template <class Impl>
Fault
LSQUnit<Impl>::executeLoad(DynInstPtr &inst)
{
    using namespace TheISA;
    // Execute a specific load.
    Fault load_fault = NoFault;

    DPRINTF(LSQUnit, "Executing load PC %s, [sn:%lli]\n",
            inst->pcState(), inst->seqNum);

    assert(!inst->isSquashed());

    load_fault = inst->initiateAcc();
	// 一般来说load指令可以分成两部分，一部分是访存，另一部分是访存数据
	// 写回寄存器操作，这里执行的是第一部分

    if (inst->isTranslationDelayed() &&
        load_fault == NoFault)
        return load_fault;
	// 如果因为TLB miss则直接返回，不进行第二部分处理

    // If the instruction faulted or predicated false, then we need to send it
    // along to commit without the instruction completing.
	// 如果访存操作出现错误，或者预测到了错误，那么这个错误需要
	// 在该指令完成之前发送到commit阶段进行处理
    if (load_fault != NoFault || !inst->readPredicate()) {
        // Send this instruction to commit, also make sure iew stage
        // realizes there is activity.  Mark it as executed unless it
        // is a strictly ordered load that needs to hit the head of
        // commit.
        if (!inst->readPredicate()) inst->forwardOldRegs();
		// ？？？
        DPRINTF(LSQUnit, "Load [sn:%lli] not executed from %s\n",
                inst->seqNum,
                (load_fault != NoFault ? "fault" : "predication"));
        if (!(inst->hasRequest() && inst->strictlyOrdered()) ||
            inst->isAtCommit()) {
            inst->setExecuted();
			// 如果该指令尚未发出请求并且不是一个严格按序执行的指令
			// 或者？？？则设置为已经执行完毕的状态
        }
        iewStage->instToCommit(inst);
        iewStage->activityThisCycle();
		// 指令会被立即交给commit阶段
    } else {
        assert(inst->effAddrValid());
        int load_idx = inst->lqIdx;
        incrLdIdx(load_idx);
		// 这里增加load_idx，是为了将自己从violation检查的目标中剔除
		
        if (checkLoads)
            return checkViolations(load_idx, inst);
		// 如果load指令执行操作没有任何问题，那么进行violation检查
	}

    return load_fault;
}

template <class Impl>
Fault
LSQUnit<Impl>::executeStore(DynInstPtr &store_inst)
{
    using namespace TheISA;
    // Make sure that a store exists.
    assert(stores != 0);

    int store_idx = store_inst->sqIdx;

    DPRINTF(LSQUnit, "Executing store PC %s [sn:%lli]\n",
            store_inst->pcState(), store_inst->seqNum);

    assert(!store_inst->isSquashed());

    // Check the recently completed loads to see if any match this store's
    // address.  If so, then we have a memory ordering violation.
    int load_idx = store_inst->lqIdx;

    Fault store_fault = store_inst->initiateAcc();

    if (store_inst->isTranslationDelayed() &&
        store_fault == NoFault)
        return store_fault;

    if (!store_inst->readPredicate()) {
        DPRINTF(LSQUnit, "Store [sn:%lli] not executed from predication\n",
                store_inst->seqNum);
        store_inst->forwardOldRegs();
        return store_fault;
    }
	// 上面的处理和executeLoad类似

    if (storeQueue[store_idx].size == 0) {
        DPRINTF(LSQUnit,"Fault on Store PC %s, [sn:%lli], Size = 0\n",
                store_inst->pcState(), store_inst->seqNum);

        return store_fault;
    }
	// 这里处理的是store指令没有存放任何数据的情况

    assert(store_fault == NoFault);

    if (store_inst->isStoreConditional()) {
        // Store conditionals need to set themselves as able to
        // writeback if we haven't had a fault by here.
        storeQueue[store_idx].canWB = true;
		// 如果发现该store指令在没有延迟的情况下完成了读取操作，那么
		// 标记该指令为为canWB表示可以写回了
        ++storesToWB;
    }

    return checkViolations(load_idx, store_inst);
	// 执行完毕后进行violation的相关检查。

}

template <class Impl>
void
LSQUnit<Impl>::commitLoad()
{
    assert(loadQueue[loadHead]);

    DPRINTF(LSQUnit, "Committing head load instruction, PC %s\n",
            loadQueue[loadHead]->pcState());

    loadQueue[loadHead] = NULL;

    incrLdIdx(loadHead);

    --loads;
}

template <class Impl>
void
LSQUnit<Impl>::commitLoads(InstSeqNum &youngest_inst)
{
    assert(loads == 0 || loadQueue[loadHead]);

    while (loads != 0 && loadQueue[loadHead]->seqNum <= youngest_inst) {
        commitLoad();
    }
}

template <class Impl>
void
LSQUnit<Impl>::commitStores(InstSeqNum &youngest_inst)
{
    assert(stores == 0 || storeQueue[storeHead].inst);

    int store_idx = storeHead;

    while (store_idx != storeTail) {
        assert(storeQueue[store_idx].inst);
        // Mark any stores that are now committed and have not yet
        // been marked as able to write back.
        if (!storeQueue[store_idx].canWB) {
            if (storeQueue[store_idx].inst->seqNum > youngest_inst) {
                break;
            }
            DPRINTF(LSQUnit, "Marking store as able to write back, PC "
                    "%s [sn:%lli]\n",
                    storeQueue[store_idx].inst->pcState(),
                    storeQueue[store_idx].inst->seqNum);

            storeQueue[store_idx].canWB = true;

            ++storesToWB;
        }

        incrStIdx(store_idx);
    }
}

template <class Impl>
void
LSQUnit<Impl>::writebackPendingStore()
{
    if (hasPendingPkt) {
        assert(pendingPkt != NULL);

        // If the cache is blocked, this will store the packet for retry.
        if (sendStore(pendingPkt)) {
            storePostSend(pendingPkt);
			// 进入这里表示sendStore成功得到了处理
        }
		// 这里没有考虑是否依然阻塞的情况，只有一种可能，那就是该函数
		// 会在Cache port退出阻塞状态后优先执行，因此可以保证此处的
		// pendingPkt一定能够得到有效的处理
        pendingPkt = NULL;
        hasPendingPkt = false;
    }
}

template <class Impl>
void
LSQUnit<Impl>::writebackStores()
{
    // First writeback the second packet from any split store that didn't
    // complete last cycle because there weren't enough cache ports available.
    if (TheISA::HasUnalignedMemAcc) {
        writebackPendingStore();
    }
	// 首先尝试调用writebackPendingStore函数处理因为Cache阻塞没有完成写回的，
	// store指令的第二个minor Packet

    while (storesToWB > 0 &&
		// 这里要求存在可以写回的store指令
           storeWBIdx != storeTail &&
		// 其实本条件和上一个条件等效
           storeQueue[storeWBIdx].inst &&
		// 标记可以写回store指令的SQ表项有效
           storeQueue[storeWBIdx].canWB &&
		// 标记可以写回store指令的SQ表项canWB标志位有效
           ((!needsTSO) || (!storeInFlight)) &&
		// 当前不遵守TSO模型（要求store按序处理）或者当前没有in-flight的store
		// 指令，我们要明确一点所有in-flight的指令的canWB一定被设置且执行了
		// 本函数，因此没有in-flight指令可以保证执行当前store指令的writeback
		// 不会导致SQ中指令进行乱序的writeback
           usedStorePorts < cacheStorePorts) {
		// 当前拥有足够的Cache port执行store指令的写回操作
		
        if (isStoreBlocked) {
            DPRINTF(LSQUnit, "Unable to write back any more stores, cache"
                    " is blocked!\n");
            break;
        }
		// 发现当前Cache处于blocked状态，因此无法对更多的store指令进行写回

        // Store didn't write any data so no need to write it back to memory.
        if (storeQueue[storeWBIdx].size == 0) {
            completeStore(storeWBIdx);
            incrStIdx(storeWBIdx);
            continue;
        }
		// 对于不写任何数据的store指令，此处直接完成该指令即可

        ++usedStorePorts;
		// 先登记一个Cache port用于本store指令的写回

        if (storeQueue[storeWBIdx].inst->isDataPrefetch()) {
            incrStIdx(storeWBIdx);
            continue;
        }
		// 对于预取指令不需要进行写回处理

        assert(storeQueue[storeWBIdx].req);
        assert(!storeQueue[storeWBIdx].committed);

        if (TheISA::HasUnalignedMemAcc && storeQueue[storeWBIdx].isSplit) {
            assert(storeQueue[storeWBIdx].sreqLow);
            assert(storeQueue[storeWBIdx].sreqHigh);
        }

        DynInstPtr inst = storeQueue[storeWBIdx].inst;
		// 获取当前可以写回的指令
        Request *req = storeQueue[storeWBIdx].req;
        RequestPtr sreqLow = storeQueue[storeWBIdx].sreqLow;
        RequestPtr sreqHigh = storeQueue[storeWBIdx].sreqHigh;
		// 获取该指令的Request
        storeQueue[storeWBIdx].committed = true;
		// 设置该指令为已提交状态？？？

        assert(!inst->memData);
        inst->memData = new uint8_t[req->getSize()];
        if (storeQueue[storeWBIdx].isAllZeros)
            memset(inst->memData, 0, req->getSize());
        else
            memcpy(inst->memData, storeQueue[storeWBIdx].data, req->getSize());
		// 即使是store指令，也需要设置一个容器存放准备写入到mem中的数据
		// 这个数据被记录在SQ表项中的data中
		
        PacketPtr data_pkt;
        PacketPtr snd_data_pkt = NULL;

        LSQSenderState *state = new LSQSenderState;
        state->isLoad = false;
        state->idx = storeWBIdx;
        state->inst = inst;
		// store指令和load指令不同，在executeStore的时候没有真正的发送
		// Packet给Mem System进行处理，而是在writeBack阶段进行操作
		// 因此此处生成了一个LSQSenderState来记录Packet处理信息

        if (!TheISA::HasUnalignedMemAcc || !storeQueue[storeWBIdx].isSplit) {
            // Build a single data packet if the store isn't split.
            data_pkt = Packet::createWrite(req);
            data_pkt->dataStatic(inst->memData);
            data_pkt->senderState = state;
			// 对于无需拆分的对齐访存操作，只生成一个main Packet即可
        } else {
            // Create two packets if the store is split in two.
            data_pkt = Packet::createWrite(sreqLow);
            snd_data_pkt = Packet::createWrite(sreqHigh);

            data_pkt->dataStatic(inst->memData);
            snd_data_pkt->dataStatic(inst->memData + sreqLow->getSize());

            data_pkt->senderState = state;
            snd_data_pkt->senderState = state;

            state->isSplit = true;
            state->outstanding = 2;
			// 对于拆分的非对齐内存访问的处理
			
            // Can delete the main request now.
            delete req;
            req = sreqLow;
			// 删除main Request，保留两个minor Request
        }

        DPRINTF(LSQUnit, "D-Cache: Writing back store idx:%i PC:%s "
                "to Addr:%#x, data:%#x [sn:%lli]\n",
                storeWBIdx, inst->pcState(),
                req->getPaddr(), (int)*(inst->memData),
                inst->seqNum);

        // @todo: Remove this SC hack once the memory system handles it.
        if (inst->isStoreConditional()) {
            assert(!storeQueue[storeWBIdx].isSplit);
			// 从该assert语句来看，SC操作似乎必须是对齐内存访问
			
            // Disable recording the result temporarily.  Writing to
            // misc regs normally updates the result, but this is not
            // the desired behavior when handling store conditionals.
            inst->recordResult(false);
            bool success = TheISA::handleLockedWrite(inst.get(), req, cacheBlockMask);
            inst->recordResult(true);
			// 关于SC的处理依然不清楚？？？可能SC检查的flag和地址都存放在
			// MiscReg中

            if (!success) {
                // Instantly complete this store.
                DPRINTF(LSQUnit, "Store conditional [sn:%lli] failed.  "
                        "Instantly completing it.\n",
                        inst->seqNum);
                WritebackEvent *wb = new WritebackEvent(inst, data_pkt, this);
                // 生成SC指令需要的write back事件进行处理，SC的write back主要
				// 是对SC处理结果状态对应寄存器的写操作
				cpu->schedule(wb, curTick() + 1);
				// 在下一个周期处理该SC的write back
                completeStore(storeWBIdx);
                incrStIdx(storeWBIdx);
                continue;
				// SC指令成功得到了处理就会继续处理下一条指令
            }
        } else {
            // Non-store conditionals do not need a writeback.
            state->noWB = true;
			// 对于非SC类指令在完成了写Mem操作后就算完成了，无需进行额外的写回操作
        }

        bool split =
            TheISA::HasUnalignedMemAcc && storeQueue[storeWBIdx].isSplit;

        ThreadContext *thread = cpu->tcBase(lsqID);

        if (req->isMmappedIpr()) {
            assert(!inst->isStoreConditional());
            TheISA::handleIprWrite(thread, data_pkt);
			// 对于映射到IO外设寄存器的内存地址访问，需要使用专门的函数
			// 因为这些内存是non cacheable的
            delete data_pkt;
            if (split) {
                assert(snd_data_pkt->req->isMmappedIpr());
                TheISA::handleIprWrite(thread, snd_data_pkt);
				// 如果是一个非对齐的内存访问，发出第二个minor packet
                delete snd_data_pkt;
                delete sreqLow;
                delete sreqHigh;
            }
            delete state;
            delete req;
			// 删除Request和LSQSenderState以及minor Packet
            completeStore(storeWBIdx);
            incrStIdx(storeWBIdx);
			// 完成该store指令，需要注意的是，这里并不考虑TLB miss产生的
			// delay问题，是因为non cacheable区域压根不会访问Cache，因此
			// 并不会走TLB
        } else if (!sendStore(data_pkt)) {
			// 如果不是固定寄存器映射的内存地址访问，那么这里会发出第一个
			// Packet，对于对齐内存访问，该Packet就是main packet
			
			// 进入到这里说明第一个Packet处理时遇到了阻塞问题，但是
			// 这个Packet并没有失效，他会被放到retryPkt中，在之后重试操作
			// 重试发送该Packet
            DPRINTF(IEW, "D-Cache became blocked when writing [sn:%lli], will"
                    "retry later\n",
                    inst->seqNum);

            // Need to store the second packet, if split.
            if (split) {
                state->pktToSend = true;
				// 设置该标记表示该Packet后面还有一个Packet需要处理
                state->pendingPacket = snd_data_pkt;
            }
			// 处理完第一个Packet之后，并不会在当前周期立即处理第二个minor
			// Packet，这个Packet会在进行writebackStores的时候执行
        } else {
			// 进入到这里只能说明第一个Packet执行过程没有出现问题

            // If split, try to send the second packet too
            // 如果存在第二个minor packet，那么尝试发送第二个packet
			if (split) {
                assert(snd_data_pkt);

                // Ensure there are enough ports to use.
                if (usedStorePorts < cacheStorePorts) {
					// 只有拥有足够的Store Cache Port才可以处理这个Packet
                    ++usedStorePorts;
                    if (sendStore(snd_data_pkt)) {
                        storePostSend(snd_data_pkt);
						// 如果处理成功则进行后续的处理
                    } else {
                        DPRINTF(IEW, "D-Cache became blocked when writing"
                                " [sn:%lli] second packet, will retry later\n",
                                inst->seqNum);
                    }
                } else {
					// 如果没有足够的Cache Port，那么将记录未完成的第二个
					// minor Packet，这时候
					
                    // Store the packet for when there's free ports.
                    assert(pendingPkt == NULL);
                    pendingPkt = snd_data_pkt;
                    hasPendingPkt = true;
                }
            } else {
                // Not a split store.
                storePostSend(data_pkt);
				// 如果没有额外的Packet处理，即非拆分的内存访问，那么
				// 进行后续处理即可
            }
        }
    }

    // Not sure this should set it to 0.
    usedStorePorts = 0;
	// 作者不确定，我也很怀疑这一点，首先可以确定每发出一个Packet
	// 就会使用一个Cache store port，但是什么时候取消占用呢？？？
	// 如果在完成的时候取消占用，那么这里绝对不能清0；如果在发送
	// 出去以后便不需要占用的话，这里的处理是合理的

    assert(stores >= 0 && storesToWB >= 0);
}

/*template <class Impl>
void
LSQUnit<Impl>::removeMSHR(InstSeqNum seqNum)
{
    list<InstSeqNum>::iterator mshr_it = find(mshrSeqNums.begin(),
                                              mshrSeqNums.end(),
                                              seqNum);

    if (mshr_it != mshrSeqNums.end()) {
        mshrSeqNums.erase(mshr_it);
        DPRINTF(LSQUnit, "Removing MSHR. count = %i\n",mshrSeqNums.size());
    }
}*/

template <class Impl>
void
LSQUnit<Impl>::squash(const InstSeqNum &squashed_num)
{
    DPRINTF(LSQUnit, "Squashing until [sn:%lli]!"
            "(Loads:%i Stores:%i)\n", squashed_num, loads, stores);
	
	// 下面这一部分是对LQ的squash操作，递减尾部指针load_idx
	// 使之指向最后一个存放有有效指令的位置
    int load_idx = loadTail;
    decrLdIdx(load_idx);

    while (loads != 0 && loadQueue[load_idx]->seqNum > squashed_num) {
		// 从尾部开始遍历整个LQ，寻找比给定编号更新的指令squash
        DPRINTF(LSQUnit,"Load Instruction PC %s squashed, "
                "[sn:%lli]\n",
                loadQueue[load_idx]->pcState(),
                loadQueue[load_idx]->seqNum);

        if (isStalled() && load_idx == stallingLoadIdx) {
            stalled = false;
            stallingStoreIsn = 0;
            stallingLoadIdx = 0;
        }
		// 如果squash的指令正好是stalled的load指令，那么stall相关的
		// 标志变量就会被重置

        // Clear the smart pointer to make sure it is decremented.
        loadQueue[load_idx]->setSquashed();
        loadQueue[load_idx] = NULL;
        --loads;
		// 清除LQ中该指令的存在
        // Inefficient!
        loadTail = load_idx;
		// 更新循环队列索引
        decrLdIdx(load_idx);
        ++lsqSquashedLoads;
    }

    if (memDepViolator && squashed_num < memDepViolator->seqNum) {
        memDepViolator = NULL;
    }
	// 如果发现产生访存violation的指令处于被squash的范围内，那么就
	// 不需要记录这个memDepViolator了，记录毕竟也是为了squash

    int store_idx = storeTail;
    decrStIdx(store_idx);

    while (stores != 0 &&
           storeQueue[store_idx].inst->seqNum > squashed_num) {
        // Instructions marked as can WB are already committed.
        if (storeQueue[store_idx].canWB) {
            break;
			// 对于已经生成了Req，设置了canWB的指令均不再squash
			// 我们对这个处理保留意见，因为不完全的squash是否会
			// 产生错误的结果呢？？？
        }

        DPRINTF(LSQUnit,"Store Instruction PC %s squashed, "
                "idx:%i [sn:%lli]\n",
                storeQueue[store_idx].inst->pcState(),
                store_idx, storeQueue[store_idx].inst->seqNum);

        // I don't think this can happen.  It should have been cleared
        // by the stalling load.
		// 上面说的有道理，但是如果只设置了stallingStoreIsn，那么可能会
		// 走到这里并进行下面的处理
        if (isStalled() &&
            storeQueue[store_idx].inst->seqNum == stallingStoreIsn) {
            panic("Is stalled should have been cleared by stalling load!\n");
            stalled = false;
            stallingStoreIsn = 0;
        }

        // Clear the smart pointer to make sure it is decremented.
        storeQueue[store_idx].inst->setSquashed();
        storeQueue[store_idx].inst = NULL;
        storeQueue[store_idx].canWB = 0;
		// 清除SQ的表项数据

        // Must delete request now that it wasn't handed off to
        // memory.  This is quite ugly.  @todo: Figure out the proper
        // place to really handle request deletes.
		
        delete storeQueue[store_idx].req;
        if (TheISA::HasUnalignedMemAcc && storeQueue[store_idx].isSplit) {
            delete storeQueue[store_idx].sreqLow;
            delete storeQueue[store_idx].sreqHigh;

            storeQueue[store_idx].sreqLow = NULL;
            storeQueue[store_idx].sreqHigh = NULL;
        }
		// 删除所有的Request并重置相应的表项

        storeQueue[store_idx].req = NULL;
        --stores;

        // Inefficient!
        storeTail = store_idx;

        decrStIdx(store_idx);
        ++lsqSquashedStores;
    }
}

template <class Impl>
void
LSQUnit<Impl>::storePostSend(PacketPtr pkt)
{
	// 根据调用的情况来看，这里处理的一定是刚刚执行了写回操作的指令
    if (isStalled() &&
        storeQueue[storeWBIdx].inst->seqNum == stallingStoreIsn) {
		// 如果当前存在stall指令且正好是第一个可以写回的store指令
		// 那么将会重新执行stall的load指令？？？
        DPRINTF(LSQUnit, "Unstalling, stalling store [sn:%lli] "
                "load idx:%i\n",
                stallingStoreIsn, stallingLoadIdx);
        stalled = false;
        stallingStoreIsn = 0;
        iewStage->replayMemInst(loadQueue[stallingLoadIdx]);
    }

    if (!storeQueue[storeWBIdx].inst->isStoreConditional()) {
        // The store is basically completed at this time. This
        // only works so long as the checker doesn't try to
        // verify the value in memory for stores.
        storeQueue[storeWBIdx].inst->setCompleted();

        if (cpu->checker) {
            cpu->checker->verify(storeQueue[storeWBIdx].inst);
        }
		// 对于非SC指令，可以直接设置该指令为完成状态，而SC由于还要
		// 写成功状态标志寄存器，故而还不能标记为完成状态
    }

    if (needsTSO) {
        storeInFlight = true;
    }
	// 为了保证store指令按序执行，此处会设置in-flight标记
	// 该标记会在执行completeStore函数的时候清除

    incrStIdx(storeWBIdx);
	// 更新最老的可以写回store指令索引
}

template <class Impl>
void
LSQUnit<Impl>::writeback(DynInstPtr &inst, PacketPtr pkt)
{
    iewStage->wakeCPU();

    // Squashed instructions do not need to complete their access.
    if (inst->isSquashed()) {
        assert(!inst->isStore());
        ++lsqIgnoredResponses;
        return;
    }
	// 跳过已经被标记为squash的指令

    if (!inst->isExecuted()) {
        inst->setExecuted();
		// 对于没有标记为已经执行的指令，标记为执行完毕，
		// 毕竟已经到了写回部分了

        if (inst->fault == NoFault) {
            // Complete access to copy data to proper place.
            inst->completeAcc(pkt);
			// 这里对于load指令来说是有意义的，即将load到的数据写回
			// 到寄存器中
        } else {
            // If the instruction has an outstanding fault, we cannot complete
            // the access as this discards the current fault.

            // If we have an outstanding fault, the fault should only be of
            // type ReExec.
            assert(dynamic_cast<ReExec*>(inst->fault.get()) != nullptr);

            DPRINTF(LSQUnit, "Not completing instruction [sn:%lli] access "
                    "due to pending fault.\n", inst->seqNum);
			// 发现指令执行出现了错误
		}
    }

    // Need to insert instruction into queue to commit
    iewStage->instToCommit(inst);
    iewStage->activityThisCycle();
	// 激活IEW阶段并将指令发送给commit阶段

    // see if this load changed the PC
    iewStage->checkMisprediction(inst);
	// 最后由IEW检查是否存在错误的分支一遍进行squash
}

template <class Impl>
void
LSQUnit<Impl>::completeStore(int store_idx)
{
    assert(storeQueue[store_idx].inst);
    storeQueue[store_idx].completed = true;
	// 标记该指令为已完成状态
    --storesToWB;
    // A bit conservative because a store completion may not free up entries,
    // but hopefully avoids two store completions in one cycle from making
    // the CPU tick twice.
    cpu->wakeCPU();
    cpu->activityThisCycle();
	// 激活CPU以便完成store的相关操作

    if (store_idx == storeHead) {
        do {
            incrStIdx(storeHead);

            --stores;
        } while (storeQueue[storeHead].completed &&
                 storeHead != storeTail);
		// 尝试将store队列头部靠前连续的已经完成的指令，通过更新
		// SQ顶端索引的方式从SQ中删除
        iewStage->updateLSQNextCycle = true;
		// 标记表示下一个周期LSQ会进行更新
    }

    DPRINTF(LSQUnit, "Completing store [sn:%lli], idx:%i, store head "
            "idx:%i\n",
            storeQueue[store_idx].inst->seqNum, store_idx, storeHead);

#if TRACING_ON
    if (DTRACE(O3PipeView)) {
        storeQueue[store_idx].inst->storeTick =
            curTick() - storeQueue[store_idx].inst->fetchTick;
		// 如果开启了debug模式，则记录存储指令完成的相对tick位置
    }
#endif

    if (isStalled() &&
        storeQueue[store_idx].inst->seqNum == stallingStoreIsn) {
        DPRINTF(LSQUnit, "Unstalling, stalling store [sn:%lli] "
                "load idx:%i\n",
                stallingStoreIsn, stallingLoadIdx);
        stalled = false;
        stallingStoreIsn = 0;
        iewStage->replayMemInst(loadQueue[stallingLoadIdx]);
    }
	// 如果当前存在stall指令且正好是第一个可以写回的store指令
	// 那么将会重新执行stall的load指令？？？

    storeQueue[store_idx].inst->setCompleted();
	// 将该指令内设置为完成状态

    if (needsTSO) {
        storeInFlight = false;
    }
	// 由于之前进行writeback的指令已经完成，重置storeInFlight以便
	// LSQ可以处理下一个指令

    // Tell the checker we've completed this instruction.  Some stores
    // may get reported twice to the checker, but the checker can
    // handle that case.

    // Store conditionals cannot be sent to the checker yet, they have
    // to update the misc registers first which should take place
    // when they commit
	// SC指令由于写回的MiscReg寄存器，因此当且仅当SC指令被提交的
	// 时候才能完成真正的写回操作
    if (cpu->checker && !storeQueue[store_idx].inst->isStoreConditional()) {
        cpu->checker->verify(storeQueue[store_idx].inst);
    }
}

template <class Impl>
bool
LSQUnit<Impl>::sendStore(PacketPtr data_pkt)
{
    if (!dcachePort->sendTimingReq(data_pkt)) {
        // Need to handle becoming blocked on a store.
		// 返回0表示该Packet因为阻塞的原因不能立即完成
        isStoreBlocked = true;
        ++lsqCacheBlocked;
        assert(retryPkt == NULL);
        retryPkt = data_pkt;
		// 没有成功处理的Packet，会被放到retryPkt，在之后recvRetry
		// 函数中发送出去
        return false;
    }
    return true;
}

template <class Impl>
void
LSQUnit<Impl>::recvRetry()
{
    if (isStoreBlocked) {
		// 如果当前存在retryPkt，那么会设置isStoreBlocked（仅在设置retryPkt
		// 的时候会设置为true），因此只有当该标志设置的时候才有必要进行
		// retryPkt的处理
        DPRINTF(LSQUnit, "Receiving retry: store blocked\n");
        assert(retryPkt != NULL);

        LSQSenderState *state =
            dynamic_cast<LSQSenderState *>(retryPkt->senderState);

        if (dcachePort->sendTimingReq(retryPkt)) {
			// 进入这里表示retryPkt得到了处理
            // Don't finish the store unless this is the last packet.
            if (!TheISA::HasUnalignedMemAcc || !state->pktToSend ||
                    state->pendingPacket == retryPkt) {
				// 如果该指令
                state->pktToSend = false;
                storePostSend(retryPkt);
            }
            retryPkt = NULL;
            isStoreBlocked = false;

            // Send any outstanding packet.
            if (TheISA::HasUnalignedMemAcc && state->pktToSend) {
                assert(state->pendingPacket);
                if (sendStore(state->pendingPacket)) {
                    storePostSend(state->pendingPacket);
                }
            }
        } else {
            // Still blocked!
            ++lsqCacheBlocked;
			// 进入这里表示目前依然没有退出阻塞状态，下次依然需要
			// 调用recvRetry处理retryPkt
        }
    }
}

template <class Impl>
inline void
LSQUnit<Impl>::incrStIdx(int &store_idx) const
{
    if (++store_idx >= SQEntries)
        store_idx = 0;
}

template <class Impl>
inline void
LSQUnit<Impl>::decrStIdx(int &store_idx) const
{
    if (--store_idx < 0)
        store_idx += SQEntries;
}

template <class Impl>
inline void
LSQUnit<Impl>::incrLdIdx(int &load_idx) const
{
    if (++load_idx >= LQEntries)
        load_idx = 0;
}

template <class Impl>
inline void
LSQUnit<Impl>::decrLdIdx(int &load_idx) const
{
    if (--load_idx < 0)
        load_idx += LQEntries;
}

template <class Impl>
void
LSQUnit<Impl>::dumpInsts() const
{
    cprintf("Load store queue: Dumping instructions.\n");
    cprintf("Load queue size: %i\n", loads);
    cprintf("Load queue: ");

    int load_idx = loadHead;

    while (load_idx != loadTail && loadQueue[load_idx]) {
        const DynInstPtr &inst(loadQueue[load_idx]);
        cprintf("%s.[sn:%i] ", inst->pcState(), inst->seqNum);

        incrLdIdx(load_idx);
    }
    cprintf("\n");

    cprintf("Store queue size: %i\n", stores);
    cprintf("Store queue: ");

    int store_idx = storeHead;

    while (store_idx != storeTail && storeQueue[store_idx].inst) {
        const DynInstPtr &inst(storeQueue[store_idx].inst);
        cprintf("%s.[sn:%i] ", inst->pcState(), inst->seqNum);

        incrStIdx(store_idx);
    }

    cprintf("\n");
}

#endif//__CPU_O3_LSQ_UNIT_IMPL_HH__
