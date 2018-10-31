/*
 * Copyright (c) 2010-2014 ARM Limited
 * Copyright (c) 2012-2013 AMD
 * All rights reserved.
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
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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

#ifndef __CPU_O3_FETCH_IMPL_HH__
#define __CPU_O3_FETCH_IMPL_HH__

#include <algorithm>
#include <cstring>
#include <list>
#include <map>
#include <queue>

#include "arch/generic/tlb.hh"
#include "arch/isa_traits.hh"
#include "arch/utility.hh"
#include "arch/vtophys.hh"
#include "base/random.hh"
#include "base/types.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
//#include "cpu/checker/cpu.hh"
#include "cpu/o3/fetch.hh"
#include "cpu/exetrace.hh"
#include "debug/Activity.hh"
#include "debug/Drain.hh"
#include "debug/Fetch.hh"
#include "debug/O3PipeView.hh"
#include "mem/packet.hh"
#include "params/DerivO3CPU.hh"
#include "sim/byteswap.hh"
#include "sim/core.hh"
#include "sim/eventq.hh"
#include "sim/full_system.hh"
#include "sim/system.hh"
#include "cpu/o3/isa_specific.hh"

using namespace std;

template<class Impl>
DefaultFetch<Impl>::DefaultFetch(O3CPU *_cpu, DerivO3CPUParams *params)
    : cpu(_cpu),
      decodeToFetchDelay(params->decodeToFetchDelay),
      renameToFetchDelay(params->renameToFetchDelay),
      iewToFetchDelay(params->iewToFetchDelay),
      commitToFetchDelay(params->commitToFetchDelay),
      fetchWidth(params->fetchWidth),
      decodeWidth(params->decodeWidth),
      retryPkt(NULL),
      retryTid(InvalidThreadID),
      cacheBlkSize(cpu->cacheLineSize()),
      fetchBufferSize(params->fetchBufferSize),
      fetchBufferMask(fetchBufferSize - 1),
      fetchQueueSize(params->fetchQueueSize),
      numThreads(params->numThreads),
      numFetchingThreads(params->smtNumFetchingThreads),
      finishTranslationEvent(this)
{
    if (numThreads > Impl::MaxThreads)
        fatal("numThreads (%d) is larger than compiled limit (%d),\n"
              "\tincrease MaxThreads in src/cpu/o3/impl.hh\n",
              numThreads, static_cast<int>(Impl::MaxThreads));
    if (fetchWidth > Impl::MaxWidth)
        fatal("fetchWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/impl.hh\n",
             fetchWidth, static_cast<int>(Impl::MaxWidth));
    if (fetchBufferSize > cacheBlkSize)
        fatal("fetch buffer size (%u bytes) is greater than the cache "
              "block size (%u bytes)\n", fetchBufferSize, cacheBlkSize);
    if (cacheBlkSize % fetchBufferSize)
        fatal("cache block (%u bytes) is not a multiple of the "
              "fetch buffer (%u bytes)\n", cacheBlkSize, fetchBufferSize);
	// 对数据大小进行检查

    std::string policy = params->smtFetchPolicy;

    // Convert string to lowercase
    std::transform(policy.begin(), policy.end(), policy.begin(),
                   (int(*)(int)) tolower);

    // Figure out fetch policy
    if (policy == "singlethread") {
        fetchPolicy = SingleThread;
        if (numThreads > 1)
            panic("Invalid Fetch Policy for a SMT workload.");
    } else if (policy == "roundrobin") {
        fetchPolicy = RoundRobin;
        DPRINTF(Fetch, "Fetch policy set to Round Robin\n");
    } else if (policy == "branch") {
        fetchPolicy = Branch;
        DPRINTF(Fetch, "Fetch policy set to Branch Count\n");
    } else if (policy == "iqcount") {
        fetchPolicy = IQ;
        DPRINTF(Fetch, "Fetch policy set to IQ count\n");
    } else if (policy == "lsqcount") {
        fetchPolicy = LSQ;
        DPRINTF(Fetch, "Fetch policy set to LSQ count\n");
    } else {
        fatal("Invalid Fetch Policy. Options Are: {SingleThread,"
              " RoundRobin,LSQcount,IQcount}\n");
    }
	// 将params给出的policy字符串转换成小写并进行相应的设置

    // Get the size of an instruction.
    instSize = sizeof(TheISA::MachInst);

    for (int i = 0; i < Impl::MaxThreads; i++) {
        decoder[i] = NULL;
        fetchBuffer[i] = NULL;
        fetchBufferPC[i] = 0;
        fetchBufferValid[i] = false;
    }

    branchPred = params->branchPred;

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        decoder[tid] = new TheISA::Decoder(params->isa[tid]);
        // Create space to buffer the cache line data,
        // which may not hold the entire cache line.
        fetchBuffer[tid] = new uint8_t[fetchBufferSize];
    }
}

template <class Impl>
std::string
DefaultFetch<Impl>::name() const
{
    return cpu->name() + ".fetch";
}

template <class Impl>
void
DefaultFetch<Impl>::regProbePoints()
{
    ppFetch = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(), "Fetch");
    ppFetchRequestSent = new ProbePointArg<RequestPtr>(cpu->getProbeManager(),
                                                       "FetchRequest");

}

template <class Impl>
void
DefaultFetch<Impl>::regStats()
{
    icacheStallCycles
        .name(name() + ".icacheStallCycles")
        .desc("Number of cycles fetch is stalled on an Icache miss")
        .prereq(icacheStallCycles);

    fetchedInsts
        .name(name() + ".Insts")
        .desc("Number of instructions fetch has processed")
        .prereq(fetchedInsts);

    fetchedBranches
        .name(name() + ".Branches")
        .desc("Number of branches that fetch encountered")
        .prereq(fetchedBranches);

    predictedBranches
        .name(name() + ".predictedBranches")
        .desc("Number of branches that fetch has predicted taken")
        .prereq(predictedBranches);

    fetchCycles
        .name(name() + ".Cycles")
        .desc("Number of cycles fetch has run and was not squashing or"
              " blocked")
        .prereq(fetchCycles);

    fetchSquashCycles
        .name(name() + ".SquashCycles")
        .desc("Number of cycles fetch has spent squashing")
        .prereq(fetchSquashCycles);

    fetchTlbCycles
        .name(name() + ".TlbCycles")
        .desc("Number of cycles fetch has spent waiting for tlb")
        .prereq(fetchTlbCycles);

    fetchIdleCycles
        .name(name() + ".IdleCycles")
        .desc("Number of cycles fetch was idle")
        .prereq(fetchIdleCycles);

    fetchBlockedCycles
        .name(name() + ".BlockedCycles")
        .desc("Number of cycles fetch has spent blocked")
        .prereq(fetchBlockedCycles);

    fetchedCacheLines
        .name(name() + ".CacheLines")
        .desc("Number of cache lines fetched")
        .prereq(fetchedCacheLines);

    fetchMiscStallCycles
        .name(name() + ".MiscStallCycles")
        .desc("Number of cycles fetch has spent waiting on interrupts, or "
              "bad addresses, or out of MSHRs")
        .prereq(fetchMiscStallCycles);

    fetchPendingDrainCycles
        .name(name() + ".PendingDrainCycles")
        .desc("Number of cycles fetch has spent waiting on pipes to drain")
        .prereq(fetchPendingDrainCycles);

    fetchNoActiveThreadStallCycles
        .name(name() + ".NoActiveThreadStallCycles")
        .desc("Number of stall cycles due to no active thread to fetch from")
        .prereq(fetchNoActiveThreadStallCycles);

    fetchPendingTrapStallCycles
        .name(name() + ".PendingTrapStallCycles")
        .desc("Number of stall cycles due to pending traps")
        .prereq(fetchPendingTrapStallCycles);

    fetchPendingQuiesceStallCycles
        .name(name() + ".PendingQuiesceStallCycles")
        .desc("Number of stall cycles due to pending quiesce instructions")
        .prereq(fetchPendingQuiesceStallCycles);

    fetchIcacheWaitRetryStallCycles
        .name(name() + ".IcacheWaitRetryStallCycles")
        .desc("Number of stall cycles due to full MSHR")
        .prereq(fetchIcacheWaitRetryStallCycles);

    fetchIcacheSquashes
        .name(name() + ".IcacheSquashes")
        .desc("Number of outstanding Icache misses that were squashed")
        .prereq(fetchIcacheSquashes);

    fetchTlbSquashes
        .name(name() + ".ItlbSquashes")
        .desc("Number of outstanding ITLB misses that were squashed")
        .prereq(fetchTlbSquashes);

    fetchNisnDist
        .init(/* base value */ 0,
              /* last value */ fetchWidth,
              /* bucket size */ 1)
        .name(name() + ".rateDist")
        .desc("Number of instructions fetched each cycle (Total)")
        .flags(Stats::pdf);

    idleRate
        .name(name() + ".idleRate")
        .desc("Percent of cycles fetch was idle")
        .prereq(idleRate);
    idleRate = fetchIdleCycles * 100 / cpu->numCycles;

    branchRate
        .name(name() + ".branchRate")
        .desc("Number of branch fetches per cycle")
        .flags(Stats::total);
    branchRate = fetchedBranches / cpu->numCycles;

    fetchRate
        .name(name() + ".rate")
        .desc("Number of inst fetches per cycle")
        .flags(Stats::total);
    fetchRate = fetchedInsts / cpu->numCycles;
}

template<class Impl>
void
DefaultFetch<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *time_buffer)
{
    timeBuffer = time_buffer;

    // Create wires to get information from proper places in time buffer.
    fromDecode = timeBuffer->getWire(-decodeToFetchDelay);
    fromRename = timeBuffer->getWire(-renameToFetchDelay);
    fromIEW = timeBuffer->getWire(-iewToFetchDelay);
    fromCommit = timeBuffer->getWire(-commitToFetchDelay);
}

template<class Impl>
void
DefaultFetch<Impl>::setActiveThreads(std::list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;
}

template<class Impl>
void
DefaultFetch<Impl>::setFetchQueue(TimeBuffer<FetchStruct> *ftb_ptr)
{
    // Create wire to write information to proper place in fetch time buf.
    toDecode = ftb_ptr->getWire(0);
}

template<class Impl>
void
DefaultFetch<Impl>::startupStage()
{
    assert(priorityList.empty());
	// 要求线程列表不能为空
    resetStage();
    // Fetch needs to start fetching instructions at the very beginning,
    // so it must start up in active state.
    switchToActive();
}

template<class Impl>
void
DefaultFetch<Impl>::resetStage()
{
    numInst = 0;
    interruptPending = false;
    cacheBlocked = false;

    priorityList.clear();

    // Setup PC and nextPC with initial state.
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        fetchStatus[tid] = Running;
		// 线程状态被设置为Running
        pc[tid] = cpu->pcState(tid);
		// 设置对应线程的PC寄存器
        fetchOffset[tid] = 0;
        macroop[tid] = NULL;

        delayedCommit[tid] = false;
        memReq[tid] = NULL;

        stalls[tid].decode = false;
        stalls[tid].drain = false;

        fetchBufferPC[tid] = 0;
        fetchBufferValid[tid] = false;

        fetchQueue[tid].clear();
        priorityList.push_back(tid);
    }

    wroteToTimeBuffer = false;
    _status = Inactive;
}

template<class Impl>
void
DefaultFetch<Impl>::processCacheCompletion(PacketPtr pkt)
{
    ThreadID tid = cpu->contextToThread(pkt->req->contextId());
	// 找到当初发出Cache访问请求的线程ID

    DPRINTF(Fetch, "[tid:%u] Waking up from cache miss.\n", tid);
    assert(!cpu->switchedOut());

    // Only change the status if it's still waiting on the icache access
    // to return.
    if (fetchStatus[tid] != IcacheWaitResponse ||
        pkt->req != memReq[tid]) {
        ++fetchIcacheSquashes;
        delete pkt->req;
        delete pkt;
        return;
		// 如果反馈的线程和需求线程不一致，或者两者请求不一致
		// 则说明该请求出现了squash操作，因此删除该请求的影响
		// 注意此处并不清理memReq中数据，因为可能还会重试
    }
	// 如果当前线程确实正在等待自己的Cache访问请求结束，则进行后面的处理

    memcpy(fetchBuffer[tid], pkt->getConstPtr<uint8_t>(), fetchBufferSize);
    fetchBufferValid[tid] = true;
	// 将获取的数据存放到fetchBuffer中

    // Wake up the CPU (if it went to sleep and was waiting on
    // this completion event).
    cpu->wakeCPU();
	// 唤醒可能处于等待fetch获取完成的CPU
	// 如果CPU已经处于唤醒状态，则这里不会进行任何处理

    DPRINTF(Activity, "[tid:%u] Activating fetch due to cache completion\n",
            tid);

    switchToActive();
	// 将可能因为该事件导致fetch停止的fetch阶段切换到活跃状态

    // Only switch to IcacheAccessComplete if we're not stalled as well.
    if (checkStall(tid)) {
        fetchStatus[tid] = Blocked;
    } else {
        fetchStatus[tid] = IcacheAccessComplete;
    }
	// 如果当前线程处于stall状态，那么将进入Blocked状态，
	// 以便防止更多的ICache访问数据无处存放，因为fetchBuffer
	// 已经被本次的数据填满，而stall的线程无法处理掉这些数据

    pkt->req->setAccessLatency();
	// 设置访存延迟的统计数据
    cpu->ppInstAccessComplete->notify(pkt);
	// ？？？
    // Reset the mem req to NULL.
    delete pkt->req;
    delete pkt;
    memReq[tid] = NULL;
	// 删除相关的状态和记录，由于Icache访问成功因此memReq的纪录被清除
}

template <class Impl>
void
DefaultFetch<Impl>::drainResume()
{
    for (ThreadID i = 0; i < numThreads; ++i) {
        stalls[i].decode = false;
        stalls[i].drain = false;
    }
}

template <class Impl>
void
DefaultFetch<Impl>::drainSanityCheck() const
{
    assert(isDrained());
    assert(retryPkt == NULL);
	// 不存在需要重试的访存操作
    assert(retryTid == InvalidThreadID);
	// 不存在需要重试的线程
    assert(!cacheBlocked);
	// 这里要求当前不能处于阻塞状态
    assert(!interruptPending);
	// 当前不能存在尚未处理的中断

    for (ThreadID i = 0; i < numThreads; ++i) {
        assert(!memReq[i]);
		// 请求队列中不能存在访存请求
        assert(fetchStatus[i] == Idle || stalls[i].drain);
		// 要么当前fetch阶段空闲要么因为排空而停止
    }

    branchPred->drainSanityCheck();
}

template <class Impl>
bool
DefaultFetch<Impl>::isDrained() const
{
    /* Make sure that threads are either idle of that the commit stage
     * has signaled that draining has completed by setting the drain
     * stall flag. This effectively forces the pipeline to be disabled
     * until the whole system is drained (simulation may continue to
     * drain other components).
     */
    for (ThreadID i = 0; i < numThreads; ++i) {
        // Verify fetch queues are drained
        if (!fetchQueue[i].empty())
            return false;
		// 如果fetchQueue不为空，则尚未排空

        // Return false if not idle or drain stalled
        if (fetchStatus[i] != Idle) {
            if (fetchStatus[i] == Blocked && stalls[i].drain)
                continue;
            else
                return false;
        }
		// 如果一个线程的状态不处于空闲状态，那么当前线程必须处于Blocked状态
		// 同时是因为drain操作停止的
    }

    /* The pipeline might start up again in the middle of the drain
     * cycle if the finish translation event is scheduled, so make
     * sure that's not the case.
     */
    return !finishTranslationEvent.scheduled();
	// 确保排空操作期间不存在可以调度的fetch地址翻译操作
}

template <class Impl>
void
DefaultFetch<Impl>::takeOverFrom()
{
    assert(cpu->getInstPort().isConnected());
    resetStage();

}

template <class Impl>
void
DefaultFetch<Impl>::drainStall(ThreadID tid)
{
    assert(cpu->isDraining());
    assert(!stalls[tid].drain);
	// 此时相关标志一定没有被设置
    DPRINTF(Drain, "%i: Thread drained.\n", tid);
    stalls[tid].drain = true;
}

template <class Impl>
void
DefaultFetch<Impl>::wakeFromQuiesce()
{
    DPRINTF(Fetch, "Waking up from quiesce\n");
    // Hopefully this is safe
    // @todo: Allow other threads to wake from quiesce.
    fetchStatus[0] = Running;
}

template <class Impl>
inline void
DefaultFetch<Impl>::switchToActive()
{
    if (_status == Inactive) {
        DPRINTF(Activity, "Activating stage.\n");

        cpu->activateStage(O3CPU::FetchIdx);

        _status = Active;
    }
}

template <class Impl>
inline void
DefaultFetch<Impl>::switchToInactive()
{
    if (_status == Active) {
        DPRINTF(Activity, "Deactivating stage.\n");

        cpu->deactivateStage(O3CPU::FetchIdx);

        _status = Inactive;
    }
}

template <class Impl>
void
DefaultFetch<Impl>::deactivateThread(ThreadID tid)
{
    // Update priority list
    auto thread_it = std::find(priorityList.begin(), priorityList.end(), tid);
    if (thread_it != priorityList.end()) {
        priorityList.erase(thread_it);
    }
}

template <class Impl>
bool
DefaultFetch<Impl>::lookupAndUpdateNextPC(
        DynInstPtr &inst, TheISA::PCState &nextPC)
{
    // Do branch prediction check here.
    // A bit of a misnomer...next_PC is actually the current PC until
    // this function updates it.
    bool predict_taken;

    if (!inst->isControl()) {
        TheISA::advancePC(nextPC, inst->staticInst);
        inst->setPredTarg(nextPC);
        inst->setPredTaken(false);
		// 每个指令自身都包含了指令的分支信息
        return false;
    }
	// 指令并非分支指令则一定不存在分支

    ThreadID tid = inst->threadNumber;
    predict_taken = branchPred->predict(inst->staticInst, inst->seqNum,
                                        nextPC, tid);
	// 调用分支单元的predict预测分支指令是否被taken

    if (predict_taken) {
        DPRINTF(Fetch, "[tid:%i]: [sn:%i]:  Branch predicted to be taken to %s.\n",
                tid, inst->seqNum, nextPC);
    } else {
        DPRINTF(Fetch, "[tid:%i]: [sn:%i]:Branch predicted to be not taken.\n",
                tid, inst->seqNum);
    }

    DPRINTF(Fetch, "[tid:%i]: [sn:%i] Branch predicted to go to %s.\n",
            tid, inst->seqNum, nextPC);
    inst->setPredTarg(nextPC);
    inst->setPredTaken(predict_taken);
	// 在指令中设置分支相关的信息

    ++fetchedBranches;
	// 递增fetch阶段处理的分支指令数目计数器

    if (predict_taken) {
        ++predictedBranches;
		// 递增fetch阶段处理的分支指令被taken次数的计数器
    }

    return predict_taken;
}

template <class Impl>
bool
DefaultFetch<Impl>::fetchCacheLine(Addr vaddr, ThreadID tid, Addr pc)
{
    Fault fault = NoFault;

    assert(!cpu->switchedOut());
	// 如果CPU被切换出去了，那还取啥Cacheline

    // @todo: not sure if these should block translation.
    //AlphaDep
    if (cacheBlocked) {
        DPRINTF(Fetch, "[tid:%i] Can't fetch cache line, cache blocked\n",
                tid);
        return false;
		// 如果当前Cache处于阻塞状态，那么不允许新的Cache访问操作
    } else if (checkInterrupt(pc) && !delayedCommit[tid]) {
        // Hold off fetch from getting new instructions when:
        // Cache is blocked, or
        // while an interrupt is pending and we're not in PAL mode, or
        // fetch is switched out.
        DPRINTF(Fetch, "[tid:%i] Can't fetch cache line, interrupt pending\n",
                tid);
		// 当前存在需要处理的中断同时fetch不允许？？？
        return false;
    }

    // Align the fetch address to the start of a fetch buffer segment.
    Addr fetchBufferBlockPC = fetchBufferAlignPC(vaddr);

    DPRINTF(Fetch, "[tid:%i] Fetching cache line %#x for addr %#x\n",
            tid, fetchBufferBlockPC, vaddr);

    // Setup the memReq to do a read of the first instruction's address.
    // Set the appropriate read size and flags as well.
    // Build request here.
    RequestPtr mem_req =
        new Request(tid, fetchBufferBlockPC, fetchBufferSize,
                    Request::INST_FETCH, cpu->instMasterId(), pc,
                    cpu->thread[tid]->contextId());
	// Request类型包含了多个信息，包括pc地址和取数长度，请求类型等等
	
    mem_req->taskId(cpu->taskId());
    memReq[tid] = mem_req;
	// 生成一个新的访存请求，加入到mem_req中

    // Initiate translation of the icache block
    fetchStatus[tid] = ItlbWait;
	// 设置状态表示fetch阶段开始等待ITLB生成地址翻译结果
    FetchTranslation *trans = new FetchTranslation(this);
	// 生成一个新的取指地址翻译请求操作类
    cpu->itb->translateTiming(mem_req, cpu->thread[tid]->getTC(),
                              trans, BaseTLB::Execute);
	// 调用CPU的itb安排翻译处理，这里翻译存在延迟，因此并不会立即执行
    return true;
}

template <class Impl>
void
DefaultFetch<Impl>::finishTranslation(const Fault &fault, RequestPtr mem_req)
{
    ThreadID tid = cpu->contextToThread(mem_req->contextId());
    Addr fetchBufferBlockPC = mem_req->getVaddr();
	// 获取访存请求对应的线程id和fetchBuffer的PC地址

    assert(!cpu->switchedOut());

    // Wake up CPU if it was idle
    cpu->wakeCPU();
	// 尝试唤醒空闲状态的CPU

    if (fetchStatus[tid] != ItlbWait || mem_req != memReq[tid] ||
        mem_req->getVaddr() != memReq[tid]->getVaddr()) {
		// 如果发出翻译请求的线程并没有处于ItlbWait状态，或者
		// mem_req中记录的请求和当前处理的不符合、或者两者希望
		// 翻译的地址不一致，则将本次翻译结果squash掉
        DPRINTF(Fetch, "[tid:%i] Ignoring itlb completed after squash\n", tid);
        ++fetchTlbSquashes;
        delete mem_req;
		// 清除请求操作对象
        return;
    }
	// 发现翻译处理没有合适的线程来处理，故取消TLB的翻译影响


    // If translation was successful, attempt to read the icache block.
    if (fault == NoFault) {
		// 如果地址翻译成功则进行下面的处理
        // Check that we're not going off into random memory
        // If we have, just wait around for commit to squash something and put
        // us on the right track
        if (!cpu->system->isMemAddr(mem_req->getPaddr())) {
            warn("Address %#x is outside of physical memory, stopping fetch\n",
                    mem_req->getPaddr());
            fetchStatus[tid] = NoGoodAddr;
            delete mem_req;
            memReq[tid] = NULL;
            return;
        }
		// 首先判断物理地址的合法性，不合法的地址会导致线程状态变化
		// 同时请求也会被删除。

        // Build packet here.
        PacketPtr data_pkt = new Packet(mem_req, MemCmd::ReadReq);
        data_pkt->dataDynamic(new uint8_t[fetchBufferSize]);
		// 生成一个内存读取的Packet，指明请求和数据写入的地址。

        fetchBufferPC[tid] = fetchBufferBlockPC;
		// 设置fetchBuffer对应的虚拟起始地址
        fetchBufferValid[tid] = false;
		// 有效位初始化为false
        DPRINTF(Fetch, "Fetch: Doing instruction read.\n");

        fetchedCacheLines++;

        // Access the cache.
        if (!cpu->getInstPort().sendTimingReq(data_pkt)) {
			// 尝试向Icache的接口发送一个Packet对应的读写请求
            assert(retryPkt == NULL);
            assert(retryTid == InvalidThreadID);
			// 要求当前没有需要重试的Packet
            DPRINTF(Fetch, "[tid:%i] Out of MSHRs!\n", tid);

            fetchStatus[tid] = IcacheWaitRetry;
            retryPkt = data_pkt;
            retryTid = tid;
            cacheBlocked = true;
			// 如果发送处理请求失败，则设置状态和重试内容
			// 设置cacheBlocked防止生成新的内存访问请求
        } else {
            DPRINTF(Fetch, "[tid:%i]: Doing Icache access.\n", tid);
            DPRINTF(Activity, "[tid:%i]: Activity: Waiting on I-cache "
                    "response.\n", tid);
            lastIcacheStall[tid] = curTick();
            fetchStatus[tid] = IcacheWaitResponse;
			// 设置状态表示正在等待一个ICache访问处理
            // Notify Fetch Request probe when a packet containing a fetch
            // request is successfully sent
            ppFetchRequestSent->notify(mem_req);
			// 表明成功发送了ICache访问请求
        }
    } else {
		// 下面的处理是因为地址翻译过程出现了错误
        // Don't send an instruction to decode if we can't handle it.
        if (!(numInst < fetchWidth) || !(fetchQueue[tid].size() < fetchQueueSize)) {
            // 如果获取的指令大于取指的宽度限制或者
			// fetchQueue队列的实际大小已经超过了规定的大小上限
			assert(!finishTranslationEvent.scheduled());
			// 此时不能有可以调度的地址翻译请求
            finishTranslationEvent.setFault(fault);
            finishTranslationEvent.setReq(mem_req);
			// 记录错误类型和请求到内部类finishTranslationEvent中
            cpu->schedule(finishTranslationEvent,
                          cpu->clockEdge(Cycles(1)));
			// 由CPU对该错误进行调度处理，可见该错误导致了1个周期的延迟
            return;
        }
        DPRINTF(Fetch, "[tid:%i] Got back req with addr %#x but expected %#x\n",
                tid, mem_req->getVaddr(), memReq[tid]->getVaddr());
        // Translation faulted, icache request won't be sent.
        delete mem_req;
        memReq[tid] = NULL;
		// 删除访存请求的相关内容

        // Send the fault to commit.  This thread will not do anything
        // until commit handles the fault.  The only other way it can
        // wake up is if a squash comes along and changes the PC.
        TheISA::PCState fetchPC = pc[tid];
        DPRINTF(Fetch, "[tid:%i]: Translation faulted, building noop.\n", tid);
        // We will use a nop in ordier to carry the fault.
        DynInstPtr instruction = buildInst(tid, StaticInst::nopStaticInstPtr,
                                           NULL, fetchPC, fetchPC, false);
        instruction->setNotAnInst();
        instruction->setPredTarg(fetchPC);
        instruction->fault = fault;
		// 设置了一个空的nop指令还存放本次翻译的错误信息
        wroteToTimeBuffer = true;

        DPRINTF(Activity, "Activity this cycle.\n");
        cpu->activityThisCycle();
		// 通知CPU可以开始本周期的处理工作了

        fetchStatus[tid] = TrapPending;
		// 由于翻译出现了错误，设置状态表示正在处理这个错误对应的陷阱

        DPRINTF(Fetch, "[tid:%i]: Blocked, need to handle the trap.\n", tid);
        DPRINTF(Fetch, "[tid:%i]: fault (%s) detected @ PC %s.\n",
                tid, fault->name(), pc[tid]);
    }
    _status = updateFetchStatus();
	// 更新fetch阶段的状态
}

template <class Impl>
inline void
DefaultFetch<Impl>::doSquash(const TheISA::PCState &newPC,
                             const DynInstPtr squashInst, ThreadID tid)
{
    DPRINTF(Fetch, "[tid:%i]: Squashing, setting PC to: %s.\n",
            tid, newPC);

    pc[tid] = newPC;
    fetchOffset[tid] = 0;
	
    if (squashInst && squashInst->pcState().instAddr() == newPC.instAddr())
        macroop[tid] = squashInst->macroop;
    else
        macroop[tid] = NULL;
	// 如果希望squash的指令不为空并且newPC中的PC正好是该指令的PC
	// 则它的微操作将会被记录下来？？？
	// 否则macroop被重置
    decoder[tid]->reset();

    // Clear the icache miss if it's outstanding.
    if (fetchStatus[tid] == IcacheWaitResponse) {
        DPRINTF(Fetch, "[tid:%i]: Squashing outstanding Icache miss.\n",
                tid);
        memReq[tid] = NULL;
    } else if (fetchStatus[tid] == ItlbWait) {
        DPRINTF(Fetch, "[tid:%i]: Squashing outstanding ITLB miss.\n",
                tid);
        memReq[tid] = NULL;
    }
	// 如果当前线程的状态显示memReq存在一个请求，可以是
	// 地址翻译请求或者是一个ICache访问请求，那么将其重置

    // Get rid of the retrying packet if it was from this thread.
    if (retryTid == tid) {
        assert(cacheBlocked);
		// 这里判断其实只是想证实存在需要重试的访存请求时
		// cacheBlocked一定被设置为true
        if (retryPkt) {
            delete retryPkt->req;
            delete retryPkt;
        }
        retryPkt = NULL;
        retryTid = InvalidThreadID;
    }
	// 如果存在重试信息，则将其删除并重置

    fetchStatus[tid] = Squashing;
	// 设置状态表示正在进行或者即将完成squash处理

    // Empty fetch queue
    fetchQueue[tid].clear();

    // microops are being squashed, it is not known wheather the
    // youngest non-squashed microop was  marked delayed commit
    // or not. Setting the flag to true ensures that the
    // interrupts are not handled when they cannot be, though
    // some opportunities to handle interrupts may be missed.
    delayedCommit[tid] = true;
	// 该变量并没有按照重置的处理进行设置

    ++fetchSquashCycles;
	// 递增表示squash占用周期数目的计数器 
}

template<class Impl>
void
DefaultFetch<Impl>::squashFromDecode(const TheISA::PCState &newPC,
                                     const DynInstPtr squashInst,
                                     const InstSeqNum seq_num, ThreadID tid)
{
    DPRINTF(Fetch, "[tid:%i]: Squashing from decode.\n", tid);

    doSquash(newPC, squashInst, tid);

    // Tell the CPU to remove any instructions that are in flight between
    // fetch and decode.
    cpu->removeInstsUntil(seq_num, tid);
}

template<class Impl>
bool
DefaultFetch<Impl>::checkStall(ThreadID tid) const
{
    bool ret_val = false;

    if (stalls[tid].drain) {
        assert(cpu->isDraining());
        DPRINTF(Fetch,"[tid:%i]: Drain stall detected.\n",tid);
        ret_val = true;
    }

    return ret_val;
}

template<class Impl>
typename DefaultFetch<Impl>::FetchStatus
DefaultFetch<Impl>::updateFetchStatus()
{
    //Check Running
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
		// 那么这里为什么要加1呢？？？

        if (fetchStatus[tid] == Running ||
            fetchStatus[tid] == Squashing ||
            fetchStatus[tid] == IcacheAccessComplete) {

            if (_status == Inactive) {
                DPRINTF(Activity, "[tid:%i]: Activating stage.\n",tid);

                if (fetchStatus[tid] == IcacheAccessComplete) {
                    DPRINTF(Activity, "[tid:%i]: Activating fetch due to cache"
                            "completion\n",tid);
                }

                cpu->activateStage(O3CPU::FetchIdx);
				// 如果当前fetch阶段并没有处于活跃状态，则进行激活
            }
            return Active;
			// 遍历activeThreads中的所有线程，如果存在一个满足要求的
			// 线程就会将fetch阶段设置为Active
        }
    }

    // Stage is switching from active to inactive, notify CPU of it.
    if (_status == Active) {
        DPRINTF(Activity, "Deactivating stage.\n");

        cpu->deactivateStage(O3CPU::FetchIdx);
		// 在没有任何线程满足上述要求的情况下fetch将会进入不活跃状态
    }

    return Inactive;
}

template <class Impl>
void
DefaultFetch<Impl>::squash(const TheISA::PCState &newPC,
                           const InstSeqNum seq_num, DynInstPtr squashInst,
                           ThreadID tid)
{
    DPRINTF(Fetch, "[tid:%u]: Squash from commit.\n", tid);

    doSquash(newPC, squashInst, tid);

    // Tell the CPU to remove any instructions that are not in the ROB.
    cpu->removeInstsNotInROB(tid);
}

template <class Impl>
void
DefaultFetch<Impl>::tick()
{
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();
    bool status_change = false;

    wroteToTimeBuffer = false;
	// 重置wroteToTimeBuffer变量为false

    for (ThreadID i = 0; i < numThreads; ++i) {
        issuePipelinedIfetch[i] = false;
    }

    while (threads != end) {
        ThreadID tid = *threads++;

        // Check the signals for each thread to determine the proper status
        // for each thread.
        bool updated_status = checkSignalsAndUpdate(tid);
        status_change =  status_change || updated_status;
		// 确定一个线程是否需要进行状态更新
    }

    DPRINTF(Fetch, "Running stage.\n");

    if (FullSystem) {
        if (fromCommit->commitInfo[0].interruptPending) {
            interruptPending = true;
        }

        if (fromCommit->commitInfo[0].clearInterrupt) {
            interruptPending = false;
        }
    }
	// 全系统模式下的中断处理，此处不考虑

    for (threadFetched = 0; threadFetched < numFetchingThreads;
         threadFetched++) {
        // Fetch each of the actively fetching threads.
        fetch(status_change);
    }
	// 对所有处于活跃状态的线程执行实际的取指处理

    // Record number of instructions fetched this cycle for distribution.
    fetchNisnDist.sample(numInst);
	// 分别记录每个线程本周期获得的指令数目到对应的统计计数器中

    if (status_change) {
        // Change the fetch stage status if there was a status change.
        _status = updateFetchStatus();
    }
	// 如果发现任何一个线程存在状态的变化便尝试进行状态更新

    // Issue the next I-cache request if possible.
    for (ThreadID i = 0; i < numThreads; ++i) {
        if (issuePipelinedIfetch[i]) {
            pipelineIcacheAccesses(i);
        }
    }
	// 如果一个线程存在需要发送到ICache中的请求则发送出去

    // Send instructions enqueued into the fetch queue to decode.
    // Limit rate by fetchWidth.  Stall if decode is stalled.
    unsigned insts_to_decode = 0;
    unsigned available_insts = 0;

    for (auto tid : *activeThreads) {
        if (!stalls[tid].decode) {
			// 要求对应线程当前decoder可用
            available_insts += fetchQueue[tid].size();
        }
    }
	// 计算每个线程fetchQueue中的全部指令数目

    // Pick a random thread to start trying to grab instructions from
    auto tid_itr = activeThreads->begin();
    std::advance(tid_itr, random_mt.random<uint8_t>(0, activeThreads->size() - 1));
	// 随机选取一个线程对应的迭代器进行处理
	
    while (available_insts != 0 && insts_to_decode < decodeWidth) {
		// 从fetchQueue中获取小于当前decodeWidth的指令进行处理
        ThreadID tid = *tid_itr;
        if (!stalls[tid].decode && !fetchQueue[tid].empty()) {
			// 要求当前线程decode处理可用
            auto inst = fetchQueue[tid].front();
            toDecode->insts[toDecode->size++] = inst;
			// 将需要进行译码操作的指令填充到fetch阶段到decode阶段的
			// 数据传递结构中
            DPRINTF(Fetch, "[tid:%i][sn:%i]: Sending instruction to decode from "
                    "fetch queue. Fetch queue size: %i.\n",
                    tid, inst->seqNum, fetchQueue[tid].size());

            wroteToTimeBuffer = true;
            fetchQueue[tid].pop_front();
			// 每个发出到下一个阶段的指令都会从fetchQueue中清除
            insts_to_decode++;
            available_insts--;
        }

        tid_itr++;
        // Wrap around if at end of active threads list
        if (tid_itr == activeThreads->end())
            tid_itr = activeThreads->begin();
		// 进入到下一个线程的指令选取
    }
	// 注意一下这里如何从多个线程获取的指令总选取进行译码的指令

    // If there was activity this cycle, inform the CPU of it.
    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity this cycle.\n");
        cpu->activityThisCycle();
		// 激活活动记录器来记录timeBuffer的行为
    }

    // Reset the number of the instruction we've fetched.
    numInst = 0;
}

template <class Impl>
bool
DefaultFetch<Impl>::checkSignalsAndUpdate(ThreadID tid)
{
    // Update the per thread stall statuses.
    if (fromDecode->decodeBlock[tid]) {
        stalls[tid].decode = true;
		// 如果发现decode阶段返回的信号表明下一个周期无法进行译码操作
		// 那么将会设置stalls的decode标志位表示decode阶段阻塞导致fetch停止
    }

    if (fromDecode->decodeUnblock[tid]) {
        assert(stalls[tid].decode);
		// 要求发出该信号之前一定进行了前面的stall处理
        assert(!fromDecode->decodeBlock[tid]);
		// 两个信号一定的是相反的
        stalls[tid].decode = false;
    }

    // Check squash signals from commit.
    if (fromCommit->commitInfo[tid].squash) {
        DPRINTF(Fetch, "[tid:%u]: Squashing instructions due to squash "
                "from commit.\n",tid);
        // In any case, squash.
        squash(fromCommit->commitInfo[tid].pc,
               fromCommit->commitInfo[tid].doneSeqNum,
               fromCommit->commitInfo[tid].squashInst, tid);
		// 如果发现从commit阶段传回的信号要求fetch阶段进行squash操作
		// 则调用squash进行相应的处理

        // If it was a branch mispredict on a control instruction, update the
        // branch predictor with that instruction, otherwise just kill the
        // invalid state we generated in after sequence number
        if (fromCommit->commitInfo[tid].mispredictInst &&
            fromCommit->commitInfo[tid].mispredictInst->isControl()) {
            branchPred->squash(fromCommit->commitInfo[tid].doneSeqNum,
                              fromCommit->commitInfo[tid].pc,
                              fromCommit->commitInfo[tid].branchTaken,
                              tid);
		// 如果发生的是一个因为错误分支预测导致的squash操作，那么
		// 只需要对分支预测器中比分支预测更新的指令进行squash操作。
        } else {
            branchPred->squash(fromCommit->commitInfo[tid].doneSeqNum,
                              tid);
		// 如果不是分支错误产生的squash，那么整个线程的指令都需要从
		// 分支预测器中squash掉
        }

        return true;
    } else if (fromCommit->commitInfo[tid].doneSeqNum) {
        // Update the branch predictor if it wasn't a squashed instruction
        // that was broadcasted.
        branchPred->update(fromCommit->commitInfo[tid].doneSeqNum, tid);
		// 如果commit阶段完成一个全局序列编号之前的所有指令，则对
		// 分支预测器的状态进行更新，因为其中的分支预测确实被token了
    }

    // Check squash signals from decode.
    if (fromDecode->decodeInfo[tid].squash) {
        DPRINTF(Fetch, "[tid:%u]: Squashing instructions due to squash "
                "from decode.\n",tid);

        // Update the branch predictor.
        if (fromDecode->decodeInfo[tid].branchMispredict) {
            branchPred->squash(fromDecode->decodeInfo[tid].doneSeqNum,
                              fromDecode->decodeInfo[tid].nextPC,
                              fromDecode->decodeInfo[tid].branchTaken,
                              tid);
        } else {
            branchPred->squash(fromDecode->decodeInfo[tid].doneSeqNum,
                              tid);
        }
		// 这里是来自于decode阶段的squash操作，总的来说和commit阶段的处理一致
		// 但是decode怎么知道会不会发生missPredict，可能是因为有些指令
		// 在译码之后就知道分支目标地址了，并不需要进行计算

        if (fetchStatus[tid] != Squashing) {

            DPRINTF(Fetch, "Squashing from decode with PC = %s\n",
                fromDecode->decodeInfo[tid].nextPC);
            // Squash unless we're already squashing
			// squash操作不能进行叠加，当前只有一个squash操作进行
			// 多个squash操作中一定会有多个操作被忽略
            squashFromDecode(fromDecode->decodeInfo[tid].nextPC,
                             fromDecode->decodeInfo[tid].squashInst,
                             fromDecode->decodeInfo[tid].doneSeqNum,
                             tid);

            return true;
        }
    }

    if (checkStall(tid) &&
        fetchStatus[tid] != IcacheWaitResponse &&
        fetchStatus[tid] != IcacheWaitRetry &&
        fetchStatus[tid] != ItlbWait &&
        fetchStatus[tid] != QuiescePending) {
		// 如果发现当前因为排空流水线导致fetch阶段停止，同时
		// 不等于上述几个状态的时候会进入Blocked状态
		// 之所以这几个状态不允许处理进入Blocked，是因为这些状态
		// 属于正常的处理状态，无需进行状态更新，其他状态下说明stall
		// 确实因为drain发生了，因此需要更新到blocked状态？？？
        DPRINTF(Fetch, "[tid:%i]: Setting to blocked\n",tid);

        fetchStatus[tid] = Blocked;

        return true;
    }

    if (fetchStatus[tid] == Blocked ||
        fetchStatus[tid] == Squashing) {
        // Switch status to running if fetch isn't being told to block or
        // squash this cycle.
        DPRINTF(Fetch, "[tid:%i]: Done squashing, switching to running.\n",
                tid);

        fetchStatus[tid] = Running;
		// 如果fetch阶段没有被要求进行block或者squash则刷新进入运行状态
        return true;
    }

    // If we've reached this point, we have not gotten any signals that
    // cause fetch to change its status.  Fetch remains the same as before.
    return false;
}

template<class Impl>
typename Impl::DynInstPtr
DefaultFetch<Impl>::buildInst(ThreadID tid, StaticInstPtr staticInst,
                              StaticInstPtr curMacroop, TheISA::PCState thisPC,
                              TheISA::PCState nextPC, bool trace)
{
    // Get a sequence number.
    InstSeqNum seq = cpu->getAndIncrementInstSeq();
	// 这里获取了一个新的全局序列编号

    // Create a new DynInst from the instruction fetched.
    DynInstPtr instruction =
        new DynInst(staticInst, curMacroop, thisPC, nextPC, seq, cpu);
    instruction->setTid(tid);

    instruction->setASID(tid);

    instruction->setThreadState(cpu->thread[tid]);

    DPRINTF(Fetch, "[tid:%i]: Instruction PC %#x (%d) created "
            "[sn:%lli].\n", tid, thisPC.instAddr(),
            thisPC.microPC(), seq);

    DPRINTF(Fetch, "[tid:%i]: Instruction is: %s\n", tid,
            instruction->staticInst->
            disassemble(thisPC.instAddr()));
	// 初始化一个动态指令并设置相应的的数据
	
#if TRACING_ON
    if (trace) {
        instruction->traceData =
            cpu->getTracer()->getInstRecord(curTick(), cpu->tcBase(tid),
                    instruction->staticInst, thisPC, curMacroop);
    }
#else
    instruction->traceData = NULL;
#endif

    // Add instruction to the CPU's list of instructions.
    instruction->setInstListIt(cpu->addInst(instruction));
	// 将指令设置到CPU的instList中，并获取对应该指令在instList中的迭代器

    // Write the instruction to the first slot in the queue
    // that heads to decode.
    assert(numInst < fetchWidth);
    fetchQueue[tid].push_back(instruction);
	// 指令被添加到fetchQueue中
    assert(fetchQueue[tid].size() <= fetchQueueSize);
    DPRINTF(Fetch, "[tid:%i]: Fetch queue entry created (%i/%i).\n",
            tid, fetchQueue[tid].size(), fetchQueueSize);
    //toDecode->insts[toDecode->size++] = instruction;

    // Keep track of if we can take an interrupt at this boundary
    delayedCommit[tid] = instruction->isDelayedCommit();
	// 说明delayedCommit和指令本身有关

    return instruction;
}

template<class Impl>
void
DefaultFetch<Impl>::fetch(bool &status_change)
{
    //////////////////////////////////////////
    // Start actual fetch
    //////////////////////////////////////////
    ThreadID tid = getFetchingThread(fetchPolicy);
	// 根据取值策略选择一个合适的线程进行取指处理
	// 通过策略算法可以发现每个可用的活跃线程都会被选择一次

    assert(!cpu->switchedOut());

    if (tid == InvalidThreadID) {
        // Breaks looping condition in tick()
        threadFetched = numFetchingThreads;

        if (numThreads == 1) {  // @todo Per-thread stats
            profileStall(0);
			// 如果当前的总线程数目为1，则更新统计计数器
        }
		// 如果getFetchingThread返回了一个InvalidThreadID
		// 表示当前没有可以进行取指操作的线程，threadFetched直接等于
		// 需要取指的线程数目表示取指过程已经提前结束

        return;
    }

    DPRINTF(Fetch, "Attempting to fetch from [tid:%i]\n", tid);

    // The current PC.
    TheISA::PCState thisPC = pc[tid];
    Addr pcOffset = fetchOffset[tid];
    Addr fetchAddr = (thisPC.instAddr() + pcOffset) & BaseCPU::PCMask;
	// 实际的取指地址由offset和PC的加和给出，PCMask则提取其中有效的部分
	// 这里的offset可能和微操作有关？？？

    bool inRom = isRomMicroPC(thisPC.microPC());
	// 和微操作位域解释有关，定义于gem5/src/base/types.hh
	// 经过试验测试，该数值在Riscv中似乎恒为false

    // If returning from the delay of a cache miss, then update the status
    // to running, otherwise do the cache access.  Possibly move this up
    // to tick() function.
    if (fetchStatus[tid] == IcacheAccessComplete) {
        DPRINTF(Fetch, "[tid:%i]: Icache miss is complete.\n", tid);

        fetchStatus[tid] = Running;
        status_change = true;
		// 如果发现当前线程的ICache访问正好完成，则将fetch阶段设置为
		// Running状态，接下来不必进行相关处理，因为数据已经处理好了
    } else if (fetchStatus[tid] == Running) {
		// 如果本来就处于Running状态，说明当前不处于访存的任何阶段
		// 因此需要启动新的Cache访问
        // Align the fetch PC so its at the start of a fetch buffer segment.
        Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);
		// 根据实际的取指地址生成对应的fetchBuffer大小对齐的取指地址

        // If buffer is no longer valid or fetchAddr has moved to point
        // to the next cache block, AND we have no remaining ucode
        // from a macro-op, then start fetch from icache.
        if (!(fetchBufferValid[tid] && fetchBufferBlockPC == fetchBufferPC[tid])
            && !inRom && !macroop[tid]) {		
			// 这里考虑了一个特殊的情况，那就是两次取指PC对应于同一个
			// fetchBuffer对齐的区域，如果两者确实处于一个fetchBuffer对齐的
			// 区域，而恰好上次取得了这个区域到fetchBuffer中，那么本次
			// 取指就可以直接从fetchBuffer中获取指令了。如果是其他情况，
			// 那么我们必须从Cache中获取一个新的区域到fetchBuffer中
            DPRINTF(Fetch, "[tid:%i]: Attempting to translate and read "
                    "instruction, starting at PC %s.\n", tid, thisPC);

            fetchCacheLine(fetchAddr, tid, thisPC.instAddr());
			// 开始获取一个CacheLine，这里压根不管fetch请求是否处理成功
			// 因为返回值没有被处理

            if (fetchStatus[tid] == IcacheWaitResponse)
                ++icacheStallCycles;
            else if (fetchStatus[tid] == ItlbWait)
                ++fetchTlbCycles;
            else
                ++fetchMiscStallCycles;
			// 更新相应的周期计数器
			
            return;
        } else if ((checkInterrupt(thisPC.instAddr()) && !delayedCommit[tid])) {
            // Stall CPU if an interrupt is posted and we're not issuing
            // an delayed commit micro-op currently (delayed commit instructions
            // are not interruptable by interrupts, only faults)
            ++fetchMiscStallCycles;
            DPRINTF(Fetch, "[tid:%i]: Fetch is stalled!\n", tid);
			// 如果当前存在中断需要处理或者？？？则不能执行取指操作
			// 而不得不的等待，该情况只有在FS模式下可能出现
            return;
        }
    } else {
        if (fetchStatus[tid] == Idle) {
            ++fetchIdleCycles;
            DPRINTF(Fetch, "[tid:%i]: Fetch is idle!\n", tid);
        }
		// 如果当前状态是空闲状态那么即使按照正常处理调用了
		// 该函数也不会进行任何操作，事实证明该情况几乎不会触发

        // Status is Idle, so fetch should do nothing.
        return;
    }
	// 上面的过程处理的是缓存行的获取操作，获得的是数据存放在fetchBuffer中
	// 需要下面的过程进行具体指令提取的处理

    ++fetchCycles;

    TheISA::PCState nextPC = thisPC;

    StaticInstPtr staticInst = NULL;
    StaticInstPtr curMacroop = macroop[tid];

	// 如果处理进入到这个位置，说明指令所在的数据已经被获取到了
	// fetchBuffer中，因此可以直接继续将指令提取出来了
    // If the read of the first instruction was successful, then grab the
    // instructions from the rest of the cache line and put them into the
    // queue heading to decode.

    DPRINTF(Fetch, "[tid:%i]: Adding instructions to queue to "
            "decode.\n", tid);

    // Need to keep track of whether or not a predicted branch
    // ended this fetch block.
    bool predictedBranch = false;

    // Need to halt fetch if quiesce instruction detected
    bool quiesce = false;

    TheISA::MachInst *cacheInsts =
        reinterpret_cast<TheISA::MachInst *>(fetchBuffer[tid]);

    const unsigned numInsts = fetchBufferSize / instSize;
    unsigned blkOffset = (fetchAddr - fetchBufferPC[tid]) / instSize;
	// 获得所需指令在fetchBuffer中的偏移位置

    // Loop through instruction memory from the cache.
    // Keep issuing while fetchWidth is available and branch is not
    // predicted taken
    while (numInst < fetchWidth && fetchQueue[tid].size() < fetchQueueSize
           && !predictedBranch && !quiesce) {
        // We need to process more memory if we aren't going to get a
        // StaticInst from the rom, the current macroop, or what's already
        // in the decoder.
        bool needMem = !inRom && !curMacroop &&
            !decoder[tid]->instReady();
		// 对于riscv来说needMem其实就是decoder中的单指令连续读取
		// 操作的标志，表示上次读取指令只读取了一半，因为每次decoder
		// 能够获取的数据一定是一个32bit数据sizeof(TheISA::MachInst)
		// 由于riscv存在16bit压缩指令，因此会产生不对齐的32bit指令
        fetchAddr = (thisPC.instAddr() + pcOffset) & BaseCPU::PCMask;
        Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);

        if (needMem) {
			// 这里表示获取单个指令的操作没有完全完成，需要等待进行
			// 下一步操作，这些操作参见decoder.cc/hh文件中的说明
            // If buffer is no longer valid or fetchAddr has moved to point
            // to the next cache block then start fetch from icache.
            if (!fetchBufferValid[tid] ||
                fetchBufferBlockPC != fetchBufferPC[tid])
				// 如果已经无法从当前的fetchBuffer中获取指令，则退出
				// 出现这种情况是因为一个指令横跨两个fetchBuffer区域
                break;

            if (blkOffset >= numInsts) {
                // We need to process more memory, but we've run out of the
                // current block.
				// 这种情况应该是发生在一个指令处于两个CacheLine中
                break;
            }

            MachInst inst = TheISA::gtoh(cacheInsts[blkOffset]);
			// 根据大小端设置将获取的数据进行翻转或者不翻转
            decoder[tid]->moreBytes(thisPC, fetchAddr, inst);
			// 由于decoder[tid]->instReady()返回false，表示一个指令
			// 没有读取完成，因此继续读取指令的后半部分

            if (decoder[tid]->needMoreBytes()) {
                blkOffset++;
                fetchAddr += instSize;
                pcOffset += instSize;
            }
			// 如果处理完当前指令后半部分之后，进入到了一个新的32bit
			// 对齐指令数据区域，那么递增相关数据，进行下一个指令的处理
			// 否则说明当前32bit数据只用了16bit，无需获取新的32bit数据
        }

        // Extract as many instructions and/or microops as we can from
        // the memory we've processed so far.
		// 下面将尝试从获取到的CacheLine中处理出指令来
        do {
            if (!(curMacroop || inRom)) {
			// 根据实验发现这个if一定是满足的
                if (decoder[tid]->instReady()) {
                    staticInst = decoder[tid]->decode(thisPC);
					// 如果一个指令处理完毕则更新PC为NPC，这里调用
					// decode更新是因为上一个指令可能是压缩指令
					// 当然这里会根据指定的PC确定对应的静态指令

                    // Increment stat of fetched instructions.
                    ++fetchedInsts;

                    if (staticInst->isMacroop()) {
                        curMacroop = staticInst;
                    } else {
                        pcOffset = 0;
						// 如果指令不包含微操作，则offset被设置为0
                    }
                } else {
                    // We need more bytes for this instruction so blkOffset and
                    // pcOffset will be updated
					// 这里说明我们对一个指令的处理只进行了一半
                    break;
                }
            }
            // Whether we're moving to a new macroop because we're at the
            // end of the current one, or the branch predictor incorrectly
            // thinks we are...
            bool newMacro = false;
            if (curMacroop || inRom) {
                if (inRom) {
                    staticInst = cpu->microcodeRom.fetchMicroop(
                            thisPC.microPC(), curMacroop);
                } else {
                    staticInst = curMacroop->fetchMicroop(thisPC.microPC());
                }
                newMacro |= staticInst->isLastMicroop();
            }
			// ？？？尝试获取一个微操作，对于不支持微操作的
			// 指令来说，这里的微操作就是指令

            DynInstPtr instruction =
                buildInst(tid, staticInst, curMacroop,
                          thisPC, nextPC, true);
			// 根据获取的静态指令生成一个可以用于后续处理的动态指令

            ppFetch->notify(instruction);
            numInst++;

#if TRACING_ON
            if (DTRACE(O3PipeView)) {
                instruction->fetchTick = curTick();
            }
			// 记录该指令被获取的对应周期位置
#endif

            nextPC = thisPC;
			// 这里的PC表示的是未分支情况下的下一个指令对应的PC

            // If we're branching after this instruction, quit fetching
            // from the same block.
            predictedBranch |= thisPC.branching();
            predictedBranch |=
                lookupAndUpdateNextPC(instruction, nextPC);
			// 如果当前指令是一个分支指令并且被token，那么predictedBranch
			// 会变为true，意味着下一个PC很可能不再当前的Block中，预测的
			// 相关信息都会被记录到指令中，nextPC会被更新为分支目标地址
			// 非分支指令或者预测不采用的分支指令该变量都是false。
            if (predictedBranch) {
                DPRINTF(Fetch, "Branch detected with PC = %s\n", thisPC);
            }

            newMacro |= thisPC.instAddr() != nextPC.instAddr();
			// 如果发生了分支指令token导致新的指令和当前指令不在一个
			// cacheblock中，则会使得newMacro变为false

            // Move to the next instruction, unless we have a branch.
			// 开始处理下一个指令
            thisPC = nextPC;
			// 更新PC为nextPC，分支目标PC或者正常递增的PC地址
            inRom = isRomMicroPC(thisPC.microPC());

            if (newMacro) {
                fetchAddr = thisPC.instAddr() & BaseCPU::PCMask;
                blkOffset = (fetchAddr - fetchBufferPC[tid]) / instSize;
                pcOffset = 0;
                curMacroop = NULL;
				// 计算新的指令在fetchBuffer中的位置信息
            }

            if (instruction->isQuiesce()) {
				// 如果发现了一个静默指令，则进行相应的处理
                DPRINTF(Fetch,
                        "Quiesce instruction encountered, halting fetch!\n");
                fetchStatus[tid] = QuiescePending;
                status_change = true;
                quiesce = true;
				// 这里将直接退出指令处理的循环
                break;
            }
        } while ((curMacroop || decoder[tid]->instReady()) &&
                 numInst < fetchWidth &&
                 fetchQueue[tid].size() < fetchQueueSize);
		// 该循环过程中不会更新blkOffset，因此该循环只负责处理一个32bit指令
		// 区域，但是一个32bit指令区域可能获得两个压缩指令，因此这里也会
		// 进行取指总数的限制判断。

        // Re-evaluate whether the next instruction to fetch is in micro-op ROM
        // or not.
        inRom = isRomMicroPC(thisPC.microPC());
    }
	// 连续指令处理的循环结束处

    if (predictedBranch) {
        DPRINTF(Fetch, "[tid:%i]: Done fetching, predicted branch "
                "instruction encountered.\n", tid);
    } else if (numInst >= fetchWidth) {
        DPRINTF(Fetch, "[tid:%i]: Done fetching, reached fetch bandwidth "
                "for this cycle.\n", tid);
    } else if (blkOffset >= fetchBufferSize) {
        DPRINTF(Fetch, "[tid:%i]: Done fetching, reached the end of the"
                "fetch buffer.\n", tid);
    }

    macroop[tid] = curMacroop;
    fetchOffset[tid] = pcOffset;
	// 记录本次取指的相关信息

    if (numInst > 0) {
        wroteToTimeBuffer = true;
    }

    pc[tid] = thisPC;
	// 在完成整个fetch操作之后正式修改fetch阶段不同线程的PC数值

    // pipeline a fetch if we're crossing a fetch buffer boundary and not in
    // a state that would preclude fetching
    fetchAddr = (thisPC.instAddr() + pcOffset) & BaseCPU::PCMask;
    Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);
	// 判断给定的新PC地址在新的fetchBuffer对齐区域
	
    issuePipelinedIfetch[tid] = fetchBufferBlockPC != fetchBufferPC[tid] &&
        fetchStatus[tid] != IcacheWaitResponse &&
        fetchStatus[tid] != ItlbWait &&
        fetchStatus[tid] != IcacheWaitRetry &&
        fetchStatus[tid] != QuiescePending &&
        !curMacroop;
	// 如果发现当前无法连续获取新的指令是因为当前fetchBuffer的数据已经用尽
	// 并且当前状态允许获取一个新的CacheLine数据到fetchBuffer中，则该标志位
	// 会被设置并在tick阶段执行fetch之后发送一个新的请求
}

template<class Impl>
void
DefaultFetch<Impl>::recvReqRetry()
{
    if (retryPkt != NULL) {
        assert(cacheBlocked);
        assert(retryTid != InvalidThreadID);
        assert(fetchStatus[retryTid] == IcacheWaitRetry);

        if (cpu->getInstPort().sendTimingReq(retryPkt)) {
			// 尝试重新将重试ICache访问请求发送给ICache处理
            fetchStatus[retryTid] = IcacheWaitResponse;
            // Notify Fetch Request probe when a retryPkt is successfully sent.
            // Note that notify must be called before retryPkt is set to NULL.
            ppFetchRequestSent->notify(retryPkt->req);
            retryPkt = NULL;
            retryTid = InvalidThreadID;
            cacheBlocked = false;
			// 然后更新相应的状态，并重置重试信息
        }
    } else {
        assert(retryTid == InvalidThreadID);
        // Access has been squashed since it was sent out.  Just clear
        // the cache being blocked.
        cacheBlocked = false;
		// 这里发现压根没有需要重试的内容，重置cacheBlocked标志位
    }
}

///////////////////////////////////////
//                                   //
//  SMT FETCH POLICY MAINTAINED HERE //
//                                   //
///////////////////////////////////////

template<class Impl>
ThreadID
DefaultFetch<Impl>::getFetchingThread(FetchPriority &fetch_priority)
{
    if (numThreads > 1) {
        switch (fetch_priority) {

          case SingleThread:
            return 0;

          case RoundRobin:
            return roundRobin();

          case IQ:
            return iqCount();

          case LSQ:
            return lsqCount();

          case Branch:
            return branchCount();

          default:
            return InvalidThreadID;
        }
		// 如果当前确实有多个线程，则调用对应的策略算法返回正确的线程
    } else {
        list<ThreadID>::iterator thread = activeThreads->begin();
        if (thread == activeThreads->end()) {
            return InvalidThreadID;
        }
		// 如果当前没有活跃的线程，则返回InvalidThreadID

        ThreadID tid = *thread;

        if (fetchStatus[tid] == Running ||
            fetchStatus[tid] == IcacheAccessComplete ||
            fetchStatus[tid] == Idle) {
			// 对于单线程的情况，只有上面的三种情况允许获取新的指令
            return tid;
        } else {
            return InvalidThreadID;
        }
    }
}


template<class Impl>
ThreadID
DefaultFetch<Impl>::roundRobin()
{
    list<ThreadID>::iterator pri_iter = priorityList.begin();
    list<ThreadID>::iterator end      = priorityList.end();

    ThreadID high_pri;

    while (pri_iter != end) {
        high_pri = *pri_iter;

        assert(high_pri <= numThreads);

        if (fetchStatus[high_pri] == Running ||
            fetchStatus[high_pri] == IcacheAccessComplete ||
            fetchStatus[high_pri] == Idle) {

            priorityList.erase(pri_iter);
            priorityList.push_back(high_pri);

            return high_pri;
        }

        pri_iter++;
    }

    return InvalidThreadID;
}

template<class Impl>
ThreadID
DefaultFetch<Impl>::iqCount()
{
    //sorted from lowest->highest
    std::priority_queue<unsigned,vector<unsigned>,
                        std::greater<unsigned> > PQ;
	// 这里的vector<unsigned>标识的是priority_queue使用的容器并不是一个元素
    std::map<unsigned, ThreadID> threadMap;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
        unsigned iqCount = fromIEW->iewInfo[tid].iqCount;

        //we can potentially get tid collisions if two threads
        //have the same iqCount, but this should be rare.
        PQ.push(iqCount);
        threadMap[iqCount] = tid;
    }

    while (!PQ.empty()) {
        ThreadID high_pri = threadMap[PQ.top()];

        if (fetchStatus[high_pri] == Running ||
            fetchStatus[high_pri] == IcacheAccessComplete ||
            fetchStatus[high_pri] == Idle)
            return high_pri;
        else
            PQ.pop();

    }

    return InvalidThreadID;
}

template<class Impl>
ThreadID
DefaultFetch<Impl>::lsqCount()
{
    //sorted from lowest->highest
    std::priority_queue<unsigned,vector<unsigned>,
                        std::greater<unsigned> > PQ;
    std::map<unsigned, ThreadID> threadMap;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
        unsigned ldstqCount = fromIEW->iewInfo[tid].ldstqCount;

        //we can potentially get tid collisions if two threads
        //have the same iqCount, but this should be rare.
        PQ.push(ldstqCount);
        threadMap[ldstqCount] = tid;
    }

    while (!PQ.empty()) {
        ThreadID high_pri = threadMap[PQ.top()];

        if (fetchStatus[high_pri] == Running ||
            fetchStatus[high_pri] == IcacheAccessComplete ||
            fetchStatus[high_pri] == Idle)
            return high_pri;
        else
            PQ.pop();
    }

    return InvalidThreadID;
}

template<class Impl>
ThreadID
DefaultFetch<Impl>::branchCount()
{
#if 0
    list<ThreadID>::iterator thread = activeThreads->begin();
    assert(thread != activeThreads->end());
    ThreadID tid = *thread;
#endif

    panic("Branch Count Fetch policy unimplemented\n");
    return InvalidThreadID;
}

template<class Impl>
void
DefaultFetch<Impl>::pipelineIcacheAccesses(ThreadID tid)
{
    if (!issuePipelinedIfetch[tid]) {
        return;
    }

    // The next PC to access.
    TheISA::PCState thisPC = pc[tid];

    if (isRomMicroPC(thisPC.microPC())) {
        return;
    }

    Addr pcOffset = fetchOffset[tid];
    Addr fetchAddr = (thisPC.instAddr() + pcOffset) & BaseCPU::PCMask;

    // Align the fetch PC so its at the start of a fetch buffer segment.
    Addr fetchBufferBlockPC = fetchBufferAlignPC(fetchAddr);

    // Unless buffer already got the block, fetch it from icache.
    if (!(fetchBufferValid[tid] && fetchBufferBlockPC == fetchBufferPC[tid])) {
        DPRINTF(Fetch, "[tid:%i]: Issuing a pipelined I-cache access, "
                "starting at PC %s.\n", tid, thisPC);

        fetchCacheLine(fetchAddr, tid, thisPC.instAddr());
    }
}

template<class Impl>
void
DefaultFetch<Impl>::profileStall(ThreadID tid) {
    DPRINTF(Fetch,"There are no more threads available to fetch from.\n");

    // @todo Per-thread stats

    if (stalls[tid].drain) {
        ++fetchPendingDrainCycles;
        DPRINTF(Fetch, "Fetch is waiting for a drain!\n");
    } else if (activeThreads->empty()) {
        ++fetchNoActiveThreadStallCycles;
        DPRINTF(Fetch, "Fetch has no active thread!\n");
    } else if (fetchStatus[tid] == Blocked) {
        ++fetchBlockedCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is blocked!\n", tid);
    } else if (fetchStatus[tid] == Squashing) {
        ++fetchSquashCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is squashing!\n", tid);
    } else if (fetchStatus[tid] == IcacheWaitResponse) {
        ++icacheStallCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is waiting cache response!\n",
                tid);
    } else if (fetchStatus[tid] == ItlbWait) {
        ++fetchTlbCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is waiting ITLB walk to "
                "finish!\n", tid);
    } else if (fetchStatus[tid] == TrapPending) {
        ++fetchPendingTrapStallCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is waiting for a pending trap!\n",
                tid);
    } else if (fetchStatus[tid] == QuiescePending) {
        ++fetchPendingQuiesceStallCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is waiting for a pending quiesce "
                "instruction!\n", tid);
    } else if (fetchStatus[tid] == IcacheWaitRetry) {
        ++fetchIcacheWaitRetryStallCycles;
        DPRINTF(Fetch, "[tid:%i]: Fetch is waiting for an I-cache retry!\n",
                tid);
    } else if (fetchStatus[tid] == NoGoodAddr) {
            DPRINTF(Fetch, "[tid:%i]: Fetch predicted non-executable address\n",
                    tid);
    } else {
        DPRINTF(Fetch, "[tid:%i]: Unexpected fetch stall reason (Status: %i).\n",
             tid, fetchStatus[tid]);
    }
}

#endif//__CPU_O3_FETCH_IMPL_HH__
