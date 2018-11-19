/*
 * Copyright (c) 2010-2013 ARM Limited
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 */

#ifndef __CPU_O3_IEW_IMPL_IMPL_HH__
#define __CPU_O3_IEW_IMPL_IMPL_HH__

// @todo: Fix the instantaneous communication among all the stages within
// iew.  There's a clear delay between issue and execute, yet backwards
// communication happens simultaneously.

#include <queue>

#include "arch/utility.hh"
#include "config/the_isa.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/o3/fu_pool.hh"
#include "cpu/o3/iew.hh"
#include "cpu/timebuf.hh"
#include "debug/Activity.hh"
#include "debug/Drain.hh"
#include "debug/IEW.hh"
#include "debug/O3PipeView.hh"
#include "params/DerivO3CPU.hh"

using namespace std;

template<class Impl>
DefaultIEW<Impl>::DefaultIEW(O3CPU *_cpu, DerivO3CPUParams *params)
    : issueToExecQueue(params->backComSize, params->forwardComSize),
      cpu(_cpu),
      instQueue(_cpu, this, params),
      ldstQueue(_cpu, this, params),
      fuPool(params->fuPool),
      commitToIEWDelay(params->commitToIEWDelay),
      renameToIEWDelay(params->renameToIEWDelay),
      issueToExecuteDelay(params->issueToExecuteDelay),
      dispatchWidth(params->dispatchWidth),
      issueWidth(params->issueWidth),
      wbWidth(params->wbWidth),
      numThreads(params->numThreads)
{
    if (dispatchWidth > Impl::MaxWidth)
        fatal("dispatchWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/impl.hh\n",
             dispatchWidth, static_cast<int>(Impl::MaxWidth));
    if (issueWidth > Impl::MaxWidth)
        fatal("issueWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/impl.hh\n",
             issueWidth, static_cast<int>(Impl::MaxWidth));
    if (wbWidth > Impl::MaxWidth)
        fatal("wbWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/impl.hh\n",
             wbWidth, static_cast<int>(Impl::MaxWidth));

    _status = Active;
    exeStatus = Running;
    wbStatus = Idle;

    // Setup wire to read instructions coming from issue.
    fromIssue = issueToExecQueue.getWire(-issueToExecuteDelay);

    // Instruction queue needs the queue between issue and execute.
    instQueue.setIssueToExecuteQueue(&issueToExecQueue);

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        dispatchStatus[tid] = Running;
        fetchRedirect[tid] = false;
    }

    updateLSQNextCycle = false;

    skidBufferMax = (renameToIEWDelay + 1) * params->renameWidth;
}

template <class Impl>
std::string
DefaultIEW<Impl>::name() const
{
    return cpu->name() + ".iew";
}

template <class Impl>
void
DefaultIEW<Impl>::regProbePoints()
{
    ppDispatch = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(), "Dispatch");
    ppMispredict = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(), "Mispredict");
    /**
     * Probe point with dynamic instruction as the argument used to probe when
     * an instruction starts to execute.
     */
    ppExecute = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(),
                                              "Execute");
    /**
     * Probe point with dynamic instruction as the argument used to probe when
     * an instruction execution completes and it is marked ready to commit.
     */
    ppToCommit = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(),
                                               "ToCommit");
}

template <class Impl>
void
DefaultIEW<Impl>::regStats()
{
    using namespace Stats;

    instQueue.regStats();
    ldstQueue.regStats();

    iewIdleCycles
        .name(name() + ".iewIdleCycles")
        .desc("Number of cycles IEW is idle");

    iewSquashCycles
        .name(name() + ".iewSquashCycles")
        .desc("Number of cycles IEW is squashing");

    iewBlockCycles
        .name(name() + ".iewBlockCycles")
        .desc("Number of cycles IEW is blocking");

    iewUnblockCycles
        .name(name() + ".iewUnblockCycles")
        .desc("Number of cycles IEW is unblocking");

    iewDispatchedInsts
        .name(name() + ".iewDispatchedInsts")
        .desc("Number of instructions dispatched to IQ");

    iewDispSquashedInsts
        .name(name() + ".iewDispSquashedInsts")
        .desc("Number of squashed instructions skipped by dispatch");

    iewDispLoadInsts
        .name(name() + ".iewDispLoadInsts")
        .desc("Number of dispatched load instructions");

    iewDispStoreInsts
        .name(name() + ".iewDispStoreInsts")
        .desc("Number of dispatched store instructions");

    iewDispNonSpecInsts
        .name(name() + ".iewDispNonSpecInsts")
        .desc("Number of dispatched non-speculative instructions");

    iewIQFullEvents
        .name(name() + ".iewIQFullEvents")
        .desc("Number of times the IQ has become full, causing a stall");

    iewLSQFullEvents
        .name(name() + ".iewLSQFullEvents")
        .desc("Number of times the LSQ has become full, causing a stall");

    memOrderViolationEvents
        .name(name() + ".memOrderViolationEvents")
        .desc("Number of memory order violations");

    predictedTakenIncorrect
        .name(name() + ".predictedTakenIncorrect")
        .desc("Number of branches that were predicted taken incorrectly");

    predictedNotTakenIncorrect
        .name(name() + ".predictedNotTakenIncorrect")
        .desc("Number of branches that were predicted not taken incorrectly");

    branchMispredicts
        .name(name() + ".branchMispredicts")
        .desc("Number of branch mispredicts detected at execute");

    branchMispredicts = predictedTakenIncorrect + predictedNotTakenIncorrect;

    iewExecutedInsts
        .name(name() + ".iewExecutedInsts")
        .desc("Number of executed instructions");

    iewExecLoadInsts
        .init(cpu->numThreads)
        .name(name() + ".iewExecLoadInsts")
        .desc("Number of load instructions executed")
        .flags(total);

    iewExecSquashedInsts
        .name(name() + ".iewExecSquashedInsts")
        .desc("Number of squashed instructions skipped in execute");

    iewExecutedSwp
        .init(cpu->numThreads)
        .name(name() + ".exec_swp")
        .desc("number of swp insts executed")
        .flags(total);

    iewExecutedNop
        .init(cpu->numThreads)
        .name(name() + ".exec_nop")
        .desc("number of nop insts executed")
        .flags(total);

    iewExecutedRefs
        .init(cpu->numThreads)
        .name(name() + ".exec_refs")
        .desc("number of memory reference insts executed")
        .flags(total);

    iewExecutedBranches
        .init(cpu->numThreads)
        .name(name() + ".exec_branches")
        .desc("Number of branches executed")
        .flags(total);

    iewExecStoreInsts
        .name(name() + ".exec_stores")
        .desc("Number of stores executed")
        .flags(total);
    iewExecStoreInsts = iewExecutedRefs - iewExecLoadInsts;

    iewExecRate
        .name(name() + ".exec_rate")
        .desc("Inst execution rate")
        .flags(total);

    iewExecRate = iewExecutedInsts / cpu->numCycles;

    iewInstsToCommit
        .init(cpu->numThreads)
        .name(name() + ".wb_sent")
        .desc("cumulative count of insts sent to commit")
        .flags(total);

    writebackCount
        .init(cpu->numThreads)
        .name(name() + ".wb_count")
        .desc("cumulative count of insts written-back")
        .flags(total);

    producerInst
        .init(cpu->numThreads)
        .name(name() + ".wb_producers")
        .desc("num instructions producing a value")
        .flags(total);

    consumerInst
        .init(cpu->numThreads)
        .name(name() + ".wb_consumers")
        .desc("num instructions consuming a value")
        .flags(total);

    wbFanout
        .name(name() + ".wb_fanout")
        .desc("average fanout of values written-back")
        .flags(total);

    wbFanout = producerInst / consumerInst;

    wbRate
        .name(name() + ".wb_rate")
        .desc("insts written-back per cycle")
        .flags(total);
    wbRate = writebackCount / cpu->numCycles;
}

template<class Impl>
void
DefaultIEW<Impl>::startupStage()
{
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        toRename->iewInfo[tid].usedIQ = true;
        toRename->iewInfo[tid].freeIQEntries =
            instQueue.numFreeEntries(tid);
		// 说明IEW使用IQ以及IQ的空闲表项数目

        toRename->iewInfo[tid].usedLSQ = true;
        toRename->iewInfo[tid].freeLQEntries = ldstQueue.numFreeLoadEntries(tid);
        toRename->iewInfo[tid].freeSQEntries = ldstQueue.numFreeStoreEntries(tid);
		// 说明IEW使用LSQ以及LQ、SQ各自的空闲表项数
		// 上面信号中的空闲表项数目在执行过程中都会更新
	}

    // Initialize the checker's dcache port here
    if (cpu->checker) {
        cpu->checker->setDcachePort(&cpu->getDataPort());
    }

    cpu->activateStage(O3CPU::IEWIdx);
}

template<class Impl>
void
DefaultIEW<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    timeBuffer = tb_ptr;

    // Setup wire to read information from time buffer, from commit.
    fromCommit = timeBuffer->getWire(-commitToIEWDelay);

    // Setup wire to write information back to previous stages.
    toRename = timeBuffer->getWire(0);

    toFetch = timeBuffer->getWire(0);

    // Instruction queue also needs main time buffer.
    instQueue.setTimeBuffer(tb_ptr);
}

template<class Impl>
void
DefaultIEW<Impl>::setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr)
{
    renameQueue = rq_ptr;

    // Setup wire to read information from rename queue.
    fromRename = renameQueue->getWire(-renameToIEWDelay);
}

template<class Impl>
void
DefaultIEW<Impl>::setIEWQueue(TimeBuffer<IEWStruct> *iq_ptr)
{
    iewQueue = iq_ptr;

    // Setup wire to write instructions to commit.
    toCommit = iewQueue->getWire(0);
}

template<class Impl>
void
DefaultIEW<Impl>::setActiveThreads(list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;

    ldstQueue.setActiveThreads(at_ptr);
    instQueue.setActiveThreads(at_ptr);
}

template<class Impl>
void
DefaultIEW<Impl>::setScoreboard(Scoreboard *sb_ptr)
{
    scoreboard = sb_ptr;
}

template <class Impl>
bool
DefaultIEW<Impl>::isDrained() const
{
    bool drained = ldstQueue.isDrained() && instQueue.isDrained();

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (!insts[tid].empty()) {
            DPRINTF(Drain, "%i: Insts not empty.\n", tid);
            drained = false;
        }
        if (!skidBuffer[tid].empty()) {
            DPRINTF(Drain, "%i: Skid buffer not empty.\n", tid);
            drained = false;
        }
        drained = drained && dispatchStatus[tid] == Running;
    }

    // Also check the FU pool as instructions are "stored" in FU
    // completion events until they are done and not accounted for
    // above
    if (drained && !fuPool->isDrained()) {
        DPRINTF(Drain, "FU pool still busy.\n");
        drained = false;
    }

    return drained;
}

template <class Impl>
void
DefaultIEW<Impl>::drainSanityCheck() const
{
    assert(isDrained());

    instQueue.drainSanityCheck();
    ldstQueue.drainSanityCheck();
}

template <class Impl>
void
DefaultIEW<Impl>::takeOverFrom()
{
    // Reset all state.
    _status = Active;
    exeStatus = Running;
    wbStatus = Idle;

    instQueue.takeOverFrom();
    ldstQueue.takeOverFrom();
    fuPool->takeOverFrom();

    startupStage();
    cpu->activityThisCycle();
	// 为什么要向CPU说明本周期的主要活动内容？

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        dispatchStatus[tid] = Running;
        fetchRedirect[tid] = false;
    }

    updateLSQNextCycle = false;

    for (int i = 0; i < issueToExecQueue.getSize(); ++i) {
        issueToExecQueue.advance();
		// 该操作无法理解？？？
    }
}

template<class Impl>
void
DefaultIEW<Impl>::squash(ThreadID tid)
{
    DPRINTF(IEW, "[tid:%i]: Squashing all instructions.\n", tid);

    // Tell the IQ to start squashing.
    instQueue.squash(tid);

    // Tell the LDSTQ to start squashing.
    ldstQueue.squash(fromCommit->commitInfo[tid].doneSeqNum, tid);
	// LSQ不支持对单个线程的所有指令进行squash操作，因此这里只能
	// 根据最近提交的指令全局编号进行squash，以便达到相同的效果；因为
	// 该操作会将比该指令更新的指令全部squash，和IQ的squash效果一致
    updatedQueues = true;
	// 由于进行了squash操作，设置该标志为表示队列变化以便广播新的空闲表项数

    // Clear the skid buffer in case it has any data in it.
    DPRINTF(IEW, "[tid:%i]: Removing skidbuffer instructions until [sn:%i].\n",
            tid, fromCommit->commitInfo[tid].doneSeqNum);

    while (!skidBuffer[tid].empty()) {
        if (skidBuffer[tid].front()->isLoad()) {
            toRename->iewInfo[tid].dispatchedToLQ++;
        }
        if (skidBuffer[tid].front()->isStore()) {
            toRename->iewInfo[tid].dispatchedToSQ++;
        }

        toRename->iewInfo[tid].dispatched++;

        skidBuffer[tid].pop();
		// 更新这些通信数据以便确保rename阶段能够获得正确的空闲表项数目
    }

    emptyRenameInsts(tid);
	// 清空insts队列
}

template<class Impl>
void
DefaultIEW<Impl>::squashDueToBranch(DynInstPtr &inst, ThreadID tid)
{
    DPRINTF(IEW, "[tid:%i]: Squashing from a specific instruction, PC: %s "
            "[sn:%i].\n", tid, inst->pcState(), inst->seqNum);

    if (!toCommit->squash[tid] ||
            inst->seqNum < toCommit->squashedSeqNum[tid]) {
		// 如果当前没有标记squash状态或者新的squash请求位置比旧的squash
		// 位置更新，需要squash更多的指令，则进行下面的信号更新
        toCommit->squash[tid] = true;
        toCommit->squashedSeqNum[tid] = inst->seqNum;
        toCommit->branchTaken[tid] = inst->pcState().branching();

        TheISA::PCState pc = inst->pcState();
        TheISA::advancePC(pc, inst->staticInst);

        toCommit->pc[tid] = pc;
        toCommit->mispredictInst[tid] = inst;
        toCommit->includeSquashInst[tid] = false;
		// 该信号表示是否需要将出错的指令也squash掉，对于分支指令
		// 来说，显然分支指令是不需要的
		
		// 设置错误分支预测的相关信息和squash信号

        wroteToTimeBuffer = true;
    }

}

template<class Impl>
void
DefaultIEW<Impl>::squashDueToMemOrder(DynInstPtr &inst, ThreadID tid)
{
    DPRINTF(IEW, "[tid:%i]: Memory violation, squashing violator and younger "
            "insts, PC: %s [sn:%i].\n", tid, inst->pcState(), inst->seqNum);
    // Need to include inst->seqNum in the following comparison to cover the
    // corner case when a branch misprediction and a memory violation for the
    // same instruction (e.g. load PC) are detected in the same cycle.  In this
    // case the memory violator should take precedence over the branch
    // misprediction because it requires the violator itself to be included in
    // the squash.
    if (!toCommit->squash[tid] ||
            inst->seqNum <= toCommit->squashedSeqNum[tid]) {
		// 判断当前没有squash操作或者需要更新squash标记
        toCommit->squash[tid] = true;

        toCommit->squashedSeqNum[tid] = inst->seqNum;
        toCommit->pc[tid] = inst->pcState();
        toCommit->mispredictInst[tid] = NULL;

        // Must include the memory violator in the squash.
        toCommit->includeSquashInst[tid] = true;
		// 由于访存指令顺序调换产生的错误要求发生错误的指令也要被squash

        wroteToTimeBuffer = true;
    }
}

template<class Impl>
void
DefaultIEW<Impl>::block(ThreadID tid)
{
    DPRINTF(IEW, "[tid:%u]: Blocking.\n", tid);

    if (dispatchStatus[tid] != Blocked &&
        dispatchStatus[tid] != Unblocking) {
        toRename->iewBlock[tid] = true;
        wroteToTimeBuffer = true;
    }

    // Add the current inputs to the skid buffer so they can be
    // reprocessed when this stage unblocks.
    skidInsert(tid);

    dispatchStatus[tid] = Blocked;
}

template<class Impl>
void
DefaultIEW<Impl>::unblock(ThreadID tid)
{
    DPRINTF(IEW, "[tid:%i]: Reading instructions out of the skid "
            "buffer %u.\n",tid, tid);

    // If the skid bufffer is empty, signal back to previous stages to unblock.
    // Also switch status to running.
    if (skidBuffer[tid].empty()) {
        toRename->iewUnblock[tid] = true;
        wroteToTimeBuffer = true;
        DPRINTF(IEW, "[tid:%i]: Done unblocking.\n",tid);
        dispatchStatus[tid] = Running;
    }
}

template<class Impl>
void
DefaultIEW<Impl>::wakeDependents(DynInstPtr &inst)
{
    instQueue.wakeDependents(inst);
}

template<class Impl>
void
DefaultIEW<Impl>::rescheduleMemInst(DynInstPtr &inst)
{
    instQueue.rescheduleMemInst(inst);
}

template<class Impl>
void
DefaultIEW<Impl>::replayMemInst(DynInstPtr &inst)
{
    instQueue.replayMemInst(inst);
}

template<class Impl>
void
DefaultIEW<Impl>::blockMemInst(DynInstPtr& inst)
{
    instQueue.blockMemInst(inst);
}

template<class Impl>
void
DefaultIEW<Impl>::cacheUnblocked()
{
    instQueue.cacheUnblocked();
}

template<class Impl>
void
DefaultIEW<Impl>::instToCommit(DynInstPtr &inst)
{
    // This function should not be called after writebackInsts in a
    // single cycle.  That will cause problems with an instruction
    // being added to the queue to commit without being processed by
    // writebackInsts prior to being sent to commit.
	
	// 此处说明该函数不可以于同一个周期，在执行了writebackInsts函数
	// 之后立即调用；这样会导致一个指令进入了commit阶段的指令队列
	// 但是没有完成写回操作？？？

    // First check the time slot that this instruction will write
    // to.  If there are free write ports at the time, then go ahead
    // and write the instruction to that time.  If there are not,
    // keep looking back to see where's the first time there's a
    // free slot.
    while ((*iewQueue)[wbCycle].insts[wbNumInst]) {
        ++wbNumInst;
        if (wbNumInst == wbWidth) {
            ++wbCycle;
            wbNumInst = 0;
        }
    }
	// 此处按照wbCycle设置队列中的指令，是因为只有当指令确定写回
	// 操作完成的时候才能够提交，因此在确定了一个指令的wbCycle后
	// 根据wbCycle将指令加入对应的队列中
	
	// wbCycle的增加是根据wbWidth决定的，如果当前写回操作指令数
	// 超过wbWidth，那么新的写回指令就需要等待额外的一个周期来
	// 完成写回操作。

    DPRINTF(IEW, "Current wb cycle: %i, width: %i, numInst: %i\nwbActual:%i\n",
            wbCycle, wbWidth, wbNumInst, wbCycle * wbWidth + wbNumInst);
    // Add finished instruction to queue to commit.
    (*iewQueue)[wbCycle].insts[wbNumInst] = inst;
    (*iewQueue)[wbCycle].size++;
	// 设置指令并更新队列大小
}

template <class Impl>
unsigned
DefaultIEW<Impl>::validInstsFromRename()
{
    unsigned inst_count = 0;

    for (int i=0; i<fromRename->size; i++) {
        if (!fromRename->insts[i]->isSquashed())
            inst_count++;
    }

    return inst_count;
}

template<class Impl>
void
DefaultIEW<Impl>::skidInsert(ThreadID tid)
{
    DynInstPtr inst = NULL;

    while (!insts[tid].empty()) {
        inst = insts[tid].front();

        insts[tid].pop();

        DPRINTF(IEW,"[tid:%i]: Inserting [sn:%lli] PC:%s into "
                "dispatch skidBuffer %i\n",tid, inst->seqNum,
                inst->pcState(),tid);

        skidBuffer[tid].push(inst);
    }

    assert(skidBuffer[tid].size() <= skidBufferMax &&
           "Skidbuffer Exceeded Max Size");
}

template<class Impl>
int
DefaultIEW<Impl>::skidCount()
{
    int max=0;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
        unsigned thread_count = skidBuffer[tid].size();
        if (max < thread_count)
            max = thread_count;
    }

    return max;
}

template<class Impl>
bool
DefaultIEW<Impl>::skidsEmpty()
{
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!skidBuffer[tid].empty())
            return false;
    }

    return true;
}

template <class Impl>
void
DefaultIEW<Impl>::updateStatus()
{
    bool any_unblocking = false;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (dispatchStatus[tid] == Unblocking) {
            any_unblocking = true;
            break;
        }
    }
	// 判断是否存在任何一个线程处于unBlocking状态

    // If there are no ready instructions waiting to be scheduled by the IQ,
    // and there's no stores waiting to write back, and dispatch is not
    // unblocking, then there is no internal activity for the IEW stage.
    instQueue.intInstQueueReads++;
    if (_status == Active && !instQueue.hasReadyInsts() &&
        !ldstQueue.willWB() && !any_unblocking) {
        DPRINTF(IEW, "IEW switching to idle\n");

        deactivateStage();
		// 如果当前IEW阶段处于活跃状态，但是IQ、LSQ没有任何可以处理的
		// 指令，而且没有线程处于unblocking状态，则IEW进入不活跃状态

        _status = Inactive;
    } else if (_status == Inactive && (instQueue.hasReadyInsts() ||
                                       ldstQueue.willWB() ||
                                       any_unblocking)) {
        // Otherwise there is internal activity.  Set to active.
        DPRINTF(IEW, "IEW switching to active\n");

        activateStage();

        _status = Active;
		// 如果当前IEW阶段处于不活跃状态，但是IQ或LSQ有可以处理的
		// 指令，或者有线程处于unblocking状态，则IEW进入活跃状态
    }
}

template <class Impl>
void
DefaultIEW<Impl>::resetEntries()
{
    instQueue.resetEntries();
    ldstQueue.resetEntries();
}

template <class Impl>
bool
DefaultIEW<Impl>::checkStall(ThreadID tid)
{
    bool ret_val(false);

    if (fromCommit->commitInfo[tid].robSquashing) {
        DPRINTF(IEW,"[tid:%i]: Stall from Commit stage detected.\n",tid);
        ret_val = true;
    } else if (instQueue.isFull(tid)) {
        DPRINTF(IEW,"[tid:%i]: Stall: IQ  is full.\n",tid);
        ret_val = true;
    }

    return ret_val;
}

template <class Impl>
void
DefaultIEW<Impl>::checkSignalsAndUpdate(ThreadID tid)
{
    // Check if there's a squash signal, squash if there is
    // Check stall signals, block if there is.
    // If status was Blocked
    //     if so then go to unblocking
    // If status was Squashing
    //     check if squashing is not high.  Switch to running this cycle.

    if (fromCommit->commitInfo[tid].squash) {
        squash(tid);

        if (dispatchStatus[tid] == Blocked ||
            dispatchStatus[tid] == Unblocking) {
            toRename->iewUnblock[tid] = true;
			// 注意此处仅仅更新了信号
            wroteToTimeBuffer = true;
        }

        dispatchStatus[tid] = Squashing;
        fetchRedirect[tid] = false;
        return;
    }
	// 如果检测到来自commit阶段的squash信号则进行squash操作
	// squash操作会触发退出阻塞的操作，并进入squashing状态

    if (fromCommit->commitInfo[tid].robSquashing) {
        DPRINTF(IEW, "[tid:%i]: ROB is still squashing.\n", tid);

        dispatchStatus[tid] = Squashing;
        emptyRenameInsts(tid);
		// 此处操作为何？？？
        wroteToTimeBuffer = true;
    }
	// 对于ROB仍然处于squashing状态的情况维持squashing状态

    if (checkStall(tid)) {
        block(tid);
        dispatchStatus[tid] = Blocked;
        return;
    }
	// 如果存在需要进行阻塞的因素则执行阻塞操作，然后返回

    if (dispatchStatus[tid] == Blocked) {
        // Status from previous cycle was blocked, but there are no more stall
        // conditions.  Switch over to unblocking.
        DPRINTF(IEW, "[tid:%i]: Done blocking, switching to unblocking.\n",
                tid);

        dispatchStatus[tid] = Unblocking;

        unblock(tid);

        return;
    }
	// 如果当前是阻塞状态并且没有因素导致IEW阶段继续阻塞下去，
	// 则尝试退出阻塞状态执行unblock

    if (dispatchStatus[tid] == Squashing) {
        // Switch status to running if rename isn't being told to block or
        // squash this cycle.
        DPRINTF(IEW, "[tid:%i]: Done squashing, switching to running.\n",
                tid);

        dispatchStatus[tid] = Running;

        return;
    }
	// 如果本周期无需继续维持squashing，则进入Running状态
}

template <class Impl>
void
DefaultIEW<Impl>::sortInsts()
{
    int insts_from_rename = fromRename->size;
#ifdef DEBUG
    for (ThreadID tid = 0; tid < numThreads; tid++)
        assert(insts[tid].empty());
#endif
    for (int i = 0; i < insts_from_rename; ++i) {
        insts[fromRename->insts[i]->threadNumber].push(fromRename->insts[i]);
    }
}

template <class Impl>
void
DefaultIEW<Impl>::emptyRenameInsts(ThreadID tid)
{
    DPRINTF(IEW, "[tid:%i]: Removing incoming rename instructions\n", tid);

    while (!insts[tid].empty()) {

        if (insts[tid].front()->isLoad()) {
            toRename->iewInfo[tid].dispatchedToLQ++;
        }
        if (insts[tid].front()->isStore()) {
            toRename->iewInfo[tid].dispatchedToSQ++;
        }

        toRename->iewInfo[tid].dispatched++;

        insts[tid].pop();
    }
	// 更新相关的计数器以保证rename阶段计算有效空闲表项数时可以有效的处理
}

template <class Impl>
void
DefaultIEW<Impl>::wakeCPU()
{
    cpu->wakeCPU();
}

template <class Impl>
void
DefaultIEW<Impl>::activityThisCycle()
{
    DPRINTF(Activity, "Activity this cycle.\n");
    cpu->activityThisCycle();
}

template <class Impl>
inline void
DefaultIEW<Impl>::activateStage()
{
    DPRINTF(Activity, "Activating stage.\n");
    cpu->activateStage(O3CPU::IEWIdx);
}

template <class Impl>
inline void
DefaultIEW<Impl>::deactivateStage()
{
    DPRINTF(Activity, "Deactivating stage.\n");
    cpu->deactivateStage(O3CPU::IEWIdx);
}

template<class Impl>
void
DefaultIEW<Impl>::dispatch(ThreadID tid)
{
    // If status is Running or idle,
    //     call dispatchInsts()
    // If status is Unblocking,
    //     buffer any instructions coming from rename
    //     continue trying to empty skid buffer
    //     check if stall conditions have passed

    if (dispatchStatus[tid] == Blocked) {
        ++iewBlockCycles;
    } else if (dispatchStatus[tid] == Squashing) {
        ++iewSquashCycles;
    }
	// 更新相关状态的周期计数器

    // Dispatch should try to dispatch as many instructions as its bandwidth
    // will allow, as long as it is not currently blocked.
    if (dispatchStatus[tid] == Running ||
        dispatchStatus[tid] == Idle) {
        DPRINTF(IEW, "[tid:%i] Not blocked, so attempting to run "
                "dispatch.\n", tid);
        dispatchInsts(tid);
		// 如果处于Running状态或者Idle，则进行指令分发
    } else if (dispatchStatus[tid] == Unblocking) {
        // Make sure that the skid buffer has something in it if the
        // status is unblocking.
        assert(!skidsEmpty());

        // If the status was unblocking, then instructions from the skid
        // buffer were used.  Remove those instructions and handle
        // the rest of unblocking.
        dispatchInsts(tid);

        ++iewUnblockCycles;

        if (validInstsFromRename()) {
            // Add the current inputs to the skid buffer so they can be
            // reprocessed when this stage unblocks.
            skidInsert(tid);
        }

        unblock(tid);
		// 如果处于unblocking状态，进行指令分发，在获取新的指令到skidBuffer
		// 后尝试退出unblocking状态
    }
}

template <class Impl>
void
DefaultIEW<Impl>::dispatchInsts(ThreadID tid)
{
    // Obtain instructions from skid buffer if unblocking, or queue from rename
    // otherwise.
    std::queue<DynInstPtr> &insts_to_dispatch =
        dispatchStatus[tid] == Unblocking ?
        skidBuffer[tid] : insts[tid];
    int insts_to_add = insts_to_dispatch.size();
	// 选择分发指令的来源并统计对应队列的指令总数

    DynInstPtr inst;
    bool add_to_iq = false;
	// 该变量表示是否需要在进行特殊处理之后将指令插入到IQ中
    int dis_num_inst = 0;

    // Loop through the instructions, putting them in the instruction
    // queue.
    for ( ; dis_num_inst < insts_to_add &&
              dis_num_inst < dispatchWidth;
          ++dis_num_inst)
    {
		// 每个周期都会尝试分发尽量多，但是不超过dispatchWidth的指令
        inst = insts_to_dispatch.front();

        if (dispatchStatus[tid] == Unblocking) {
            DPRINTF(IEW, "[tid:%i]: Issue: Examining instruction from skid "
                    "buffer\n", tid);
        }

        // Make sure there's a valid instruction there.
        assert(inst);

        DPRINTF(IEW, "[tid:%i]: Issue: Adding PC %s [sn:%lli] [tid:%i] to "
                "IQ.\n",
                tid, inst->pcState(), inst->seqNum, inst->threadNumber);

        // Be sure to mark these instructions as ready so that the
        // commit stage can go ahead and execute them, and mark
        // them as issued so the IQ doesn't reprocess them.

        // Check for squashed instructions.
        if (inst->isSquashed()) {
            DPRINTF(IEW, "[tid:%i]: Issue: Squashed instruction encountered, "
                    "not adding to IQ.\n", tid);

            ++iewDispSquashedInsts;

            insts_to_dispatch.pop();

            //Tell Rename That An Instruction has been processed
            if (inst->isLoad()) {
                toRename->iewInfo[tid].dispatchedToLQ++;
            }
            if (inst->isStore()) {
                toRename->iewInfo[tid].dispatchedToSQ++;
            }

            toRename->iewInfo[tid].dispatched++;

            continue;
			// 对于被标记squashed的指令，跳过但是需要更新相关的计数器
        }

        // Check for full conditions.
        if (instQueue.isFull(tid)) {
            DPRINTF(IEW, "[tid:%i]: Issue: IQ has become full.\n", tid);

            // Call function to start blocking.
            block(tid);
			// 如果在分发过程中发现IQ已满，则立即进行阻塞

            // Set unblock to false. Special case where we are using
            // skidbuffer (unblocking) instructions but then we still
            // get full in the IQ.
            toRename->iewUnblock[tid] = false;
			// 这里设置Unblock为false，是因为block函数不设置Unblock标志位
			// 这里更新因为Unblocking状态会将Unblock设置为true

            ++iewIQFullEvents;
            break;
			// 更新计数器以后结束分发新的指令
        }

        // Check LSQ if inst is LD/ST
        if ((inst->isLoad() && ldstQueue.lqFull(tid)) ||
            (inst->isStore() && ldstQueue.sqFull(tid))) {
            DPRINTF(IEW, "[tid:%i]: Issue: %s has become full.\n",tid,
                    inst->isLoad() ? "LQ" : "SQ");

            // Call function to start blocking.
            block(tid);

            // Set unblock to false. Special case where we are using
            // skidbuffer (unblocking) instructions but then we still
            // get full in the IQ.
            toRename->iewUnblock[tid] = false;

            ++iewLSQFullEvents;
            break;
			// 该部分处理和IQ溢出时的处理一致
        }

        // Otherwise issue the instruction just fine.
        if (inst->isLoad()) {
            DPRINTF(IEW, "[tid:%i]: Issue: Memory instruction "
                    "encountered, adding to LSQ.\n", tid);

            // Reserve a spot in the load store queue for this
            // memory access.
            ldstQueue.insertLoad(inst);
            ++iewDispLoadInsts;
            add_to_iq = true;

            toRename->iewInfo[tid].dispatchedToLQ++;
			// load指令仅仅会插入到LSQ中
        } else if (inst->isStore()) {
            DPRINTF(IEW, "[tid:%i]: Issue: Memory instruction "
                    "encountered, adding to LSQ.\n", tid);

            ldstQueue.insertStore(inst);

            ++iewDispStoreInsts;

            if (inst->isStoreConditional()) {
                // Store conditionals need to be set as "canCommit()"
                // so that commit can process them when they reach the
                // head of commit.
                // @todo: This is somewhat specific to Alpha.
                inst->setCanCommit();
                instQueue.insertNonSpec(inst);
				// 指令被同事按照非推测执行的指令插入到IQ中？？？
                add_to_iq = false;

                ++iewDispNonSpecInsts;
            } else {
                add_to_iq = true;
            }
			// 对于SC指令，为什么直接就设置为可以Commit呢？？？

            toRename->iewInfo[tid].dispatchedToSQ++;
        } else if (inst->isMemBarrier() || inst->isWriteBarrier()) {
            // Same as non-speculative stores.
            inst->setCanCommit();
            instQueue.insertBarrier(inst);
			// 按照Barrier将指令插入到IQ中
            add_to_iq = false;
			// 由于此处已经将指令插入到了IQ中，因此add_to_iq设置为false
			// 阻止后面再次将该指令插入到IQ中
        } else if (inst->isNop()) {
            DPRINTF(IEW, "[tid:%i]: Issue: Nop instruction encountered, "
                    "skipping.\n", tid);

            inst->setIssued();
            inst->setExecuted();
            inst->setCanCommit();
			// 对于NOP指令将所有标志设置为true，一路放行

            instQueue.recordProducer(inst);
			// 但是这里为什么依然将该指令作为生产者记录呢？？？

            iewExecutedNop[tid]++;

            add_to_iq = false;
        } else {
            assert(!inst->isExecuted());
			// 此处不可能存在一个已经执行过的指令
            add_to_iq = true;
        }
		// 上面处理的是除了将指令直接加入到IQ以外的特殊操作
		// 比如加入到LSQ或者特殊指令的IQ插入操作

        if (add_to_iq && inst->isNonSpeculative()) {
            DPRINTF(IEW, "[tid:%i]: Issue: Nonspeculative instruction "
                    "encountered, skipping.\n", tid);

            // Same as non-speculative stores.
            inst->setCanCommit();

            // Specifically insert it as nonspeculative.
            instQueue.insertNonSpec(inst);

            ++iewDispNonSpecInsts;

            add_to_iq = false;
			// 将非推测执行的指令插入到IQ中
        }

        // If the instruction queue is not full, then add the
        // instruction.
        if (add_to_iq) {
            instQueue.insert(inst);
			// 这里处理了推测式指令的插入操作
        }

        insts_to_dispatch.pop();

        toRename->iewInfo[tid].dispatched++;

        ++iewDispatchedInsts;

#if TRACING_ON
        inst->dispatchTick = curTick() - inst->fetchTick;
		// 如果开启了debug操作，那么记录指令被分发的周期位置
#endif
        ppDispatch->notify(inst);
		// 更新Probe Points
    }

    if (!insts_to_dispatch.empty()) {
        DPRINTF(IEW,"[tid:%i]: Issue: Bandwidth Full. Blocking.\n", tid);
        block(tid);
        toRename->iewUnblock[tid] = false;
		// 如果发现指令队列不为空，说明队列溢出，或者dispatchWidth已满
		// 则进行阻塞，但是dispatchWidth已满不应该阻塞啊？？？
    }

    if (dispatchStatus[tid] == Idle && dis_num_inst) {
        dispatchStatus[tid] = Running;
        updatedQueues = true;
    }
	// 如果本周期处理了指令，那么状态则会由Idle进入Running状态

    dis_num_inst = 0;
}

template <class Impl>
void
DefaultIEW<Impl>::printAvailableInsts()
{
    int inst = 0;

    std::cout << "Available Instructions: ";

    while (fromIssue->insts[inst]) {

        if (inst%3==0) std::cout << "\n\t";

        std::cout << "PC: " << fromIssue->insts[inst]->pcState()
             << " TN: " << fromIssue->insts[inst]->threadNumber
             << " SN: " << fromIssue->insts[inst]->seqNum << " | ";

        inst++;

    }

    std::cout << "\n";
}
// debug用函数

template <class Impl>
void
DefaultIEW<Impl>::executeInsts()
{
    wbNumInst = 0;
    wbCycle = 0;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
        fetchRedirect[tid] = false;
    }
	// 在执行新的指令以前，首先重置所有活跃线程的fetchRedirect标志位

    // Uncomment this if you want to see all available instructions.
    // @todo This doesn't actually work anymore, we should fix it.
	// printAvailableInsts();

    // Execute/writeback any instructions that are available.
    int insts_to_execute = fromIssue->size;
    int inst_num = 0;
	// 执行阶段会处理尽量多的指令
    for (; inst_num < insts_to_execute;
          ++inst_num) {

        DPRINTF(IEW, "Execute: Executing instructions from IQ.\n");

        DynInstPtr inst = instQueue.getInstToExecute();
		// 从IQ获取可以开始执行的指令

        DPRINTF(IEW, "Execute: Processing PC %s, [tid:%i] [sn:%i].\n",
                inst->pcState(), inst->threadNumber,inst->seqNum);

        // Notify potential listeners that this instruction has started
        // executing
        ppExecute->notify(inst);

        // Check if the instruction is squashed; if so then skip it
        if (inst->isSquashed()) {
            DPRINTF(IEW, "Execute: Instruction was squashed. PC: %s, [tid:%i]"
                         " [sn:%i]\n", inst->pcState(), inst->threadNumber,
                         inst->seqNum);

            // Consider this instruction executed so that commit can go
            // ahead and retire the instruction.
            inst->setExecuted();
			// 对于已经标记为squashed的指令认为立即执行完毕

            // Not sure if I should set this here or just let commit try to
            // commit any squashed instructions.  I like the latter a bit more.
            inst->setCanCommit();
			// 并且可以立即提交

            ++iewExecSquashedInsts;

            continue;
			// 跳过已经被标记为squashed的指令，这里检查到的标记应该来自于
			// IQ的squash操作，再往前的squash处理会在分发时过滤掉
        }

        Fault fault = NoFault;
		// 初始化显示执行过程错误的变量

        // Execute instruction.
        // Note that if the instruction faults, it will be handled
        // at the commit stage.
        if (inst->isMemRef()) {
			// 即使是load/store指令也由IQ获得指令，由LSQ处理执行操作
            DPRINTF(IEW, "Execute: Calculating address for memory "
                    "reference.\n");

            // Tell the LDSTQ to execute this instruction (if it is a load).
            if (inst->isLoad()) {
                // Loads will mark themselves as executed, and their writeback
                // event adds the instruction to the queue to commit
                fault = ldstQueue.executeLoad(inst);
				// 执行对应指令并将对应的指令标记为已经执行的

                if (inst->isTranslationDelayed() &&
                    fault == NoFault) {
                    // A hw page table walk is currently going on; the
                    // instruction must be deferred.
                    DPRINTF(IEW, "Execute: Delayed translation, deferring "
                            "load.\n");
                    instQueue.deferMemInst(inst);
					// 由于TLB miss而进行table walk，该指令需要延迟完成执行
					// 直接继续处理下一个指令
                    continue;
                }

                if (inst->isDataPrefetch() || inst->isInstPrefetch()) {
                    inst->fault = NoFault;
					// 对于预取指令，则会忽略其产生的错误
                }
            } else if (inst->isStore()) {
                fault = ldstQueue.executeStore(inst);

                if (inst->isTranslationDelayed() &&
                    fault == NoFault) {
                    // A hw page table walk is currently going on; the
                    // instruction must be deferred.
                    DPRINTF(IEW, "Execute: Delayed translation, deferring "
                            "store.\n");
                    instQueue.deferMemInst(inst);
                    continue;
					// 这里和load一样因为TLB miss而延迟完成该指令
                }

                // If the store had a fault then it may not have a mem req
                if (fault != NoFault || !inst->readPredicate() ||
                        !inst->isStoreConditional()) {
					// 这里要求指令执行出错（需要在完成之前传递给commit阶段）
					// 或者该指令并非推测读或者并非一个SC指令？？？
                    // If the instruction faulted, then we need to send it along
                    // to commit without the instruction completing.
                    // Send this instruction to commit, also make sure iew stage
                    // realizes there is activity.
                    inst->setExecuted();
					// 标记为已经执行完毕的状态
                    instToCommit(inst);
					// 将指令放到toCommit的指令队列
                    activityThisCycle();
					// 激活本周期来完成相关工作
                }

                // Store conditionals will mark themselves as
                // executed, and their writeback event will add the
                // instruction to the queue to commit.
				// SC指令会由进行disapatch函数单独处理
            } else {
                panic("Unexpected memory type!\n");
            }

        } else {
            // If the instruction has already faulted, then skip executing it.
            // Such case can happen when it faulted during ITLB translation.
            // If we execute the instruction (even if it's a nop) the fault
            // will be replaced and we will lose it.
            if (inst->getFault() == NoFault) {
                inst->execute();
                if (!inst->readPredicate()){
                    inst->forwardOldRegs();
					// 进行前递操作？？？
				}
            }
			// 对于非内存访问指令，如果指令没有出现错误，调用execute函数执行
			// 主要操作，看过staticInst的execute函数，就可以发现这里实际上不仅
			// 执行了execute部分的操作，还执行了几乎包括读操作数/写操作数的内容
			// 因此gem5实际上只是负责模拟状态而已，实际执行效果在保证模拟正确性
			// 的范围内得到有效的处理

            inst->setExecuted();
            instToCommit(inst);
			// 如果指令出错也会设置为执行完成状态，并提交到Commit阶段
        }
		// 至此处，可以执行的指令已经执行

        updateExeInstStats(inst);
		// 根据指令执行错误情况更新线程的状态

        // Check if branch prediction was correct, if not then we need
        // to tell commit to squash in flight instructions.  Only
        // handle this if there hasn't already been something that
        // redirects fetch in this group of instructions.

        // This probably needs to prioritize the redirects if a different
        // scheduler is used.  Currently the scheduler schedules the oldest
        // instruction first, so the branch resolution order will be correct.
		
        ThreadID tid = inst->threadNumber;

        if (!fetchRedirect[tid] ||
            !toCommit->squash[tid] ||
            toCommit->squashedSeqNum[tid] > inst->seqNum) {
			// 下面检查执行的分支指令是否检测到了missPrediction
			// 这里要求之前的操作没有产生commit阶段squash操作请求
			// 或者如果存在squash指令，但是当前的指令不属于被squash的指令

            // Prevent testing for misprediction on load instructions,
            // that have not been executed.
            bool loadNotExecuted = !inst->isExecuted() && inst->isLoad();
			// 避免处理到已经执行过的load指令？？？

            if (inst->mispredicted() && !loadNotExecuted) {
				// 发现了错误预测的分支指令
                fetchRedirect[tid] = true;

                DPRINTF(IEW, "Execute: Branch mispredict detected.\n");
                DPRINTF(IEW, "Predicted target was PC: %s.\n",
                        inst->readPredTarg());
                DPRINTF(IEW, "Execute: Redirecting fetch to PC: %s.\n",
                        inst->pcState());
                // If incorrect, then signal the ROB that it must be squashed.
                squashDueToBranch(inst, tid);
				// 发出squash信号给commit阶段，并标记分支预测的错误信息

                ppMispredict->notify(inst);
				// 更新probe points

                if (inst->readPredTaken()) {
                    predictedTakenIncorrect++;
                } else {
                    predictedNotTakenIncorrect++;
                }
				// 更新相关计数器
            } else if (ldstQueue.violation(tid)) {
				// 执行的时候由LSQ发现了访存顺序错乱问题
                assert(inst->isMemRef());
                // If there was an ordering violation, then get the
                // DynInst that caused the violation.  Note that this
                // clears the violation signal.
                DynInstPtr violator;
                violator = ldstQueue.getMemDepViolator(tid);
				// 从LSQ中获得发生violation的指令

                DPRINTF(IEW, "LDSTQ detected a violation. Violator PC: %s "
                        "[sn:%lli], inst PC: %s [sn:%lli]. Addr is: %#x.\n",
                        violator->pcState(), violator->seqNum,
                        inst->pcState(), inst->seqNum, inst->physEffAddrLow);

                fetchRedirect[tid] = true;
				// violation也需要重新进行fetch重定向

                // Tell the instruction queue that a violation has occured.
                instQueue.violation(inst, violator);
				// 由IQ处理violation，实际上最后会交给MemdepUnit处理该问题

                // Squash.
                squashDueToMemOrder(violator, tid);

                ++memOrderViolationEvents;
            }
        } else {
			// 这种情况是当前不需要进行新的squash检查：
            // Reset any state associated with redirects that will not
            // be used.
            if (ldstQueue.violation(tid)) {
                assert(inst->isMemRef());

                DynInstPtr violator = ldstQueue.getMemDepViolator(tid);

                DPRINTF(IEW, "LDSTQ detected a violation.  Violator PC: "
                        "%s, inst PC: %s.  Addr is: %#x.\n",
                        violator->pcState(), inst->pcState(),
                        inst->physEffAddrLow);
                DPRINTF(IEW, "Violation will not be handled because "
                        "already squashing\n");

                ++memOrderViolationEvents;
				// 对于访存顺序错乱依然需要获取产生错误的指令
            }
        }
    }

    // Update and record activity if we processed any instructions.
    if (inst_num) {
        if (exeStatus == Idle) {
            exeStatus = Running;
        }

        updatedQueues = true;

        cpu->activityThisCycle();
		// 如果本周期处理了至少一条指令，那么更新执行阶段状态
		// 并激活IEW阶段
    }

    // Need to reset this in case a writeback event needs to write into the
    // iew queue.  That way the writeback event will write into the correct
    // spot in the queue.
    wbNumInst = 0;

}

template <class Impl>
void
DefaultIEW<Impl>::writebackInsts()
{
    // Loop through the head of the time buffer and wake any
    // dependents.  These instructions are about to write back.  Also
    // mark scoreboard that this instruction is finally complete.
    // Either have IEW have direct access to scoreboard, or have this
    // as part of backwards communication.
    for (int inst_num = 0; inst_num < wbWidth &&
             toCommit->insts[inst_num]; inst_num++) {
		// 每次最多会处理wbWidth对应的指令的完成操作，toCommit可能对应的
		// 就是iewQueue[0]->insts队列（即本周期就可以完成写回的指令）？？？
        DynInstPtr inst = toCommit->insts[inst_num];
        ThreadID tid = inst->threadNumber;

        DPRINTF(IEW, "Sending instructions to commit, [sn:%lli] PC %s.\n",
                inst->seqNum, inst->pcState());

        iewInstsToCommit[tid]++;
        // Notify potential listeners that execution is complete for this
        // instruction.
        ppToCommit->notify(inst);
		// 更新计数器和probe Points

        // Some instructions will be sent to commit without having
        // executed because they need commit to handle them.
        // E.g. Strictly ordered loads have not actually executed when they
        // are first sent to commit.  Instead commit must tell the LSQ
        // when it's ready to execute the strictly ordered load.
        if (!inst->isSquashed() && inst->isExecuted() && inst->getFault() == NoFault) {
            // 选择一个执行完毕，没有被标记为squash并且执行过程没有产生错误
			// 或者说指令本身没有携带错误的指令进行写回处理
			int dependents = instQueue.wakeDependents(inst);
			// 调用IQ唤醒相关指令，返回数值表示唤醒的相关指令个数

            for (int i = 0; i < inst->numDestRegs(); i++) {
                //mark as Ready
                DPRINTF(IEW,"Setting Destination Register %i (%s)\n",
                        inst->renamedDestRegIdx(i)->index(),
                        inst->renamedDestRegIdx(i)->className());
                scoreboard->setReg(inst->renamedDestRegIdx(i));
				// 在记分板中将该指令的所有目的寄存器设置为ready状态
            }

            if (dependents) {
                producerInst[tid]++;
                consumerInst[tid]+= dependents;
            }
            writebackCount[tid]++;
        }
    }
}

template<class Impl>
void
DefaultIEW<Impl>::tick()
{
    wbNumInst = 0;
    wbCycle = 0;

    wroteToTimeBuffer = false;
    updatedQueues = false;

    sortInsts();
	// 将fromRename的指令按照线程移入insts队列中

    // Free function units marked as being freed this cycle.
    fuPool->processFreeUnits();

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    // Check stall and squash signals, dispatch any instructions.
    while (threads != end) {
        ThreadID tid = *threads++;

        DPRINTF(IEW,"Issue: Processing [tid:%i]\n",tid);

        checkSignalsAndUpdate(tid);
        dispatch(tid);
    }
	// 更新每个线程的状态，并进行指令的分发

    if (exeStatus != Squashing) {
        executeInsts();
        writebackInsts();
		// 如果执行阶段没有进行squash操作，就执行指令并写回指令

        // Have the instruction queue try to schedule any ready instructions.
        // (In actuality, this scheduling is for instructions that will
        // be executed next cycle.)
        instQueue.scheduleReadyInsts();
		// 由于新的指令完成，会唤醒相关的指令，此处进行相应的调度操作
		// 将下一个周期可以执行的指令放到ready list中

        // Also should advance its own time buffers if the stage ran.
        // Not the best place for it, but this works (hopefully).
        issueToExecQueue.advance();
		// 该操作会将issue本周期处理的可以发射的指令发送到execute阶段
    }

    bool broadcast_free_entries = false;

    if (updatedQueues || exeStatus == Running || updateLSQNextCycle) {
		// 如果本周期更新了任何一个队列或者当前的执行状态是Running
		// 表示IQ、LSQ中有新的空闲位置出现了
        exeStatus = Idle;
        updateLSQNextCycle = false;
		// 重置状态并设置标志位表示需要更新空闲标志位
        broadcast_free_entries = true;
    }

    // Writeback any stores using any leftover bandwidth.
    ldstQueue.writebackStores();
	// 这里才会使用剩余的写回带宽将store指令进行写回处理？？？

    // Check the committed load/store signals to see if there's a load
    // or store to commit.  Also check if it's being told to execute a
    // nonspeculative instruction.
    // This is pretty inefficient...
	
	// 检查是否存在提交过的load/store指令，以及是否需要非推测式的执行指令
    threads = activeThreads->begin();
    while (threads != end) {
        ThreadID tid = (*threads++);

        DPRINTF(IEW,"Processing [tid:%i]\n",tid);

        // Update structures based on instructions committed.
        if (fromCommit->commitInfo[tid].doneSeqNum != 0 &&
            !fromCommit->commitInfo[tid].squash &&
            !fromCommit->commitInfo[tid].robSquashing) {
			// 发现本周期commit阶段提交了一个指令，并且没有发出squash相关信号

            ldstQueue.commitStores(fromCommit->commitInfo[tid].doneSeqNum,tid);
            ldstQueue.commitLoads(fromCommit->commitInfo[tid].doneSeqNum,tid);
            updateLSQNextCycle = true;
            instQueue.commit(fromCommit->commitInfo[tid].doneSeqNum,tid);
			// 尝试让IQ和LSQ提交在当前commit提交指令之前的所有指令
        }

        if (fromCommit->commitInfo[tid].nonSpecSeqNum != 0) {
			// commit阶段要求非推测式的执行一个给定的指令
            //DPRINTF(IEW,"NonspecInst from thread %i",tid);
            if (fromCommit->commitInfo[tid].strictlyOrdered) {
                instQueue.replayMemInst(
                    fromCommit->commitInfo[tid].strictlyOrderedLoad);
                fromCommit->commitInfo[tid].strictlyOrderedLoad->setAtCommit();
				// 严格按序的所有需要replay的访存指令执行
            } else {
                instQueue.scheduleNonSpec(
                    fromCommit->commitInfo[tid].nonSpecSeqNum);
				// 将需要指令的指令按照非推测式执行调度
            }
        }

        if (broadcast_free_entries) {
            toFetch->iewInfo[tid].iqCount =
                instQueue.getCount(tid);
            toFetch->iewInfo[tid].ldstqCount =
                ldstQueue.getCount(tid);

            toRename->iewInfo[tid].usedIQ = true;
            toRename->iewInfo[tid].freeIQEntries =
                instQueue.numFreeEntries(tid);
            toRename->iewInfo[tid].usedLSQ = true;

            toRename->iewInfo[tid].freeLQEntries =
                ldstQueue.numFreeLoadEntries(tid);
            toRename->iewInfo[tid].freeSQEntries =
                ldstQueue.numFreeStoreEntries(tid);

            wroteToTimeBuffer = true;
        }
		// 更新不同队列的空闲表项信息到toFetch阶段

        DPRINTF(IEW, "[tid:%i], Dispatch dispatched %i instructions.\n",
                tid, toRename->iewInfo[tid].dispatched);
    }

    DPRINTF(IEW, "IQ has %i free entries (Can schedule: %i).  "
            "LQ has %i free entries. SQ has %i free entries.\n",
            instQueue.numFreeEntries(), instQueue.hasReadyInsts(),
            ldstQueue.numFreeLoadEntries(), ldstQueue.numFreeStoreEntries());

    updateStatus();
	// 更新IEW阶段的整体的活跃状态

    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity this cycle.\n");
        cpu->activityThisCycle();
    }
}

template <class Impl>
void
DefaultIEW<Impl>::updateExeInstStats(DynInstPtr &inst)
{
    ThreadID tid = inst->threadNumber;

    iewExecutedInsts++;

#if TRACING_ON
    if (DTRACE(O3PipeView)) {
        inst->completeTick = curTick() - inst->fetchTick;
    }
	// 更新指令完成执行的时间信息
#endif

    //
    //  Control operations
    //
    if (inst->isControl())
        iewExecutedBranches[tid]++;

    //
    //  Memory operations
    //
    if (inst->isMemRef()) {
        iewExecutedRefs[tid]++;

        if (inst->isLoad()) {
            iewExecLoadInsts[tid]++;
        }
    }
	// 更新相关计数器
}

template <class Impl>
void
DefaultIEW<Impl>::checkMisprediction(DynInstPtr &inst)
{
    ThreadID tid = inst->threadNumber;

    if (!fetchRedirect[tid] ||
        !toCommit->squash[tid] ||
        toCommit->squashedSeqNum[tid] > inst->seqNum) {
		// 要求此时尚未出现fetch重定向操作以及squash操作
		// 或者该指令导致的新squash操作会更新之前的squash操作

        if (inst->mispredicted()) {
            fetchRedirect[tid] = true;

            DPRINTF(IEW, "Execute: Branch mispredict detected.\n");
            DPRINTF(IEW, "Predicted target was PC:%#x, NPC:%#x.\n",
                    inst->predInstAddr(), inst->predNextInstAddr());
            DPRINTF(IEW, "Execute: Redirecting fetch to PC: %#x,"
                    " NPC: %#x.\n", inst->nextInstAddr(),
                    inst->nextInstAddr());
            // If incorrect, then signal the ROB that it must be squashed.
            squashDueToBranch(inst, tid);
			// 发出squash请求信号
			
            if (inst->readPredTaken()) {
                predictedTakenIncorrect++;
            } else {
                predictedNotTakenIncorrect++;
            }
			// 更新相关的错误分支预测计数器
        }
    }
}

#endif//__CPU_O3_IEW_IMPL_IMPL_HH__
