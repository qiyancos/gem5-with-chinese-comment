/*
 * Copyright 2014 Google, Inc.
 * Copyright (c) 2010-2014, 2017 ARM Limited
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
#ifndef __CPU_O3_COMMIT_IMPL_HH__
#define __CPU_O3_COMMIT_IMPL_HH__

#include <algorithm>
#include <set>
#include <string>

#include "arch/utility.hh"
#include "base/loader/symtab.hh"
#include "base/cp_annotate.hh"
#include "config/the_isa.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/o3/commit.hh"
#include "cpu/o3/thread_state.hh"
#include "cpu/base.hh"
#include "cpu/exetrace.hh"
#include "cpu/timebuf.hh"
#include "debug/Activity.hh"
#include "debug/Commit.hh"
#include "debug/CommitRate.hh"
#include "debug/Drain.hh"
#include "debug/ExecFaulting.hh"
#include "debug/O3PipeView.hh"
#include "params/DerivO3CPU.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"

using namespace std;

template <class Impl>
void
DefaultCommit<Impl>::processTrapEvent(ThreadID tid)
{
    // This will get reset by commit if it was switched out at the
    // time of this event processing.
    trapSquash[tid] = true;
}

template <class Impl>
DefaultCommit<Impl>::DefaultCommit(O3CPU *_cpu, DerivO3CPUParams *params)
    : cpu(_cpu),
      iewToCommitDelay(params->iewToCommitDelay),
      commitToIEWDelay(params->commitToIEWDelay),
      renameToROBDelay(params->renameToROBDelay),
      fetchToCommitDelay(params->commitToFetchDelay),
      renameWidth(params->renameWidth),
      commitWidth(params->commitWidth),
      numThreads(params->numThreads),
      drainPending(false),
      drainImminent(false),
      trapLatency(params->trapLatency),
      canHandleInterrupts(true),
      avoidQuiesceLiveLock(false)
{
    if (commitWidth > Impl::MaxWidth)
        fatal("commitWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/impl.hh\n",
             commitWidth, static_cast<int>(Impl::MaxWidth));

    _status = Active;
    _nextStatus = Inactive;
    std::string policy = params->smtCommitPolicy;

    //Convert string to lowercase
    std::transform(policy.begin(), policy.end(), policy.begin(),
                   (int(*)(int)) tolower);

    //Assign commit policy
    if (policy == "aggressive"){
        commitPolicy = Aggressive;

        DPRINTF(Commit,"Commit Policy set to Aggressive.\n");
    } else if (policy == "roundrobin"){
        commitPolicy = RoundRobin;

        //Set-Up Priority List
        for (ThreadID tid = 0; tid < numThreads; tid++) {
            priority_list.push_back(tid);
        }

        DPRINTF(Commit,"Commit Policy set to Round Robin.\n");
    } else if (policy == "oldestready"){
        commitPolicy = OldestReady;

        DPRINTF(Commit,"Commit Policy set to Oldest Ready.");
    } else {
        assert(0 && "Invalid SMT Commit Policy. Options Are: {Aggressive,"
               "RoundRobin,OldestReady}");
    }

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        commitStatus[tid] = Idle;
        changedROBNumEntries[tid] = false;
        checkEmptyROB[tid] = false;
        trapInFlight[tid] = false;
        committedStores[tid] = false;
        trapSquash[tid] = false;
        tcSquash[tid] = false;
        pc[tid].set(0);
        lastCommitedSeqNum[tid] = 0;
        squashAfterInst[tid] = NULL;
    }
    interrupt = NoFault;
}

template <class Impl>
std::string
DefaultCommit<Impl>::name() const
{
    return cpu->name() + ".commit";
}

template <class Impl>
void
DefaultCommit<Impl>::regProbePoints()
{
    ppCommit = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(), "Commit");
    ppCommitStall = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(), "CommitStall");
    ppSquash = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(), "Squash");
}

template <class Impl>
void
DefaultCommit<Impl>::regStats()
{
    using namespace Stats;
    commitSquashedInsts
        .name(name() + ".commitSquashedInsts")
        .desc("The number of squashed insts skipped by commit")
        .prereq(commitSquashedInsts);

    commitNonSpecStalls
        .name(name() + ".commitNonSpecStalls")
        .desc("The number of times commit has been forced to stall to "
              "communicate backwards")
        .prereq(commitNonSpecStalls);

    branchMispredicts
        .name(name() + ".branchMispredicts")
        .desc("The number of times a branch was mispredicted")
        .prereq(branchMispredicts);

    numCommittedDist
        .init(0,commitWidth,1)
        .name(name() + ".committed_per_cycle")
        .desc("Number of insts commited each cycle")
        .flags(Stats::pdf)
        ;

    instsCommitted
        .init(cpu->numThreads)
        .name(name() + ".committedInsts")
        .desc("Number of instructions committed")
        .flags(total)
        ;

    opsCommitted
        .init(cpu->numThreads)
        .name(name() + ".committedOps")
        .desc("Number of ops (including micro ops) committed")
        .flags(total)
        ;

    statComSwp
        .init(cpu->numThreads)
        .name(name() + ".swp_count")
        .desc("Number of s/w prefetches committed")
        .flags(total)
        ;

    statComRefs
        .init(cpu->numThreads)
        .name(name() +  ".refs")
        .desc("Number of memory references committed")
        .flags(total)
        ;

    statComLoads
        .init(cpu->numThreads)
        .name(name() +  ".loads")
        .desc("Number of loads committed")
        .flags(total)
        ;

    statComMembars
        .init(cpu->numThreads)
        .name(name() +  ".membars")
        .desc("Number of memory barriers committed")
        .flags(total)
        ;

    statComBranches
        .init(cpu->numThreads)
        .name(name() + ".branches")
        .desc("Number of branches committed")
        .flags(total)
        ;

    statComFloating
        .init(cpu->numThreads)
        .name(name() + ".fp_insts")
        .desc("Number of committed floating point instructions.")
        .flags(total)
        ;

    statComVector
        .init(cpu->numThreads)
        .name(name() + ".vec_insts")
        .desc("Number of committed Vector instructions.")
        .flags(total)
        ;

    statComInteger
        .init(cpu->numThreads)
        .name(name()+".int_insts")
        .desc("Number of committed integer instructions.")
        .flags(total)
        ;

    statComFunctionCalls
        .init(cpu->numThreads)
        .name(name()+".function_calls")
        .desc("Number of function calls committed.")
        .flags(total)
        ;

    statCommittedInstType
        .init(numThreads,Enums::Num_OpClass)
        .name(name() + ".op_class")
        .desc("Class of committed instruction")
        .flags(total | pdf | dist)
        ;
    statCommittedInstType.ysubnames(Enums::OpClassStrings);

    commitEligibleSamples
        .name(name() + ".bw_lim_events")
        .desc("number cycles where commit BW limit reached")
        ;
}

template <class Impl>
void
DefaultCommit<Impl>::setThreads(std::vector<Thread *> &threads)
{
    thread = threads;
}

template <class Impl>
void
DefaultCommit<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    timeBuffer = tb_ptr;

    // Setup wire to send information back to IEW.
    toIEW = timeBuffer->getWire(0);

    // Setup wire to read data from IEW (for the ROB).
    robInfoFromIEW = timeBuffer->getWire(-iewToCommitDelay);
}

template <class Impl>
void
DefaultCommit<Impl>::setFetchQueue(TimeBuffer<FetchStruct> *fq_ptr)
{
    fetchQueue = fq_ptr;

    // Setup wire to get instructions from rename (for the ROB).
    fromFetch = fetchQueue->getWire(-fetchToCommitDelay);
}

template <class Impl>
void
DefaultCommit<Impl>::setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr)
{
    renameQueue = rq_ptr;

    // Setup wire to get instructions from rename (for the ROB).
    fromRename = renameQueue->getWire(-renameToROBDelay);
}

template <class Impl>
void
DefaultCommit<Impl>::setIEWQueue(TimeBuffer<IEWStruct> *iq_ptr)
{
    iewQueue = iq_ptr;

    // Setup wire to get instructions from IEW.
    fromIEW = iewQueue->getWire(-iewToCommitDelay);
}

template <class Impl>
void
DefaultCommit<Impl>::setIEWStage(IEW *iew_stage)
{
    iewStage = iew_stage;
}

template<class Impl>
void
DefaultCommit<Impl>::setActiveThreads(list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;
}

template <class Impl>
void
DefaultCommit<Impl>::setRenameMap(RenameMap rm_ptr[])
{
    for (ThreadID tid = 0; tid < numThreads; tid++)
        renameMap[tid] = &rm_ptr[tid];
}

template <class Impl>
void
DefaultCommit<Impl>::setROB(ROB *rob_ptr)
{
    rob = rob_ptr;
}

template <class Impl>
void
DefaultCommit<Impl>::startupStage()
{
    rob->setActiveThreads(activeThreads);
    rob->resetEntries();

    // Broadcast the number of free entries.
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        toIEW->commitInfo[tid].usedROB = true;
        toIEW->commitInfo[tid].freeROBEntries = rob->numFreeEntries(tid);
        toIEW->commitInfo[tid].emptyROB = true;
		// 表示使用ROB，但是当前ROB为空，没有存放任何指令
    }

    // Commit must broadcast the number of free entries it has at the
    // start of the simulation, so it starts as active.
    cpu->activateStage(O3CPU::CommitIdx);

    cpu->activityThisCycle();
}

template <class Impl>
void
DefaultCommit<Impl>::drain()
{
    drainPending = true;
}

template <class Impl>
void
DefaultCommit<Impl>::drainResume()
{
    drainPending = false;
    drainImminent = false;
}

template <class Impl>
void
DefaultCommit<Impl>::drainSanityCheck() const
{
    assert(isDrained());
    rob->drainSanityCheck();
}

template <class Impl>
bool
DefaultCommit<Impl>::isDrained() const
{
    /* Make sure no one is executing microcode. There are two reasons
     * for this:
     * - Hardware virtualized CPUs can't switch into the middle of a
     *   microcode sequence.
     * - The current fetch implementation will most likely get very
     *   confused if it tries to start fetching an instruction that
     *   is executing in the middle of a ucode sequence that changes
     *   address mappings. This can happen on for example x86.
     */
	// 之所以要求当前不能执行微操作，首先排空流水线一般可能和CPU之间
	// 进行switch操作有关，而进入微操作的话排空的可能只是当前执行的一
	// 部分微操作，尚未执行的微操作会被切断，这样会出现问题；第二个问
	// 并题没有理解？？？
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (pc[tid].microPC() != 0)
            return false;
    }
	// 这里其实只是要求每个线程都恰好提交到单个指令的微操作边界

    /* Make sure that all instructions have finished committing before
     * declaring the system as drained. We want the pipeline to be
     * completely empty when we declare the CPU to be drained. This
     * makes debugging easier since CPU handover and restoring from a
     * checkpoint with a different CPU should have the same timing.
     */
	// 确定有效的处理了所有的中断并且提交/或者squash掉位于ROB中的指令
    return rob->isEmpty() &&
        interrupt == NoFault;
}

template <class Impl>
void
DefaultCommit<Impl>::takeOverFrom()
{
    _status = Active;
    _nextStatus = Inactive;
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        commitStatus[tid] = Idle;
        changedROBNumEntries[tid] = false;
        trapSquash[tid] = false;
        tcSquash[tid] = false;
        squashAfterInst[tid] = NULL;
    }
    rob->takeOverFrom();
}

template <class Impl>
void
DefaultCommit<Impl>::deactivateThread(ThreadID tid)
{
    list<ThreadID>::iterator thread_it = std::find(priority_list.begin(),
            priority_list.end(), tid);

    if (thread_it != priority_list.end()) {
        priority_list.erase(thread_it);
    }
}


template <class Impl>
void
DefaultCommit<Impl>::updateStatus()
{
    // reset ROB changed variable
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;
        changedROBNumEntries[tid] = false;
		// 在本周期末重置临时标志位

        // Also check if any of the threads has a trap pending
        if (commitStatus[tid] == TrapPending ||
            commitStatus[tid] == FetchTrapPending) {
            _nextStatus = Active;
			// 如果当前是因为trap处理停止，尝试在下一个周期进入活跃状态
        }
    }

    if (_nextStatus == Inactive && _status == Active) {
        DPRINTF(Activity, "Deactivating stage.\n");
        cpu->deactivateStage(O3CPU::CommitIdx);
    } else if (_nextStatus == Active && _status == Inactive) {
        DPRINTF(Activity, "Activating stage.\n");
        cpu->activateStage(O3CPU::CommitIdx);
    }
	// 由CPU激活或者停止commit阶段

    _status = _nextStatus;
	// 在本周期末更新commit阶段的整体状态
}

template <class Impl>
bool
DefaultCommit<Impl>::changedROBEntries()
{
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (changedROBNumEntries[tid]) {
            return true;
        }
    }

    return false;
}

template <class Impl>
size_t
DefaultCommit<Impl>::numROBFreeEntries(ThreadID tid)
{
    return rob->numFreeEntries(tid);
}

template <class Impl>
void
DefaultCommit<Impl>::generateTrapEvent(ThreadID tid, Fault inst_fault)
{
    DPRINTF(Commit, "Generating trap event for [tid:%i]\n", tid);

    EventFunctionWrapper *trap = new EventFunctionWrapper(
        [this, tid]{ processTrapEvent(tid); },
        "Trap", true, Event::CPU_Tick_Pri);

    Cycles latency = dynamic_pointer_cast<SyscallRetryFault>(inst_fault) ?
                     cpu->syscallRetryLatency : trapLatency;
	// 确定处理该trap的延迟，如果是一个重试的syscall，则使用对应的延迟

    cpu->schedule(trap, cpu->clockEdge(latency));
	// 告知CPU在给定延迟后结束对该trap的处理，恢复状态
    trapInFlight[tid] = true;
    thread[tid]->trapPending = true;
}

template <class Impl>
void
DefaultCommit<Impl>::generateTCEvent(ThreadID tid)
{
    assert(!trapInFlight[tid]);
    DPRINTF(Commit, "Generating TC squash event for [tid:%i]\n", tid);

    tcSquash[tid] = true;
}

template <class Impl>
void
DefaultCommit<Impl>::squashAll(ThreadID tid)
{
    // If we want to include the squashing instruction in the squash,
    // then use one older sequence number.
    // Hopefully this doesn't mess things up.  Basically I want to squash
    // all instructions of this thread.
    InstSeqNum squashed_inst = rob->isEmpty(tid) ?
        lastCommitedSeqNum[tid] : rob->readHeadInst(tid)->seqNum - 1;
	// 如果当前该线程ROB为空，则将该线程上一次提交指令的编号作为squash起点
	// 否则获取ROB顶部指令，即当前线程ROB中最老的指令编号减1作为squash起点
	// 从上述选择来看所有尚未提交的该线程的指令都会被删除

    // All younger instructions will be squashed. Set the sequence
    // number as the youngest instruction in the ROB (0 in this case.
    // Hopefully nothing breaks.)
    youngestSeqNum[tid] = lastCommitedSeqNum[tid];
	// 由于ROB中的指令均会被squash掉，因此这里将最近提交的该线程
	// 的指令作为ROB中最新的且有效的指令（虽然实际和描述不相符）
	// 但是似乎并不会产生影响

    rob->squash(squashed_inst, tid);
    changedROBNumEntries[tid] = true;
	// 调用rob的squash函数清理该线程的指令，并设置标志位表示
	// 该线程ROB的空闲表项数有所更新

    // Send back the sequence number of the squashed instruction.
    toIEW->commitInfo[tid].doneSeqNum = squashed_inst;
    // Send back the squash signal to tell stages that they should
    // squash.
    toIEW->commitInfo[tid].squash = true;

    // Send back the rob squashing signal so other stages know that
    // the ROB is in the process of squashing.
    toIEW->commitInfo[tid].robSquashing = true;

    toIEW->commitInfo[tid].mispredictInst = NULL;
    toIEW->commitInfo[tid].squashInst = NULL;

    toIEW->commitInfo[tid].pc = pc[tid];
	// 设置到IEW阶段的squash请求和相关信息，对于IEW之前阶段的squash
	// 由IEW阶段逐步向前传递
}

template <class Impl>
void
DefaultCommit<Impl>::squashFromTrap(ThreadID tid)
{
    squashAll(tid);

    DPRINTF(Commit, "Squashing from trap, restarting at PC %s\n", pc[tid]);

    thread[tid]->trapPending = false;
    thread[tid]->noSquashFromTC = false;
    trapInFlight[tid] = false;

    trapSquash[tid] = false;

    commitStatus[tid] = ROBSquashing;
    cpu->activityThisCycle();
}

template <class Impl>
void
DefaultCommit<Impl>::squashFromTC(ThreadID tid)
{
    squashAll(tid);

    DPRINTF(Commit, "Squashing from TC, restarting at PC %s\n", pc[tid]);

    thread[tid]->noSquashFromTC = false;
    assert(!thread[tid]->trapPending);

    commitStatus[tid] = ROBSquashing;
    cpu->activityThisCycle();

    tcSquash[tid] = false;
}

template <class Impl>
void
DefaultCommit<Impl>::squashFromSquashAfter(ThreadID tid)
{
    DPRINTF(Commit, "Squashing after squash after request, "
            "restarting at PC %s\n", pc[tid]);

    squashAll(tid);
	// 此处直接squashAll了是因为调用该函数时，commit函数处理到了被
	// 标记为squashAfter的指令后面的指令，被标记指令以及之后的指令
	// 均将被squash。因此这里可以保证不该被squash的指令均已经提交
	
    // Make sure to inform the fetch stage of which instruction caused
    // the squash. It'll try to re-fetch an instruction executing in
    // microcode unless this is set.
    toIEW->commitInfo[tid].squashInst = squashAfterInst[tid];
    squashAfterInst[tid] = NULL;
	// 使用过的squashAfter标记被重置，注意对于其他阶段来说，该标记
	// 对应指令之后的（不包含该指令）才会被squash
	
    commitStatus[tid] = ROBSquashing;
	// 这里设置是多余的，因为squash一定会设置为该状态
    cpu->activityThisCycle();
}

template <class Impl>
void
DefaultCommit<Impl>::squashAfter(ThreadID tid, DynInstPtr &head_inst)
{
    DPRINTF(Commit, "Executing squash after for [tid:%i] inst [sn:%lli]\n",
            tid, head_inst->seqNum);

    assert(!squashAfterInst[tid] || squashAfterInst[tid] == head_inst);
    commitStatus[tid] = SquashAfterPending;
    squashAfterInst[tid] = head_inst;
}

template <class Impl>
void
DefaultCommit<Impl>::tick()
{
    wroteToTimeBuffer = false;
    _nextStatus = Inactive;
	// 设置默认的状态变化

    if (activeThreads->empty())
        return;
	// 如果没有活跃线程则直接返回，不需要提交任何线程的指令

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    // Check if any of the threads are done squashing.  Change the
    // status if they are done.
	// 检查所有的活跃线程的ROB squash操作是否完成，并更新相应的状态
    while (threads != end) {
        ThreadID tid = *threads++;

        // Clear the bit saying if the thread has committed stores
        // this cycle.
        committedStores[tid] = false;
		// 重置该标志位

        if (commitStatus[tid] == ROBSquashing) {

            if (rob->isDoneSquashing(tid)) {
				// 如果该线程当前正在处理ROB的squash，并且ROB状态表明
				// ROB确实完成了squash操作，那么将状态转换到Running
                commitStatus[tid] = Running;
            } else {
                DPRINTF(Commit,"[tid:%u]: Still Squashing, cannot commit any"
                        " insts this cycle.\n", tid);
                rob->doSquash(tid);
                toIEW->commitInfo[tid].robSquashing = true;
                wroteToTimeBuffer = true;
				// 否则调用ROB的doSquash函数处理本周期的squash操作
				// 注意ROB并不会自己tick相信的一周期的操作
            }
        }
    }

    commit();
	// commit阶段的核心函数，尝试提交ROB中的指令

    markCompletedInsts();
	// 根据IEW阶段的信号将给定指令标记为完成状态，注意
	// commit函数刚刚已经提交了指令，因此当前周期完成的
	// 指令是不会在当前周期提交的

    threads = activeThreads->begin();
	
	// 根据ROB中最顶部的指令是否完成，来确定下一个周期的状态
    while (threads != end) {
        ThreadID tid = *threads++;

        if (!rob->isEmpty(tid) && rob->readHeadInst(tid)->readyToCommit()) {
			
            // The ROB has more instructions it can commit. Its next status
            // will be active.
            _nextStatus = Active;

            DynInstPtr inst = rob->readHeadInst(tid);

            DPRINTF(Commit,"[tid:%i]: Instruction [sn:%lli] PC %s is head of"
                    " ROB and ready to commit\n",
                    tid, inst->seqNum, inst->pcState());
			// 如果该线程的ROB不为空，并且其顶部的指令可以提交，那么下个
			// 周期的Commit阶段将会标记为活跃状态
			
        } else if (!rob->isEmpty(tid)) {
            DynInstPtr inst = rob->readHeadInst(tid);

            ppCommitStall->notify(inst);
			// 通过Probe point说明因为哪一个指令导致commit阶段阻塞

            DPRINTF(Commit,"[tid:%i]: Can't commit, Instruction [sn:%lli] PC "
                    "%s is head of ROB and not ready\n",
                    tid, inst->seqNum, inst->pcState());
        }

        DPRINTF(Commit, "[tid:%i]: ROB has %d insts & %d free entries.\n",
                tid, rob->countInsts(tid), rob->numFreeEntries(tid));
    }


    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity This Cycle.\n");
        cpu->activityThisCycle();
		// 如果commit阶段本周期更新了timeBuffer，那么将会激活CPU下周期运行
    }

    updateStatus();
	// 在本周期末更新_status为_nextStatus
}

template <class Impl>
void
DefaultCommit<Impl>::handleInterrupt()
{
    // Verify that we still have an interrupt to handle
    if (!cpu->checkInterrupts(cpu->tcBase(0))) {
		// 判断对应线程的上下文中是否记录了需要处理的中断
        DPRINTF(Commit, "Pending interrupt is cleared by master before "
                "it got handled. Restart fetching from the orig path.\n");
        toIEW->commitInfo[0].clearInterrupt = true;
        interrupt = NoFault;
        avoidQuiesceLiveLock = true;
		// 这里表示没有发现需要处理的中断，重设标志位和通信信号
        return;
    }

    // Wait until all in flight instructions are finished before enterring
    // the interrupt.
    if (canHandleInterrupts && cpu->instList.empty()) {
		// 在处理中断前居然要求所有instList中的指令完成，这是一定的
		// 因为在遇到interrupt的时候，会进行阻塞并将中断之后的所有
		// 指令都squash掉，这时候完成所有instList指令是ok的
		
        // Squash or record that I need to squash this cycle if
        // an interrupt needed to be handled.
        DPRINTF(Commit, "Interrupt detected.\n");

        // Clear the interrupt now that it's going to be handled
        toIEW->commitInfo[0].clearInterrupt = true;
		// 依然预先设置标志位表示中断已经处理完毕

        assert(!thread[0]->noSquashFromTC);
        thread[0]->noSquashFromTC = true;
		// 设置该标志位放置因为线程上下文变化导致的squash操作影响
		// 下面的中断的处理

        if (cpu->checker) {
            cpu->checker->handlePendingInt();
        }

        // CPU will handle interrupt. Note that we ignore the local copy of
        // interrupt. This is because the local copy may no longer be the
        // interrupt that the interrupt controller thinks is being handled.
        cpu->processInterrupts(cpu->getInterrupts());
		// 由CPU的函数处理中断

        thread[0]->noSquashFromTC = false;
		// 恢复noSquashFromTC

        commitStatus[0] = TrapPending;
        interrupt = NoFault;
		// 清空中断信息，并进入TrapPending状态，表示正在处理一个
		// trap，等待执行状态更新恢复到Running状态？？？

        // Generate trap squash event.
        generateTrapEvent(0, interrupt);
		// 设置该中断处理的延迟和相应的标志位

        avoidQuiesceLiveLock = false;
    } else {
        DPRINTF(Commit, "Interrupt pending: instruction is %sin "
                "flight, ROB is %sempty\n",
                canHandleInterrupts ? "not " : "",
                cpu->instList.empty() ? "" : "not " );
		// 有中断但是不满足中断的处理条件
    }
}

template <class Impl>
void
DefaultCommit<Impl>::propagateInterrupt()
{
    // Don't propagate intterupts if we are currently handling a trap or
    // in draining and the last observable instruction has been committed.
    if (commitStatus[0] == TrapPending || interrupt || trapSquash[0] ||
            tcSquash[0] || drainImminent)
        return;
	// 如果现在已经存在处理中的中断或者存在中断或者正在处理中断相关的内容
	// 或正在进行squash操作或者准备进行排空流水线处理，则不做处理

    // Process interrupts if interrupts are enabled, not in PAL
    // mode, and no other traps or external squashes are currently
    // pending.
    // @todo: Allow other threads to handle interrupts.

    // Get any interrupt that happened
    interrupt = cpu->getInterrupts();

    // Tell fetch that there is an interrupt pending.  This
    // will make fetch wait until it sees a non PAL-mode PC,
    // at which point it stops fetching instructions.
    if (interrupt != NoFault)
        toIEW->commitInfo[0].interruptPending = true;
	// 这里会要求fetch阶段只能获取和interrupt处理相关的pal指令
	// 一旦遇到非pal指令将会停止取指，指导trap处理完毕
}

template <class Impl>
void
DefaultCommit<Impl>::commit()
{
    if (FullSystem) {
        // Check if we have a interrupt and get read to handle it
        if (cpu->checkInterrupts(cpu->tcBase(0)))
            propagateInterrupt();
    }

    ////////////////////////////////////
    // Check for any possible squashes, handle them first
    ////////////////////////////////////
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    int num_squashing_threads = 0;

    while (threads != end) {
        ThreadID tid = *threads++;

        // Not sure which one takes priority.  I think if we have
        // both, that's a bad sign.
		// 此处的设计不考虑处理不同原因导致squash的优先级，是因为
		// 作者认为trapSquash和tcSquash并不会同时发生
        if (trapSquash[tid]) {
            assert(!tcSquash[tid]);
            squashFromTrap(tid);
        } else if (tcSquash[tid]) {
            assert(commitStatus[tid] != TrapPending);
            squashFromTC(tid);
		// 根据不同的标记进行相应的squash操作
        } else if (commitStatus[tid] == SquashAfterPending) {
            // A squash from the previous cycle of the commit stage (i.e.,
            // commitInsts() called squashAfter) is pending. Squash the
            // thread now.
            squashFromSquashAfter(tid);
			// 如果当前正在处理一个suqashAfter操作，则进行squashFromSquashAfter
			// 的squash处理；这里可以直接开始squash，是因为改状态是在上个周期
			// 提交到标记位squashAfter指令之后才设置的，因此可以直接开始squash
        }

        // Squashed sequence number must be older than youngest valid
        // instruction in the ROB. This prevents squashes from younger
        // instructions overriding squashes from older instructions.
        if (fromIEW->squash[tid] &&
            commitStatus[tid] != TrapPending &&
            fromIEW->squashedSeqNum[tid] <= youngestSeqNum[tid]) {
			// 确定当前IEW阶段发送到Commit阶段的squash请求，会导致ROB进行
			// squash（包含了ROB中的指令），才会执行后面的处理

            if (fromIEW->mispredictInst[tid]) {
                DPRINTF(Commit,
                    "[tid:%i]: Squashing due to branch mispred PC:%#x [sn:%i]\n",
                    tid,
                    fromIEW->mispredictInst[tid]->instAddr(),
                    fromIEW->squashedSeqNum[tid]);
            } else {
                DPRINTF(Commit,
                    "[tid:%i]: Squashing due to order violation [sn:%i]\n",
                    tid, fromIEW->squashedSeqNum[tid]);
            }
			// 两种来IEW的squash可能，一个是错误的分支预测，另一个是访存
			// 顺序violation

            DPRINTF(Commit, "[tid:%i]: Redirecting to PC %#x\n",
                    tid,
                    fromIEW->pc[tid].nextInstAddr());

            commitStatus[tid] = ROBSquashing;
			// 设置状态表示当前周期需要进行ROB的squash处理

            // If we want to include the squashing instruction in the squash,
            // then use one older sequence number.
            InstSeqNum squashed_inst = fromIEW->squashedSeqNum[tid];

            if (fromIEW->includeSquashInst[tid]) {
                squashed_inst--;
            }
			// 根据IEW信息确定squash操作的起点，最后squash仅会将squash_inst
			// 之后（即比该指令更新的）的指令squash掉

            // All younger instructions will be squashed. Set the sequence
            // number as the youngest instruction in the ROB.
            youngestSeqNum[tid] = squashed_inst;
			// 重新设置youngestSeqNum，但是这个东西直接设置可能不准确？？？
			// 因为该标记对应与ROB的表项可能不是一个有效的指令

            rob->squash(squashed_inst, tid);
            changedROBNumEntries[tid] = true;
            toIEW->commitInfo[tid].doneSeqNum = squashed_inst;
            toIEW->commitInfo[tid].squash = true;
			// 执行ROB的squash操作，设置标志位表示需要更新ROB空闲表项
			// 并告知IEW阶段一并处理squash操作

            // Send back the rob squashing signal so other stages know that
            // the ROB is in the process of squashing.
            toIEW->commitInfo[tid].robSquashing = true;
			// ROB正在进行squash的状态标志位
            toIEW->commitInfo[tid].mispredictInst =
                fromIEW->mispredictInst[tid];
			// 错误分支预测的指令
            toIEW->commitInfo[tid].branchTaken =
                fromIEW->branchTaken[tid];
			// 错误分支预测的实际taken情况
            toIEW->commitInfo[tid].squashInst =
                                    rob->findInst(tid, squashed_inst);
			// ROB进行squash操作的标记位置对应指令在ROB中的索引
            if (toIEW->commitInfo[tid].mispredictInst) {
                if (toIEW->commitInfo[tid].mispredictInst->isUncondCtrl()) {
                     toIEW->commitInfo[tid].branchTaken = true;
                }
                ++branchMispredicts;
            }
            toIEW->commitInfo[tid].pc = fromIEW->pc[tid];
			// 分支预测正确预测后对应的正确PC状态
			// 设置返回给前面阶段的信息，通过IEW逐个阶段的传递
        }

        if (commitStatus[tid] == ROBSquashing) {
            num_squashing_threads++;
			// 表明当前执行squash的线程数量
        }
    }

    // If commit is currently squashing, then it will have activity for the
    // next cycle. Set its next status as active.
    if (num_squashing_threads) {
        _nextStatus = Active;
    }
	// 如果由任何线程执行squash，激活下一个周期的状态，处理squash
	// 之后的相关工作

    if (num_squashing_threads != numThreads) {
        // If we're not currently squashing, then get instructions.
        getInsts();
		// 从rename阶段获取指令到Commit阶段的ROB中

        // Try to commit any instructions.
        commitInsts();
		// 进行实际的指令提交
		
		// 如果本周期并不是所有的线程都在处理squash，那么就可以对
		// 那些不用squash的线程进行指令提交处理
    }

    //Check for any activity
    threads = activeThreads->begin();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (changedROBNumEntries[tid]) {
            toIEW->commitInfo[tid].usedROB = true;
            toIEW->commitInfo[tid].freeROBEntries = rob->numFreeEntries(tid);

            wroteToTimeBuffer = true;
            changedROBNumEntries[tid] = false;
            if (rob->isEmpty(tid))
                checkEmptyROB[tid] = true;
        }
		// 更新活跃线程的ROB空闲表项数目

        // ROB is only considered "empty" for previous stages if: a)
        // ROB is empty, b) there are no outstanding stores, c) IEW
        // stage has received any information regarding stores that
        // committed.
		
        // c) is checked by making sure to not consider the ROB empty
        // on the same cycle as when stores have been committed.
        // @todo: Make this handle multi-cycle communication between
        // commit and IEW.
        if (checkEmptyROB[tid] && rob->isEmpty(tid) &&
            !iewStage->hasStoresToWB(tid) && !committedStores[tid]) {
			// 确定是否应该告知IEW阶段ROB为空了，要求ROB本身为空
			// 并且IEW阶段没有可以写回的store指令，本周期没有提交
			// store指令（store指令在本周期提交并不能当作提交完成）
			// 这里的处理依然需要重新思考？？？
            checkEmptyROB[tid] = false;
            toIEW->commitInfo[tid].usedROB = true;
            toIEW->commitInfo[tid].emptyROB = true;
            toIEW->commitInfo[tid].freeROBEntries = rob->numFreeEntries(tid);
            wroteToTimeBuffer = true;
        }

    }
}

template <class Impl>
void
DefaultCommit<Impl>::commitInsts()
{
    ////////////////////////////////////
    // Handle commit
    // Note that commit will be handled prior to putting new
    // instructions in the ROB so that the ROB only tries to commit
    // instructions it has in this current cycle, and not instructions
    // it is writing in during this cycle.  Can't commit and squash
    // things at the same time...
    ////////////////////////////////////
	
	// 这里说对指令提交只会对当前ROB中已经确定可以提交的指令进行
	// 对于本周期刚从IEW阶段传递过来表示已经完成可以提交的指令
	// 则不会处理，同时指令的commit和squash操作是互斥的

    DPRINTF(Commit, "Trying to commit instructions in the ROB.\n");

    unsigned num_committed = 0;

    DynInstPtr head_inst;

    // Commit as many instructions as possible until the commit bandwidth
    // limit is reached, or it becomes impossible to commit any more.
    while (num_committed < commitWidth) {
        // Check for any interrupt that we've already squashed for
        // and start processing it.
        if (interrupt != NoFault)
            handleInterrupt();
		// 如果提交到了包含中断操作的指令，则立即对中断进行处理

        ThreadID commit_thread = getCommittingThread();
		// 每次遍历都会选择一个线程提交对应的指令，选择是根据
		// commitPolicy决定的，这里如果发现没有满足特定状态的
		// 线程（后面处理中进行squash以及中断处理等等，均会导
		// 致该线程无法继续提交指令），就会返回-1

        if (commit_thread == -1 || !rob->isHeadReady(commit_thread))
            break;
		// 如果发现当前没有选择到可用的线程，或者该线程ROB顶部的指令
		// 尚未准备好，就会结束提交指令的过程

        head_inst = rob->readHeadInst(commit_thread);

        ThreadID tid = head_inst->threadNumber;

        assert(tid == commit_thread);

        DPRINTF(Commit, "Trying to commit head instruction, [sn:%i] [tid:%i]\n",
                head_inst->seqNum, tid);

        // If the head instruction is squashed, it is ready to retire
        // (be removed from the ROB) at any time.
        if (head_inst->isSquashed()) {

            DPRINTF(Commit, "Retiring squashed instruction from "
                    "ROB.\n");

            rob->retireHead(commit_thread);
			// 将给定的指令提交，并完全retire

            ++commitSquashedInsts;
            // Notify potential listeners that this instruction is squashed
            ppSquash->notify(head_inst);

            // Record that the number of ROB entries has changed.
            changedROBNumEntries[tid] = true;
			// 对于一个被标记为squashed的指令，可以直接retire/提交
        } else {
            pc[tid] = head_inst->pcState();
			// 更新commit阶段的PC状态，根据这个PC进行提交和fetch重定向

            // Increment the total number of non-speculative instructions
            // executed.
            // Hack for now: it really shouldn't happen until after the
            // commit is deemed to be successful, but this count is needed
            // for syscalls.
            thread[tid]->funcExeInst++;

            // Try to commit the head instruction.
            bool commit_success = commitHead(head_inst, num_committed);
			// 尝试提交该指令

            if (commit_success) {
                ++num_committed;
				// 根据提交成功与否的操作递增num_committed
                statCommittedInstType[tid][head_inst->opClass()]++;
                ppCommit->notify(head_inst);
				// 更新不同类型指令的提交计数，并通知Probe Points
				// 一个该指令已经提交了

                changedROBNumEntries[tid] = true;

                // Set the doneSeqNum to the youngest committed instruction.
                toIEW->commitInfo[tid].doneSeqNum = head_inst->seqNum;
				// 这里设置doneSeqNum和squash时设置的的确是同一个变量
				// 但是意义却不相同，squash表示的是squash的起点指令
				// 这里则表示当前提交指令中最新的指令编号，以便IEW阶段
				// 处理该指令以及更老指令的提交相关处理

                if (tid == 0) {
                    canHandleInterrupts =  (!head_inst->isDelayedCommit()) &&
                        // 只有当前的指令不是一个延迟提交的指令才允许处理
						// 中断，因为只有非延迟提交的指令才能确保该指令
						// 的确完成提交，中断处理的确是精确的
						((THE_ISA != ALPHA_ISA) ||
                        (!(pc[0].instAddr() & 0x3)));
						// 上面是对于ALPHA的特殊要求
                }

                // at this point store conditionals should either have
                // been completed or predicated false
                assert(!head_inst->isStoreConditional() ||
                       head_inst->isCompleted() ||
                       !head_inst->readPredicate());

                // Updates misc. registers.
                head_inst->updateMiscRegs();
				// 更新Misc寄存器，注意这里并没有

                // Check instruction execution if it successfully commits and
                // is not carrying a fault.
                if (cpu->checker) {
                    cpu->checker->verify(head_inst);
                }

                cpu->traceFunctions(pc[tid].instAddr());
				// 该函数定义在BaseCPU中而非O3CPU中

                TheISA::advancePC(pc[tid], head_inst->staticInst);
				// 根据commit阶段的PC状态，更新下一个PC

                // Keep track of the last sequence number commited
                lastCommitedSeqNum[tid] = head_inst->seqNum;

                // If this is an instruction that doesn't play nicely with
                // others squash everything and restart fetch
                if (head_inst->isSquashAfter())
                    squashAfter(tid, head_inst);
				// 对于一个标记为squashAfter指令进行相应的处理

                if (drainPending) {
					// 如果准备执行drain操作，那么必须到达一个指令边界
					// 这个指令边界是对于微操作来说的，同时要求到达前
					// 没有出现任何中断或者准备处理中断
                    if (pc[tid].microPC() == 0 && interrupt == NoFault &&
                        !thread[tid]->trapPending) {
                        // Last architectually committed instruction.
                        // Squash the pipeline, stall fetch, and use
                        // drainImminent to disable interrupts
                        DPRINTF(Drain, "Draining: %i:%s\n", tid, pc[tid]);
                        squashAfter(tid, head_inst);
						// 首先将当前指令之后的所有指令squash达到排空效果
                        cpu->commitDrained(tid);
						// 通知CPU要开始排空流水线了，fetch阶段应该停止取指
                        drainImminent = true;
						// 设置该标志位组织排空流水线期间处理中断
                    }
                }

                bool onInstBoundary = !head_inst->isMicroop() ||
                                      head_inst->isLastMicroop() ||
                                      !head_inst->isDelayedCommit();
				// 判断是否即将到达了指令边界，这里要求顶部指令不是一个
				// 微操作或者顶部指令是指令中的最后一个微操作，或者该指令
				// 不是一个延迟提交的指令？？？

                if (onInstBoundary) {
                    int count = 0;
                    Addr oldpc;
                    // Make sure we're not currently updating state while
                    // handling PC events.
                    assert(!thread[tid]->noSquashFromTC &&
                           !thread[tid]->trapPending);
                    do {
                        oldpc = pc[tid].instAddr();
                        cpu->system->pcEventQueue.service(thread[tid]->getTC());
                        count++;
                    } while (oldpc != pc[tid].instAddr());
                    if (count > 1) {
                        DPRINTF(Commit,
                                "PC skip function event, stopping commit\n");
                        break;
                    }
                }
				// 此处的处理意义不明？？？

                // Check if an instruction just enabled interrupts and we've
                // previously had an interrupt pending that was not handled
                // because interrupts were subsequently disabled before the
                // pipeline reached a place to handle the interrupt. In that
                // case squash now to make sure the interrupt is handled.
                //
                // If we don't do this, we might end up in a live lock situation
                if (!interrupt && avoidQuiesceLiveLock &&
                    onInstBoundary && cpu->checkInterrupts(cpu->tcBase(0)))
                    squashAfter(tid, head_inst);
				// 如果当前没有正在处理的中断，但是有需要处理的中断，
				// 并且位于指令边界，则执行squashAfter保证精确中断
            } else {
                DPRINTF(Commit, "Unable to commit head instruction PC:%s "
                        "[tid:%i] [sn:%i].\n",
                        head_inst->pcState(), tid ,head_inst->seqNum);
                break;
				// 如果任何一条指令没有成功提交，一般来说应该属于延迟提交
				// 那么不再继续进行提交，则在该未成功提交的指令得到处理前
				// 无法正常执行后续的提交
            }
        }
    }

    DPRINTF(CommitRate, "%i\n", num_committed);
    numCommittedDist.sample(num_committed);

    if (num_committed == commitWidth) {
        commitEligibleSamples++;
    }
	// 更新相关的计数器
}

template <class Impl>
bool
DefaultCommit<Impl>::commitHead(DynInstPtr &head_inst, unsigned inst_num)
{
	// 参数inst_num表示的是当前周期已经提交的指令数量
    assert(head_inst);

    ThreadID tid = head_inst->threadNumber;

    // If the instruction is not executed yet, then it will need extra
    // handling.  Signal backwards that it should be executed.
    if (!head_inst->isExecuted()) {
		// 发现该指令压根就没有执行完毕，还提交个屁
		
        // Keep this number correct.  We have not yet actually executed
        // and committed this instruction.
        thread[tid]->funcExeInst--;

        // Make sure we are only trying to commit un-executed instructions we
        // think are possible.
        assert(head_inst->isNonSpeculative() || head_inst->isStoreConditional()
               || head_inst->isMemBarrier() || head_inst->isWriteBarrier() ||
               (head_inst->isLoad() && head_inst->strictlyOrdered()));

        DPRINTF(Commit, "Encountered a barrier or non-speculative "
                "instruction [sn:%lli] at the head of the ROB, PC %s.\n",
                head_inst->seqNum, head_inst->pcState());

        if (inst_num > 0 || iewStage->hasStoresToWB(tid)) {
            DPRINTF(Commit, "Waiting for all stores to writeback.\n");
            return false;
			// 如果当前指令并不是本周期尝试提交的第一个指令，或者
			// IEW阶段存在可以写回的store指令？？？，则该指令无法提交
        }

        toIEW->commitInfo[tid].nonSpecSeqNum = head_inst->seqNum;

        // Change the instruction so it won't try to commit again until
        // it is executed.
        head_inst->clearCanCommit();
		// 你这里就算清除不能commit的标志位，也用不上啊，提交指令
		// 过程中似乎并没有对canCommit标志进行检查啊？？？

        if (head_inst->isLoad() && head_inst->strictlyOrdered()) {
            DPRINTF(Commit, "[sn:%lli]: Strictly ordered load, PC %s.\n",
                    head_inst->seqNum, head_inst->pcState());
            toIEW->commitInfo[tid].strictlyOrdered = true;
            toIEW->commitInfo[tid].strictlyOrderedLoad = head_inst;
			// 如果是一个严格按序执行的load指令，则会发送给IEW阶段
			// 保证该指令在执行过程严格按序
        } else {
            ++commitNonSpecStalls;
        }

        return false;
    }

    if (head_inst->isThreadSync()) {
        // Not handled for now.
        panic("Thread sync instructions are not handled yet.\n");
    }
	// 这里意思似乎并不是没有设计处理线程同步操作，而是尚未在这处理

    // Check if the instruction caused a fault.  If so, trap.
    Fault inst_fault = head_inst->getFault();

    // Stores mark themselves as completed.
    if (!head_inst->isStore() && inst_fault == NoFault) {
        head_inst->setCompleted();
    }
	// 对于非store的无错误指令，可以直接设置为完成状态

    if (inst_fault != NoFault) {
		// 下面是产生错误的指令或者携带特殊操作信息的指令
        DPRINTF(Commit, "Inst [sn:%lli] PC %s has a fault\n",
                head_inst->seqNum, head_inst->pcState());

        if (iewStage->hasStoresToWB(tid) || inst_num > 0) {
            DPRINTF(Commit, "Stores outstanding, fault must wait.\n");
            return false;
			// 这里依然要求IEW阶段的所有store指令均已经写回？？？
        }

        head_inst->setCompleted();

        // If instruction has faulted, let the checker execute it and
        // check if it sees the same fault and control flow.
        if (cpu->checker) {
            // Need to check the instruction before its fault is processed
            cpu->checker->verify(head_inst);
        }
		// 由checker检查该指令执行是否会出项相同的错误

        assert(!thread[tid]->noSquashFromTC);

        // Mark that we're in state update mode so that the trap's
        // execution doesn't generate extra squashes.
        thread[tid]->noSquashFromTC = true;
		// 设置标志位放置trap的处理被打断

        // Execute the trap.  Although it's slightly unrealistic in
        // terms of timing (as it doesn't wait for the full timing of
        // the trap event to complete before updating state), it's
        // needed to update the state as soon as possible.  This
        // prevents external agents from changing any specific state
        // that the trap need.
        cpu->trap(inst_fault, tid,
                  head_inst->notAnInst() ?
                      StaticInst::nullStaticInstPtr :
                      head_inst->staticInst);

        // Exit state update mode to avoid accidental updating.
        thread[tid]->noSquashFromTC = false;

        commitStatus[tid] = TrapPending;
		// 设置状态表示一个trap尚未处理完毕

        DPRINTF(Commit, "Committing instruction with fault [sn:%lli]\n",
            head_inst->seqNum);
        if (head_inst->traceData) {
            if (DTRACE(ExecFaulting)) {
                head_inst->traceData->setFetchSeq(head_inst->seqNum);
                head_inst->traceData->setCPSeq(thread[tid]->numOp);
                head_inst->traceData->dump();
            }
            delete head_inst->traceData;
            head_inst->traceData = NULL;
        }
		// 设置和trace有关的数据

        // Generate trap squash event.
        generateTrapEvent(tid, inst_fault);
		// 生成trap相应的squash操作和延迟处理信息，并通知到CPU
        return false;
    }

    updateComInstStats(head_inst);

    if (FullSystem) {
        if (thread[tid]->profile) {
            thread[tid]->profilePC = head_inst->instAddr();
            ProfileNode *node = thread[tid]->profile->consume(
                    thread[tid]->getTC(), head_inst->staticInst);

            if (node)
                thread[tid]->profileNode = node;
        }
        if (CPA::available()) {
            if (head_inst->isControl()) {
                ThreadContext *tc = thread[tid]->getTC();
                CPA::cpa()->swAutoBegin(tc, head_inst->nextInstAddr());
            }
        }
    }
	// 忽略FS模式下的相关处理
    DPRINTF(Commit, "Committing instruction with [sn:%lli] PC %s\n",
            head_inst->seqNum, head_inst->pcState());
    if (head_inst->traceData) {
        head_inst->traceData->setFetchSeq(head_inst->seqNum);
        head_inst->traceData->setCPSeq(thread[tid]->numOp);
        head_inst->traceData->dump();
        delete head_inst->traceData;
        head_inst->traceData = NULL;
    }
	// 再提交之后将该指令的traceData删除
	
    if (head_inst->isReturn()) {
        DPRINTF(Commit,"Return Instruction Committed [sn:%lli] PC %s \n",
                        head_inst->seqNum, head_inst->pcState());
    }

    // Update the commit rename map
    for (int i = 0; i < head_inst->numDestRegs(); i++) {
        renameMap[tid]->setEntry(head_inst->flattenedDestRegIdx(i),
                                 head_inst->renamedDestRegIdx(i));
    }
	// 设置renameMap的映射情况，请注意，这里的renameMap是commit阶段
	// 使用的renameMap而不是rename阶段的renameMap，这里表示提交后的
	// 寄存器重命名映射关系

    // Finally clear the head ROB entry.
    rob->retireHead(tid);
	// 将该指令从ROB中删除

#if TRACING_ON
    if (DTRACE(O3PipeView)) {
        head_inst->commitTick = curTick() - head_inst->fetchTick;
    }
#endif

    // If this was a store, record it for this cycle.
    if (head_inst->isStore())
        committedStores[tid] = true;
	// 如果该指令是一个store指令，设置标志位表示本周期提交了一个store指令

    // Return true to indicate that we have committed an instruction.
    return true;
}

template <class Impl>
void
DefaultCommit<Impl>::getInsts()
{
    DPRINTF(Commit, "Getting instructions from Rename stage.\n");

    // Read any renamed instructions and place them into the ROB.
    int insts_to_process = std::min((int)renameWidth, fromRename->size);
	// 处理最多不超过renameWidth的指令，且仅处理有效的指令

    for (int inst_num = 0; inst_num < insts_to_process; ++inst_num) {
        DynInstPtr inst;

        inst = fromRename->insts[inst_num];
        ThreadID tid = inst->threadNumber;

        if (!inst->isSquashed() &&
            commitStatus[tid] != ROBSquashing &&
            commitStatus[tid] != TrapPending) {
			// 要求该指令并没有标记为squashed，并且当前状态允许处理
			// 新的指令
            changedROBNumEntries[tid] = true;

            DPRINTF(Commit, "Inserting PC %s [sn:%i] [tid:%i] into ROB.\n",
                    inst->pcState(), inst->seqNum, tid);
            rob->insertInst(inst);
            assert(rob->getThreadEntries(tid) <= rob->getMaxEntries(tid));
            youngestSeqNum[tid] = inst->seqNum;
			// 插入新的指令，并更新ROB的youngestSeqNum，这里的指令
			// 是乱序的，在ROB插入时会进行整理，按照先后顺序加入指令
        } else {
            DPRINTF(Commit, "Instruction PC %s [sn:%i] [tid:%i] was "
                    "squashed, skipping.\n",
                    inst->pcState(), inst->seqNum, tid);
        }
    }
}

template <class Impl>
void
DefaultCommit<Impl>::markCompletedInsts()
{
    // Grab completed insts out of the IEW instruction queue, and mark
    // instructions completed within the ROB.
    for (int inst_num = 0; inst_num < fromIEW->size; ++inst_num) {
        assert(fromIEW->insts[inst_num]);
        if (!fromIEW->insts[inst_num]->isSquashed()) {
			// 如果IEW传递来的指令没有被标记为squashed，则标记该指令
			// 可以提交
            DPRINTF(Commit, "[tid:%i]: Marking PC %s, [sn:%lli] ready "
                    "within ROB.\n",
                    fromIEW->insts[inst_num]->threadNumber,
                    fromIEW->insts[inst_num]->pcState(),
                    fromIEW->insts[inst_num]->seqNum);

            // Mark the instruction as ready to commit.
            fromIEW->insts[inst_num]->setCanCommit();
        }
    }
}

template <class Impl>
void
DefaultCommit<Impl>::updateComInstStats(DynInstPtr &inst)
{
    ThreadID tid = inst->threadNumber;

    if (!inst->isMicroop() || inst->isLastMicroop())
        instsCommitted[tid]++;
    opsCommitted[tid]++;

    // To match the old model, don't count nops and instruction
    // prefetches towards the total commit count.
    if (!inst->isNop() && !inst->isInstPrefetch()) {
        cpu->instDone(tid, inst);
    }

    //
    //  Control Instructions
    //
    if (inst->isControl())
        statComBranches[tid]++;

    //
    //  Memory references
    //
    if (inst->isMemRef()) {
        statComRefs[tid]++;

        if (inst->isLoad()) {
            statComLoads[tid]++;
        }
    }

    if (inst->isMemBarrier()) {
        statComMembars[tid]++;
    }

    // Integer Instruction
    if (inst->isInteger())
        statComInteger[tid]++;

    // Floating Point Instruction
    if (inst->isFloating())
        statComFloating[tid]++;
    // Vector Instruction
    if (inst->isVector())
        statComVector[tid]++;

    // Function Calls
    if (inst->isCall())
        statComFunctionCalls[tid]++;

}

////////////////////////////////////////
//                                    //
//  SMT COMMIT POLICY MAINTAINED HERE //
//                                    //
////////////////////////////////////////

template <class Impl>
ThreadID
DefaultCommit<Impl>::getCommittingThread()
{
    if (numThreads > 1) {
        switch (commitPolicy) {

          case Aggressive:
            //If Policy is Aggressive, commit will call
            //this function multiple times per
            //cycle
            return oldestReady();

          case RoundRobin:
            return roundRobin();

          case OldestReady:
            return oldestReady();

          default:
            return InvalidThreadID;
        }
    } else {
        assert(!activeThreads->empty());
        ThreadID tid = activeThreads->front();

        if (commitStatus[tid] == Running ||
            commitStatus[tid] == Idle ||
            commitStatus[tid] == FetchTrapPending) {
            return tid;
        } else {
            return InvalidThreadID;
        }
		// 单个线程则不需要使用什么策略选择线程，只要满足状态
		// 要求就可以进行指令提交操作
    }
}

template<class Impl>
ThreadID
DefaultCommit<Impl>::roundRobin()
{
    list<ThreadID>::iterator pri_iter = priority_list.begin();
    list<ThreadID>::iterator end      = priority_list.end();

    while (pri_iter != end) {
        ThreadID tid = *pri_iter;

        if (commitStatus[tid] == Running ||
            commitStatus[tid] == Idle ||
            commitStatus[tid] == FetchTrapPending) {

            if (rob->isHeadReady(tid)) {
                priority_list.erase(pri_iter);
                priority_list.push_back(tid);

                return tid;
            }
        }

        pri_iter++;
    }

    return InvalidThreadID;
}

template<class Impl>
ThreadID
DefaultCommit<Impl>::oldestReady()
{
    unsigned oldest = 0;
    bool first = true;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (!rob->isEmpty(tid) &&
            (commitStatus[tid] == Running ||
             commitStatus[tid] == Idle ||
             commitStatus[tid] == FetchTrapPending)) {

            if (rob->isHeadReady(tid)) {

                DynInstPtr head_inst = rob->readHeadInst(tid);

                if (first) {
                    oldest = tid;
                    first = false;
                } else if (head_inst->seqNum < oldest) {
                    oldest = tid;
                }
            }
        }
    }

    if (!first) {
        return oldest;
    } else {
        return InvalidThreadID;
    }
}

#endif//__CPU_O3_COMMIT_IMPL_HH__
