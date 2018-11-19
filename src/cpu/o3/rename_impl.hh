/*
 * Copyright (c) 2010-2012, 2014-2016 ARM Limited
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
 *          Korey Sewell
 */

#ifndef __CPU_O3_RENAME_IMPL_HH__
#define __CPU_O3_RENAME_IMPL_HH__

#include <list>

#include "arch/isa_traits.hh"
#include "arch/registers.hh"
#include "config/the_isa.hh"
#include "cpu/o3/rename.hh"
#include "cpu/reg_class.hh"
#include "debug/Activity.hh"
#include "debug/Rename.hh"
#include "debug/O3PipeView.hh"
#include "params/DerivO3CPU.hh"

using namespace std;

template <class Impl>
DefaultRename<Impl>::DefaultRename(O3CPU *_cpu, DerivO3CPUParams *params)
    : cpu(_cpu),
      iewToRenameDelay(params->iewToRenameDelay),
      decodeToRenameDelay(params->decodeToRenameDelay),
      commitToRenameDelay(params->commitToRenameDelay),
      renameWidth(params->renameWidth),
      commitWidth(params->commitWidth),
      numThreads(params->numThreads)
{
    if (renameWidth > Impl::MaxWidth)
        fatal("renameWidth (%d) is larger than compiled limit (%d),\n"
             "\tincrease MaxWidth in src/cpu/o3/impl.hh\n",
             renameWidth, static_cast<int>(Impl::MaxWidth));

    // @todo: Make into a parameter.
    skidBufferMax = (decodeToRenameDelay + 1) * params->decodeWidth;
}

template <class Impl>
std::string
DefaultRename<Impl>::name() const
{
    return cpu->name() + ".rename";
}

template <class Impl>
void
DefaultRename<Impl>::regStats()
{
    renameSquashCycles
        .name(name() + ".SquashCycles")
        .desc("Number of cycles rename is squashing")
        .prereq(renameSquashCycles);
    renameIdleCycles
        .name(name() + ".IdleCycles")
        .desc("Number of cycles rename is idle")
        .prereq(renameIdleCycles);
    renameBlockCycles
        .name(name() + ".BlockCycles")
        .desc("Number of cycles rename is blocking")
        .prereq(renameBlockCycles);
    renameSerializeStallCycles
        .name(name() + ".serializeStallCycles")
        .desc("count of cycles rename stalled for serializing inst")
        .flags(Stats::total);
    renameRunCycles
        .name(name() + ".RunCycles")
        .desc("Number of cycles rename is running")
        .prereq(renameIdleCycles);
    renameUnblockCycles
        .name(name() + ".UnblockCycles")
        .desc("Number of cycles rename is unblocking")
        .prereq(renameUnblockCycles);
    renameRenamedInsts
        .name(name() + ".RenamedInsts")
        .desc("Number of instructions processed by rename")
        .prereq(renameRenamedInsts);
    renameSquashedInsts
        .name(name() + ".SquashedInsts")
        .desc("Number of squashed instructions processed by rename")
        .prereq(renameSquashedInsts);
    renameROBFullEvents
        .name(name() + ".ROBFullEvents")
        .desc("Number of times rename has blocked due to ROB full")
        .prereq(renameROBFullEvents);
    renameIQFullEvents
        .name(name() + ".IQFullEvents")
        .desc("Number of times rename has blocked due to IQ full")
        .prereq(renameIQFullEvents);
    renameLQFullEvents
        .name(name() + ".LQFullEvents")
        .desc("Number of times rename has blocked due to LQ full")
        .prereq(renameLQFullEvents);
    renameSQFullEvents
        .name(name() + ".SQFullEvents")
        .desc("Number of times rename has blocked due to SQ full")
        .prereq(renameSQFullEvents);
    renameFullRegistersEvents
        .name(name() + ".FullRegisterEvents")
        .desc("Number of times there has been no free registers")
        .prereq(renameFullRegistersEvents);
    renameRenamedOperands
        .name(name() + ".RenamedOperands")
        .desc("Number of destination operands rename has renamed")
        .prereq(renameRenamedOperands);
    renameRenameLookups
        .name(name() + ".RenameLookups")
        .desc("Number of register rename lookups that rename has made")
        .prereq(renameRenameLookups);
    renameCommittedMaps
        .name(name() + ".CommittedMaps")
        .desc("Number of HB maps that are committed")
        .prereq(renameCommittedMaps);
    renameUndoneMaps
        .name(name() + ".UndoneMaps")
        .desc("Number of HB maps that are undone due to squashing")
        .prereq(renameUndoneMaps);
    renamedSerializing
        .name(name() + ".serializingInsts")
        .desc("count of serializing insts renamed")
        .flags(Stats::total)
        ;
    renamedTempSerializing
        .name(name() + ".tempSerializingInsts")
        .desc("count of temporary serializing insts renamed")
        .flags(Stats::total)
        ;
    renameSkidInsts
        .name(name() + ".skidInsts")
        .desc("count of insts added to the skid buffer")
        .flags(Stats::total)
        ;
    intRenameLookups
        .name(name() + ".int_rename_lookups")
        .desc("Number of integer rename lookups")
        .prereq(intRenameLookups);
    fpRenameLookups
        .name(name() + ".fp_rename_lookups")
        .desc("Number of floating rename lookups")
        .prereq(fpRenameLookups);
    vecRenameLookups
        .name(name() + ".vec_rename_lookups")
        .desc("Number of vector rename lookups")
        .prereq(vecRenameLookups);
}

template <class Impl>
void
DefaultRename<Impl>::regProbePoints()
{
    ppRename = new ProbePointArg<DynInstPtr>(cpu->getProbeManager(), "Rename");
    ppSquashInRename = new ProbePointArg<SeqNumRegPair>(cpu->getProbeManager(),
                                                        "SquashInRename");
}

template <class Impl>
void
DefaultRename<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    timeBuffer = tb_ptr;

    // Setup wire to read information from time buffer, from IEW stage.
    fromIEW = timeBuffer->getWire(-iewToRenameDelay);

    // Setup wire to read infromation from time buffer, from commit stage.
    fromCommit = timeBuffer->getWire(-commitToRenameDelay);

    // Setup wire to write information to previous stages.
    toDecode = timeBuffer->getWire(0);
}

template <class Impl>
void
DefaultRename<Impl>::setRenameQueue(TimeBuffer<RenameStruct> *rq_ptr)
{
    renameQueue = rq_ptr;

    // Setup wire to write information to future stages.
    toIEW = renameQueue->getWire(0);
}

template <class Impl>
void
DefaultRename<Impl>::setDecodeQueue(TimeBuffer<DecodeStruct> *dq_ptr)
{
    decodeQueue = dq_ptr;

    // Setup wire to get information from decode.
    fromDecode = decodeQueue->getWire(-decodeToRenameDelay);
}

template <class Impl>
void
DefaultRename<Impl>::startupStage()
{
    resetStage();
}

template <class Impl>
void
DefaultRename<Impl>::resetStage()
{
    _status = Inactive;

    resumeSerialize = false;
    resumeUnblocking = false;

    // Grab the number of free entries directly from the stages.
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        renameStatus[tid] = Idle;

        freeEntries[tid].iqEntries = iew_ptr->instQueue.numFreeEntries(tid);
        freeEntries[tid].lqEntries = iew_ptr->ldstQueue.numFreeLoadEntries(tid);
        freeEntries[tid].sqEntries = iew_ptr->ldstQueue.numFreeStoreEntries(tid);
        freeEntries[tid].robEntries = commit_ptr->numROBFreeEntries(tid);
        emptyROB[tid] = true;

        stalls[tid].iew = false;
        serializeInst[tid] = NULL;

        instsInProgress[tid] = 0;
        loadsInProgress[tid] = 0;
        storesInProgress[tid] = 0;

        serializeOnNextInst[tid] = false;
    }
}

template<class Impl>
void
DefaultRename<Impl>::setActiveThreads(list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;
}


template <class Impl>
void
DefaultRename<Impl>::setRenameMap(RenameMap rm_ptr[])
{
    for (ThreadID tid = 0; tid < numThreads; tid++)
        renameMap[tid] = &rm_ptr[tid];
}

template <class Impl>
void
DefaultRename<Impl>::setFreeList(FreeList *fl_ptr)
{
    freeList = fl_ptr;
}

template<class Impl>
void
DefaultRename<Impl>::setScoreboard(Scoreboard *_scoreboard)
{
    scoreboard = _scoreboard;
}

template <class Impl>
bool
DefaultRename<Impl>::isDrained() const
{
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (instsInProgress[tid] != 0 ||
            !historyBuffer[tid].empty() ||
            !skidBuffer[tid].empty() ||
            !insts[tid].empty() ||
            (renameStatus[tid] != Idle && renameStatus[tid] != Running))
            return false;
    }
    return true;
}

template <class Impl>
void
DefaultRename<Impl>::takeOverFrom()
{
    resetStage();
}

template <class Impl>
void
DefaultRename<Impl>::drainSanityCheck() const
{
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        assert(historyBuffer[tid].empty());
        assert(insts[tid].empty());
        assert(skidBuffer[tid].empty());
        assert(instsInProgress[tid] == 0);
    }
}

template <class Impl>
void
DefaultRename<Impl>::squash(const InstSeqNum &squash_seq_num, ThreadID tid)
{
    DPRINTF(Rename, "[tid:%u]: Squashing instructions.\n",tid);

    // Clear the stall signal if rename was blocked or unblocking before.
    // If it still needs to block, the blocking should happen the next
    // cycle and there should be space to hold everything due to the squash.
    if (renameStatus[tid] == Blocked ||
        renameStatus[tid] == Unblocking) {
        toDecode->renameUnblock[tid] = 1;

        resumeSerialize = false;
        serializeInst[tid] = NULL;
		// 如果进行squash操作的时候，rename阶段正好处于blocked或者unblock状态
		// 那么在squash之后便会退出阻塞状态，并清除序列化执行的标志和指令
    } else if (renameStatus[tid] == SerializeStall) {
		// 如果当前因为需要进行序列化操作而stall
        if (serializeInst[tid]->seqNum <= squash_seq_num) {
            DPRINTF(Rename, "Rename will resume serializing after squash\n");
            resumeSerialize = true;
            assert(serializeInst[tid]);
			// 由于squash的指令比当前需要序列化执行的指令更新，因此
			// 在squash之后应该继续序列化执行。
        } else {
            resumeSerialize = false;
            toDecode->renameUnblock[tid] = 1;
            serializeInst[tid] = NULL;
			// 由于需要序列化执行的指令也会被squash掉，因此在squash操作之后
			// 无需继续进行序列化操作，同时序列化指令也会被清除；并通知decode
			// 退出因为序列化执行进入的阻塞状态，
        }
    }

    // Set the status to Squashing.
    renameStatus[tid] = Squashing;

    // Squash any instructions from decode.
    for (int i=0; i<fromDecode->size; i++) {
        if (fromDecode->insts[i]->threadNumber == tid &&
            fromDecode->insts[i]->seqNum > squash_seq_num) {
            fromDecode->insts[i]->setSquashed();
            wroteToTimeBuffer = true;
        }
	// 将需要squash的来自decode阶段指令设置为被squash的状态，由于
	// 进行了squash操作，因此设置wroteToTimeBuffer为true表示本周期活跃
    }

    // Clear the instruction list and skid buffer in case they have any
    // insts in them.
    insts[tid].clear();

    // Clear the skid buffer in case it has any data in it.
    skidBuffer[tid].clear();

    doSquash(squash_seq_num, tid);
}

template <class Impl>
void
DefaultRename<Impl>::tick()
{
    wroteToTimeBuffer = false;

    blockThisCycle = false;

    bool status_change = false;

    toIEWIndex = 0;
	// 每个周期尝试向IEW阶段发送指令都需要重置，因为这是一个
	// 临时变量，而非静态变量

    sortInsts();

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    // Check stall and squash signals.
    while (threads != end) {
        ThreadID tid = *threads++;

        DPRINTF(Rename, "Processing [tid:%i]\n", tid);

        status_change = checkSignalsAndUpdate(tid) || status_change;

        rename(status_change, tid);
    }

    if (status_change) {
        updateStatus();
    }

    if (wroteToTimeBuffer) {
        DPRINTF(Activity, "Activity this cycle.\n");
        cpu->activityThisCycle();
    }

    threads = activeThreads->begin();

    while (threads != end) {
        ThreadID tid = *threads++;

        // If we committed （this cycle？？？） then doneSeqNum will be > 0
        if (fromCommit->commitInfo[tid].doneSeqNum != 0 &&
            !fromCommit->commitInfo[tid].squash &&
            renameStatus[tid] != Squashing) {

            removeFromHistory(fromCommit->commitInfo[tid].doneSeqNum,
                                  tid);
        }
		// 如果上一个周期提交阶段确实提交了指令，提交阶段没有触发squash操作
		// 并且当前rename阶段没有进行suqash操纵，则根据提交指令中最新的一个对
		// rename历史表进行更新（移除已经安全提交的指令）
    }

    // @todo: make into updateProgress function
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        instsInProgress[tid] -= fromIEW->iewInfo[tid].dispatched;
        loadsInProgress[tid] -= fromIEW->iewInfo[tid].dispatchedToLQ;
        storesInProgress[tid] -= fromIEW->iewInfo[tid].dispatchedToSQ;
        assert(loadsInProgress[tid] >= 0);
        assert(storesInProgress[tid] >= 0);
        assert(instsInProgress[tid] >=0);
    }
	// 更新in-flight指令的计数器，

}

template<class Impl>
void
DefaultRename<Impl>::rename(bool &status_change, ThreadID tid)
{
    // If status is Running or idle,
    //     call renameInsts()
    // If status is Unblocking,
    //     buffer any instructions coming from decode
    //     continue trying to empty skid buffer
    //     check if stall conditions have passed

    if (renameStatus[tid] == Blocked) {
        ++renameBlockCycles;
    } else if (renameStatus[tid] == Squashing) {
        ++renameSquashCycles;
	// blocked和squashing状态会导致本周期无法进行重命名操作
	// 因此只会递增相关的周期计数器
    } else if (renameStatus[tid] == SerializeStall) {
        ++renameSerializeStallCycles;
        // If we are currently in SerializeStall and resumeSerialize
        // was set, then that means that we are resuming serializing
        // this cycle.  Tell the previous stages to block.
        if (resumeSerialize) {
            resumeSerialize = false;
            block(tid);
            toDecode->renameUnblock[tid] = false;
        }
		// 如果当前因为需要执行序列化指令而stall：
		// 这时候如果要求下一个周期继续序列化的执行指令，那么resumeSerialize
		// 将会被重置，并且阻塞rename阶段，并告知decode阶段不要停止阻塞。
		// 如果下一个周期不需要继续序列化，那么将不再阻塞rename阶段
    } else if (renameStatus[tid] == Unblocking) {
        if (resumeUnblocking) {
            block(tid);
            resumeUnblocking = false;
            toDecode->renameUnblock[tid] = false;
        }
		// 如果当前rename阶段处于Unblocking状态：
		// 这时候如果要求下一个周期继续维持该状态，那么resumeUnblocking
		// 将会被重置，并且继续阻塞rename阶段，并告知decode阶段不要停止阻塞。
		// 如果下一个周期可以退出阻塞正常执行，那么将不再阻塞rename阶段
    }
	
	// 下面根据处理后的状态选择是否对指令进行重命名操作
    if (renameStatus[tid] == Running ||
        renameStatus[tid] == Idle) {
        DPRINTF(Rename, "[tid:%u]: Not blocked, so attempting to run "
                "stage.\n", tid);

        renameInsts(tid);
    } else if (renameStatus[tid] == Unblocking) {
        renameInsts(tid);
		// 如果rename阶段依然处于尝试退出阻塞的状态，那么需要
		// 持续重命名skidBuffer中的指令，直到skidBuffer被清空

        if (validInsts()) {
            // Add the current inputs to the skid buffer so they can be
            // reprocessed when this stage unblocks.
            skidInsert(tid);
        }
		// 如果decode阶段发送来的指令，则需要将这些指令插入到skidBuffer中

        // If we switched over to blocking, then there's a potential for
        // an overall status change.
        status_change = unblock(tid) || status_change || blockThisCycle;
		// 尝试退出阻塞状态
    }
}

template <class Impl>
void
DefaultRename<Impl>::renameInsts(ThreadID tid)
{
    // Instructions can be either in the skid buffer or the queue of
    // instructions coming from decode, depending on the status.
    int insts_available = renameStatus[tid] == Unblocking ?
        skidBuffer[tid].size() : insts[tid].size();
	// 选择处理的数据，如果当前正在退出阻塞状态，那么从skidBuffer获取需要处理
	// 指令，否则从insts队列中获取需要处理的指令

    // Check the decode queue to see if instructions are available.
    // If there are no available instructions to rename, then do nothing.
    if (insts_available == 0) {
        DPRINTF(Rename, "[tid:%u]: Nothing to do, breaking out early.\n",
                tid);
        // Should I change status to idle?
        ++renameIdleCycles;
        return;
	// 当前周期需要处理的队列没有可以处理的指令，则会直接退出
    } else if (renameStatus[tid] == Unblocking) {
        ++renameUnblockCycles;
    } else if (renameStatus[tid] == Running) {
        ++renameRunCycles;
    }

    DynInstPtr inst;

    // Will have to do a different calculation for the number of free
    // entries.
    int free_rob_entries = calcFreeROBEntries(tid);
    int free_iq_entries  = calcFreeIQEntries(tid);
    int min_free_entries = free_rob_entries;

    FullSource source = ROB;

    if (free_iq_entries < min_free_entries) {
        min_free_entries = free_iq_entries;
        source = IQ;
    }
	// 确定本周期重命名指令的上限约束来自于哪一个结构的空闲表项数目
	// 这里只处理了IQ和ROB，LQ和SQ的处理在执行过程中动态判断
	// 这样可以保证不会因为LQ和SQ的限制导致重命名指令上限无故变小
	// （因为可能实际处理的指令中Store/load指令没有那么多）

    // Check if there's any space left.
    if (min_free_entries <= 0) {
        DPRINTF(Rename, "[tid:%u]: Blocking due to no free ROB/IQ/ "
                "entries.\n"
                "ROB has %i free entries.\n"
                "IQ has %i free entries.\n",
                tid,
                free_rob_entries,
                free_iq_entries);
				
        blockThisCycle = true;
        block(tid);
		// 发现存在关键队列已满的情况，因此需要在本周期将rename阻塞	
		
        incrFullStat(source);
		// 更新stall原因次数计数器

        return;
    } else if (min_free_entries < insts_available) {
        DPRINTF(Rename, "[tid:%u]: Will have to block this cycle."
                "%i insts available, but only %i insts can be "
                "renamed due to ROB/IQ/LSQ limits.\n",
                tid, insts_available, min_free_entries);

        insts_available = min_free_entries;

        blockThisCycle = true;

        incrFullStat(source);
		// 发现剩余的空闲表项数目小于当前可以处理的指令数目
		// 虽然也会进行block操作，但是依然会尽量处理和该上限一致
		// 的指令，然后才进行阻塞操作
    }

    InstQueue &insts_to_rename = renameStatus[tid] == Unblocking ?
        skidBuffer[tid] : insts[tid];
	// 选择处理指令的来源队列

    DPRINTF(Rename, "[tid:%u]: %i available instructions to "
            "send iew.\n", tid, insts_available);

    DPRINTF(Rename, "[tid:%u]: %i insts pipelining from Rename | %i insts "
            "dispatched to IQ last cycle.\n",
            tid, instsInProgress[tid], fromIEW->iewInfo[tid].dispatched);

    // Handle serializing the next instruction if necessary.
    if (serializeOnNextInst[tid]) {
        if (emptyROB[tid] && instsInProgress[tid] == 0) {
            // ROB already empty; no need to serialize.
            serializeOnNextInst[tid] = false;
			// 由于ROB已经为空，并且当前RS中没有任何指令
			// 因此无需序列化指令，即此处不需要等待之前的
			// 指令提交之后才进行重命名处理
        } else if (!insts_to_rename.empty()) {
            insts_to_rename.front()->setSerializeBefore();
			// 如果存在下一条指令，则将其设置为序列化执行
        }
    }
	// 处理序列化执行的指令

    int renamed_insts = 0;

	// 开始进行指令重命名，尽量多的处理指令，但是不能超过可用的指令
	// 以及renameWidth
    while (insts_available > 0 &&  toIEWIndex < renameWidth) {
        DPRINTF(Rename, "[tid:%u]: Sending instructions to IEW.\n", tid);

        assert(!insts_to_rename.empty());

        inst = insts_to_rename.front();

        //For all kind of instructions, check ROB and IQ first
        //For load instruction, check LQ size and take into account the inflight loads
        //For store instruction, check SQ size and take into account the inflight stores

        if (inst->isLoad()) {
            if (calcFreeLQEntries(tid) <= 0) {
                DPRINTF(Rename, "[tid:%u]: Cannot rename due to no free LQ\n");
                source = LQ;
                incrFullStat(source);
                break;
            }
        }
		// 如果动态处理过程中发现LQ已经没有空闲表项，而当前处理的是一个
		// load指令，那么会立即结束处理，更新stall的事件计数器。

        if (inst->isStore()) {
            if (calcFreeSQEntries(tid) <= 0) {
                DPRINTF(Rename, "[tid:%u]: Cannot rename due to no free SQ\n");
                source = SQ;
                incrFullStat(source);
                break;
            }
        }
		// 如果动态处理过程中发现SQ已经没有空闲表项，而当前处理的是一个
		// store指令，那么会立即结束处理，更新stall的事件计数器。
		
        insts_to_rename.pop_front();
		// 确定可以重命名该指令，因此将它从队列中弹出

        if (renameStatus[tid] == Unblocking) {
            DPRINTF(Rename,"[tid:%u]: Removing [sn:%lli] PC:%s from rename "
                    "skidBuffer\n", tid, inst->seqNum, inst->pcState());
        }

        if (inst->isSquashed()) {
            DPRINTF(Rename, "[tid:%u]: instruction %i with PC %s is "
                    "squashed, skipping.\n", tid, inst->seqNum,
                    inst->pcState());

            ++renameSquashedInsts;

            // Decrement how many instructions are available.
            --insts_available;

            continue;
        }
		// 跳过被标记为squashed的指令

        DPRINTF(Rename, "[tid:%u]: Processing instruction [sn:%lli] with "
                "PC %s.\n", tid, inst->seqNum, inst->pcState());

        // Check here to make sure there are enough destination registers
        // to rename to.  Otherwise block.
        if (!renameMap[tid]->canRename(inst->numIntDestRegs(),
                                       inst->numFPDestRegs(),
                                       inst->numVecDestRegs(),
                                       inst->numVecElemDestRegs(),
                                       inst->numCCDestRegs())) {
            DPRINTF(Rename, "Blocking due to lack of free "
                    "physical registers to rename to.\n");
            blockThisCycle = true;
            insts_to_rename.push_front(inst);
			// 由于该指令没有可用的重命名资源，因此该指令没有成功处理
			// 需要被退回到指令源队列中
            ++renameFullRegistersEvents;

            break;
        }
		// 如果发现任何一个寄存器的重命名资源不足就会阻塞rename阶段
		// 停止继续处理指令

        // Handle serializeAfter/serializeBefore instructions.
        // serializeAfter marks the next instruction as serializeBefore.
        // serializeBefore makes the instruction wait in rename until the ROB
        // is empty.
		
		// 这里似乎提供了两种序列化执行的处理，第一种serializeAfter会将
		// 下一条指令标记为serializeBefore，而被标记了serializeBefore的
		// 指令会使得该指令停在rename阶段指导ROB清空位置（即该指令之前的
		// 所有指令全部提交）。

        // In this model, IPR accesses are serialize before
        // instructions, and store conditionals are serialize after
        // instructions.  This is mainly due to lack of support for
        // out-of-order operations of either of those classes of
        // instructions.
		
		// 对于SC指令，在它后面的指令不得提前于SC指令执行，IPR是啥？？？
		
        if ((inst->isIprAccess() || inst->isSerializeBefore()) &&
            !inst->isSerializeHandled()) {
			// 对于serializeBefore类型指令的处理，这里要求该指令
			// 没有按照序列化处理过？？？
            DPRINTF(Rename, "Serialize before instruction encountered.\n");

            if (!inst->isTempSerializeBefore()) {
                renamedSerializing++;
                inst->setSerializeHandled();
				// 如果不是临时序列化执行，那么标记该指令，之后
				// 将不会按照序列化执行该指令
            } else {
                renamedTempSerializing++;
				// 存在特殊情况会要求一条指令临时按照序列化指令处理
				// 比如说错误speculative执行的load指令
            }

            // Change status over to SerializeStall so that other stages know
            // what this is blocked on.
            renameStatus[tid] = SerializeStall;
            serializeInst[tid] = inst;
            blockThisCycle = true;
			// 如果遇到serializeBefore的指令，则会导致rename阶段阻塞
			// 产生阻塞的指令则会被记录在serializeInst中
            break;
        } else if ((inst->isStoreConditional() || inst->isSerializeAfter()) &&
                   !inst->isSerializeHandled()) {
			// 对于serializeAfter类型指令的处理，这里要求该指令
			// 没有按照序列化处理过？？？
            DPRINTF(Rename, "Serialize after instruction encountered.\n");

            renamedSerializing++;

            inst->setSerializeHandled();
			// 该指令被标记，将不会再次作为序列化指令处理

            serializeAfter(insts_to_rename, tid);
			// 执行对于serializeAfter类型指令的处理
        }

        renameSrcRegs(inst, inst->threadNumber);
		// 通过查找重命名映射来重命名源寄存器

        renameDestRegs(inst, inst->threadNumber);
		// 重命名目的寄存器

        if (inst->isLoad()) {
                loadsInProgress[tid]++;
        }
        if (inst->isStore()) {
                storesInProgress[tid]++;
        }
        ++renamed_insts;
		// 递增相关的计数器
		
        // Notify potential listeners that source and destination registers for
        // this instruction have been renamed.
        ppRename->notify(inst);
		// 更新probe point变量

        // Put instruction in rename queue.
        toIEW->insts[toIEWIndex] = inst;
        ++(toIEW->size);
		// 将重命名好的指令写入到IEW阶段的timeBuffer中

        // Increment which instruction we're on.
        ++toIEWIndex;

        // Decrement how many instructions are available.
        --insts_available;
    }

    instsInProgress[tid] += renamed_insts;
    renameRenamedInsts += renamed_insts;
	// 更新计数器

    // If we wrote to the time buffer, record this.
    if (toIEWIndex) {
        wroteToTimeBuffer = true;
		// 如果确实向IEW的timeBuffer发送了至少1条重命名后的指令
		// 则设置wroteToTimeBuffer表示rename阶段本周期确实进行了
		// 一些指令的处理
    }

    // Check if there's any instructions left that haven't yet been renamed.
    // If so then block.
    if (insts_available) {
        blockThisCycle = true;
		// 如果最后发现存在剩余的指令没有处理，那么一定存在原因导致了阻塞
    }

    if (blockThisCycle) {
        block(tid);
        toDecode->renameUnblock[tid] = false;
		// 将本阶段block，并更新到decode的通信信号
    }
}

template<class Impl>
void
DefaultRename<Impl>::skidInsert(ThreadID tid)
{
    DynInstPtr inst = NULL;

    while (!insts[tid].empty()) {
        inst = insts[tid].front();

        insts[tid].pop_front();

        assert(tid == inst->threadNumber);

        DPRINTF(Rename, "[tid:%u]: Inserting [sn:%lli] PC: %s into Rename "
                "skidBuffer\n", tid, inst->seqNum, inst->pcState());

        ++renameSkidInsts;

        skidBuffer[tid].push_back(inst);
    }
	// 将对应线程insts队列中的指令放到skidBuffer中

    if (skidBuffer[tid].size() > skidBufferMax)
    {
        typename InstQueue::iterator it;
        warn("Skidbuffer contents:\n");
        for (it = skidBuffer[tid].begin(); it != skidBuffer[tid].end(); it++)
        {
            warn("[tid:%u]: %s [sn:%i].\n", tid,
                    (*it)->staticInst->disassemble(inst->instAddr()),
                    (*it)->seqNum);
        }
        panic("Skidbuffer Exceeded Max Size");
    }
	// 如果skidBuffer大小溢出，则会出现错误
}

template <class Impl>
void
DefaultRename<Impl>::sortInsts()
{
    int insts_from_decode = fromDecode->size;
    for (int i = 0; i < insts_from_decode; ++i) {
        DynInstPtr inst = fromDecode->insts[i];
        insts[inst->threadNumber].push_back(inst);
#if TRACING_ON
        if (DTRACE(O3PipeView)) {
            inst->renameTick = curTick() - inst->fetchTick;
			// 更新一个指令到达rename阶段的周期位置
        }
#endif
    }
}

template<class Impl>
bool
DefaultRename<Impl>::skidsEmpty()
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

template<class Impl>
void
DefaultRename<Impl>::updateStatus()
{
    bool any_unblocking = false;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (renameStatus[tid] == Unblocking) {
            any_unblocking = true;
            break;
        }
    }

    // Rename will have activity if it's unblocking.
    if (any_unblocking) {
	// 如果存在一个线程正在从阻塞状态中退出，就会导致rename阶段进入活跃状态
        if (_status == Inactive) {
            _status = Active;

            DPRINTF(Activity, "Activating stage.\n");

            cpu->activateStage(O3CPU::RenameIdx);
        }
    } else {
        // If it's not unblocking, then rename will not have any internal
        // activity.  Switch it to inactive.
	// 依然关心这里的Running会导致rename阶段进入不活跃状态的问题
	// 但是进入不活跃状态会有什么影响才是关键？？？
        if (_status == Active) {
            _status = Inactive;
            DPRINTF(Activity, "Deactivating stage.\n");

            cpu->deactivateStage(O3CPU::RenameIdx);
        }
    }
}

template <class Impl>
bool
DefaultRename<Impl>::block(ThreadID tid)
{
    DPRINTF(Rename, "[tid:%u]: Blocking.\n", tid);

    // Add the current inputs onto the skid buffer, so they can be
    // reprocessed when this stage unblocks.
    skidInsert(tid);
	// 在阻塞的时候需要首先将本周期没有处理的来自decode阶段的指令
	// 从insts队列中插入到skidBuffer中

    // Only signal backwards to block if the previous stages do not think
    // rename is already blocked.
    if (renameStatus[tid] != Blocked) {
        // If resumeUnblocking is set, we unblocked during the squash,
        // but now we're have unblocking status. We need to tell earlier
        // stages to block.
        if (resumeUnblocking || renameStatus[tid] != Unblocking) {
            toDecode->renameBlock[tid] = true;
            toDecode->renameUnblock[tid] = false;
			// 设置到decode阶段的通信信号以便使decode阶段阻塞
            wroteToTimeBuffer = true;
        }

        // Rename can not go from SerializeStall to Blocked, otherwise
        // it would not know to complete the serialize stall.
        if (renameStatus[tid] != SerializeStall) {
            // Set status to Blocked.
            renameStatus[tid] = Blocked;
            return true;
			// 如果当前rename阶段状态不是SerializeStall，就进入阻塞
			// 因为SerializeStall虽然和Block都是阻塞效果，但是处理不同
        }
    }

    return false;
}

template <class Impl>
bool
DefaultRename<Impl>::unblock(ThreadID tid)
{
    DPRINTF(Rename, "[tid:%u]: Trying to unblock.\n", tid);

    // Rename is done unblocking if the skid buffer is empty.
    if (skidBuffer[tid].empty() && renameStatus[tid] != SerializeStall) {

        DPRINTF(Rename, "[tid:%u]: Done unblocking.\n", tid);

        toDecode->renameUnblock[tid] = true;
		// 告知decode阶段rename阶段已经成功退出了阻塞状态
        wroteToTimeBuffer = true;

        renameStatus[tid] = Running;
        return true;
		// 当且仅当skidBuffer中的指令全部处理结束之后才可以
		// 退出unblocking状态进入running状态
    }

    return false;
}

template <class Impl>
void
DefaultRename<Impl>::doSquash(const InstSeqNum &squashed_seq_num, ThreadID tid)
{
    typename std::list<RenameHistory>::iterator hb_it =
        historyBuffer[tid].begin();

    // After a syscall squashes everything, the history buffer may be empty
    // but the ROB may still be squashing instructions.
    if (historyBuffer[tid].empty()) {
        return;
    }
	// 如果没有可以squash的重命名历史，则直接返回

    // Go through the most recent instructions, undoing the mappings
    // they did and freeing up the registers.
    while (!historyBuffer[tid].empty() &&
           hb_it->instSeqNum > squashed_seq_num) {
		// 循环遍历整个historyBuffer，将比squash指令更新的表项删除
		// 并恢复到rename_map中
        assert(hb_it != historyBuffer[tid].end());

        DPRINTF(Rename, "[tid:%u]: Removing history entry with sequence "
                "number %i.\n", tid, hb_it->instSeqNum);

        // Undo the rename mapping only if it was really a change.
        // Special regs that are not really renamed (like misc regs
        // and the zero reg) can be recognized because the new mapping
        // is the same as the old one.  While it would be merely a
        // waste of time to update the rename table, we definitely
        // don't want to put these on the free list.
		
        if (hb_it->newPhysReg != hb_it->prevPhysReg) {
			// 只有特殊的寄存器，映射前后目标物理寄存器是不变的
            // Tell the rename map to set the architected register to the
            // previous physical register that it was renamed to.
            renameMap[tid]->setEntry(hb_it->archReg, hb_it->prevPhysReg);

            // Put the renamed physical register back on the free list.
            freeList->addReg(hb_it->newPhysReg);
        }
		// 如果历史映射变更是一个真正的变更，则会将原始的映射关系写入到
		// rename_map中，并将删除映射的物理寄存器从freelist中释放

        // Notify potential listeners that the register mapping needs to be
        // removed because the instruction it was mapped to got squashed. 
		// Note that this is done before hb_it is incremented.
        ppSquashInRename->notify(std::make_pair(hb_it->instSeqNum,
                                                hb_it->newPhysReg));
		// 更新probe point

        historyBuffer[tid].erase(hb_it++);
		// 处理结束后将历史从historyBuffer中删除

        ++renameUndoneMaps;
		// 更新计数器
    }
}

template<class Impl>
void
DefaultRename<Impl>::removeFromHistory(InstSeqNum inst_seq_num, ThreadID tid)
{
    DPRINTF(Rename, "[tid:%u]: Removing a committed instruction from the "
            "history buffer %u (size=%i), until [sn:%lli].\n",
            tid, tid, historyBuffer[tid].size(), inst_seq_num);

    typename std::list<RenameHistory>::iterator hb_it =
        historyBuffer[tid].end();

    --hb_it;

    if (historyBuffer[tid].empty()) {
        DPRINTF(Rename, "[tid:%u]: History buffer is empty.\n", tid);
        return;
		// 对于空的historyBuffer无需进行处理
    } else if (hb_it->instSeqNum > inst_seq_num) {
        DPRINTF(Rename, "[tid:%u]: Old sequence number encountered.  Ensure "
                "that a syscall happened recently.\n", tid);
        return;
		// 尝试从historyBuffer中删除一条过老（比当前最老的指令还要老）
		// 而已经被删除过的指令将会立即返回
    }

    // Commit all the renames up until (and including) the committed sequence
    // number. Some or even all of the committed instructions may not have
    // rename histories if they did not have destination registers that were
    // renamed.
    while (!historyBuffer[tid].empty() &&
           hb_it != historyBuffer[tid].end() &&
           hb_it->instSeqNum <= inst_seq_num) {
		// 将historyBuffer中所有比提交指令更老的指令历史删除

        DPRINTF(Rename, "[tid:%u]: Freeing up older rename of reg %i (%s), "
                "[sn:%lli].\n",
                tid, hb_it->prevPhysReg->index(),
                hb_it->prevPhysReg->className(),
                hb_it->instSeqNum);

        // Don't free special phys regs like misc and zero regs, which
        // can be recognized because the new mapping is the same as
        // the old one.
        if (hb_it->newPhysReg != hb_it->prevPhysReg) {
            freeList->addReg(hb_it->prevPhysReg);
			// 释放已经不再使用的物理寄存器到freelist中
        }

        ++renameCommittedMaps;

        historyBuffer[tid].erase(hb_it--);
		// 将该表项释放
    }
}

template <class Impl>
inline void
DefaultRename<Impl>::renameSrcRegs(DynInstPtr &inst, ThreadID tid)
{
    ThreadContext *tc = inst->tcBase();
    RenameMap *map = renameMap[tid];
    unsigned num_src_regs = inst->numSrcRegs();

    // Get the architectual register numbers from the source and
    // operands, and redirect them to the right physical register.
    for (int src_idx = 0; src_idx < num_src_regs; src_idx++) {
		// 分别对指令中不同的源寄存器进行映射关系查找
        const RegId& src_reg = inst->srcRegIdx(src_idx);
		// 获取源寄存器的实际id，该id可以用来索引体系结构寄存器编号
        PhysRegIdPtr renamed_reg;

        renamed_reg = map->lookup(tc->flattenRegId(src_reg));
		// 在rename_map中查找该体系结构寄存器映射到的物理寄存器
        switch (src_reg.classValue()) {
          case IntRegClass:
            intRenameLookups++;
            break;
          case FloatRegClass:
            fpRenameLookups++;
            break;
          case VecRegClass:
            vecRenameLookups++;
            break;
          case CCRegClass:
          case MiscRegClass:
            break;

          default:
            panic("Invalid register class: %d.", src_reg.classValue());
        }
		// 递增映射查询计数器

        DPRINTF(Rename, "[tid:%u]: Looking up %s arch reg %i"
                ", got phys reg %i (%s)\n", tid,
                src_reg.className(), src_reg.index(),
                renamed_reg->index(),
                renamed_reg->className());

        inst->renameSrcReg(src_idx, renamed_reg);
		// 将重命名映射关系记录到指令中

        // See if the register is ready or not.
        if (scoreboard->getReg(renamed_reg)) {
            DPRINTF(Rename, "[tid:%u]: Register %d (flat: %d) (%s)"
                    " is ready.\n", tid, renamed_reg->index(),
                    renamed_reg->flatIndex(),
                    renamed_reg->className());

            inst->markSrcRegReady(src_idx);
			// 如果从记分板查到该源寄存器映射到的物理寄存器数据已经准备
			// 好了，则将该指令的源寄存器标记为ready
        } else {
            DPRINTF(Rename, "[tid:%u]: Register %d (flat: %d) (%s)"
                    " is not ready.\n", tid, renamed_reg->index(),
                    renamed_reg->flatIndex(),
                    renamed_reg->className());
        }

        ++renameRenameLookups;
		// 递增查询计数器
    }
}

template <class Impl>
inline void
DefaultRename<Impl>::renameDestRegs(DynInstPtr &inst, ThreadID tid)
{
    ThreadContext *tc = inst->tcBase();
    RenameMap *map = renameMap[tid];
    unsigned num_dest_regs = inst->numDestRegs();

    // Rename the destination registers.
    for (int dest_idx = 0; dest_idx < num_dest_regs; dest_idx++) {
		// 对目的寄存器逐个进行重命名
        const RegId& dest_reg = inst->destRegIdx(dest_idx);
        typename RenameMap::RenameInfo rename_result;
        RegId flat_dest_regid = tc->flattenRegId(dest_reg);
		// 获取目的寄存器对应的通用体系结构寄存器id

        rename_result = map->rename(flat_dest_regid);
		// 由rename_map进行重命名，并返回映射到的物理寄存器信息
		// 该信息包含了

        inst->flattenDestReg(dest_idx, flat_dest_regid);
		// 记录目的寄存器的通用体系结构寄存器编号

        // Mark Scoreboard entry as not ready
        scoreboard->unsetReg(rename_result.first);
		// 将该物理寄存器的数据标记位not ready

        DPRINTF(Rename, "[tid:%u]: Renaming arch reg %i (%s) to physical "
                "reg %i (%i).\n", tid, dest_reg.index(),
                dest_reg.className(),
                rename_result.first->index(),
                rename_result.first->flatIndex());

        // Record the rename information so that a history can be kept.
        RenameHistory hb_entry(inst->seqNum, flat_dest_regid,
                               rename_result.first,
                               rename_result.second);
		
        historyBuffer[tid].push_front(hb_entry);
		// 生成并添加一个重命名映射历史表项

        DPRINTF(Rename, "[tid:%u]: Adding instruction to history buffer "
                "(size=%i), [sn:%lli].\n",tid,
                historyBuffer[tid].size(),
                (*historyBuffer[tid].begin()).instSeqNum);

        // Tell the instruction to rename the appropriate destination
        // register (dest_idx) to the new physical register
        // (rename_result.first), and record the previous physical
        // register that the same logical register was renamed to
        // (rename_result.second).
        inst->renameDestReg(dest_idx,
                            rename_result.first,
                            rename_result.second);
		// 将映射关系记录在指令中

        ++renameRenamedOperands;
    }
}

template <class Impl>
inline int
DefaultRename<Impl>::calcFreeROBEntries(ThreadID tid)
{
    int num_free = freeEntries[tid].robEntries -
                  (instsInProgress[tid] - fromIEW->iewInfo[tid].dispatched);

    //DPRINTF(Rename,"[tid:%i]: %i rob free\n",tid,num_free);
	// 这里返回的是ROB报告的空闲表项数目减去已经离开rename阶段但是尚未登记
	// 到ROB的指令数量，即实际有效的ROB表项数目
    return num_free;
}

template <class Impl>
inline int
DefaultRename<Impl>::calcFreeIQEntries(ThreadID tid)
{
    int num_free = freeEntries[tid].iqEntries -
                  (instsInProgress[tid] - fromIEW->iewInfo[tid].dispatched);

    //DPRINTF(Rename,"[tid:%i]: %i iq free\n",tid,num_free);
	// 这里返回的是IQ报告的空闲表项数目减去已经离开rename阶段但是尚未登记
	// 到IQ的指令数量，即实际有效的IQ表项数目
    return num_free;
}

template <class Impl>
inline int
DefaultRename<Impl>::calcFreeLQEntries(ThreadID tid)
{
    int num_free = freeEntries[tid].lqEntries -
                  (loadsInProgress[tid] - fromIEW->iewInfo[tid].dispatchedToLQ);
    DPRINTF(Rename, "calcFreeLQEntries: free lqEntries: %d, loadsInProgress: %d, "
            "loads dispatchedToLQ: %d\n", freeEntries[tid].lqEntries,
            loadsInProgress[tid], fromIEW->iewInfo[tid].dispatchedToLQ);
    // 这里返回的是LQ报告的空闲表项数目减去已经离开rename阶段但是尚未登记
	// 到LQ的指令数量，即实际有效的LQ表项数目    
	return num_free;
}

template <class Impl>
inline int
DefaultRename<Impl>::calcFreeSQEntries(ThreadID tid)
{
    int num_free = freeEntries[tid].sqEntries -
                   (storesInProgress[tid] - fromIEW->iewInfo[tid].dispatchedToSQ);
    DPRINTF(Rename, "calcFreeSQEntries: free sqEntries: %d, storesInProgress: %d, "
            "stores dispatchedToSQ: %d\n", freeEntries[tid].sqEntries,
            storesInProgress[tid], fromIEW->iewInfo[tid].dispatchedToSQ);
	// 这里返回的是SQ报告的空闲表项数目减去已经离开rename阶段但是尚未登记
	// 到SQ的指令数量，即实际有效的SQ表项数目
    return num_free;
}

template <class Impl>
unsigned
DefaultRename<Impl>::validInsts()
{
    unsigned inst_count = 0;

    for (int i=0; i<fromDecode->size; i++) {
        if (!fromDecode->insts[i]->isSquashed())
            inst_count++;
    }
	// 这里只是在统计的时候去除了被squash掉的指令

    return inst_count;
}

template <class Impl>
void
DefaultRename<Impl>::readStallSignals(ThreadID tid)
{
    if (fromIEW->iewBlock[tid]) {
        stalls[tid].iew = true;
    }

    if (fromIEW->iewUnblock[tid]) {
        assert(stalls[tid].iew);
        stalls[tid].iew = false;
    }
}

template <class Impl>
bool
DefaultRename<Impl>::checkStall(ThreadID tid)
{
    bool ret_val = false;

    if (stalls[tid].iew) {
        DPRINTF(Rename,"[tid:%i]: Stall from IEW stage detected.\n", tid);
        ret_val = true;
    } else if (calcFreeROBEntries(tid) <= 0) {
        DPRINTF(Rename,"[tid:%i]: Stall: ROB has 0 free entries.\n", tid);
        ret_val = true;
    } else if (calcFreeIQEntries(tid) <= 0) {
        DPRINTF(Rename,"[tid:%i]: Stall: IQ has 0 free entries.\n", tid);
        ret_val = true;
    } else if (calcFreeLQEntries(tid) <= 0 && calcFreeSQEntries(tid) <= 0) {
        DPRINTF(Rename,"[tid:%i]: Stall: LSQ has 0 free entries.\n", tid);
        ret_val = true;
    } else if (renameMap[tid]->numFreeEntries() <= 0) {
        DPRINTF(Rename,"[tid:%i]: Stall: RenameMap has 0 free entries.\n", tid);
        ret_val = true;
    } else if (renameStatus[tid] == SerializeStall &&
               (!emptyROB[tid] || instsInProgress[tid])) {
        DPRINTF(Rename,"[tid:%i]: Stall: Serialize stall and ROB is not "
                "empty.\n", tid);
        ret_val = true;
		// 此处要求正在进行执行序列化操作，并且尚未结束
    }

    return ret_val;
}

template <class Impl>
void
DefaultRename<Impl>::readFreeEntries(ThreadID tid)
{
    if (fromIEW->iewInfo[tid].usedIQ)
        freeEntries[tid].iqEntries = fromIEW->iewInfo[tid].freeIQEntries;

    if (fromIEW->iewInfo[tid].usedLSQ) {
        freeEntries[tid].lqEntries = fromIEW->iewInfo[tid].freeLQEntries;
        freeEntries[tid].sqEntries = fromIEW->iewInfo[tid].freeSQEntries;
    }

    if (fromCommit->commitInfo[tid].usedROB) {
        freeEntries[tid].robEntries =
            fromCommit->commitInfo[tid].freeROBEntries;
        emptyROB[tid] = fromCommit->commitInfo[tid].emptyROB;
    }

    DPRINTF(Rename, "[tid:%i]: Free IQ: %i, Free ROB: %i, "
                    "Free LQ: %i, Free SQ: %i, FreeRM %i(%i %i %i %i)\n",
            tid,
            freeEntries[tid].iqEntries,
            freeEntries[tid].robEntries,
            freeEntries[tid].lqEntries,
            freeEntries[tid].sqEntries,
            renameMap[tid]->numFreeEntries(),
            renameMap[tid]->numFreeIntEntries(),
            renameMap[tid]->numFreeFloatEntries(),
            renameMap[tid]->numFreeVecEntries(),
            renameMap[tid]->numFreeCCEntries());

    DPRINTF(Rename, "[tid:%i]: %i instructions not yet in ROB\n",
            tid, instsInProgress[tid]);
}

template <class Impl>
bool
DefaultRename<Impl>::checkSignalsAndUpdate(ThreadID tid)
{
    // Check if there's a squash signal, squash if there is
    // Check stall signals, block if necessary.
    // If status was blocked
    //     check if stall conditions have passed
    //         if so then go to unblocking
    // If status was Squashing
    //     check if squashing is not high.  Switch to running this cycle.
    // If status was serialize stall
    //     check if ROB is empty and no insts are in flight to the ROB

    readFreeEntries(tid);
    readStallSignals(tid);

    if (fromCommit->commitInfo[tid].squash) {
        DPRINTF(Rename, "[tid:%u]: Squashing instructions due to squash from "
                "commit.\n", tid);

        squash(fromCommit->commitInfo[tid].doneSeqNum, tid);

        return true;
		// 如果接收到了commit阶段的squash信号，进行squash操作
    }

    if (checkStall(tid)) {
        return block(tid);
		// 如果需要stall，则阻塞rename阶段
    }

    if (renameStatus[tid] == Blocked) {
        DPRINTF(Rename, "[tid:%u]: Done blocking, switching to unblocking.\n",
                tid);
        renameStatus[tid] = Unblocking;
        unblock(tid);

        return true;
		// 如果当前周期不需要继续阻塞，则开始尝试退出阻塞状态
    }

    if (renameStatus[tid] == Squashing) {
		// 根据resume*标志来决定执行了squash操作后的状态
        // Switch status to running if rename isn't being told to block or
        // squash this cycle.
        if (resumeSerialize) {
            DPRINTF(Rename, "[tid:%u]: Done squashing, switching to serialize.\n",
                    tid);

            renameStatus[tid] = SerializeStall;
            return true;
        } else if (resumeUnblocking) {
            DPRINTF(Rename, "[tid:%u]: Done squashing, switching to unblocking.\n",
                    tid);
            renameStatus[tid] = Unblocking;
            return true;
        } else {
            DPRINTF(Rename, "[tid:%u]: Done squashing, switching to running.\n",
                    tid);

            renameStatus[tid] = Running;
			// 默认在squash后回到running的正常运行状态
            return false;
        }
    }

    if (renameStatus[tid] == SerializeStall) {
		// 如果当前正因为处理序列化执行指令而阻塞，则退出阻塞状态
		// 这里可以一定退出阻塞状态是因为如果不满足序列执行处理结束的情况
		// 在上面的checkStall函数处理就不会通过，而继续阻塞rename阶段
        // Stall ends once the ROB is free.
        DPRINTF(Rename, "[tid:%u]: Done with serialize stall, switching to "
                "unblocking.\n", tid);

        DynInstPtr serial_inst = serializeInst[tid];
        renameStatus[tid] = Unblocking;
        unblock(tid);
		// 尝试退出阻塞状态

        DPRINTF(Rename, "[tid:%u]: Processing instruction [%lli] with "
                "PC %s.\n", tid, serial_inst->seqNum, serial_inst->pcState());

        // Put instruction into queue here.
        serial_inst->clearSerializeBefore();
		// 清除指令的序列执行标记

        if (!skidBuffer[tid].empty()) {
            skidBuffer[tid].push_front(serial_inst);
        } else {
            insts[tid].push_front(serial_inst);
        }
		// 根据下一个周期的处理队列，将序列执行指令加入到对应队列的顶部

        DPRINTF(Rename, "[tid:%u]: Instruction must be processed by rename."
                " Adding to front of list.\n", tid);

        serializeInst[tid] = NULL;
		// 清空序列执行队列。

        return true;
    }

    // If we've reached this point, we have not gotten any signals that
    // cause rename to change its status.  Rename remains the same as before.
    return false;
}

template<class Impl>
void
DefaultRename<Impl>::serializeAfter(InstQueue &inst_list, ThreadID tid)
{
    if (inst_list.empty()) {
        // Mark a bit to say that I must serialize on the next instruction.
        serializeOnNextInst[tid] = true;
        return;
    }

    // Set the next instruction as serializing.
    inst_list.front()->setSerializeBefore();
}

template <class Impl>
inline void
DefaultRename<Impl>::incrFullStat(const FullSource &source)
{
    switch (source) {
      case ROB:
        ++renameROBFullEvents;
        break;
      case IQ:
        ++renameIQFullEvents;
        break;
      case LQ:
        ++renameLQFullEvents;
        break;
      case SQ:
        ++renameSQFullEvents;
        break;
      default:
        panic("Rename full stall stat should be incremented for a reason!");
        break;
    }
}

template <class Impl>
void
DefaultRename<Impl>::dumpHistory()
{
    typename std::list<RenameHistory>::iterator buf_it;

    for (ThreadID tid = 0; tid < numThreads; tid++) {

        buf_it = historyBuffer[tid].begin();

        while (buf_it != historyBuffer[tid].end()) {
            cprintf("Seq num: %i\nArch reg[%s]: %i New phys reg:"
                    " %i[%s] Old phys reg: %i[%s]\n",
                    (*buf_it).instSeqNum,
                    (*buf_it).archReg.className(),
                    (*buf_it).archReg.index(),
                    (*buf_it).newPhysReg->index(),
                    (*buf_it).newPhysReg->className(),
                    (*buf_it).prevPhysReg->index(),
                    (*buf_it).prevPhysReg->className());

            buf_it++;
        }
    }
}

#endif//__CPU_O3_RENAME_IMPL_HH__
