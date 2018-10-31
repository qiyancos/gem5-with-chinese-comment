/*
 * Copyright (c) 2011-2014 ARM Limited
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

#ifndef __CPU_O3_INST_QUEUE_IMPL_HH__
#define __CPU_O3_INST_QUEUE_IMPL_HH__

#include <limits>
#include <vector>

#include "cpu/o3/fu_pool.hh"
#include "cpu/o3/inst_queue.hh"
#include "debug/IQ.hh"
#include "enums/OpClass.hh"
#include "params/DerivO3CPU.hh"
#include "sim/core.hh"

// clang complains about std::set being overloaded with Packet::set if
// we open up the entire namespace std
using std::list;

template <class Impl>
InstructionQueue<Impl>::FUCompletion::FUCompletion(DynInstPtr &_inst,
    int fu_idx, InstructionQueue<Impl> *iq_ptr)
    : Event(Stat_Event_Pri, AutoDelete),
      inst(_inst), fuIdx(fu_idx), iqPtr(iq_ptr), freeFU(false)
{
}

template <class Impl>
void
InstructionQueue<Impl>::FUCompletion::process()
{
    iqPtr->processFUCompletion(inst, freeFU ? fuIdx : -1);
    inst = NULL;
}


template <class Impl>
const char *
InstructionQueue<Impl>::FUCompletion::description() const
{
    return "Functional unit completion";
}

template <class Impl>
InstructionQueue<Impl>::InstructionQueue(O3CPU *cpu_ptr, IEW *iew_ptr,
                                         DerivO3CPUParams *params)
    : cpu(cpu_ptr),
      iewStage(iew_ptr),
      fuPool(params->fuPool),
      numEntries(params->numIQEntries),
      totalWidth(params->issueWidth),
      commitToIEWDelay(params->commitToIEWDelay)
{
    assert(fuPool);
	// 必须指定一个执行单元池的指针

    numThreads = params->numThreads;

    // Set the number of total physical registers
    // As the vector registers have two addressing modes, they are added twice
    numPhysRegs = params->numPhysIntRegs + params->numPhysFloatRegs +
                    params->numPhysVecRegs +
                    params->numPhysVecRegs * TheISA::NumVecElemPerVecReg +
                    params->numPhysCCRegs;
	// 向量寄存器拥有两种寻址方式，一个是整体寻址，一个是元素寻址，因此加了两次

    //Create an entry for each physical register within the
    //dependency graph.
    dependGraph.resize(numPhysRegs);
	// 初始化dependGraph，为每个物理寄存器都设置一个独立的相关链表

    // Resize the register scoreboard.
    regScoreboard.resize(numPhysRegs);
	// 初始化寄存器数值准备状态的标志向量

    //Initialize Mem Dependence Units
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        memDepUnit[tid].init(params, tid);
        memDepUnit[tid].setIQ(this);
    }
	// 初始化内存相关检测/预测单元

    resetState();
	// 重置所有IQ的状态

    std::string policy = params->smtIQPolicy;
    //Convert string to lowercase
    std::transform(policy.begin(), policy.end(), policy.begin(),
                   (int(*)(int)) tolower);
	// 获得SMT IQ的策略名称并转换成小写

    //Figure out resource sharing policy
    if (policy == "dynamic") {
        iqPolicy = Dynamic;

        //Set Max Entries to Total ROB Capacity
        for (ThreadID tid = 0; tid < numThreads; tid++) {
            maxEntries[tid] = numEntries;
        }
	// 策略1：动态IQ
	// 每个线程单独分配的IQ长度都是numEntries
    } else if (policy == "partitioned") {
        iqPolicy = Partitioned;

        //@todo:make work if part_amt doesnt divide evenly.
        int part_amt = numEntries / numThreads;

        //Divide ROB up evenly
        for (ThreadID tid = 0; tid < numThreads; tid++) {
            maxEntries[tid] = part_amt;
        }

        DPRINTF(IQ, "IQ sharing policy set to Partitioned:"
                "%i entries per thread.\n",part_amt);
	// 策略2：分离的IQ
	// numEntries被均分到每个线程，每个线程的IQ只有numEntries的均分部分长度
    } else if (policy == "threshold") {
        iqPolicy = Threshold;

        double threshold =  (double)params->smtIQThreshold / 100;

        int thresholdIQ = (int)((double)threshold * numEntries);

        //Divide up by threshold amount
        for (ThreadID tid = 0; tid < numThreads; tid++) {
            maxEntries[tid] = thresholdIQ;
        }

        DPRINTF(IQ, "IQ sharing policy set to Threshold:"
                "%i entries per thread.\n",thresholdIQ);
	// 策略3：阈值模式
	// 有params给定的另一个数值（百分比，90表示90%）决定实际
	// 每个线程IQ的最大表项数目是numEntries的百分之多少
   } else {
       assert(0 && "Invalid IQ Sharing Policy.Options Are:{Dynamic,"
              "Partitioned, Threshold}");
   }
}

template <class Impl>
InstructionQueue<Impl>::~InstructionQueue()
{
    dependGraph.reset();
#ifdef DEBUG
    cprintf("Nodes traversed: %i, removed: %i\n",
            dependGraph.nodesTraversed, dependGraph.nodesRemoved);
#endif
}

template <class Impl>
std::string
InstructionQueue<Impl>::name() const
{
    return cpu->name() + ".iq";
}

template <class Impl>
void
InstructionQueue<Impl>::regStats()
{
    using namespace Stats;
    iqInstsAdded
        .name(name() + ".iqInstsAdded")
        .desc("Number of instructions added to the IQ (excludes non-spec)")
        .prereq(iqInstsAdded);

    iqNonSpecInstsAdded
        .name(name() + ".iqNonSpecInstsAdded")
        .desc("Number of non-speculative instructions added to the IQ")
        .prereq(iqNonSpecInstsAdded);

    iqInstsIssued
        .name(name() + ".iqInstsIssued")
        .desc("Number of instructions issued")
        .prereq(iqInstsIssued);

    iqIntInstsIssued
        .name(name() + ".iqIntInstsIssued")
        .desc("Number of integer instructions issued")
        .prereq(iqIntInstsIssued);

    iqFloatInstsIssued
        .name(name() + ".iqFloatInstsIssued")
        .desc("Number of float instructions issued")
        .prereq(iqFloatInstsIssued);

    iqBranchInstsIssued
        .name(name() + ".iqBranchInstsIssued")
        .desc("Number of branch instructions issued")
        .prereq(iqBranchInstsIssued);

    iqMemInstsIssued
        .name(name() + ".iqMemInstsIssued")
        .desc("Number of memory instructions issued")
        .prereq(iqMemInstsIssued);

    iqMiscInstsIssued
        .name(name() + ".iqMiscInstsIssued")
        .desc("Number of miscellaneous instructions issued")
        .prereq(iqMiscInstsIssued);

    iqSquashedInstsIssued
        .name(name() + ".iqSquashedInstsIssued")
        .desc("Number of squashed instructions issued")
        .prereq(iqSquashedInstsIssued);

    iqSquashedInstsExamined
        .name(name() + ".iqSquashedInstsExamined")
        .desc("Number of squashed instructions iterated over during squash;"
              " mainly for profiling")
        .prereq(iqSquashedInstsExamined);

    iqSquashedOperandsExamined
        .name(name() + ".iqSquashedOperandsExamined")
        .desc("Number of squashed operands that are examined and possibly "
              "removed from graph")
        .prereq(iqSquashedOperandsExamined);

    iqSquashedNonSpecRemoved
        .name(name() + ".iqSquashedNonSpecRemoved")
        .desc("Number of squashed non-spec instructions that were removed")
        .prereq(iqSquashedNonSpecRemoved);
/*
    queueResDist
        .init(Num_OpClasses, 0, 99, 2)
        .name(name() + ".IQ:residence:")
        .desc("cycles from dispatch to issue")
        .flags(total | pdf | cdf )
        ;
    for (int i = 0; i < Num_OpClasses; ++i) {
        queueResDist.subname(i, opClassStrings[i]);
    }
*/
    numIssuedDist
        .init(0,totalWidth,1)
        .name(name() + ".issued_per_cycle")
        .desc("Number of insts issued each cycle")
        .flags(pdf)
        ;
/*
    dist_unissued
        .init(Num_OpClasses+2)
        .name(name() + ".unissued_cause")
        .desc("Reason ready instruction not issued")
        .flags(pdf | dist)
        ;
    for (int i=0; i < (Num_OpClasses + 2); ++i) {
        dist_unissued.subname(i, unissued_names[i]);
    }
*/
    statIssuedInstType
        .init(numThreads,Enums::Num_OpClass)
        .name(name() + ".FU_type")
        .desc("Type of FU issued")
        .flags(total | pdf | dist)
        ;
    statIssuedInstType.ysubnames(Enums::OpClassStrings);

    //
    //  How long did instructions for a particular FU type wait prior to issue
    //
/*
    issueDelayDist
        .init(Num_OpClasses,0,99,2)
        .name(name() + ".")
        .desc("cycles from operands ready to issue")
        .flags(pdf | cdf)
        ;

    for (int i=0; i<Num_OpClasses; ++i) {
        std::stringstream subname;
        subname << opClassStrings[i] << "_delay";
        issueDelayDist.subname(i, subname.str());
    }
*/
    issueRate
        .name(name() + ".rate")
        .desc("Inst issue rate")
        .flags(total)
        ;
    issueRate = iqInstsIssued / cpu->numCycles;

    statFuBusy
        .init(Num_OpClasses)
        .name(name() + ".fu_full")
        .desc("attempts to use FU when none available")
        .flags(pdf | dist)
        ;
    for (int i=0; i < Num_OpClasses; ++i) {
        statFuBusy.subname(i, Enums::OpClassStrings[i]);
    }

    fuBusy
        .init(numThreads)
        .name(name() + ".fu_busy_cnt")
        .desc("FU busy when requested")
        .flags(total)
        ;

    fuBusyRate
        .name(name() + ".fu_busy_rate")
        .desc("FU busy rate (busy events/executed inst)")
        .flags(total)
        ;
    fuBusyRate = fuBusy / iqInstsIssued;

    for (ThreadID tid = 0; tid < numThreads; tid++) {
        // Tell mem dependence unit to reg stats as well.
        memDepUnit[tid].regStats();
    }

    intInstQueueReads
        .name(name() + ".int_inst_queue_reads")
        .desc("Number of integer instruction queue reads")
        .flags(total);

    intInstQueueWrites
        .name(name() + ".int_inst_queue_writes")
        .desc("Number of integer instruction queue writes")
        .flags(total);

    intInstQueueWakeupAccesses
        .name(name() + ".int_inst_queue_wakeup_accesses")
        .desc("Number of integer instruction queue wakeup accesses")
        .flags(total);

    fpInstQueueReads
        .name(name() + ".fp_inst_queue_reads")
        .desc("Number of floating instruction queue reads")
        .flags(total);

    fpInstQueueWrites
        .name(name() + ".fp_inst_queue_writes")
        .desc("Number of floating instruction queue writes")
        .flags(total);

    fpInstQueueWakeupAccesses
        .name(name() + ".fp_inst_queue_wakeup_accesses")
        .desc("Number of floating instruction queue wakeup accesses")
        .flags(total);

    vecInstQueueReads
        .name(name() + ".vec_inst_queue_reads")
        .desc("Number of vector instruction queue reads")
        .flags(total);

    vecInstQueueWrites
        .name(name() + ".vec_inst_queue_writes")
        .desc("Number of vector instruction queue writes")
        .flags(total);

    vecInstQueueWakeupAccesses
        .name(name() + ".vec_inst_queue_wakeup_accesses")
        .desc("Number of vector instruction queue wakeup accesses")
        .flags(total);

    intAluAccesses
        .name(name() + ".int_alu_accesses")
        .desc("Number of integer alu accesses")
        .flags(total);

    fpAluAccesses
        .name(name() + ".fp_alu_accesses")
        .desc("Number of floating point alu accesses")
        .flags(total);

    vecAluAccesses
        .name(name() + ".vec_alu_accesses")
        .desc("Number of vector alu accesses")
        .flags(total);

}

template <class Impl>
void
InstructionQueue<Impl>::resetState()
{
    //Initialize thread IQ counts
    for (ThreadID tid = 0; tid <numThreads; tid++) {
        count[tid] = 0;
        instList[tid].clear();
    }

    // Initialize the number of free IQ entries.
    freeEntries = numEntries;

    // Note that in actuality, the registers corresponding to the logical
    // registers start off as ready.  However this doesn't matter for the
    // IQ as the instruction should have been correctly told if those
    // registers are ready in rename.  Thus it can all be initialized as
    // unready.
    for (int i = 0; i < numPhysRegs; ++i) {
        regScoreboard[i] = false;
    }

    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        squashedSeqNum[tid] = 0;
    }

    for (int i = 0; i < Num_OpClasses; ++i) {
        while (!readyInsts[i].empty())
            readyInsts[i].pop();
        queueOnList[i] = false;
        readyIt[i] = listOrder.end();
    }
    nonSpecInsts.clear();
    listOrder.clear();
    deferredMemInsts.clear();
    blockedMemInsts.clear();
    retryMemInsts.clear();
    wbOutstanding = 0;
}

template <class Impl>
void
InstructionQueue<Impl>::setActiveThreads(list<ThreadID> *at_ptr)
{
    activeThreads = at_ptr;
}

template <class Impl>
void
InstructionQueue<Impl>::setIssueToExecuteQueue(TimeBuffer<IssueStruct> *i2e_ptr)
{
      issueToExecuteQueue = i2e_ptr;
}

template <class Impl>
void
InstructionQueue<Impl>::setTimeBuffer(TimeBuffer<TimeStruct> *tb_ptr)
{
    timeBuffer = tb_ptr;

    fromCommit = timeBuffer->getWire(-commitToIEWDelay);
}

template <class Impl>
bool
InstructionQueue<Impl>::isDrained() const
{
    bool drained = dependGraph.empty() &&
                   instsToExecute.empty() &&
                   wbOutstanding == 0;
    for (ThreadID tid = 0; tid < numThreads; ++tid)
        drained = drained && memDepUnit[tid].isDrained();

    return drained;
}

template <class Impl>
void
InstructionQueue<Impl>::drainSanityCheck() const
{
    assert(dependGraph.empty());
    assert(instsToExecute.empty());
    for (ThreadID tid = 0; tid < numThreads; ++tid)
        memDepUnit[tid].drainSanityCheck();
}

template <class Impl>
void
InstructionQueue<Impl>::takeOverFrom()
{
    resetState();
}

template <class Impl>
int
InstructionQueue<Impl>::entryAmount(ThreadID num_threads)
{
    if (iqPolicy == Partitioned) {
        return numEntries / num_threads;
    } else {
        return 0;
    }
}


template <class Impl>
void
InstructionQueue<Impl>::resetEntries()
{
    if (iqPolicy != Dynamic || numThreads > 1) {
		// 线程数量多于1个或者没有使用Dynamic策略
        int active_threads = activeThreads->size();

        list<ThreadID>::iterator threads = activeThreads->begin();
        list<ThreadID>::iterator end = activeThreads->end();
		// 获取当前活跃线程列表中的线程

        while (threads != end) {
            ThreadID tid = *threads++;

            if (iqPolicy == Partitioned) {
                maxEntries[tid] = numEntries / active_threads;
            } else if (iqPolicy == Threshold && active_threads == 1) {
                maxEntries[tid] = numEntries;
            }
			// 遍历所有的活跃进程，重新设置每个线程的最大表项数量
        }
    }
}

template <class Impl>
unsigned
InstructionQueue<Impl>::numFreeEntries()
{
    return freeEntries;
}

template <class Impl>
unsigned
InstructionQueue<Impl>::numFreeEntries(ThreadID tid)
{
    return maxEntries[tid] - count[tid];
}

// Might want to do something more complex if it knows how many instructions
// will be issued this cycle.
template <class Impl>
bool
InstructionQueue<Impl>::isFull()
{
    if (freeEntries == 0) {
        return(true);
    } else {
        return(false);
    }
}

template <class Impl>
bool
InstructionQueue<Impl>::isFull(ThreadID tid)
{
    if (numFreeEntries(tid) == 0) {
        return(true);
    } else {
        return(false);
    }
}

template <class Impl>
bool
InstructionQueue<Impl>::hasReadyInsts()
{
    if (!listOrder.empty()) {
        return true;
    }

    for (int i = 0; i < Num_OpClasses; ++i) {
        if (!readyInsts[i].empty()) {
            return true;
        }
    }

    return false;
}

template <class Impl>
void
InstructionQueue<Impl>::insert(DynInstPtr &new_inst)
{
    if (new_inst->isFloating()) {
        fpInstQueueWrites++;
    } else if (new_inst->isVector()) {
        vecInstQueueWrites++;
    } else {
        intInstQueueWrites++;
    }
	// 根据指令的类型更新相应的队列写操作计数器
	
    // Make sure the instruction is valid
    assert(new_inst);
	// 插入的指令不能为空

    DPRINTF(IQ, "Adding instruction [sn:%lli] PC %s to the IQ.\n",
            new_inst->seqNum, new_inst->pcState());

    assert(freeEntries != 0);
	// 当前IQ必须有位置插入指令，其实这里应该判断指令线程的队列
	// 是否有空闲位置，但是有的策略支持动态调整队列宽度，因此指令
	// 对应IQ没有空闲位置不代表无法存放该指令。
	
	// 上面的两个断言说明，在调用assert之前一定有相应的判断

    instList[new_inst->threadNumber].push_back(new_inst);
	// 将指令插入到对应线程的IQ中

    --freeEntries;
	// 更新总空闲表项数目

    new_inst->setInIQ();
	// 告知指令他所在对应的IQ，函数定义在base_dyn_inst.hh中

    // Look through its source registers (physical regs), and mark any
    // dependencies.
    addToDependents(new_inst);
    // Have this instruction set itself as the producer of its destination
    // register(s).
    addToProducers(new_inst);
	// 按照消费者和生产者的身份加入到dependGraph中

    if (new_inst->isMemRef()) {
        memDepUnit[new_inst->threadNumber].insert(new_inst);
    } else {
        addIfReady(new_inst);
    }
	// 存储访问指令丢给内存相关检测单元
	// 非存储类指令尝试加入好准备好数据发射的指令队列中

    ++iqInstsAdded;
	// 递增IQ指令添加次数计数器

    count[new_inst->threadNumber]++;
	// 递增对应线程IQ队列已使用表项数目的计数器

    assert(freeEntries == (numEntries - countInsts()));
	// 确定所有计数都准确无误
}

template <class Impl>
void
InstructionQueue<Impl>::insertNonSpec(DynInstPtr &new_inst)
{
    // @todo: Clean up this code; can do it by setting inst as unable
    // to issue, then calling normal insert on the inst.
    if (new_inst->isFloating()) {
        fpInstQueueWrites++;
    } else if (new_inst->isVector()) {
        vecInstQueueWrites++;
    } else {
        intInstQueueWrites++;
    }

    assert(new_inst);

    nonSpecInsts[new_inst->seqNum] = new_inst;

    DPRINTF(IQ, "Adding non-speculative instruction [sn:%lli] PC %s "
            "to the IQ.\n",
            new_inst->seqNum, new_inst->pcState());

    assert(freeEntries != 0);

    instList[new_inst->threadNumber].push_back(new_inst);

    --freeEntries;

    new_inst->setInIQ();

    // Have this instruction set itself as the producer of its destination
    // register(s).
    addToProducers(new_inst);

    // If it's a memory instruction, add it to the memory dependency
    // unit.
    if (new_inst->isMemRef()) {
        memDepUnit[new_inst->threadNumber].insertNonSpec(new_inst);
    }

    ++iqNonSpecInstsAdded;
	// 占用NonSpec指令队列，相关计数器递增
	
    count[new_inst->threadNumber]++;
	// 依然占用IQ

    assert(freeEntries == (numEntries - countInsts()));
}

template <class Impl>
void
InstructionQueue<Impl>::insertBarrier(DynInstPtr &barr_inst)
{
    memDepUnit[barr_inst->threadNumber].insertBarrier(barr_inst);

    insertNonSpec(barr_inst);
}

template <class Impl>
typename Impl::DynInstPtr
InstructionQueue<Impl>::getInstToExecute()
{
    assert(!instsToExecute.empty());
	
    DynInstPtr inst = instsToExecute.front();
    instsToExecute.pop_front();
	// 将instsToExecute队列最前端的指令删除
	
    if (inst->isFloating()) {
        fpInstQueueReads++;
    } else if (inst->isVector()) {
        vecInstQueueReads++;
    } else {
        intInstQueueReads++;
    }
	// 更新指令队列读次数的计数器
	
    return inst;
}

template <class Impl>
void
InstructionQueue<Impl>::addToOrderList(OpClass op_class)
{
    assert(!readyInsts[op_class].empty());
	// 希望添加的op class对应的readyInsts队列不为空

    ListOrderEntry queue_entry;

    queue_entry.queueType = op_class;
    queue_entry.oldestInst = readyInsts[op_class].top()->seqNum;
	// 生成一个listOrder的表项，包含了OPclass和全局序列号
	// 全局序列号对应于该op class的readyInsts队列中最老的指令
	
    ListOrderIt list_it = listOrder.begin();
    ListOrderIt list_end_it = listOrder.end();

    while (list_it != list_end_it) {
        if ((*list_it).oldestInst > queue_entry.oldestInst) {
            break;
        }

        list_it++;
    }
	// 循环遍历listOrder列表，找到比当前表项对应指令更新指令中最老的那个

    readyIt[op_class] = listOrder.insert(list_it, queue_entry);
	// 然后在该表项后面插入新的表项
    queueOnList[op_class] = true;
	// 同时设置queueList说明该op class数据已经被存放在listOrder中
}

template <class Impl>
void
InstructionQueue<Impl>::moveToYoungerInst(ListOrderIt list_order_it)
{
    // Get iterator of next item on the list
    // Delete the original iterator
    // Determine if the next item is either the end of the list or younger
    // than the new instruction.  If so, then add in a new iterator right here.
    // If not, then move along.
	
    ListOrderEntry queue_entry;
    OpClass op_class = (*list_order_it).queueType;
    ListOrderIt next_it = list_order_it;

    ++next_it;

    queue_entry.queueType = op_class;
    queue_entry.oldestInst = readyInsts[op_class].top()->seqNum;
	// 获取该表项对应op class的readyInsts队列中新的最老指令

    while (next_it != listOrder.end() &&
           (*next_it).oldestInst < queue_entry.oldestInst) {
        ++next_it;
    }

    readyIt[op_class] = listOrder.insert(next_it, queue_entry);
}

template <class Impl>
void
InstructionQueue<Impl>::processFUCompletion(DynInstPtr &inst, int fu_idx)
{
    DPRINTF(IQ, "Processing FU completion [sn:%lli]\n", inst->seqNum);
    assert(!cpu->switchedOut());
    // The CPU could have been sleeping until this op completed (*extremely*
    // long latency op).  Wake it if it was.  This may be overkill.
	// 处理要求当前CPU一定处于运行状态
	
   --wbOutstanding;
   // 一条in-flight的指令完成，递减该类型指令计数
    iewStage->wakeCPU();
	// 唤醒IEW阶段的处理？？？

    if (fu_idx > -1)
        fuPool->freeUnitNextCycle(fu_idx);
		// 正常情况下会在下一个周期释放掉对应的FU
	// 某些情况下给出的fu_idx为-1，表示？？？

    // @todo: Ensure that these FU Completions happen at the beginning
    // of a cycle, otherwise they could add too many instructions to
    // the queue.
    issueToExecuteQueue->access(-1)->size++;
	// 发射阶段到执行阶段的队列指针进行处理？？？
    instsToExecute.push_back(inst);
	// 指令被添加到instsToExecute的尾部
}

// @todo: Figure out a better way to remove the squashed items from the
// lists.  Checking the top item of each list to see if it's squashed
// wastes time and forces jumps.
template <class Impl>
void
InstructionQueue<Impl>::scheduleReadyInsts()
{
    DPRINTF(IQ, "Attempting to schedule ready instructions from "
            "the IQ.\n");

    IssueStruct *i2e_info = issueToExecuteQueue->access(0);
	// ？？？

    DynInstPtr mem_inst;
    while (mem_inst = getDeferredMemInstToExecute()) {
        addReadyMemInst(mem_inst);
    }
	// 首先尝试获取因为DTLB miss被延迟的访存指令并加入到
	// 准备好的访存指令队列中，此处获取可能不止一条

    // See if any cache blocked instructions are able to be executed
    while (mem_inst = getBlockedMemInstToExecute()) {
        addReadyMemInst(mem_inst);
    }
	// 再次尝试获取因为访存阻塞而被延迟的访存指令加入到
	// 准备好的访存指令队列中，此处获取可能不止一条

    // Have iterator to head of the list
    // While I haven't exceeded bandwidth or reached the end of the list,
    // Try to get a FU that can do what this op needs.
    // If successful, change the oldestInst to the new top of the list, put
    // the queue in the proper place in the list.
    // Increment the iterator.
    // This will avoid trying to schedule a certain op class if there are no
    // FUs that handle it.
	
    int total_issued = 0;
    ListOrderIt order_it = listOrder.begin();
    ListOrderIt order_end_it = listOrder.end();

    while (total_issued < totalWidth && order_it != order_end_it) {
		// 循环查找listOrder找出最老的且可以发射指令，直到填满发射宽度为止
        OpClass op_class = (*order_it).queueType;

        assert(!readyInsts[op_class].empty());

        DynInstPtr issuing_inst = readyInsts[op_class].top();
		// 找到listOrder当前遍历位置对应在readyInsts中的指令

        if (issuing_inst->isFloating()) {
            fpInstQueueReads++;
        } else if (issuing_inst->isVector()) {
            vecInstQueueReads++;
        } else {
            intInstQueueReads++;
        }
		// 递增相关计数器

        assert(issuing_inst->seqNum == (*order_it).oldestInst);

        if (issuing_inst->isSquashed()) {
			// 发现当前遍历位置对应的指令已经被squash了
            readyInsts[op_class].pop();
			// 清理该指令

            if (!readyInsts[op_class].empty()) {
                moveToYoungerInst(order_it);
            } else {
                readyIt[op_class] = listOrder.end();
                queueOnList[op_class] = false;
            }
            listOrder.erase(order_it++);
			// 因为readyInsts发生变化，更新listOrder队列
			// 将新的队首指令加入到listOrder，并删除旧的队首指令
            // 或者完全删除一个op class的相关记录
			
			++iqSquashedInstsIssued;
			// 记录因为被squash而无法发射的指令计数器

            continue;
        }

        int idx = FUPool::NoCapableFU;
        Cycles op_latency = Cycles(1);
		// 初始化的idx是一个空FU，执行延迟默认设置为1
        ThreadID tid = issuing_inst->threadNumber;

        if (op_class != No_OpClass) {
            idx = fuPool->getUnit(op_class);
			// 根据执行运算类型获取可用执行单元的索引
            
			if (issuing_inst->isFloating()) {
                fpAluAccesses++;
            } else if (issuing_inst->isVector()) {
                vecAluAccesses++;
            } else {
                intAluAccesses++;
            }
			// 递增相关计数器
			
            if (idx > FUPool::NoFreeFU) {
                op_latency = fuPool->getOpLatency(op_class);
				// 重新设置正确的执行延迟
            }
        }
		// 分配相应的执行单元操作

        // If we have an instruction that doesn't require a FU, or a
        // valid FU, then schedule for execution.
        if (idx != FUPool::NoFreeFU) {
			// 下面的处理表示成功分配到了一个空闲的执行单元
            if (op_latency == Cycles(1)) {
                i2e_info->size++;
                instsToExecute.push_back(issuing_inst);
				// 指令被放到instToExecute的队列尾部

                // Add the FU onto the list of FU's to be freed next
                // cycle if we used one.
                if (idx >= 0)
                    fuPool->freeUnitNextCycle(idx);
				// 如果给定执行单元的执行延迟是一个周期，则立即通知FUPool
				// 对应的执行单元下一个周期就空闲了
				
				// 由于之另一个周期就完成了，因此这里其实同时把发射和完成的
				// 工作都处理了，因此没有递增wbOutstanding
            } else {
                bool pipelined = fuPool->isPipelined(op_class);
				// 因为多周期指令有些使用了流水线设计的执行单元，因此
				// 该标志决定该执行单元是否是流水线化的
				
                // Generate completion event for the FU
                ++wbOutstanding;
				// 由于1个周期无法完成因此对wbOutstanding递增进行记录
                
				FUCompletion *execution = new FUCompletion(issuing_inst,
                                                           idx, this);
				// 只有多周期指令才会生成FUCompletion结构体

                cpu->schedule(execution,
                              cpu->clockEdge(Cycles(op_latency - 1)));
				// CPU对指令进行实际的调度操作，这里给出了指令执行延迟
				// 该函数被定义在？？？

                if (!pipelined) {
                    // If FU isn't pipelined, then it must be freed
                    // upon the execution completing.
                    execution->setFreeFU();
					// 说明执行结束后执行单元才可以作为空闲单元处理
                } else {
                    // Add the FU onto the list of FU's to be freed next cycle.
                    fuPool->freeUnitNextCycle(idx);
					// 流水线化之后一个执行单元在下一个周期便可以立即被使用
					// 因此调用freeUnitNextCycle在下一个周期释放执行单元
                }
            }

            DPRINTF(IQ, "Thread %i: Issuing instruction PC %s "
                    "[sn:%lli]\n",
                    tid, issuing_inst->pcState(),
                    issuing_inst->seqNum);

            readyInsts[op_class].pop();
			// 从readyInsts队列中删除相应的指令

            if (!readyInsts[op_class].empty()) {
                moveToYoungerInst(order_it);
            } else {
                readyIt[op_class] = listOrder.end();
                queueOnList[op_class] = false;
            }
			// 这里和前面一段相似，但是此处是针对非squash情况下
			// 正常发射指令后的更新操作，但是两段代码效果相同

            issuing_inst->setIssued();
			// 设置指令的标志位表示指令已经发射了
            ++total_issued;
			// 递增单周期发射指令总和计数器

#if TRACING_ON
            issuing_inst->issueTick = curTick() - issuing_inst->fetchTick;
			// 这个是对指令发射周期位置进行记录的处理，和trace有关
#endif

            if (!issuing_inst->isMemRef()) {
                // Memory instructions can not be freed from the IQ until they
                // complete.
                ++freeEntries;
                count[tid]--;
                // 更新计数器数值
				issuing_inst->clearInIQ();
				// 调用clearInIQ来清除自己在IQ所有队列中的资源
				
				// 对于非访存类的指令，在发射以后便可以从IQ中清理掉
				// 并空出来新的表项位置
            } else {
                memDepUnit[tid].issue(issuing_inst);
				// 访存类的指令会调用issue尝试发射该指令去完成访存操作
            }

            listOrder.erase(order_it++);
            statIssuedInstType[tid][op_class]++;
        } else {
            statFuBusy[op_class]++;
            fuBusy[tid]++;
			// 递增两个统计计数器
            ++order_it;
			// 继续查找下一个指令，获取一个可以发射的指令
			
			// 这里发现指令对应的执行单元处于正忙状态
			// 即没有空闲的执行单元可以分配给该指令
        }
    }

    numIssuedDist.sample(total_issued);
	// 记录每个周期的发射指令数目
    iqInstsIssued+= total_issued;
	// 递增IQ发射指令总数的计数器

    // If we issued any instructions, tell the CPU we had activity.
    // @todo If the way deferred memory instructions are handeled due to
    // translation changes then the deferredMemInsts condition should be removed
    // from the code below.
    if (total_issued || !retryMemInsts.empty() || !deferredMemInsts.empty()) {
        cpu->activityThisCycle();
		// 调度处理完成，有至少一个指令会在本周期处理（来自发射、需要重试
		// 的访存指令），则调用activityThisCycle激活本周期的处理
    } else {
        DPRINTF(IQ, "Not able to schedule any instructions.\n");
		// 否则不会CPU在本周期不会被激活，因为没有什么可以处理的指令
		// 这种情况就意味着发射阶段出现了发射空挡周期
    }
}

template <class Impl>
void
InstructionQueue<Impl>::scheduleNonSpec(const InstSeqNum &inst)
{
    DPRINTF(IQ, "Marking nonspeculative instruction [sn:%lli] as ready "
            "to execute.\n", inst);

    NonSpecMapIt inst_it = nonSpecInsts.find(inst);
	// 根据给定的全局序列号索引获取对应的非推测式指令

    assert(inst_it != nonSpecInsts.end());
	// 如果调用本函数说明一定存在可以处理的非推测执行指令

    ThreadID tid = (*inst_it).second->threadNumber;

    (*inst_it).second->setAtCommit();
	// 进行提交阶段的设置？？？

    (*inst_it).second->setCanIssue();
	// 进行设置表示可以发射？？？

    if (!(*inst_it).second->isMemRef()) {
        addIfReady((*inst_it).second);
    } else {
        memDepUnit[tid].nonSpecInstReady((*inst_it).second);
    }
	// 访存指令被加入到存储相关检测单元，其他指令则加入到readyInsts队列中

    (*inst_it).second = NULL;
    nonSpecInsts.erase(inst_it);
	// 成功调度后清理nonSpecInsts中该元素
}

template <class Impl>
void
InstructionQueue<Impl>::commit(const InstSeqNum &inst, ThreadID tid)
{
    DPRINTF(IQ, "[tid:%i]: Committing instructions older than [sn:%i]\n",
            tid,inst);

    ListIt iq_it = instList[tid].begin();

    while (iq_it != instList[tid].end() &&
           (*iq_it)->seqNum <= inst) {
        ++iq_it;
        instList[tid].pop_front();
    }
	// 将所有比给定全局序列编号更老的指令更老的指令从instList中删除
	// 注意这里没有更新空闲表项数目！

    assert(freeEntries == (numEntries - countInsts()));
}

template <class Impl>
int
InstructionQueue<Impl>::wakeDependents(DynInstPtr &completed_inst)
{
    int dependents = 0;

    // The instruction queue here takes care of both floating and int ops
    if (completed_inst->isFloating()) {
        fpInstQueueWakeupAccesses++;
    } else if (completed_inst->isVector()) {
        vecInstQueueWakeupAccesses++;
    } else {
        intInstQueueWakeupAccesses++;
    }
	// 递增相关计数器

    DPRINTF(IQ, "Waking dependents of completed instruction.\n");

    assert(!completed_inst->isSquashed());
	// 希望唤醒相关指令的指令一定不能是被squash的指令

    // Tell the memory dependence unit to wake any dependents on this
    // instruction if it is a memory instruction.  Also complete the memory
    // instruction at this point since we know it executed without issues.
    // @todo: Might want to rename "completeMemInst" to something that
    // indicates that it won't need to be replayed, and call this
    // earlier.  Might not be a big deal.
	
    if (completed_inst->isMemRef()) {
        memDepUnit[completed_inst->threadNumber].wakeDependents(completed_inst);
        completeMemInst(completed_inst);
		// 如果处理对象是一个访存指令，则使用存储相关检测单元对相关指令进行唤醒
		// 而completeMemInst则是通知存储检测单元该指令可以完成了
    } else if (completed_inst->isMemBarrier() ||
               completed_inst->isWriteBarrier()) {
		// 如果给定指令是一个访存Barrier，则会不需要唤醒相关指令
		// 直接调用completeBarrier完成该指令即可
        memDepUnit[completed_inst->threadNumber].completeBarrier(completed_inst);
    }

    for (int dest_reg_idx = 0;
         dest_reg_idx < completed_inst->numDestRegs();
         dest_reg_idx++)
    // 处理的指令可能有多余一个目的寄存器，因此需要根据每个
	// 目的寄存器的相关关系进行唤醒处理
	{
        PhysRegIdPtr dest_reg =
            completed_inst->renamedDestRegIdx(dest_reg_idx);
		// 获取该指令目的寄存器重命名后映射到物理寄存器指针

        // Special case of uniq or control registers.  They are not
        // handled by the IQ and thus have no dependency graph entry.
        if (dest_reg->isFixedMapping()) {
            DPRINTF(IQ, "Reg %d [%s] is part of a fix mapping, skipping\n",
                    dest_reg->index(), dest_reg->className());
            continue;
        }
		// 对于固定映射的物理寄存器的相关处理不由IQ进行处理

        DPRINTF(IQ, "Waking any dependents on register %i (%s).\n",
                dest_reg->index(),
                dest_reg->className());

        //Go through the dependency chain, marking the registers as
        //ready within the waiting instructions.
        DynInstPtr dep_inst = dependGraph.pop(dest_reg->flatIndex());
		// 获取对应物理寄存器的相关关系链表的头结点后面的元素
		// 注意这里的pop函数是定义在dep_graph.hh中的函数

        while (dep_inst) {
            DPRINTF(IQ, "Waking up a dependent instruction, [sn:%lli] "
                    "PC %s.\n", dep_inst->seqNum, dep_inst->pcState());

            // Might want to give more information to the instruction
            // so that it knows which of its source registers is
            // ready.  However that would mean that the dependency
            // graph entries would need to hold the src_reg_idx.
            dep_inst->markSrcRegReady();
			// 相关指令的源寄存器被标记为Ready

            addIfReady(dep_inst);
			// 如果该指令的所有源寄存器都准备好了，就会被加入到readyInsts队列中

            dep_inst = dependGraph.pop(dest_reg->flatIndex());
			// 获取下一个寄存器相关指令

            ++dependents;
			// 对唤醒的指令进行统计
        }
		// 上面的唤醒行为其实就是RS中的寄存器ready修改过程

        // Reset the head node now that all of its dependents have
        // been woken up.
        assert(dependGraph.empty(dest_reg->flatIndex()));
		// 一次处理一定会把所有相关指令都唤醒
        dependGraph.clearInst(dest_reg->flatIndex());
		// 清理dependGraph对应物理寄存器相关链表的头节点

        // Mark the scoreboard as having that register ready.
        regScoreboard[dest_reg->flatIndex()] = true;
		// 说明对应寄存器的数据已经是最新的数据，而不是正在计算中
    }
    return dependents;
}

template <class Impl>
void
InstructionQueue<Impl>::addReadyMemInst(DynInstPtr &ready_inst)
{
    OpClass op_class = ready_inst->opClass();
    readyInsts[op_class].push(ready_inst);
	// 将给定的指令加入到其op class对应的readyInsts队列中

    // Will need to reorder the list if either a queue is not on the list,
    // or it has an older instruction than last time.
    if (!queueOnList[op_class]) {
        addToOrderList(op_class);
    } else if (readyInsts[op_class].top()->seqNum  <
               (*readyIt[op_class]).oldestInst) {
        listOrder.erase(readyIt[op_class]);
        addToOrderList(op_class);
    }
	// 尝试更新listOrder队列的指令

    DPRINTF(IQ, "Instruction is ready to issue, putting it onto "
            "the ready list, PC %s opclass:%i [sn:%lli].\n",
            ready_inst->pcState(), op_class, ready_inst->seqNum);
}

template <class Impl>
void
InstructionQueue<Impl>::rescheduleMemInst(DynInstPtr &resched_inst)
{
    DPRINTF(IQ, "Rescheduling mem inst [sn:%lli]\n", resched_inst->seqNum);

    // Reset DTB translation state
    resched_inst->translationStarted(false);
    resched_inst->translationCompleted(false);
	// 将DTLB的处理状态重置为初始状态

    resched_inst->clearCanIssue();
	// 对应指令的允许发射的标志位被重置
    memDepUnit[resched_inst->threadNumber].reschedule(resched_inst);
	// 由内存相关检测单元进行重新调度
}

template <class Impl>
void
InstructionQueue<Impl>::replayMemInst(DynInstPtr &replay_inst)
{
    memDepUnit[replay_inst->threadNumber].replay();
}

template <class Impl>
void
InstructionQueue<Impl>::completeMemInst(DynInstPtr &completed_inst)
{
    ThreadID tid = completed_inst->threadNumber;

    DPRINTF(IQ, "Completing mem instruction PC: %s [sn:%lli]\n",
            completed_inst->pcState(), completed_inst->seqNum);

    ++freeEntries;
	// 这里直接更新了空闲表项计数，

    completed_inst->memOpDone(true);

    memDepUnit[tid].completed(completed_inst);
	// 该操作应该会将该指令从instList中删除
    count[tid]--;
}

template <class Impl>
void
InstructionQueue<Impl>::deferMemInst(DynInstPtr &deferred_inst)
{
    deferredMemInsts.push_back(deferred_inst);
}

template <class Impl>
void
InstructionQueue<Impl>::blockMemInst(DynInstPtr &blocked_inst)
{
    blocked_inst->translationStarted(false);
    blocked_inst->translationCompleted(false);
	// 重置处理状态
    blocked_inst->clearIssued();
    blocked_inst->clearCanIssue();
	// ？？？
    blockedMemInsts.push_back(blocked_inst);
}

template <class Impl>
void
InstructionQueue<Impl>::cacheUnblocked()
{
    retryMemInsts.splice(retryMemInsts.end(), blockedMemInsts);
	// 由于存储阻塞被解决，这里会将blockedMemInsts中的所有指令加入到
	// retryMemInsts的末尾，准备进行重试。splice被用于list的拼接。
    // Get the CPU ticking again
    cpu->wakeCPU();
	// 由于访存阻塞得到解决，可能存在新的可执行指令
}

template <class Impl>
typename Impl::DynInstPtr
InstructionQueue<Impl>::getDeferredMemInstToExecute()
{
    for (ListIt it = deferredMemInsts.begin(); it != deferredMemInsts.end();
         ++it) {
        if ((*it)->translationCompleted() || (*it)->isSquashed()) {
			// 如果希望该指令执行，则要求指令的地址翻译已经完成
			// 或者这个指令已经被squash了，需要调度函数进行处理
            DynInstPtr mem_inst = *it;
            deferredMemInsts.erase(it);
			// 被选择的队列将会从队列中删除
            return mem_inst;
        }
    }
	// 循环遍历deferredMemInsts队列查找
    return nullptr;
}

template <class Impl>
typename Impl::DynInstPtr
InstructionQueue<Impl>::getBlockedMemInstToExecute()
{
    if (retryMemInsts.empty()) {
        return nullptr;
    } else {
        DynInstPtr mem_inst = retryMemInsts.front();
        retryMemInsts.pop_front();
        return mem_inst;
    }
}

template <class Impl>
void
InstructionQueue<Impl>::violation(DynInstPtr &store,
                                  DynInstPtr &faulting_load)
{
    intInstQueueWrites++;
	// 递增相关计数器
    memDepUnit[store->threadNumber].violation(store, faulting_load);
}

template <class Impl>
void
InstructionQueue<Impl>::squash(ThreadID tid)
{
    DPRINTF(IQ, "[tid:%i]: Starting to squash instructions in "
            "the IQ.\n", tid);

    // Read instruction sequence number of last instruction out of the
    // time buffer.
    squashedSeqNum[tid] = fromCommit->commitInfo[tid].doneSeqNum;
	// 获取上一个到Commit阶段指令队列发出的指令全局编号

    doSquash(tid);
	// 调用doSquash进行实际的squash操作

    // Also tell the memory dependence unit to squash.
    memDepUnit[tid].squash(squashedSeqNum[tid], tid);
	// 通知存储相关检测单元也进行squash操作
}

template <class Impl>
void
InstructionQueue<Impl>::doSquash(ThreadID tid)
{
    // Start at the tail.
    ListIt squash_it = instList[tid].end();
    --squash_it;

    DPRINTF(IQ, "[tid:%i]: Squashing until sequence number %i!\n",
            tid, squashedSeqNum[tid]);

    // Squash any instructions younger than the squashed sequence number
    // given.
    while (squash_it != instList[tid].end() &&
           (*squash_it)->seqNum > squashedSeqNum[tid]) {
		// 这里的遍历是从instList的尾部开始的，需要squash的指令根据
		// squash的全局序列编号squashedSeqNum[tid]判断，凡是比该序列编号更大
		// 即更新的指令都会被squash掉
		DynInstPtr squashed_inst = (*squash_it);
		// 需要squash处理的指令
		
        if (squashed_inst->isFloating()) {
            fpInstQueueWrites++;
        } else if (squashed_inst->isVector()) {
            vecInstQueueWrites++;
        } else {
            intInstQueueWrites++;
        }
		// 递增相关计数器

        // Only handle the instruction if it actually is in the IQ and
        // hasn't already been squashed in the IQ.
        if (squashed_inst->threadNumber != tid ||
            squashed_inst->isSquashedInIQ()) {
			// 跳过其他线程指令以及已经被IQ squash过的指令
            --squash_it;
            continue;
        }

        if (!squashed_inst->isIssued() ||
            (squashed_inst->isMemRef() &&
             !squashed_inst->memOpDone())) {
			// 这里要求该指令没有发射或者是一个内存指令但是相关操作没有完成

            DPRINTF(IQ, "[tid:%i]: Instruction [sn:%lli] PC %s squashed.\n",
                    tid, squashed_inst->seqNum, squashed_inst->pcState());

            bool is_acq_rel = squashed_inst->isMemBarrier() &&
                         (squashed_inst->isLoad() ||
                           (squashed_inst->isStore() &&
                             !squashed_inst->isStoreConditional()));
			// 这是一个访存load/store的barrier指令，但不能是storeCondition指令
			
            // Remove the instruction from the dependency list.
            if (is_acq_rel ||
                (!squashed_inst->isNonSpeculative() &&
                 !squashed_inst->isStoreConditional() &&
                 !squashed_inst->isMemBarrier() &&
                 !squashed_inst->isWriteBarrier())) {
				// 这里要求指令是一个load/store barrier或者非NonSpeculative
				// StoreConditional、Barrier类型的指令，因为只有这一类指令才拥有
				// 源寄存器，故进行下面的处理
                for (int src_reg_idx = 0;
                     src_reg_idx < squashed_inst->numSrcRegs();
                     src_reg_idx++)
                {
					// 获取该指令的所有源寄存器
                    PhysRegIdPtr src_reg =
                        squashed_inst->renamedSrcRegIdx(src_reg_idx);
					// 找到这些源寄存器重命名后映射到的物理寄存器索引
						
                    // Only remove it from the dependency graph if it
                    // was placed there in the first place.

                    // Instead of doing a linked list traversal, we
                    // can just remove these squashed instructions
                    // either at issue time, or when the register is
                    // overwritten.  The only downside to this is it
                    // leaves more room for error.

                    if (!squashed_inst->isReadySrcRegIdx(src_reg_idx) &&
                        !src_reg->isFixedMapping()) {
						// 该指令的源寄存器数据没有准备好？？？
						// 并且该源寄存器不是固定映射的寄存器
                        dependGraph.remove(src_reg->flatIndex(),
                                           squashed_inst);
						// 从dependGraph中删除该指令的消费者记录
                    }


                    ++iqSquashedOperandsExamined;
                }
            } else if (!squashed_inst->isStoreConditional() ||
                       !squashed_inst->isCompleted()) {
				// 对于非SC指令或者未完成的指令
                NonSpecMapIt ns_inst_it =
                    nonSpecInsts.find(squashed_inst->seqNum);
				// 确认需要squash的指令是否是一个非推测执行的指令

                // we remove non-speculative instructions from
                // nonSpecInsts already when they are ready, and so we
                // cannot always expect to find them
                if (ns_inst_it == nonSpecInsts.end()) {
                    // loads that became ready but stalled on a
                    // blocked cache are alreayd removed from
                    // nonSpecInsts, and have not faulted
                    assert(squashed_inst->getFault() != NoFault ||
                           squashed_inst->isMemRef());
					// 如果不是一个非推测执行的指令，那么要么该指令是一个
					// 访存指令，要么没有出现执行错误
                } else {

                    (*ns_inst_it).second = NULL;
                    nonSpecInsts.erase(ns_inst_it);
                    ++iqSquashedNonSpecRemoved;
					// 推测执行的指令则会被清除掉
                }
            }

            // Might want to also clear out the head of the dependency graph.

            // Mark it as squashed within the IQ.
            squashed_inst->setSquashedInIQ();
			// 将对应指令标记为squashed状态

            // @todo: Remove this hack where several statuses are set so the
            // inst will flow through the rest of the pipeline.
            squashed_inst->setIssued();
            squashed_inst->setCanCommit();
            squashed_inst->clearInIQ();
			// ？？？

            //Update Thread IQ Count
            count[squashed_inst->threadNumber]--;
            ++freeEntries;
			// 更新相关计数器
        }

        instList[tid].erase(squash_it--);
        ++iqSquashedInstsExamined;
		// 从IQ中将给定指令删除
    }
}

template <class Impl>
bool
InstructionQueue<Impl>::addToDependents(DynInstPtr &new_inst)
{
    // Loop through the instruction's source registers, adding
    // them to the dependency list if they are not ready.
    int8_t total_src_regs = new_inst->numSrcRegs();
    bool return_val = false;

    for (int src_reg_idx = 0;
         src_reg_idx < total_src_regs;
         src_reg_idx++)
    {
		// 对该指令的所有源寄存器都进行相应的处理
        // Only add it to the dependency graph if it's not ready.
        if (!new_inst->isReadySrcRegIdx(src_reg_idx)) {
			// 被记录到dependGraph的源寄存器相关指令，一定是数据没有准备好
			// 同时源寄存器数据生产者已经进入IQ的指令
			
            PhysRegIdPtr src_reg = new_inst->renamedSrcRegIdx(src_reg_idx);
			// 获得该源寄存器经过重命名后映射到的物理寄存器

            // Check the IQ's scoreboard to make sure the register
            // hasn't become ready while the instruction was in flight
            // between stages.  Only if it really isn't ready should
            // it be added to the dependency graph.
			
            if (src_reg->isFixedMapping()) {
				// 固定映射寄存器的相关不由dependGraph处理
                continue;
            } else if (!regScoreboard[src_reg->flatIndex()]) {
                DPRINTF(IQ, "Instruction PC %s has src reg %i (%s) that "
                        "is being added to the dependency chain.\n",
                        new_inst->pcState(), src_reg->index(),
                        src_reg->className());

                dependGraph.insert(src_reg->flatIndex(), new_inst);
				// 该寄存器数据正在生成，因此可以正常的插入指令
                // Change the return value to indicate that something
                // was added to the dependency graph.
                return_val = true;
            } else {
                DPRINTF(IQ, "Instruction PC %s has src reg %i (%s) that "
                        "became ready before it reached the IQ.\n",
                        new_inst->pcState(), src_reg->index(),
                        src_reg->className());
                // Mark a register ready within the instruction.
                new_inst->markSrcRegReady(src_reg_idx);
				// 这里表示该指令源寄存器数据的生产者指令已经生成了结果
				// 因此需要对指令源寄存器数据ready有效位进行标记
            }
        }
    }

    return return_val;
}

template <class Impl>
void
InstructionQueue<Impl>::addToProducers(DynInstPtr &new_inst)
{
    // Nothing really needs to be marked when an instruction becomes
    // the producer of a register's value, but for convenience a ptr
    // to the producing instruction will be placed in the head node of
    // the dependency links.
    int8_t total_dest_regs = new_inst->numDestRegs();

    for (int dest_reg_idx = 0;
         dest_reg_idx < total_dest_regs;
         dest_reg_idx++)
    {
		// 对指令的所有目的寄存器都进行相应的处理
        PhysRegIdPtr dest_reg = new_inst->renamedDestRegIdx(dest_reg_idx);
		// 获取目的寄存器重命名后映射的物理寄存器索引

        // Some registers have fixed mapping, and there is no need to track
        // dependencies as these instructions must be executed at commit.
        if (dest_reg->isFixedMapping()) {
            continue;
        }
		// 固定映射寄存器的相关不由dependGraph处理

        if (!dependGraph.empty(dest_reg->flatIndex())) {
			// 错误处理
			// 说明上一个生产者的数据没有使用完毕，就要插入新的生产者
			// 这样其实意思是寄存器相关关系记录出现问题，因为新的生产者
			// 到来的时候应该会被重命名到其他物理寄存器以避免生产者冲突
            dependGraph.dump();
            panic("Dependency graph %i (%s) (flat: %i) not empty!",
                  dest_reg->index(), dest_reg->className(),
                  dest_reg->flatIndex());
        }

        dependGraph.setInst(dest_reg->flatIndex(), new_inst);
		// 设置dependGraph头节点的指令数据

        // Mark the scoreboard to say it's not yet ready.
        regScoreboard[dest_reg->flatIndex()] = false;
		// 由于新的生产这刚刚进入IQ，数据没有生成，因此regScoreboard设置为false
    }
}

template <class Impl>
void
InstructionQueue<Impl>::addIfReady(DynInstPtr &inst)
{
    // If the instruction now has all of its source registers
    // available, then add it to the list of ready instructions.
    if (inst->readyToIssue()) {
		// 判断指令的数据都已经准备就绪

        //Add the instruction to the proper ready list.
        if (inst->isMemRef()) {

            DPRINTF(IQ, "Checking if memory instruction can issue.\n");

            // Message to the mem dependence unit that this instruction has
            // its registers ready.
            memDepUnit[inst->threadNumber].regsReady(inst);

            return;
        }
		// 访存相关指令将会被存储相关检测单元接收

        OpClass op_class = inst->opClass();

        DPRINTF(IQ, "Instruction is ready to issue, putting it onto "
                "the ready list, PC %s opclass:%i [sn:%lli].\n",
                inst->pcState(), op_class, inst->seqNum);

        readyInsts[op_class].push(inst);
		// 其他指令则根据op class加入到readyInsts队列中

        // Will need to reorder the list if either a queue is not on the list,
        // or it has an older instruction than last time.
        if (!queueOnList[op_class]) {
            addToOrderList(op_class);
        } else if (readyInsts[op_class].top()->seqNum  <
                   (*readyIt[op_class]).oldestInst) {
            listOrder.erase(readyIt[op_class]);
            addToOrderList(op_class);
        }
		// 更新listOrder队列
    }
}

template <class Impl>
int
InstructionQueue<Impl>::countInsts()
{
#if 0
    //ksewell:This works but definitely could use a cleaner write
    //with a more intuitive way of counting. Right now it's
    //just brute force ....
    // Change the #if if you want to use this method.
    int total_insts = 0;

    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        ListIt count_it = instList[tid].begin();

        while (count_it != instList[tid].end()) {
            if (!(*count_it)->isSquashed() && !(*count_it)->isSquashedInIQ()) {
                if (!(*count_it)->isIssued()) {
                    ++total_insts;
                } else if ((*count_it)->isMemRef() &&
                           !(*count_it)->memOpDone) {
                    // Loads that have not been marked as executed still count
                    // towards the total instructions.
                    ++total_insts;
                }
            }

            ++count_it;
        }
    }
	// 和下面的统计不同，此处统计对squash标记的指令进行的删除
	// 统计的是IQ表项中被指令使用并且是有效指令的数目
	// 其实下面的方法就行，压根不用这么费劲的统计

    return total_insts;
#else
    return numEntries - freeEntries;
#endif
}

template <class Impl>
void
InstructionQueue<Impl>::dumpLists()
{
    for (int i = 0; i < Num_OpClasses; ++i) {
        cprintf("Ready list %i size: %i\n", i, readyInsts[i].size());

        cprintf("\n");
    }

    cprintf("Non speculative list size: %i\n", nonSpecInsts.size());

    NonSpecMapIt non_spec_it = nonSpecInsts.begin();
    NonSpecMapIt non_spec_end_it = nonSpecInsts.end();

    cprintf("Non speculative list: ");

    while (non_spec_it != non_spec_end_it) {
        cprintf("%s [sn:%lli]", (*non_spec_it).second->pcState(),
                (*non_spec_it).second->seqNum);
        ++non_spec_it;
    }

    cprintf("\n");

    ListOrderIt list_order_it = listOrder.begin();
    ListOrderIt list_order_end_it = listOrder.end();
    int i = 1;

    cprintf("List order: ");

    while (list_order_it != list_order_end_it) {
        cprintf("%i OpClass:%i [sn:%lli] ", i, (*list_order_it).queueType,
                (*list_order_it).oldestInst);

        ++list_order_it;
        ++i;
    }

    cprintf("\n");
}


template <class Impl>
void
InstructionQueue<Impl>::dumpInsts()
{
    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        int num = 0;
        int valid_num = 0;
        ListIt inst_list_it = instList[tid].begin();

        while (inst_list_it != instList[tid].end()) {
            cprintf("Instruction:%i\n", num);
            if (!(*inst_list_it)->isSquashed()) {
                if (!(*inst_list_it)->isIssued()) {
                    ++valid_num;
                    cprintf("Count:%i\n", valid_num);
                } else if ((*inst_list_it)->isMemRef() &&
                           !(*inst_list_it)->memOpDone()) {
                    // Loads that have not been marked as executed
                    // still count towards the total instructions.
                    ++valid_num;
                    cprintf("Count:%i\n", valid_num);
                }
            }

            cprintf("PC: %s\n[sn:%lli]\n[tid:%i]\n"
                    "Issued:%i\nSquashed:%i\n",
                    (*inst_list_it)->pcState(),
                    (*inst_list_it)->seqNum,
                    (*inst_list_it)->threadNumber,
                    (*inst_list_it)->isIssued(),
                    (*inst_list_it)->isSquashed());

            if ((*inst_list_it)->isMemRef()) {
                cprintf("MemOpDone:%i\n", (*inst_list_it)->memOpDone());
            }

            cprintf("\n");

            inst_list_it++;
            ++num;
        }
    }

    cprintf("Insts to Execute list:\n");

    int num = 0;
    int valid_num = 0;
    ListIt inst_list_it = instsToExecute.begin();

    while (inst_list_it != instsToExecute.end())
    {
        cprintf("Instruction:%i\n",
                num);
        if (!(*inst_list_it)->isSquashed()) {
            if (!(*inst_list_it)->isIssued()) {
                ++valid_num;
                cprintf("Count:%i\n", valid_num);
            } else if ((*inst_list_it)->isMemRef() &&
                       !(*inst_list_it)->memOpDone()) {
                // Loads that have not been marked as executed
                // still count towards the total instructions.
                ++valid_num;
                cprintf("Count:%i\n", valid_num);
            }
        }

        cprintf("PC: %s\n[sn:%lli]\n[tid:%i]\n"
                "Issued:%i\nSquashed:%i\n",
                (*inst_list_it)->pcState(),
                (*inst_list_it)->seqNum,
                (*inst_list_it)->threadNumber,
                (*inst_list_it)->isIssued(),
                (*inst_list_it)->isSquashed());

        if ((*inst_list_it)->isMemRef()) {
            cprintf("MemOpDone:%i\n", (*inst_list_it)->memOpDone());
        }

        cprintf("\n");

        inst_list_it++;
        ++num;
    }
}

#endif//__CPU_O3_INST_QUEUE_IMPL_HH__
