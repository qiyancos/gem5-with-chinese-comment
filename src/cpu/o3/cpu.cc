/*
 * Copyright (c) 2011-2012, 2014, 2016, 2017 ARM Limited
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
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
 * Copyright (c) 2011 Regents of the University of California
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
 *          Rick Strong
 */

#include "cpu/o3/cpu.hh"

#include "arch/generic/traits.hh"
#include "arch/kernel_stats.hh"
#include "config/the_isa.hh"
#include "cpu/activity.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/checker/thread_context.hh"
#include "cpu/o3/isa_specific.hh"
#include "cpu/o3/thread_context.hh"
#include "cpu/quiesce_event.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "debug/Activity.hh"
#include "debug/Drain.hh"
#include "debug/O3CPU.hh"
#include "debug/Quiesce.hh"
#include "enums/MemoryMode.hh"
#include "sim/core.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"
#include "sim/stat_control.hh"
#include "sim/system.hh"

#if THE_ISA == ALPHA_ISA
#include "arch/alpha/osfpal.hh"
#include "debug/Activity.hh"

#endif

struct BaseCPUParams;

using namespace TheISA;
using namespace std;

BaseO3CPU::BaseO3CPU(BaseCPUParams *params)
    : BaseCPU(params)
{
}

void
BaseO3CPU::regStats()
{
    BaseCPU::regStats();
}

template<class Impl>
bool
FullO3CPU<Impl>::IcachePort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(O3CPU, "Fetch unit received timing\n");
    // We shouldn't ever get a cacheable block in Modified state
    assert(pkt->req->isUncacheable() ||
           !(pkt->cacheResponding() && !pkt->hasSharers()));
	// ？？？
    fetch->processCacheCompletion(pkt);

    return true;
}

template<class Impl>
void
FullO3CPU<Impl>::IcachePort::recvReqRetry()
{
    fetch->recvReqRetry();
}

template <class Impl>
bool
FullO3CPU<Impl>::DcachePort::recvTimingResp(PacketPtr pkt)
{
    return lsq->recvTimingResp(pkt);
}

template <class Impl>
void
FullO3CPU<Impl>::DcachePort::recvTimingSnoopReq(PacketPtr pkt)
{
    for (ThreadID tid = 0; tid < cpu->numThreads; tid++) {
        if (cpu->getCpuAddrMonitor(tid)->doMonitor(pkt)) {
            cpu->wakeup(tid);
        }
    }
    lsq->recvTimingSnoopReq(pkt);
}

template <class Impl>
void
FullO3CPU<Impl>::DcachePort::recvReqRetry()
{
    lsq->recvReqRetry();
}

template <class Impl>
FullO3CPU<Impl>::FullO3CPU(DerivO3CPUParams *params)
    : BaseO3CPU(params),
      itb(params->itb),
      dtb(params->dtb),
      tickEvent([this]{ tick(); }, "FullO3CPU tick",
                false, Event::CPU_Tick_Pri),
#ifndef NDEBUG
      instcount(0),
#endif
      removeInstsThisCycle(false),
      fetch(this, params),
      decode(this, params),
      rename(this, params),
      iew(this, params),
      commit(this, params),

      /* It is mandatory that all SMT threads use the same renaming mode as
       * they are sharing registers and rename */
      vecMode(initRenameMode<TheISA::ISA>::mode(params->isa[0])),
      regFile(params->numPhysIntRegs,
              params->numPhysFloatRegs,
              params->numPhysVecRegs,
              params->numPhysCCRegs,
              vecMode),

      freeList(name() + ".freelist", &regFile),

      rob(this, params),

      scoreboard(name() + ".scoreboard",
                 regFile.totalNumPhysRegs()),

      isa(numThreads, NULL),

      icachePort(&fetch, this),
      dcachePort(&iew.ldstQueue, this),

      timeBuffer(params->backComSize, params->forwardComSize),
      fetchQueue(params->backComSize, params->forwardComSize),
      decodeQueue(params->backComSize, params->forwardComSize),
      renameQueue(params->backComSize, params->forwardComSize),
      iewQueue(params->backComSize, params->forwardComSize),
      activityRec(name(), NumStages,
                  params->backComSize + params->forwardComSize,
                  params->activity),

      globalSeqNum(1),
      system(params->system),
      lastRunningCycle(curCycle())
{
    if (!params->switched_out) {
        _status = Running;
    } else {
        _status = SwitchedOut;
    }
	// 根据参数确定CPU的初始化状态，只有在多核情况下可能初始化为SwitchedOut状态

    if (params->checker) {
        BaseCPU *temp_checker = params->checker;
        checker = dynamic_cast<Checker<Impl> *>(temp_checker);
        checker->setIcachePort(&icachePort);
        checker->setSystem(params->system);
    } else {
        checker = NULL;
    }
	// 如果设置了checker那么checker就会被设置为params给定的checker
	// 同时会设置相应的信息给checker，以便checker能够正常进行工作
	// 如果没有设置那么该变量被赋予空指针

    if (!FullSystem) {
        thread.resize(numThreads);
        tids.resize(numThreads);
    }
	// 如果当前不处于FS模式，那么thread相关的问题将会由gem5管理
	// 因此此处需要将线程相关的两个向量大小resize到运行二进制文件实际的线程数量

    // The stages also need their CPU pointer setup.  However this
    // must be done at the upper level CPU because they have pointers
    // to the upper level CPU, and not this FullO3CPU.

    // Set up Pointers to the activeThreads list for each stage
    fetch.setActiveThreads(&activeThreads);
    decode.setActiveThreads(&activeThreads);
    rename.setActiveThreads(&activeThreads);
    iew.setActiveThreads(&activeThreads);
    commit.setActiveThreads(&activeThreads);
	// 将当前的活跃进程列表告知每个阶段对应的类

    // Give each of the stages the time buffer they will use.
    fetch.setTimeBuffer(&timeBuffer);
    decode.setTimeBuffer(&timeBuffer);
    rename.setTimeBuffer(&timeBuffer);
    iew.setTimeBuffer(&timeBuffer);
    commit.setTimeBuffer(&timeBuffer);
	// 为每个阶段设置反向传递的数据

    // Also setup each of the stages' queues.
    fetch.setFetchQueue(&fetchQueue);
    decode.setFetchQueue(&fetchQueue);
	decode.setDecodeQueue(&decodeQueue);
    rename.setDecodeQueue(&decodeQueue);
    rename.setRenameQueue(&renameQueue);
    iew.setRenameQueue(&renameQueue);
    iew.setIEWQueue(&iewQueue);
    commit.setFetchQueue(&fetchQueue);
	commit.setIEWQueue(&iewQueue);
    commit.setRenameQueue(&renameQueue);
	// 设置每个阶段需要使用到的队列

    commit.setIEWStage(&iew);
    rename.setIEWStage(&iew);
    rename.setCommitStage(&commit);
	// 设置必要的阶段需要调用的其他阶段

    ThreadID active_threads;
    if (FullSystem) {
        active_threads = 1;
    } else {
        active_threads = params->workload.size();

        if (active_threads > Impl::MaxThreads) {
            panic("Workload Size too large. Increase the 'MaxThreads' "
                  "constant in your O3CPU impl. file (e.g. o3/alpha/impl.hh) "
                  "or edit your workload size.");
			// 很明显gem5并不支持单进程中的线程调度
        }
    }
	// FS模式下只可能有一个线程是活跃的
	// 而在非FS模式下对多可以有当前CPU支持SMT个数个活跃的线程
	// 而workload.size()决定着当前执行二进制进程的线程数目

    //Make Sure That this a Valid Architeture
    assert(params->numPhysIntRegs   >= numThreads * TheISA::NumIntRegs);
    //printf(">> Arm need %d arch float regs!\n", TheISA::NumFloatRegs);
	assert(params->numPhysFloatRegs >= numThreads * TheISA::NumFloatRegs);
    assert(params->numPhysVecRegs >= numThreads * TheISA::NumVecRegs);
    assert(params->numPhysCCRegs >= numThreads * TheISA::NumCCRegs);
	// 确保参数中寄存器数目的正确性，这里要求每拥有一个thread拥有一套ISA定义的寄存器

    rename.setScoreboard(&scoreboard);
    iew.setScoreboard(&scoreboard);
	// IEW和重命名阶段都会使用到记分板

    // Setup the rename map for whichever stages need it.
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        isa[tid] = params->isa[tid];
        assert(initRenameMode<TheISA::ISA>::equals(isa[tid], isa[0]));

        // Only Alpha has an FP zero register, so for other ISAs we
        // use an invalid FP register index to avoid special treatment
        // of any valid FP reg.
        RegIndex invalidFPReg = TheISA::NumFloatRegs + 1;
		// 这里专门增加了一个invalid的浮点寄存器作为alpha中的浮点0寄存器
        RegIndex fpZeroReg =
            (THE_ISA == ALPHA_ISA) ? TheISA::ZeroReg : invalidFPReg;
		// 对于非Alpha的ISA中0寄存器只有一个，因此浮点0寄存器被重定位到整数0寄存器
		// 而Alpha则会被定位到给定的一个invalid浮点寄存器上面

        commitRenameMap[tid].init(&regFile, TheISA::ZeroReg, fpZeroReg,
                                  &freeList,
                                  vecMode);

        renameMap[tid].init(&regFile, TheISA::ZeroReg, fpZeroReg,
                            &freeList, vecMode);
		// 初始化每个线程的重命名映射表（重命名阶段和提交阶段各一个）
    }
	// 从params中载入每个线程对应的TheISA::ISA类

    // Initialize rename map to assign physical registers to the
    // architectural registers for active threads only.
	PhysRegIdPtr phys_reg_zero = NULL;
    for (ThreadID tid = 0; tid < active_threads; tid++) {
        for (RegIndex ridx = 0; ridx < TheISA::NumIntRegs; ++ridx) {
            // Note that we can't use the rename() method because we don't
            // want special treatment for the zero register at this point
			PhysRegIdPtr phys_reg;
			if (ridx == TheISA::ZeroReg) {
				if (tid == 0){
            		phys_reg = freeList.getIntReg();
					phys_reg_zero = phys_reg;
				// 在第一次处理零寄存器时记录对应的物理寄存器指针
				}
				else phys_reg = phys_reg_zero;
				// 之后所有的零寄存器均会映射到同一个0寄存器
			}
			else {
				phys_reg = freeList.getIntReg();
				// 从freelist对应的物理寄存器中获得一个整数物理寄存器
			}
            renameMap[tid].setEntry(RegId(IntRegClass, ridx), phys_reg);
			// 在重命名表中的表项中，建立一个从体系结构整数寄存器到
			// 获得的整数物理寄存器之间的映射
            commitRenameMap[tid].setEntry(RegId(IntRegClass, ridx), phys_reg);
			// 同时在提交阶段的映射表中也进行同样的记录
        }
		// 为每一个体系结构的整数寄存器初始化一个到物理寄存器的映射

        for (RegIndex ridx = 0; ridx < TheISA::NumFloatRegs; ++ridx) {
            PhysRegIdPtr phys_reg = freeList.getFloatReg();
            renameMap[tid].setEntry(RegId(FloatRegClass, ridx), phys_reg);
            commitRenameMap[tid].setEntry(
                    RegId(FloatRegClass, ridx), phys_reg);
        }
		// 对体系结构浮点寄存器重命名映射进行初始化设置

        /* Here we need two 'interfaces' the 'whole register' and the
         * 'register element'. At any point only one of them will be
         * active. */
        if (vecMode == Enums::Full) {
            /* Initialize the full-vector interface */
            for (RegIndex ridx = 0; ridx < TheISA::NumVecRegs; ++ridx) {
                RegId rid = RegId(VecRegClass, ridx);
                PhysRegIdPtr phys_reg = freeList.getVecReg();
                renameMap[tid].setEntry(rid, phys_reg);
                commitRenameMap[tid].setEntry(rid, phys_reg);
            }
        } 
		// 上面对整个向量寄存器进行绑定在一起的映射初始化
		else {
            /* Initialize the vector-element interface */
            for (RegIndex ridx = 0; ridx < TheISA::NumVecRegs; ++ridx) {
                for (ElemIndex ldx = 0; ldx < TheISA::NumVecElemPerVecReg;
                        ++ldx) {
                    RegId lrid = RegId(VecElemClass, ridx, ldx);
                    PhysRegIdPtr phys_elem = freeList.getVecElem();
                    renameMap[tid].setEntry(lrid, phys_elem);
                    commitRenameMap[tid].setEntry(lrid, phys_elem);
                }
            }
        }
		// 这里进行的针对向量寄存器每个元素的单独映射初始化

        for (RegIndex ridx = 0; ridx < TheISA::NumCCRegs; ++ridx) {
            PhysRegIdPtr phys_reg = freeList.getCCReg();
            renameMap[tid].setEntry(RegId(CCRegClass, ridx), phys_reg);
            commitRenameMap[tid].setEntry(RegId(CCRegClass, ridx), phys_reg);
        }
		// 对CCReg进行重命名的映射初始化，对于RISCV无意义
    }

    rename.setRenameMap(renameMap);
    commit.setRenameMap(commitRenameMap);
    rename.setFreeList(&freeList);
	// 根据上面对freeList、renameMap、commitRenameMap的初始化结果
	// 对rename和commit阶段进行设置

    // Setup the ROB for whichever stages need it.
    commit.setROB(&rob);
	// 设置提交阶段的ROB表项对象

    lastActivatedCycle = 0;
#if 0
    // Give renameMap & rename stage access to the freeList;
    for (ThreadID tid = 0; tid < numThreads; tid++)
        globalSeqNum[tid] = 1;
	// 多线程的情况下会执行到这里
#endif

    DPRINTF(O3CPU, "Creating O3CPU object.\n");

    // Setup any thread state.
    this->thread.resize(this->numThreads);
	// 为什么这里又resize了一遍？

    for (ThreadID tid = 0; tid < this->numThreads; ++tid) {
        if (FullSystem) {
            // SMT is not supported in FS mode yet.
            assert(this->numThreads == 1);
			// 在FS模式下，只能gem5控制的只有一个线程，而多线程是由系统处理的
            this->thread[tid] = new Thread(this, 0, NULL);
			// 在FS模式下只会生成一个0号线程，且没有workload输入
        } else {
            if (tid < params->workload.size()) {
				// 对于当前工作集中的线程会正常进行初始化
				// 并记录到thread数组中
                DPRINTF(O3CPU, "Workload[%i] process is %#x",
                        tid, this->thread[tid]);
						
                this->thread[tid] = new typename FullO3CPU<Impl>::Thread(
                        (typename Impl::O3CPU *)(this),
                        tid, params->workload[tid]);

                //usedTids[tid] = true;
                //threadMap[tid] = tid;
            } else {
				// 这里处理的是SMT中空闲的线程位置
				// 这些位置中会被放入空线程对象
                //Allocate Empty thread so M5 can use later
                //when scheduling threads to CPU
                Process* dummy_proc = NULL;

                this->thread[tid] = new typename FullO3CPU<Impl>::Thread(
                        (typename Impl::O3CPU *)(this),
                        tid, dummy_proc);
                //usedTids[tid] = false;
            }
        }

        ThreadContext *tc;

        // Setup the TC that will serve as the interface to the threads/CPU.
        O3ThreadContext<Impl> *o3_tc = new O3ThreadContext<Impl>;
		// 设置了一个线程上下文对象，每个CPU中的线程都需要一个上下文

        tc = o3_tc;

        // If we're using a checker, then the TC should be the
        // CheckerThreadContext.
        if (params->checker) {
            tc = new CheckerThreadContext<O3ThreadContext<Impl> >(
                o3_tc, this->checker);
        }
		// 如果设置了checker，则对线程上下文生成一个检查器
		// 每个线程都会设置一个属于自己的上下文检查器

        o3_tc->cpu = (typename Impl::O3CPU *)(this);
        assert(o3_tc->cpu);
        o3_tc->thread = this->thread[tid];
		// 将当前CPU类和对应的线程向量设置到线程上下文中

        // Setup quiesce event.
        this->thread[tid]->quiesceEvent = new EndQuiesceEvent(tc);

        // Give the thread the TC.
        this->thread[tid]->tc = tc;

        // Add the TC to the CPU's list of TC's.
        this->threadContexts.push_back(tc);
		// threadContexts表示当前CPU中所有线程上下文
		// 该对象定义在BaseCPU中
    }

    // FullO3CPU always requires an interrupt controller.
    if (!params->switched_out && interrupts.empty()) {
        fatal("FullO3CPU %s has no interrupt controller.\n"
              "Ensure createInterruptController() is called.\n", name());
    }

    for (ThreadID tid = 0; tid < this->numThreads; tid++)
        this->thread[tid]->setFuncExeInst(0);
	// ？？？
}

template <class Impl>
FullO3CPU<Impl>::~FullO3CPU()
{
}

template <class Impl>
void
FullO3CPU<Impl>::regProbePoints()
{
    BaseCPU::regProbePoints();

    ppInstAccessComplete = new ProbePointArg<PacketPtr>(getProbeManager(), "InstAccessComplete");
    ppDataAccessComplete = new ProbePointArg<std::pair<DynInstPtr, PacketPtr> >(getProbeManager(), "DataAccessComplete");

    fetch.regProbePoints();
    rename.regProbePoints();
    iew.regProbePoints();
    commit.regProbePoints();
}
// ？？？

template <class Impl>
void
FullO3CPU<Impl>::regStats()
{
    BaseO3CPU::regStats();

    // Register any of the O3CPU's stats here.
    timesIdled
        .name(name() + ".timesIdled")
        .desc("Number of times that the entire CPU went into an idle state and"
              " unscheduled itself")
        .prereq(timesIdled);
		// 这里和下面的格式一样，都对一个Stats::Scalar变量的函数调用
		// 该类型的主要实体变量定义于gem5/src/base/stats/info.hh
		// 的info类中以及gem5/src/base/statistics.hh的StatStor类中。
		// 其中info类被管理在一个以其为元素的向量the_map中
		// 而.name设置名称name，.desc设置描述desc，.prereq设置info指针
		// 并在结束的时候打印出来，此处只是初始化描述，不涉及计数器数值的初始化

    idleCycles
        .name(name() + ".idleCycles")
        .desc("Total number of cycles that the CPU has spent unscheduled due "
              "to idling")
        .prereq(idleCycles);

    quiesceCycles
        .name(name() + ".quiesceCycles")
        .desc("Total number of cycles that CPU has spent quiesced or waiting "
              "for an interrupt")
        .prereq(quiesceCycles);

    // Number of Instructions simulated
    // --------------------------------
    // Should probably be in Base CPU but need templated
    // MaxThreads so put in here instead
    committedInsts
        .init(numThreads)
        .name(name() + ".committedInsts")
        .desc("Number of Instructions Simulated")
        .flags(Stats::total);
		// 该变量是一个Stats::Vector类型，该类型的实体变量和Stats::Scalar一致
		// .init函数用来初始化大小size，而size大小决定了StatStor中
		// Counter变量的个数，即每个线程都会分配一个Counter变量（一个StatStor类）
		// .flags函数将info类中的Flag类型变量设置为Stats::total
		// Stats::total是定义域info.hh中的一个FlagsType类型常量

    committedOps
        .init(numThreads)
        .name(name() + ".committedOps")
        .desc("Number of Ops (including micro ops) Simulated")
        .flags(Stats::total);

    cpi
        .name(name() + ".cpi")
        .desc("CPI: Cycles Per Instruction")
        .precision(6);
    cpi = numCycles / committedInsts;
	// 该变量是Stats::Formula类型，其实体变量依赖于info类型、Formula类型
	// 以及FormulaInfoProxy类型，Formula类型包含了NodePtr，而FormulaInfoProxy类型
	// 包含了VCounter和VResult变量。.precision函数将info中的precision
	// 进行设置，而cpi调用了重载运算符"="在源代码中实际上没有任何操作？？？
	// 只能根据意义猜测该操作对其中的计数器实体进行的赋值，但是不可能每次
	// 数据更新都进行赋值，因此次数可能说明的是计算方法/公式
	// 如果尝试获取其计数器数值，则会调用该公式进行计算

    totalCpi
        .name(name() + ".cpi_total")
        .desc("CPI: Total CPI of All Threads")
        .precision(6);
    totalCpi = numCycles / sum(committedInsts);
	// 此处sum表示将committedInsts中每个线程对应的计数器数据加在一起

    ipc
        .name(name() + ".ipc")
        .desc("IPC: Instructions Per Cycle")
        .precision(6);
    ipc =  committedInsts / numCycles;

    totalIpc
        .name(name() + ".ipc_total")
        .desc("IPC: Total IPC of All Threads")
        .precision(6);
    totalIpc =  sum(committedInsts) / numCycles;

    this->fetch.regStats();
    this->decode.regStats();
    this->rename.regStats();
    this->iew.regStats();
    this->commit.regStats();
    this->rob.regStats();

    intRegfileReads
        .name(name() + ".int_regfile_reads")
        .desc("number of integer regfile reads")
        .prereq(intRegfileReads);
	// 该变量和后面的都属于Stats::Scalar类型，此处不做赘述

    intRegfileWrites
        .name(name() + ".int_regfile_writes")
        .desc("number of integer regfile writes")
        .prereq(intRegfileWrites);

    fpRegfileReads
        .name(name() + ".fp_regfile_reads")
        .desc("number of floating regfile reads")
        .prereq(fpRegfileReads);

    fpRegfileWrites
        .name(name() + ".fp_regfile_writes")
        .desc("number of floating regfile writes")
        .prereq(fpRegfileWrites);

    vecRegfileReads
        .name(name() + ".vec_regfile_reads")
        .desc("number of vector regfile reads")
        .prereq(vecRegfileReads);

    vecRegfileWrites
        .name(name() + ".vec_regfile_writes")
        .desc("number of vector regfile writes")
        .prereq(vecRegfileWrites);

    ccRegfileReads
        .name(name() + ".cc_regfile_reads")
        .desc("number of cc regfile reads")
        .prereq(ccRegfileReads);

    ccRegfileWrites
        .name(name() + ".cc_regfile_writes")
        .desc("number of cc regfile writes")
        .prereq(ccRegfileWrites);

    miscRegfileReads
        .name(name() + ".misc_regfile_reads")
        .desc("number of misc regfile reads")
        .prereq(miscRegfileReads);

    miscRegfileWrites
        .name(name() + ".misc_regfile_writes")
        .desc("number of misc regfile writes")
        .prereq(miscRegfileWrites);
}

template <class Impl>
void
FullO3CPU<Impl>::tick()
{
    DPRINTF(O3CPU, "\n\nFullO3CPU: Ticking main, FullO3CPU.\n");
    assert(!switchedOut());
	// 如果CPU处于被切换掉的状态，是不可能调用tick函数步进的
	// 此时该CPU处于停止状态，但真实的CPU又是怎么做的呢？
    assert(drainState() != DrainState::Drained);
	// 如果当前CPU正在排空流水线，是不能调用tick步进的
	// 这样处理是为什么呢，不步进那么将如何排空流水线？？？

    ++numCycles;
	// 递增Cycle总数
    updateCycleCounters(BaseCPU::CPU_STATE_ON);
	// 该函数定义在gem5/src/cpu/base.hh中
	// 通过调用notify对和周期相关的变量进行更新？？？

	// activity = false;
	
	/*
	int tempTid;
	//printf(">> Tick to %lu\n", curTick());
	if (curTick() > 257000 ){
		printf(">> Check at tick: %lu\n", curTick());
		for(tempTid = 0; tempTid < numThreads; tempTid++){
			printf(">> Thread::%d\n", tempTid);
			renameMap[tempTid].dumpInsts();
		}
	}
	*/

    //Tick each of the stages
    fetch.tick();

    decode.tick();

    rename.tick();

    iew.tick();

    commit.tick();
	
	//按照顺序步进每一个周期，其实是同时步进的
	
    // Now advance the time buffers
    timeBuffer.advance();
	// 步进不同阶段后更新反向传播的数据
	
    fetchQueue.advance();
    decodeQueue.advance();
    renameQueue.advance();
    iewQueue.advance();
	// 不同阶段指令队列按顺序推进

    activityRec.advance();
	// 活动记录器步进

    if (removeInstsThisCycle) {
        cleanUpRemovedInsts();
    }
	// 如果标志位要求本周起清理指令，则将removeList中的指令清理掉

    if (!tickEvent.scheduled()) {
        if (_status == SwitchedOut) {
            DPRINTF(O3CPU, "Switched out!\n");
            // increment stat
            lastRunningCycle = curCycle();
		// 如果当前处理器状态变成了SwitchOut
		// 则设置lastRunningCycle为当前时钟位值
        } else if (!activityRec.active() || _status == Idle) {
            DPRINTF(O3CPU, "Idle!\n");
            lastRunningCycle = curCycle();
            timesIdled++;
		// 如果当前活动记录器不活跃或者CPU状态处于闲置状态
		// 也会设置lastRunningCycle，同时将闲置周期计数器递增
        } else {
            schedule(tickEvent, clockEdge(Cycles(1)));
            DPRINTF(O3CPU, "Scheduling next tick!\n");
			// 如果都不满足，则进行调度，进入下一个周期的处理
        }
    }
	// 如果tickEvent还没有被调度处理，根据当前状况选择是否进行下一部分调度
	// 这里调用schedule只会标记下一次调度的状态，不会立即进行调度

    if (!FullSystem)
        updateThreadPriority();
	// 如果当前系统不处于FS模式，需要gem5手动更新不同线程的执行优先级

    tryDrain();
	// 尝试排空流水线，这里会根据是否需要排空流水线进行操作
}

template <class Impl>
void
FullO3CPU<Impl>::init()
{
    BaseCPU::init();

    for (ThreadID tid = 0; tid < numThreads; ++tid) {
        // Set noSquashFromTC so that the CPU doesn't squash when initially
        // setting up registers.
        thread[tid]->noSquashFromTC = true;
		// 设置为true表示CPU初始化时不可以进行squash的处理
        // Initialise the ThreadContext's memory proxies
        thread[tid]->initMemProxies(thread[tid]->getTC());
		// 为每个线程设置对应的上下文
    }

    if (FullSystem && !params()->switched_out) {
        for (ThreadID tid = 0; tid < numThreads; tid++) {
            ThreadContext *src_tc = threadContexts[tid];
            TheISA::initCPU(src_tc, src_tc->contextId());
        }
    }
	// 对于非FS模式下运行并且初始化为Running状态的CPU
	// 中的线程进行初始化

    // Clear noSquashFromTC.
    for (int tid = 0; tid < numThreads; ++tid)
        thread[tid]->noSquashFromTC = false;
	// 线程初始化操作完毕后才允许squash操作出现

    commit.setThreads(thread);
	// 提交阶段设置线程相关的信息
}

template <class Impl>
void
FullO3CPU<Impl>::startup()
{
    BaseCPU::startup();
    for (int tid = 0; tid < numThreads; ++tid)
        isa[tid]->startup(threadContexts[tid]);
	// 设置每个线程的TheISA类型进行初始化

    fetch.startupStage();
    decode.startupStage();
    iew.startupStage();
    rename.startupStage();
    commit.startupStage();
	// 启动每个阶段的执行操作
}

template <class Impl>
void
FullO3CPU<Impl>::activateThread(ThreadID tid)
{
    list<ThreadID>::iterator isActive =
        std::find(activeThreads.begin(), activeThreads.end(), tid);
		// 在begin和end之间查找id等于tid的线程对应额度iterator
		
		
    DPRINTF(O3CPU, "[tid:%i]: Calling activate thread.\n", tid);
    assert(!switchedOut());
	// 进行线程调度的时候CPU一定处于运行状态

    if (isActive == activeThreads.end()) {
		// 如果没有在activeThreads中找到对应的线程则返回.end对应的iterator
        DPRINTF(O3CPU, "[tid:%i]: Adding to active threads list\n",
                tid);

        activeThreads.push_back(tid);
		// 将制定的线程加入到活跃进程列表中
    }
}

template <class Impl>
void
FullO3CPU<Impl>::deactivateThread(ThreadID tid)
{
    //Remove From Active List, if Active
    list<ThreadID>::iterator thread_it =
        std::find(activeThreads.begin(), activeThreads.end(), tid);

    DPRINTF(O3CPU, "[tid:%i]: Calling deactivate thread.\n", tid);
    assert(!switchedOut());

    if (thread_it != activeThreads.end()) {
        DPRINTF(O3CPU,"[tid:%i]: Removing from active threads list\n",
                tid);
        activeThreads.erase(thread_it);
    }
	// 除了要将给定线程从activeThreads列表中删除以外

    fetch.deactivateThread(tid);
    commit.deactivateThread(tid);
	// 还需要将它从取指和提交阶段中的活跃队列中删除
}

template <class Impl>
Counter
FullO3CPU<Impl>::totalInsts() const
{
    Counter total(0);

    ThreadID size = thread.size();
	// 获取当前CPU中的总线程数
    for (ThreadID i = 0; i < size; i++)
        total += thread[i]->numInst;
	
	// 计算当前的每个线程执行的指令数目的加和
    return total;
}

template <class Impl>
Counter
FullO3CPU<Impl>::totalOps() const
{
    Counter total(0);

    ThreadID size = thread.size();
    for (ThreadID i = 0; i < size; i++)
        total += thread[i]->numOp;

    return total;
}
// 处理和上面的totalInsts函数一致，只不过这里处理的是总操作数
// 对于指令不包含微操作的ISA，该数值和totalInsts的返回数值是一样的

template <class Impl>
void
FullO3CPU<Impl>::activateContext(ThreadID tid)
{
    assert(!switchedOut());

    // Needs to set each stage to running as well.
    activateThread(tid);

    // We don't want to wake the CPU if it is drained. In that case,
    // we just want to flag the thread as active and schedule the tick
    // event from drainResume() instead.
    if (drainState() == DrainState::Drained)
        return;
	// 如果发现当前CPU正在排空流水线，则不做额外的处理

    // If we are time 0 or if the last activation time is in the past,
    // schedule the next tick and wake up the fetch unit
    if (lastActivatedCycle == 0 || lastActivatedCycle < curTick()) {
        scheduleTickEvent(Cycles(0));
		// 如果发现当前CPU处于第一次初始化或者CPU在
		// 当前tick第一次调用本函数处理新的线程时才进行处理
		// lastActivatedCycle < curTick()表示当前tick第一次调用本函数

        // Be sure to signal that there's some activity so the CPU doesn't
        // deschedule itself.
        activityRec.activity();
        fetch.wakeFromQuiesce();
		// 激活活动记录器并唤醒Fetch阶段

        Cycles cycles(curCycle() - lastRunningCycle);
		// 表示和上次运行之间的间隔周期数
        // @todo: This is an oddity that is only here to match the stats
        if (cycles != 0)
            --cycles;
        quiesceCycles += cycles;
		// 更新quiesceCycles为新的间隔周期数总和数值

        lastActivatedCycle = curTick();
		// 因为CPU会因为一个线程激活，因此记录CPU新的CPU激活对应的tick
		
        _status = Running;
        BaseCPU::activateContext(tid);
		// 启动CPU进行处理，可以确定的是收起启动
		// 该函数一定会被调用
    }
}
// 上述处理一般被用于新线程生成而激活CPU的情景

template <class Impl>
void
FullO3CPU<Impl>::suspendContext(ThreadID tid)
{
    DPRINTF(O3CPU,"[tid: %i]: Suspending Thread Context.\n", tid);
    assert(!switchedOut());

    deactivateThread(tid);
	// 将对应线程

    // If this was the last thread then unschedule the tick event.
    if (activeThreads.size() == 0) {
        unscheduleTickEvent();
		// 解调度？？？
        lastRunningCycle = curCycle();
        // 设置CPU最近一次运行的周期位置
		_status = Idle;
    }
	// 如果发现当前CPU最后的线程被处理掉了，则将CPU挂起到空闲状态

    DPRINTF(Quiesce, "Suspending Context\n");

    BaseCPU::suspendContext(tid);
}

template <class Impl>
void
FullO3CPU<Impl>::haltContext(ThreadID tid)
{
    //For now, this is the same as deallocate
    DPRINTF(O3CPU,"[tid:%i]: Halt Context called. Deallocating", tid);
    assert(!switchedOut());

    deactivateThread(tid);
    removeThread(tid);

    updateCycleCounters(BaseCPU::CPU_STATE_SLEEP);
}
// 挂起一个给定的线程

template <class Impl>
void
FullO3CPU<Impl>::insertThread(ThreadID tid)
{
    DPRINTF(O3CPU,"[tid:%i] Initializing thread into CPU");
    // Will change now that the PC and thread state is internal to the CPU
    // and not in the ThreadContext.
    ThreadContext *src_tc;
    if (FullSystem)
        src_tc = system->threadContexts[tid];
    else
        src_tc = tcBase(tid);
	// 在FS模式下会从system中获取线程上下文
	// 否则会从tcBase中获得上下文内容，可见上下文内容是由tcBase处理的

    //Bind Int Regs to Rename Map

    for (RegId reg_id(IntRegClass, 0); reg_id.index() < TheISA::NumIntRegs;
         reg_id.index()++) {
        PhysRegIdPtr phys_reg = freeList.getIntReg();
        renameMap[tid].setEntry(reg_id, phys_reg);
        scoreboard.setReg(phys_reg);
		// 注意，这里在scoreboard中也记录了相应的物理寄存器
    }
	// 载入上下文中的整数寄存器内容到流水线中
	// 为此需要对其重命名映射进行初始化设置

    //Bind Float Regs to Rename Map
    for (RegId reg_id(FloatRegClass, 0); reg_id.index() < TheISA::NumFloatRegs;
         reg_id.index()++) {
        PhysRegIdPtr phys_reg = freeList.getFloatReg();
        renameMap[tid].setEntry(reg_id, phys_reg);
        scoreboard.setReg(phys_reg);
    }
	// 进行浮点寄存器的映射初始化操作

    //Bind condition-code Regs to Rename Map
    for (RegId reg_id(CCRegClass, 0); reg_id.index() < TheISA::NumCCRegs;
         reg_id.index()++) {
        PhysRegIdPtr phys_reg = freeList.getCCReg();
        renameMap[tid].setEntry(reg_id, phys_reg);
        scoreboard.setReg(phys_reg);
    }
	// CCReg在Riscv中没有使用

    //Copy Thread Data Into RegFile
    //this->copyFromTC(tid);

    //Set PC/NPC/NNPC
    pcState(src_tc->pcState(), tid);
	// 设置线程的NPC和PC寄存器

    src_tc->setStatus(ThreadContext::Active);
	// 标记对应线程的上下文为活跃状态，表示该上下文对应的线程正在运行

    activateContext(tid);
	// 将初始化好的线程上下文激活

    //Reset ROB/IQ/LSQ Entries
    commit.rob->resetEntries();
    iew.resetEntries();
	// 重新计算并设置ROB/IQ/LSQ的表项数目信息
}
// 将tid标记的线程放到插入到CPU中

template <class Impl>
void
FullO3CPU<Impl>::removeThread(ThreadID tid)
{
    DPRINTF(O3CPU,"[tid:%i] Removing thread context from CPU.\n", tid);

    // Copy Thread Data From RegFile
    // If thread is suspended, it might be re-allocated
    // this->copyToTC(tid);

    // @todo: 2-27-2008: Fix how we free up rename mappings
    // here to alleviate the case for double-freeing registers
    // in SMT workloads.

    // Unbind Int Regs from Rename Map
    for (RegId reg_id(IntRegClass, 0); reg_id.index() < TheISA::NumIntRegs;
         reg_id.index()++) {
        PhysRegIdPtr phys_reg = renameMap[tid].lookup(reg_id);
        scoreboard.unsetReg(phys_reg);
        freeList.addReg(phys_reg);
    }

    // Unbind Float Regs from Rename Map
    for (RegId reg_id(FloatRegClass, 0); reg_id.index() < TheISA::NumFloatRegs;
         reg_id.index()++) {
        PhysRegIdPtr phys_reg = renameMap[tid].lookup(reg_id);
        scoreboard.unsetReg(phys_reg);
        freeList.addReg(phys_reg);
    }

    // Unbind condition-code Regs from Rename Map
    for (RegId reg_id(CCRegClass, 0); reg_id.index() < TheISA::NumCCRegs;
         reg_id.index()++) {
        PhysRegIdPtr phys_reg = renameMap[tid].lookup(reg_id);
        scoreboard.unsetReg(phys_reg);
        freeList.addReg(phys_reg);
    }
	// 对当前线程的所有寄存器的重命名映射进行解除
	// 这里并没有对相应的寄存器数据进行存储

    // Squash Throughout Pipeline
    DynInstPtr inst = commit.rob->readHeadInst(tid);
	// 获取ROB中属于当前线程的最新的指令
    InstSeqNum squash_seq_num = inst->seqNum;
	// 得到该指令的序列号
    fetch.squash(0, squash_seq_num, inst, tid);
    decode.squash(tid);
    rename.squash(squash_seq_num, tid);
    iew.squash(tid);
	// 对所有的阶段中属于该线程的指令进行squash操作
    iew.ldstQueue.squash(squash_seq_num, tid);
    commit.rob->squash(squash_seq_num, tid);
	// 将该线程的在LSQ和ROB中的指令squash掉

    assert(iew.instQueue.getCount(tid) == 0);
    assert(iew.ldstQueue.getCount(tid) == 0);
	// 清理完毕后对应线程在IQ和LSQ中的数目应该是0

    // Reset ROB/IQ/LSQ Entries

    // Commented out for now.  This should be possible to do by
    // telling all the pipeline stages to drain first, and then
    // checking until the drain completes.  Once the pipeline is
    // drained, call resetEntries(). - 10-09-06 ktlim
/*
    if (activeThreads.size() >= 1) {
        commit.rob->resetEntries();
        iew.resetEntries();
    }
*/
}

template <class Impl>
Fault
FullO3CPU<Impl>::hwrei(ThreadID tid)
{
#if THE_ISA == ALPHA_ISA
    // Need to clear the lock flag upon returning from an interrupt.
    this->setMiscRegNoEffect(AlphaISA::MISCREG_LOCKFLAG, false, tid);

    this->thread[tid]->kernelStats->hwrei();

    // FIXME: XXX check for interrupts? XXX
#endif
    return NoFault;
}
// 作用比较模糊？？？

template <class Impl>
bool
FullO3CPU<Impl>::simPalCheck(int palFunc, ThreadID tid)
{
#if THE_ISA == ALPHA_ISA
    if (this->thread[tid]->kernelStats)
        this->thread[tid]->kernelStats->callpal(palFunc,
                                                this->threadContexts[tid]);
	
    switch (palFunc) {
      case PAL::halt:
        halt();
        if (--System::numSystemsRunning == 0)
            exitSimLoop("all cpus halted");
        break;

      case PAL::bpt:
      case PAL::bugchk:
        if (this->system->breakpoint())
            return false;
        break;
    }
#endif
    return true;
}
// 尝试模拟特权指令的调用操作，仅限于Alpha

template <class Impl>
Fault
FullO3CPU<Impl>::getInterrupts()
{
    // Check if there are any outstanding interrupts
    return this->interrupts[0]->getInterrupt(this->threadContexts[0]);
}

template <class Impl>
void
FullO3CPU<Impl>::processInterrupts(const Fault &interrupt)
{
    // Check for interrupts here.  For now can copy the code that
    // exists within isa_fullsys_traits.hh.  Also assume that thread 0
    // is the one that handles the interrupts.
    // @todo: Possibly consolidate the interrupt checking code.
    // @todo: Allow other threads to handle interrupts.

    assert(interrupt != NoFault);
	// 中断不可能属于NoFault
    this->interrupts[0]->updateIntrInfo(this->threadContexts[0]);
	//更新中断信息
	
    DPRINTF(O3CPU, "Interrupt %s being handled\n", interrupt->name());
    this->trap(interrupt, 0, nullptr);
	// 调用trap函数对中断进行处理
}
// 处理中断响应的函数

template <class Impl>
void
FullO3CPU<Impl>::trap(const Fault &fault, ThreadID tid,
                      const StaticInstPtr &inst)
{
    // Pass the thread's TC into the invoke method.
    fault->invoke(this->threadContexts[tid], inst);
}
// 中断服务程序调用函数

template <class Impl>
void
FullO3CPU<Impl>::syscall(int64_t callnum, ThreadID tid, Fault *fault)
{
    DPRINTF(O3CPU, "[tid:%i] Executing syscall().\n\n", tid);

    DPRINTF(Activity,"Activity: syscall() called.\n");

    // Temporarily increase this by one to account for the syscall
    // instruction.
    ++(this->thread[tid]->funcExeInst);
	// 递增引发系统调用指令的计数器，表示一个新指令需要处理对应的syscall

    // Execute the actual syscall.
    this->thread[tid]->syscall(callnum, fault);
	// 根据系统调用编号，执行相应的系统调用处理函数

    // Decrease funcExeInst by one as the normal commit will handle
    // incrementing it.
    --(this->thread[tid]->funcExeInst);
	// 由于对应的syscall已经得到处理，对应的计数被递减
}
// 处理系统调用的函数

template <class Impl>
void
FullO3CPU<Impl>::serializeThread(CheckpointOut &cp, ThreadID tid) const
{
    thread[tid]->serialize(cp);
}

template <class Impl>
void
FullO3CPU<Impl>::unserializeThread(CheckpointIn &cp, ThreadID tid)
{
    thread[tid]->unserialize(cp);
}

template <class Impl>
DrainState
FullO3CPU<Impl>::drain()
{
    // Deschedule any power gating event (if any)
    deschedulePowerGatingEvent();
	// ？？？

    // If the CPU isn't doing anything, then return immediately.
    if (switchedOut())
        return DrainState::Drained;
	// 如果当前的CPU压根不在运行，那么不做任何操作直接
	// 返回表示流水线已经排干的状态

    DPRINTF(Drain, "Draining...\n");

    // We only need to signal a drain to the commit stage as this
    // initiates squashing controls the draining. Once the commit
    // stage commits an instruction where it is safe to stop, it'll
    // squash the rest of the instructions in the pipeline and force
    // the fetch stage to stall. The pipeline will be drained once all
    // in-flight instructions have retired.
    commit.drain();
	// 首先将提交阶段的指令排空，这里其实只是发送了信号并不会立即处理
	// 而提交阶段在遇到一个可以安全的开始squash操作的指令后，便会由通过
	// 提交排空流水线转到通过squash操作排空流水线并强制fetch停止取指，
	// 也就是说安全指令之后的指令都将被squash掉。

    // Wake the CPU and record activity so everything can drain out if
    // the CPU was not able to immediately drain.
    if (!isDrained())  {
        // If a thread is suspended, wake it up so it can be drained
        for (auto t : threadContexts) {
			// 此处表示在threadContexts中进行枚举的处理操作
            if (t->status() == ThreadContext::Suspended){
                DPRINTF(Drain, "Currently suspended so activate %i \n",
                        t->threadId());
                t->activate();
                // As the thread is now active, change the power state as well
                activateContext(t->threadId());
            }
        }
		// 如果发现存在被挂起，则将其激活

        wakeCPU();
        activityRec.activity();
		// 唤醒CPU并激活活动记录器

        DPRINTF(Drain, "CPU not drained\n");

        return DrainState::Draining;
		// 如果检查当前没有处于流水线排空状态，则进行相应的处理
		// 返回表示正在排空流水线的状态
    } else {
        DPRINTF(Drain, "CPU is already drained\n");
        if (tickEvent.scheduled())
            deschedule(tickEvent);
		// 由于流水线已经排空，因此需要对tickEvent解调度

        // Flush out any old data from the time buffers.  In
        // particular, there might be some data in flight from the
        // fetch stage that isn't visible in any of the CPU buffers we
        // test in isDrained().
        for (int i = 0; i < timeBuffer.getSize(); ++i) {
            timeBuffer.advance();
            fetchQueue.advance();
            decodeQueue.advance();
            renameQueue.advance();
            iewQueue.advance();
        }
		// 通过调用advance操作将流水线的中间数据排除掉
		// 当反向传输的数据timeBuffer被排空的时候，所有队列也就被排空了
		// 这里的advance不会进行有效的处理操作？？？

        drainSanityCheck();
		// 进行排空流水线的bug处理情况检查
        return DrainState::Drained;
		// 这里检查发现CPU已经排空了流水线
    }
}
// 开始进行流水线排空操作

template <class Impl>
bool
FullO3CPU<Impl>::tryDrain()
{
    if (drainState() != DrainState::Draining || !isDrained())
        return false;
	// 如果当前不处于正在排空流水线的状态或者不处于已经排空的状态
	// 则返回false表示没有排空流水线的相关请求被处理或者正在被处理

	// 进入到这里只有一种情况，那便是流水线正在处理排空操作
	// 而排空操作已经执行完毕，下面则开始进入排空完成状态的处理
    if (tickEvent.scheduled())
        deschedule(tickEvent);

    DPRINTF(Drain, "CPU done draining, processing drain event\n");
    signalDrainDone();
	// 发出信号表示排空操作已经完成

    return true;
}
// 尝试完成/结束一个正在处理的流水线排空操作

template <class Impl>
void
FullO3CPU<Impl>::drainSanityCheck() const
{
    assert(isDrained());
	// 检查之前必须保证当前流水线已经排空
    fetch.drainSanityCheck();
    decode.drainSanityCheck();
    rename.drainSanityCheck();
    iew.drainSanityCheck();
    commit.drainSanityCheck();
}
// 排空流水线的错误处理状态检查，如果发现存在没有处理的错误，将会处理

template <class Impl>
bool
FullO3CPU<Impl>::isDrained() const
{
    bool drained(true);

    if (!instList.empty() || !removeList.empty()) {
        DPRINTF(Drain, "Main CPU structures not drained.\n");
        drained = false;
    }

    if (!fetch.isDrained()) {
        DPRINTF(Drain, "Fetch not drained.\n");
        drained = false;
    }

    if (!decode.isDrained()) {
        DPRINTF(Drain, "Decode not drained.\n");
        drained = false;
    }

    if (!rename.isDrained()) {
        DPRINTF(Drain, "Rename not drained.\n");
        drained = false;
    }

    if (!iew.isDrained()) {
        DPRINTF(Drain, "IEW not drained.\n");
        drained = false;
    }

    if (!commit.isDrained()) {
        DPRINTF(Drain, "Commit not drained.\n");
        drained = false;
    }

    return drained;
}
// 该函数用来检查排空流水线操作是否已经完成

template <class Impl>
void
FullO3CPU<Impl>::commitDrained(ThreadID tid)
{
    fetch.drainStall(tid);
}
// 通知取指阶段stall以排空流水线

template <class Impl>
void
FullO3CPU<Impl>::drainResume()
{
    if (switchedOut())
        return;
	// 如果CPU被切换出去了，则什么都不做

    DPRINTF(Drain, "Resuming...\n");
    verifyMemoryMode();
	// ？？？

    fetch.drainResume();
    commit.drainResume();
	// 恢复取指和提交阶段

    _status = Idle;
    for (ThreadID i = 0; i < thread.size(); i++) {
        if (thread[i]->status() == ThreadContext::Active) {
            DPRINTF(Drain, "Activating thread: %i\n", i);
            activateThread(i);
            _status = Running;
        }
    }
	// 尝试激活所有的线程，如果没有线程CPU则无法进入running状态

    assert(!tickEvent.scheduled());
	// 此时的tickEvent不可能处于正在调度的状态
    if (_status == Running)
        schedule(tickEvent, nextCycle());
	// 如果由可以执行的线程则CPU开启tickEvent调度以继续执行指令

    // Reschedule any power gating event (if any)
    schedulePowerGatingEvent();
}
// 尝试在流水线排空之后继续执行新的指令

template <class Impl>
void
FullO3CPU<Impl>::switchOut()
{
    DPRINTF(O3CPU, "Switching out\n");
    BaseCPU::switchOut();

    activityRec.reset();
	// 活动记录器被重置

    _status = SwitchedOut;

    if (checker)
        checker->switchOut();
	// 如果设置了checker，cheker也会进行切换操作
}
// 将从当前的CPU切换掉

template <class Impl>
void
FullO3CPU<Impl>::takeOverFrom(BaseCPU *oldCPU)
{
    BaseCPU::takeOverFrom(oldCPU);

    fetch.takeOverFrom();
    decode.takeOverFrom();
    rename.takeOverFrom();
    iew.takeOverFrom();
    commit.takeOverFrom();

    assert(!tickEvent.scheduled());
	// 要求当前CPU不能处于执行状态

    FullO3CPU<Impl> *oldO3CPU = dynamic_cast<FullO3CPU<Impl>*>(oldCPU);
    if (oldO3CPU)
        globalSeqNum = oldO3CPU->globalSeqNum;
	// 继承全局的序列编号

    lastRunningCycle = curCycle();
    _status = Idle;
	// 将处理器设置为空闲状态
}
// 从指定的CPU中接管数据

template <class Impl>
void
FullO3CPU<Impl>::verifyMemoryMode() const
{
    if (!system->isTimingMode()) {
        fatal("The O3 CPU requires the memory system to be in "
              "'timing' mode.\n");
    }
}
// 检查是否支持内存系统，来进入时序模式

template <class Impl>
TheISA::MiscReg
FullO3CPU<Impl>::readMiscRegNoEffect(int misc_reg, ThreadID tid) const
{
    return this->isa[tid]->readMiscRegNoEffect(misc_reg);
}

template <class Impl>
TheISA::MiscReg
FullO3CPU<Impl>::readMiscReg(int misc_reg, ThreadID tid)
{
    miscRegfileReads++;
	// 递增相关读取计数
    return this->isa[tid]->readMiscReg(misc_reg, tcBase(tid));
}

template <class Impl>
void
FullO3CPU<Impl>::setMiscRegNoEffect(int misc_reg,
        const TheISA::MiscReg &val, ThreadID tid)
{
    this->isa[tid]->setMiscRegNoEffect(misc_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setMiscReg(int misc_reg,
        const TheISA::MiscReg &val, ThreadID tid)
{
    miscRegfileWrites++;
    this->isa[tid]->setMiscReg(misc_reg, val, tcBase(tid));
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readIntReg(PhysRegIdPtr phys_reg)
{
    intRegfileReads++;
    return regFile.readIntReg(phys_reg);
}

template <class Impl>
FloatReg
FullO3CPU<Impl>::readFloatReg(PhysRegIdPtr phys_reg)
{
    fpRegfileReads++;
    return regFile.readFloatReg(phys_reg);
}

template <class Impl>
FloatRegBits
FullO3CPU<Impl>::readFloatRegBits(PhysRegIdPtr phys_reg)
{
    fpRegfileReads++;
    return regFile.readFloatRegBits(phys_reg);
}

template <class Impl>
auto
FullO3CPU<Impl>::readVecReg(PhysRegIdPtr phys_reg) const
        -> const VecRegContainer&
		// 这个语法没有见到过，似乎在说明返回的数值类型
{
    vecRegfileReads++;
    return regFile.readVecReg(phys_reg);
}

template <class Impl>
auto
FullO3CPU<Impl>::getWritableVecReg(PhysRegIdPtr phys_reg)
        -> VecRegContainer&
{
    vecRegfileWrites++;
    return regFile.getWritableVecReg(phys_reg);
}

template <class Impl>
auto
FullO3CPU<Impl>::readVecElem(PhysRegIdPtr phys_reg) const -> const VecElem&
{
    vecRegfileReads++;
    return regFile.readVecElem(phys_reg);
}

template <class Impl>
CCReg
FullO3CPU<Impl>::readCCReg(PhysRegIdPtr phys_reg)
{
    ccRegfileReads++;
    return regFile.readCCReg(phys_reg);
}

template <class Impl>
void
FullO3CPU<Impl>::setIntReg(PhysRegIdPtr phys_reg, uint64_t val)
{
    intRegfileWrites++;
    regFile.setIntReg(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setFloatReg(PhysRegIdPtr phys_reg, FloatReg val)
{
    fpRegfileWrites++;
    regFile.setFloatReg(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setFloatRegBits(PhysRegIdPtr phys_reg, FloatRegBits val)
{
    fpRegfileWrites++;
    regFile.setFloatRegBits(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setVecReg(PhysRegIdPtr phys_reg, const VecRegContainer& val)
{
    vecRegfileWrites++;
    regFile.setVecReg(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setVecElem(PhysRegIdPtr phys_reg, const VecElem& val)
{
    vecRegfileWrites++;
    regFile.setVecElem(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setCCReg(PhysRegIdPtr phys_reg, CCReg val)
{
    ccRegfileWrites++;
    regFile.setCCReg(phys_reg, val);
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readArchIntReg(int reg_idx, ThreadID tid)
{
    intRegfileReads++;
    PhysRegIdPtr phys_reg = commitRenameMap[tid].lookup(
            RegId(IntRegClass, reg_idx));
	// 说明寄存器类型和对应的的编号才可以找到重命名的映射关系

    return regFile.readIntReg(phys_reg);
}

template <class Impl>
float
FullO3CPU<Impl>::readArchFloatReg(int reg_idx, ThreadID tid)
{
    fpRegfileReads++;
    PhysRegIdPtr phys_reg = commitRenameMap[tid].lookup(
        RegId(FloatRegClass, reg_idx));

    return regFile.readFloatReg(phys_reg);
}

template <class Impl>
uint64_t
FullO3CPU<Impl>::readArchFloatRegInt(int reg_idx, ThreadID tid)
{
    fpRegfileReads++;
    PhysRegIdPtr phys_reg = commitRenameMap[tid].lookup(
        RegId(FloatRegClass, reg_idx));

    return regFile.readFloatRegBits(phys_reg);
}

template <class Impl>
auto
FullO3CPU<Impl>::readArchVecReg(int reg_idx, ThreadID tid) const
        -> const VecRegContainer&
{
    PhysRegIdPtr phys_reg = commitRenameMap[tid].lookup(
                RegId(VecRegClass, reg_idx));
    return readVecReg(phys_reg);
}

template <class Impl>
auto
FullO3CPU<Impl>::getWritableArchVecReg(int reg_idx, ThreadID tid)
        -> VecRegContainer&
{
    PhysRegIdPtr phys_reg = commitRenameMap[tid].lookup(
                RegId(VecRegClass, reg_idx));
    return getWritableVecReg(phys_reg);
}

template <class Impl>
auto
FullO3CPU<Impl>::readArchVecElem(const RegIndex& reg_idx, const ElemIndex& ldx,
                                 ThreadID tid) const -> const VecElem&
{
    PhysRegIdPtr phys_reg = commitRenameMap[tid].lookup(
                                RegId(VecRegClass, reg_idx, ldx));
    return readVecElem(phys_reg);
}

template <class Impl>
CCReg
FullO3CPU<Impl>::readArchCCReg(int reg_idx, ThreadID tid)
{
    ccRegfileReads++;
    PhysRegIdPtr phys_reg = commitRenameMap[tid].lookup(
        RegId(CCRegClass, reg_idx));

    return regFile.readCCReg(phys_reg);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchIntReg(int reg_idx, uint64_t val, ThreadID tid)
{
    intRegfileWrites++;
    PhysRegIdPtr phys_reg = commitRenameMap[tid].lookup(
            RegId(IntRegClass, reg_idx));

    regFile.setIntReg(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchFloatReg(int reg_idx, float val, ThreadID tid)
{
    fpRegfileWrites++;
    PhysRegIdPtr phys_reg = commitRenameMap[tid].lookup(
            RegId(FloatRegClass, reg_idx));

    regFile.setFloatReg(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchFloatRegInt(int reg_idx, uint64_t val, ThreadID tid)
{
    fpRegfileWrites++;
    PhysRegIdPtr phys_reg = commitRenameMap[tid].lookup(
            RegId(FloatRegClass, reg_idx));

    regFile.setFloatRegBits(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchVecReg(int reg_idx, const VecRegContainer& val,
                               ThreadID tid)
{
    PhysRegIdPtr phys_reg = commitRenameMap[tid].lookup(
                RegId(VecRegClass, reg_idx));
    setVecReg(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchVecElem(const RegIndex& reg_idx, const ElemIndex& ldx,
                                const VecElem& val, ThreadID tid)
{
    PhysRegIdPtr phys_reg = commitRenameMap[tid].lookup(
                RegId(VecRegClass, reg_idx, ldx));
    setVecElem(phys_reg, val);
}

template <class Impl>
void
FullO3CPU<Impl>::setArchCCReg(int reg_idx, CCReg val, ThreadID tid)
{
    ccRegfileWrites++;
    PhysRegIdPtr phys_reg = commitRenameMap[tid].lookup(
            RegId(CCRegClass, reg_idx));

    regFile.setCCReg(phys_reg, val);
}

template <class Impl>
TheISA::PCState
FullO3CPU<Impl>::pcState(ThreadID tid)
{
    return commit.pcState(tid);
}

template <class Impl>
void
FullO3CPU<Impl>::pcState(const TheISA::PCState &val, ThreadID tid)
{
    commit.pcState(val, tid);
}

template <class Impl>
Addr
FullO3CPU<Impl>::instAddr(ThreadID tid)
{
    return commit.instAddr(tid);
}

template <class Impl>
Addr
FullO3CPU<Impl>::nextInstAddr(ThreadID tid)
{
    return commit.nextInstAddr(tid);
}

template <class Impl>
MicroPC
FullO3CPU<Impl>::microPC(ThreadID tid)
{
    return commit.microPC(tid);
}

template <class Impl>
void
FullO3CPU<Impl>::squashFromTC(ThreadID tid)
{
    this->thread[tid]->noSquashFromTC = true;
    this->commit.generateTCEvent(tid);
}

template <class Impl>
typename FullO3CPU<Impl>::ListIt
FullO3CPU<Impl>::addInst(DynInstPtr &inst)
{
    instList.push_back(inst);

    return --(instList.end());
}

template <class Impl>
void
FullO3CPU<Impl>::instDone(ThreadID tid, DynInstPtr &inst)
{
    // Keep an instruction count.
    if (!inst->isMicroop() || inst->isLastMicroop()) {
		
        thread[tid]->numInst++;
        thread[tid]->numInsts++;
        committedInsts[tid]++;
        system->totalNumInsts++;

        // Check for instruction-count-based events.
        comInstEventQueue[tid]->serviceEvents(thread[tid]->numInst);
        system->instEventQueue.serviceEvents(system->totalNumInsts);
		// 检查和指令数据相关的事件，比如gem5支持在执行若干条指令后停止
    }
	// 上面是完成一个指令而非微操作后的处理
    thread[tid]->numOp++;
    thread[tid]->numOps++;
    committedOps[tid]++;
	// 微操作和指令共享的计数器，即未不设置微操作则该数值对应于指令的统计数据

    probeInstCommit(inst->staticInst);
	// ？？？
}
// 一个指令或者微操作完成以后的相关数据更新和处理

template <class Impl>
void
FullO3CPU<Impl>::removeFrontInst(DynInstPtr &inst)
{
    DPRINTF(O3CPU, "Removing committed instruction [tid:%i] PC %s "
            "[sn:%lli]\n",
            inst->threadNumber, inst->pcState(), inst->seqNum);

    removeInstsThisCycle = true;

    // Remove the front instruction.
    removeList.push(inst->getInstListIt());
}
// 将制定的指令放到removeList中去，并设置标志位

template <class Impl>
void
FullO3CPU<Impl>::removeInstsNotInROB(ThreadID tid)
{
    DPRINTF(O3CPU, "Thread %i: Deleting instructions from instruction"
            " list.\n", tid);

    ListIt end_it;

    bool rob_empty = false;

    if (instList.empty()) {
        return;
		// 如果InstList为空则一定没有可以执行的操作
    } else if (rob.isEmpty(tid)) {
        DPRINTF(O3CPU, "ROB is empty, squashing all insts.\n");
        end_it = instList.begin();
        rob_empty = true;
		// 如果ROB没有指令，那么instList中的指令需要全部清除
    } else {
        end_it = (rob.readTailInst(tid))->getInstListIt();
		// 获取ROB中最老的指令对应与instList中的位置
        DPRINTF(O3CPU, "ROB is not empty, squashing insts not in ROB.\n");
    }

    removeInstsThisCycle = true;

    ListIt inst_it = instList.end();
	// 找到squash结束的位置，这里对应的是最老的指令

    inst_it--;
	// 似乎.end对应的元素不会处理？？？

    // Walk through the instruction list, removing any instructions
    // that were inserted after the given instruction iterator, end_it.
    while (inst_it != end_it) {
        assert(!instList.empty());

        squashInstIt(inst_it, tid);

        inst_it--;
    }
	// 将instList中所有需要squash的指令处理掉

    // If the ROB was empty, then we actually need to remove the first
    // instruction as well.
    if (rob_empty) {
        squashInstIt(inst_it, tid);
		// 如果ROB是空的说明inst_it（此时等于end_it）对应的也是需要处理的指令
    }
}

template <class Impl>
void
FullO3CPU<Impl>::removeInstsUntil(const InstSeqNum &seq_num, ThreadID tid)
{
    assert(!instList.empty());
	// 执行该指令的时候instList不能为空，否则就不会有可以移除的指令了

    removeInstsThisCycle = true;

    ListIt inst_iter = instList.end();

    inst_iter--;

    DPRINTF(O3CPU, "Deleting instructions from instruction "
            "list that are from [tid:%i] and above [sn:%lli] (end=%lli).\n",
            tid, seq_num, (*inst_iter)->seqNum);

    while ((*inst_iter)->seqNum > seq_num) {

        bool break_loop = (inst_iter == instList.begin());

        squashInstIt(inst_iter, tid);

        inst_iter--;

        if (break_loop)
            break;
    }
}
// 从instList中将所有序列编号小于给定编号的指令squash掉

template <class Impl>
inline void
FullO3CPU<Impl>::squashInstIt(const ListIt &instIt, ThreadID tid)
{
    if ((*instIt)->threadNumber == tid) {
        DPRINTF(O3CPU, "Squashing instruction, "
                "[tid:%i] [sn:%lli] PC %s\n",
                (*instIt)->threadNumber,
                (*instIt)->seqNum,
                (*instIt)->pcState());

        // Mark it as squashed.
        (*instIt)->setSquashed();

        // @todo: Formulate a consistent method for deleting
        // instructions from the instruction list
        // Remove the instruction from the list.
        removeList.push(instIt);
    }
}
// 将指定线程的给定指令squash掉，并放到removeList中

template <class Impl>
void
FullO3CPU<Impl>::cleanUpRemovedInsts()
{
    while (!removeList.empty()) {
        DPRINTF(O3CPU, "Removing instruction, "
                "[tid:%i] [sn:%lli] PC %s\n",
                (*removeList.front())->threadNumber,
                (*removeList.front())->seqNum,
                (*removeList.front())->pcState());

        instList.erase(removeList.front());

        removeList.pop();
    }

    removeInstsThisCycle = false;
}
// 从instList和removeList中删除所有位于removeList中的指令

/*
template <class Impl>
void
FullO3CPU<Impl>::removeAllInsts()
{
    instList.clear();
}
*/
template <class Impl>
void
FullO3CPU<Impl>::dumpInsts()
{
    int num = 0;

    ListIt inst_list_it = instList.begin();

    cprintf("Dumping Instruction List\n");

    while (inst_list_it != instList.end()) {
        cprintf("Instruction:%i\nPC:%#x\n[tid:%i]\n[sn:%lli]\nIssued:%i\n"
                "Squashed:%i\n\n",
                num, (*inst_list_it)->instAddr(), (*inst_list_it)->threadNumber,
                (*inst_list_it)->seqNum, (*inst_list_it)->isIssued(),
                (*inst_list_it)->isSquashed());
        inst_list_it++;
        ++num;
    }
}
// 用于debug时打印instList中的指令
/*
template <class Impl>
void
FullO3CPU<Impl>::wakeDependents(DynInstPtr &inst)
{
    iew.wakeDependents(inst);
}
*/
template <class Impl>
void
FullO3CPU<Impl>::wakeCPU()
{
    if (activityRec.active() || tickEvent.scheduled()) {
        DPRINTF(Activity, "CPU already running.\n");
        return;
    }
	// 发现活动记录器活跃或者tickEvent正常调度，说明CPU无需唤醒

    DPRINTF(Activity, "Waking up CPU\n");

    Cycles cycles(curCycle() - lastRunningCycle);
    // @todo: This is an oddity that is only here to match the stats
    if (cycles > 1) {
        --cycles;
        idleCycles += cycles;
        numCycles += cycles;
		// 设置相应的周期计数
    }

    schedule(tickEvent, clockEdge());
	// 开始调度tickEvent继续运行CPU
}
// 唤醒当前CPU

template <class Impl>
void
FullO3CPU<Impl>::wakeup(ThreadID tid)
{
    if (this->thread[tid]->status() != ThreadContext::Suspended)
        return;
	// 如果该线程无需唤醒直接返回

    this->wakeCPU();
	// 首先尝试唤醒CPU

    DPRINTF(Quiesce, "Suspended Processor woken\n");
    this->threadContexts[tid]->activate();
	// 激活对应线程的上下文
}
// 唤醒CPU执行给定的线程

template <class Impl>
ThreadID
FullO3CPU<Impl>::getFreeTid()
{
    for (ThreadID tid = 0; tid < numThreads; tid++) {
        if (!tids[tid]) {
            tids[tid] = true;
            return tid;
        }
    }

    return InvalidThreadID;
}
// 获取一个未使用的线程id

template <class Impl>
void
FullO3CPU<Impl>::updateThreadPriority()
{
    if (activeThreads.size() > 1) {
        //DEFAULT TO ROUND ROBIN SCHEME
        //e.g. Move highest priority to end of thread list
        list<ThreadID>::iterator list_begin = activeThreads.begin();

        unsigned high_thread = *list_begin;

        activeThreads.erase(list_begin);

        activeThreads.push_back(high_thread);
    }
}
// 更新活跃进程的优先级

// Forward declaration of FullO3CPU.
template class FullO3CPU<O3CPUImpl>;
