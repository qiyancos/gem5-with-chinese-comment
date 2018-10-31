/*
 * Copyright (c) 2011, 2016 ARM Limited
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
 *          Nathanael Premillieu
 */

#ifndef __CPU_O3_COMM_HH__
#define __CPU_O3_COMM_HH__

#include <vector>

#include "arch/types.hh"
#include "base/types.hh"
#include "cpu/inst_seq.hh"
#include "sim/faults.hh"

/** Physical register index type.
 * Although the Impl might be a better for this, but there are a few classes
 * that need this typedef yet are not templated on the Impl.
 */
using PhysRegIndex = short int;

/** Physical register ID.
 * Like a register ID but physical. The inheritance is private because the
 * only relationship between this types is functional, and it is done to
 * prevent code replication. */
class PhysRegId : private RegId {
  private:
    PhysRegIndex flatIdx;

  public:
    explicit PhysRegId() : RegId(IntRegClass, -1), flatIdx(-1) {}

    /** Scalar PhysRegId constructor. */
    explicit PhysRegId(RegClass _regClass, PhysRegIndex _regIdx,
              PhysRegIndex _flatIdx)
        : RegId(_regClass, _regIdx), flatIdx(_flatIdx)
    {}

    /** Vector PhysRegId constructor (w/ elemIndex). */
    explicit PhysRegId(RegClass _regClass, PhysRegIndex _regIdx,
              ElemIndex elem_idx, PhysRegIndex flat_idx)
        : RegId(_regClass, _regIdx, elem_idx), flatIdx(flat_idx) { }

    /** Visible RegId methods */
    /** @{ */
    using RegId::index;
    using RegId::classValue;
    using RegId::isZeroReg;
    using RegId::className;
    using RegId::elemIndex;
     /** @} */
    /**
     * Explicit forward methods, to prevent comparisons of PhysRegId with
     * RegIds.
     */
    /** @{ */
    bool operator<(const PhysRegId& that) const {
        return RegId::operator<(that);
    }

    bool operator==(const PhysRegId& that) const {
        return RegId::operator==(that);
    }

    bool operator!=(const PhysRegId& that) const {
        return RegId::operator!=(that);
    }
    /** @} */

    /** @return true if it is an integer physical register. */
    bool isIntPhysReg() const { return isIntReg(); }

    /** @return true if it is a floating-point physical register. */
    bool isFloatPhysReg() const { return isFloatReg(); }

    /** @Return true if it is a  condition-code physical register. */
    bool isCCPhysReg() const { return isCCReg(); }

    /** @Return true if it is a vector physical register. */
    bool isVectorPhysReg() const { return isVecReg(); }

    /** @Return true if it is a vector element physical register. */
    bool isVectorPhysElem() const { return isVecElem(); }

    /** @Return true if it is a  condition-code physical register. */
    bool isMiscPhysReg() const { return isMiscReg(); }

    /**
     * Returns true if this register is always associated to the same
     * architectural register.
     */
    bool isFixedMapping() const
    {
        return !isRenameable();
    }

    /** Flat index accessor */
    const PhysRegIndex& flatIndex() const { return flatIdx; }

    static PhysRegId elemId(const PhysRegId* vid, ElemIndex elem)
    {
        assert(vid->isVectorPhysReg());
        return PhysRegId(VecElemClass, vid->index(), elem);
    }
};

/** Constant pointer definition.
 * PhysRegIds only need to be created once and then we can just share
 * pointers */
using PhysRegIdPtr = const PhysRegId*;

/** Struct that defines the information passed from fetch to decode. */
template<class Impl>
struct DefaultFetchDefaultDecode {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth];
    Fault fetchFault;
    InstSeqNum fetchFaultSN;
    bool clearFetchFault;
};
// 这里对应于Fetch到Decode阶段
// 流水线不同级别之间的寄存器，Latch overhead

/** Struct that defines the information passed from decode to rename. */
template<class Impl>
struct DefaultDecodeDefaultRename {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth];
};
// 这里对应于Deocde阶段到Rename阶段
// 很明显Decode并没有对DynInstPtr进行转换
// 后面的结构体定义同理

/** Struct that defines the information passed from rename to IEW. */
template<class Impl>
struct DefaultRenameDefaultIEW {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth];
};
// 似乎DynInstPtr这个类包含了CPU执行所需要的全部信息

/** Struct that defines the information passed from IEW to commit. */
template<class Impl>
struct DefaultIEWDefaultCommit {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;

    DynInstPtr insts[Impl::MaxWidth];
    DynInstPtr mispredictInst[Impl::MaxThreads];
	// 错误预测的指令，如果支持多线程，那么该错误预测的指令数目与SMT宽度相等
    Addr mispredPC[Impl::MaxThreads];
	// 错误预测的PC
    InstSeqNum squashedSeqNum[Impl::MaxThreads];
	// ？？？
    TheISA::PCState pc[Impl::MaxThreads];
	// 计算得到的新PC
	
    bool squash[Impl::MaxThreads];
    // 是否进行
	bool branchMispredict[Impl::MaxThreads];
	// 之前的分支预测是否发生了错误
    bool branchTaken[Impl::MaxThreads];
	// 给定的分支指令是否被采用
    bool includeSquashInst[Impl::MaxThreads];
	// ？？？
};
// 这一部分与之前的结构体明显不同，包含了执行后结果相关的很多信息

template<class Impl>
struct IssueStruct {
    typedef typename Impl::DynInstPtr DynInstPtr;

    int size;
	// ？？？

    DynInstPtr insts[Impl::MaxWidth];
};
// 发射部分的结构体，对应了当前多发射可以发射的几条指令

/** Struct that defines all backwards communication. */
template<class Impl>
struct TimeBufStruct {
    typedef typename Impl::DynInstPtr DynInstPtr;
    struct decodeComm {
        TheISA::PCState nextPC;
		// 下一个PC设定数据
        DynInstPtr mispredictInst;
		// 错误预测的指令
        DynInstPtr squashInst;
        // 效果被冲刷掉的指令
		InstSeqNum doneSeqNum;
        // ？？？
		Addr mispredPC;
        // 产生错误预测的指令PC
		uint64_t branchAddr;
		// 预测分支的目标地址
        unsigned branchCount;
		// ？？？
        bool squash;
		// 指令是否被冲刷
        bool predIncorrect;
		// 预测是否正确？？？
        bool branchMispredict;
		// 分支预测错误
        bool branchTaken;
		// 分支目标被采用
    };
	// 由Decode阶段产生的数据

    decodeComm decodeInfo[Impl::MaxThreads];
	// 为SMT的每个线程创建一个结构体实体
	
    struct renameComm {
    };

    renameComm renameInfo[Impl::MaxThreads];
	// 重命名阶段没有产生数据
	
    struct iewComm {
        // Also eventually include skid buffer space.
        unsigned freeIQEntries;
		// 空闲的Issue Queue数目
        unsigned freeLQEntries;
		// 空闲的Load Queue数目
        unsigned freeSQEntries;
        // 空闲的Store Queue数目
		unsigned dispatchedToLQ;
        // ？？？
		unsigned dispatchedToSQ;
		// ？？？
		
        unsigned iqCount;
		// Issue Queue的计数
        unsigned ldstqCount;
		// LSQ的计数

        unsigned dispatched;
        bool usedIQ;
        bool usedLSQ;
		// ？？？
    };

    iewComm iewInfo[Impl::MaxThreads];
	// IEW这个阶段生成的数据
	
    struct commitComm {
        /////////////////////////////////////////////////////////////////////
        // This code has been re-structured for better packing of variables
        // instead of by stage which is the more logical way to arrange the
        // data.
        // F = Fetch
        // D = Decode
        // I = IEW
        // R = Rename
        // As such each member is annotated with who consumes it
        // e.g. bool variable name // *F,R for Fetch and Rename
        /////////////////////////////////////////////////////////////////////
		
		// 这里表示的是由Commit阶段生成的数据
		
        /// The pc of the next instruction to execute. This is the next
        /// instruction for a branch mispredict, but the same instruction for
        /// order violation and the like
        TheISA::PCState pc; // *F
		// *F表示发送给Fetch阶段的数据，后面同理
		// 在分支预测错误情况下的下一条指令的PC？？？
		
        /// Provide fetch the instruction that mispredicted, if this
        /// pointer is not-null a misprediction occured
        DynInstPtr mispredictInst;  // *F
		// 说明发生错误预测的指令，该项不为空则说明发生了错误预测
		
        /// Instruction that caused the a non-mispredict squash
        DynInstPtr squashInst; // *F
		// 非错误预测导致squash操作的指令？？？

        /// Hack for now to send back a strictly ordered access to the
        /// IEW stage.
        DynInstPtr strictlyOrderedLoad; // *I
		// 表示一个需要严格按需处理的Load指令
		// 这个操作在alpha的TSO模型中可以看到
		// 因为发现有的Load不能推测执行
		// 因此被处理成严格按序（Load和Store）执行的指令

        /// Communication specifically to the IQ to tell the IQ that it can
        /// schedule a non-speculative instruction.
        InstSeqNum nonSpecSeqNum; // *I
		// ？？？
		
        /// Represents the instruction that has either been retired or
        /// squashed.  Similar to having a single bus that broadcasts the
        /// retired or squashed sequence number.
        InstSeqNum doneSeqNum; // *F, I
		// 广播一个指令完成到CDB时用来进行识别的一个序列编号

        /// Tell Rename how many free entries it has in the ROB
        unsigned freeROBEntries; // *R
		// ROB表中空闲表项的个数

        bool squash; // *F, D, R, I
		// 是否冲刷单条指令
        bool robSquashing; // *F, D, R, I
		// ROB是否进行冲刷中
		
        /// Rename should re-read number of free rob entries
        bool usedROB; // *R
		// Rename阶段需要应该更新它那边记录的ROB数量
		// 从而进行Rename分配的调整

        /// Notify Rename that the ROB is empty
        bool emptyROB; // *R
		// 告知Rename阶段ROB表是否为空

        /// Was the branch taken or not
        bool branchTaken; // *F
		// 分支是否被采用
		
        /// If an interrupt is pending and fetch should stall
        bool interruptPending; // *F
		// 中断的处理相关内容（并不是指令执行，而是控制相关的处理）
		// 没有完成，这段时间取指阶段将被stall暂停
        /// If the interrupt ended up being cleared before being handled
        bool clearInterrupt; // *F
		// 表示处理的中断没有进入正常的中断处理程序而被清除了

        /// Hack for now to send back an strictly ordered access to
        /// the IEW stage.
        bool strictlyOrdered; // *I
		// ？？？

    };

    commitComm commitInfo[Impl::MaxThreads];

    bool decodeBlock[Impl::MaxThreads];
    bool decodeUnblock[Impl::MaxThreads];
    bool renameBlock[Impl::MaxThreads];
    bool renameUnblock[Impl::MaxThreads];
    bool iewBlock[Impl::MaxThreads];
    bool iewUnblock[Impl::MaxThreads];
	// ？？？
};
// 该结构体定义的所有与流水线指令执行反向传播的数据

#endif //__CPU_O3_COMM_HH__
