/*
 * Copyright (c) 2011, 2014 ARM Limited
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
 */

#include "cpu/pred/tournament.hh"

#include "base/bitfield.hh"
#include "base/intmath.hh"

TournamentBP::TournamentBP(const TournamentBPParams *params)
    : BPredUnit(params),
      localPredictorSize(params->localPredictorSize),
      localCtrBits(params->localCtrBits),
      localHistoryTableSize(params->localHistoryTableSize),
      localHistoryBits(ceilLog2(params->localPredictorSize)),
      globalPredictorSize(params->globalPredictorSize),
      globalCtrBits(params->globalCtrBits),
      globalHistory(params->numThreads, 0),
      globalHistoryBits(
          ceilLog2(params->globalPredictorSize) >
          ceilLog2(params->choicePredictorSize) ?
          ceilLog2(params->globalPredictorSize) :
          ceilLog2(params->choicePredictorSize)),
      choicePredictorSize(params->choicePredictorSize),
      choiceCtrBits(params->choiceCtrBits)
{
    if (!isPowerOf2(localPredictorSize)) {
        fatal("Invalid local predictor size!\n");
    }

    if (!isPowerOf2(globalPredictorSize)) {
        fatal("Invalid global predictor size!\n");
    }

    //Set up the array of counters for the local predictor
    localCtrs.resize(localPredictorSize);
	// 根据大小设置配置局部分支预测器的计数器表

    for (int i = 0; i < localPredictorSize; ++i)
        localCtrs[i].setBits(localCtrBits);
	// 根据有效位数设置局部分支预测器计数器的最大值

    localPredictorMask = mask(localHistoryBits);
	// 根据有效位数生成一个64bit的mask，该操作定义于bitfield.hh

    if (!isPowerOf2(localHistoryTableSize)) {
        fatal("Invalid local history table size!\n");
    }

    //Setup the history table for the local table
    localHistoryTable.resize(localHistoryTableSize);

    for (int i = 0; i < localHistoryTableSize; ++i)
        localHistoryTable[i] = 0;

    //Setup the array of counters for the global predictor
    globalCtrs.resize(globalPredictorSize);

    for (int i = 0; i < globalPredictorSize; ++i)
        globalCtrs[i].setBits(globalCtrBits);
	// 根据有效位数设置全局分支预测器计数器的最大值

    // Set up the global history mask
    // this is equivalent to mask(log2(globalPredictorSize)
    globalHistoryMask = globalPredictorSize - 1;
	// 此处没有使用globalHistoryBits生成mask是因为，该有效位数是
	// 由锦标赛选择计数器表和全局分支预测计数器表共同决定的，因此
	// 使用它生成mask可能是不准确的。

    if (!isPowerOf2(choicePredictorSize)) {
        fatal("Invalid choice predictor size!\n");
    }

    // Set up choiceHistoryMask
    // this is equivalent to mask(log2(choicePredictorSize)
    choiceHistoryMask = choicePredictorSize - 1;
	// 与上面同理

    //Setup the array of counters for the choice predictor
    choiceCtrs.resize(choicePredictorSize);

    for (int i = 0; i < choicePredictorSize; ++i)
        choiceCtrs[i].setBits(choiceCtrBits);

    //Set up historyRegisterMask
    historyRegisterMask = mask(globalHistoryBits);

    //Check that predictors don't use more bits than they have available
    if (globalHistoryMask > historyRegisterMask) {
        fatal("Global predictor too large for global history bits!\n");
    }
    if (choiceHistoryMask > historyRegisterMask) {
        fatal("Choice predictor too large for global history bits!\n");
    }

    if (globalHistoryMask < historyRegisterMask &&
        choiceHistoryMask < historyRegisterMask) {
        inform("More global history bits than required by predictors\n");
    }
	// 要求能够存储的全局历史位数一定不能小于需要使用到的GHR位数

    // Set thresholds for the three predictors' counters
    // This is equivalent to (2^(Ctr))/2 - 1
    localThreshold  = (ULL(1) << (localCtrBits  - 1)) - 1;
    globalThreshold = (ULL(1) << (globalCtrBits - 1)) - 1;
    choiceThreshold = (ULL(1) << (choiceCtrBits - 1)) - 1;
	// 计算区分taken和not taken之间的阈值
}

inline
unsigned
TournamentBP::calcLocHistIdx(Addr &branch_addr)
{
    // Get low order bits after removing instruction offset.
    return (branch_addr >> instShiftAmt) & (localHistoryTableSize - 1);
}

inline
void
TournamentBP::updateGlobalHistTaken(ThreadID tid)
{
    globalHistory[tid] = (globalHistory[tid] << 1) | 1;
    globalHistory[tid] = globalHistory[tid] & historyRegisterMask;
}

inline
void
TournamentBP::updateGlobalHistNotTaken(ThreadID tid)
{
    globalHistory[tid] = (globalHistory[tid] << 1);
    globalHistory[tid] = globalHistory[tid] & historyRegisterMask;
}

inline
void
TournamentBP::updateLocalHistTaken(unsigned local_history_idx)
{
    localHistoryTable[local_history_idx] =
        (localHistoryTable[local_history_idx] << 1) | 1;
}

inline
void
TournamentBP::updateLocalHistNotTaken(unsigned local_history_idx)
{
    localHistoryTable[local_history_idx] =
        (localHistoryTable[local_history_idx] << 1);
}


void
TournamentBP::btbUpdate(ThreadID tid, Addr branch_addr, void * &bp_history)
{
    unsigned local_history_idx = calcLocHistIdx(branch_addr);
	// 首先根据分支指令PC计算出局部分支预测器历史表的索引
	
    //Update Global History to Not Taken (clear LSB)
    globalHistory[tid] &= (historyRegisterMask & ~ULL(1));
    // 重置全局历史寄存器最近一次的历史信息为not taken
	
	//Update Local History to Not Taken
    localHistoryTable[local_history_idx] =
       localHistoryTable[local_history_idx] & (localPredictorMask & ~ULL(1));
	// 重置局部历史表对应表项最近一次的历史信息为not taken
}

bool
TournamentBP::lookup(ThreadID tid, Addr branch_addr, void * &bp_history)
{
    bool local_prediction;
    unsigned local_history_idx;
    unsigned local_predictor_idx;

    bool global_prediction;
    bool choice_prediction;

    //Lookup in the local predictor to get its branch prediction
    local_history_idx = calcLocHistIdx(branch_addr);
    local_predictor_idx = localHistoryTable[local_history_idx]
        & localPredictorMask;
    local_prediction = localCtrs[local_predictor_idx].read() > localThreshold;
	// 确定局部分支预测器中的预测结果，索引历史表得到分支历史，然后索引计数器
	// 表格获得计数器数值，最后根据阈值确定是否taken

    //Lookup in the global predictor to get its branch prediction
    global_prediction = globalThreshold <
      globalCtrs[globalHistory[tid] & globalHistoryMask].read();
	// 确定全局分支预测器中的预测结果，直接根据该线程对应的GHR的有效位
	// 索引全局分支预测器的计数器表获得数值，和阈值对比后得到

    //Lookup in the choice predictor to see which one to use
    choice_prediction = choiceThreshold <
      choiceCtrs[globalHistory[tid] & choiceHistoryMask].read();
	// 确定锦标赛的选择结果，索引方法和全局分支预测器预测的处理类似

    // Create BPHistory and pass it back to be recorded.
    BPHistory *history = new BPHistory;
    history->globalHistory = globalHistory[tid];
    history->localPredTaken = local_prediction;
    history->globalPredTaken = global_prediction;
    history->globalUsed = choice_prediction;
    history->localHistoryIdx = local_history_idx;
    history->localHistory = local_predictor_idx;
    bp_history = (void *)history;
	// 生成一个结构体存放预测的详细信息

    assert(local_history_idx < localHistoryTableSize);
	// 这个错误不应该出现啊？？？

    // Speculative update of the global history and the
    // selected local history.
    if (choice_prediction) {
        if (global_prediction) {
            updateGlobalHistTaken(tid);
            updateLocalHistTaken(local_history_idx);
            return true;
        } else {
            updateGlobalHistNotTaken(tid);
            updateLocalHistNotTaken(local_history_idx);
            return false;
        }
    } else {
        if (local_prediction) {
            updateGlobalHistTaken(tid);
            updateLocalHistTaken(local_history_idx);
            return true;
        } else {
            updateGlobalHistNotTaken(tid);
            updateLocalHistNotTaken(local_history_idx);
            return false;
        }
    }
	// 根据锦标赛的结果选择使用局部还是全局的分支预测器预测结果作为
	// 最后的结果，然后根据预测的结果推测式的更新分支预测器
}

void
TournamentBP::uncondBranch(ThreadID tid, Addr pc, void * &bp_history)
{
    // Create BPHistory and pass it back to be recorded.
    BPHistory *history = new BPHistory;
    history->globalHistory = globalHistory[tid];
    history->localPredTaken = true;
    history->globalPredTaken = true;
    history->globalUsed = true;
    history->localHistoryIdx = invalidPredictorIndex;
    history->localHistory = invalidPredictorIndex;
    bp_history = static_cast<void *>(history);

    updateGlobalHistTaken(tid);
}

void
TournamentBP::update(ThreadID tid, Addr branch_addr, bool taken,
                     void *bp_history, bool squashed)
{
    assert(bp_history);
	// 在更新的时候，给出的BPHistory必须存在，否则无法找到操作
	// 的分支预测器表项对象

    BPHistory *history = static_cast<BPHistory *>(bp_history);

    unsigned local_history_idx = calcLocHistIdx(branch_addr);

    assert(local_history_idx < localHistoryTableSize);

    // Unconditional branches do not use local history.
    bool old_local_pred_valid = history->localHistory !=
            invalidPredictorIndex;
	// 对于非条件分支并不记录实际的局部分支历史，而是使用无效索引作为
	// 该数值填充，因此此处确定是否使用了局部分支预测器

    // If this is a misprediction, restore the speculatively
    // updated state (global history register and local history)
    // and update again.
    if (squashed) {
		// 如果该指令错误预测导致了squash，进行下面的操作进行更新
		// 需要注意，这里是在BPHistory结构体记录的分支历史基础上更新的
		// 因此实际在更新正确预测结果之前相当于进行了历史恢复
		
        // Global history restore and update
        globalHistory[tid] = (history->globalHistory << 1) | taken;
        globalHistory[tid] &= historyRegisterMask;
		// 更新全局分支历史寄存器

        // Local history restore and update.
        if (old_local_pred_valid) {
            localHistoryTable[local_history_idx] =
                        (history->localHistory << 1) | taken;
        }
		// 只有条件分支指令才会更新局部分支历史记录表信息

        return;
    }
	// 上面是错误预测指令的更新操作

    unsigned old_local_pred_index = history->localHistory &
        localPredictorMask;

    assert(old_local_pred_index < localPredictorSize);
	// 获取局部分支预测器的计数器表索引

    // Update the choice predictor to tell it which one was correct if
    // there was a prediction.
    if (history->localPredTaken != history->globalPredTaken &&
        old_local_pred_valid)
    {
	// 如果当时对该指令进行预测时，全局分支预测器结果和局部分支预测器
	// 的结果不相同并且确实使用了局部分支预测器，那么更新锦标赛计数器
		
         // If the local prediction matches the actual outcome,
         // decrement the counter. Otherwise increment the
         // counter.
         unsigned choice_predictor_idx =
           history->globalHistory & choiceHistoryMask;
         if (history->localPredTaken == taken) {
             choiceCtrs[choice_predictor_idx].decrement();
         } else if (history->globalPredTaken == taken) {
             choiceCtrs[choice_predictor_idx].increment();
         }
		 // 锦标赛中全局分支预测器成功预测则增加计数器，
		 // 局部分支预测器成功预测则递减计数器。
    }

    // Update the counters with the proper
    // resolution of the branch. Histories are updated
    // speculatively, restored upon squash() calls, and
    // recomputed upon update(squash = true) calls,
    // so they do not need to be updated.
    unsigned global_predictor_idx =
            history->globalHistory & globalHistoryMask;
    if (taken) {
          globalCtrs[global_predictor_idx].increment();
          if (old_local_pred_valid) {
                 localCtrs[old_local_pred_index].increment();
          }
    } else {
          globalCtrs[global_predictor_idx].decrement();
          if (old_local_pred_valid) {
              localCtrs[old_local_pred_index].decrement();
          }
    }
	// 根据结果同时更新局部分支预测器和全局分支预测器的计数器
	// taken则递增，not taken递减

    // We're done with this history, now delete it.
    delete history;
	// 该指令的更新完成，不再需要它的BPHistory结构体，将其删除
}

void
TournamentBP::squash(ThreadID tid, void *bp_history)
{
    BPHistory *history = static_cast<BPHistory *>(bp_history);

    // Restore global history to state prior to this branch.
    globalHistory[tid] = history->globalHistory;

    // Restore local history
    if (history->localHistoryIdx != invalidPredictorIndex) {
        localHistoryTable[history->localHistoryIdx] = history->localHistory;
    }
	// 恢复两个分支预测器的历史之后，将记录历史的表项删除
	
    // Delete this BPHistory now that we're done with it.
    delete history;
}

TournamentBP*
TournamentBPParams::create()
{
    return new TournamentBP(this);
	// 构造一个使用该参数的锦标赛分支预测器对象
}

unsigned
TournamentBP::getGHR(ThreadID tid, void *bp_history) const
{
    return static_cast<BPHistory *>(bp_history)->globalHistory;
}

#ifdef DEBUG
int
TournamentBP::BPHistory::newCount = 0;
// 初始化BPHistory的debug用变量为0
#endif
