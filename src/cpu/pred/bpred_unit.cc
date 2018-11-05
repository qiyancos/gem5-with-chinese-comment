/*
 * Copyright (c) 2011-2012, 2014 ARM Limited
 * Copyright (c) 2010 The University of Edinburgh
 * Copyright (c) 2012 Mark D. Hill and David A. Wood
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
 */

#include "cpu/pred/bpred_unit.hh"

#include <algorithm>

#include "arch/isa_traits.hh"
#include "arch/types.hh"
#include "arch/utility.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "debug/Branch.hh"

BPredUnit::BPredUnit(const Params *params)
    : SimObject(params),
      numThreads(params->numThreads),
      predHist(numThreads),
      BTB(params->BTBEntries,
          params->BTBTagSize,
          params->instShiftAmt,
          params->numThreads),
      RAS(numThreads),
      useIndirect(params->useIndirect),
      iPred(params->indirectHashGHR,
            params->indirectHashTargets,
            params->indirectSets,
            params->indirectWays,
            params->indirectTagSize,
            params->indirectPathLength,
            params->instShiftAmt,
            params->numThreads),
      instShiftAmt(params->instShiftAmt)
{
    for (auto& r : RAS)
        r.init(params->RASSize);
}

void
BPredUnit::regStats()
{
    SimObject::regStats();

    lookups
        .name(name() + ".lookups")
        .desc("Number of BP lookups")
        ;

    condPredicted
        .name(name() + ".condPredicted")
        .desc("Number of conditional branches predicted")
        ;

    condIncorrect
        .name(name() + ".condIncorrect")
        .desc("Number of conditional branches incorrect")
        ;

    BTBLookups
        .name(name() + ".BTBLookups")
        .desc("Number of BTB lookups")
        ;

    BTBHits
        .name(name() + ".BTBHits")
        .desc("Number of BTB hits")
        ;

    BTBCorrect
        .name(name() + ".BTBCorrect")
        .desc("Number of correct BTB predictions (this stat may not "
              "work properly.")
        ;

    BTBHitPct
        .name(name() + ".BTBHitPct")
        .desc("BTB Hit Percentage")
        .precision(6);
    BTBHitPct = (BTBHits / BTBLookups) * 100;

    usedRAS
        .name(name() + ".usedRAS")
        .desc("Number of times the RAS was used to get a target.")
        ;

    RASIncorrect
        .name(name() + ".RASInCorrect")
        .desc("Number of incorrect RAS predictions.")
        ;

    indirectLookups
        .name(name() + ".indirectLookups")
        .desc("Number of indirect predictor lookups.")
        ;

    indirectHits
        .name(name() + ".indirectHits")
        .desc("Number of indirect target hits.")
        ;

    indirectMisses
        .name(name() + ".indirectMisses")
        .desc("Number of indirect misses.")
        ;

    indirectMispredicted
        .name(name() + "indirectMispredicted")
        .desc("Number of mispredicted indirect branches.")
        ;

}

ProbePoints::PMUUPtr
BPredUnit::pmuProbePoint(const char *name)
{
    ProbePoints::PMUUPtr ptr;
    ptr.reset(new ProbePoints::PMU(getProbeManager(), name));

    return ptr;
}

void
BPredUnit::regProbePoints()
{
    ppBranches = pmuProbePoint("Branches");
    ppMisses = pmuProbePoint("Misses");
}

void
BPredUnit::drainSanityCheck() const
{
    // We shouldn't have any outstanding requests when we resume from
    // a drained system.
    for (const auto& ph M5_VAR_USED : predHist)
        assert(ph.empty());
}

bool
BPredUnit::predict(const StaticInstPtr &inst, const InstSeqNum &seqNum,
                   TheISA::PCState &pc, ThreadID tid)
{
    // See if branch predictor predicts taken.
    // If so, get its target addr either from the BTB or the RAS.
    // Save off record of branch stuff so the RAS can be fixed
    // up once it's done.

    bool pred_taken = false;
    TheISA::PCState target = pc;
	// 这里的pc参数默认表示的是无分支情况下的下一个PC
	// 而分支token标志位则初始化为false

    ++lookups;
    ppBranches->notify(1);
	// ？？？

    void *bp_history = NULL;

    if (inst->isUncondCtrl()) {
        DPRINTF(Branch, "[tid:%i]: Unconditional control.\n", tid);
        pred_taken = true;
		// 对于无条件分支指令，都预测token
        // Tell the BP there was an unconditional branch.
        uncondBranch(tid, pc.instAddr(), bp_history);
    } else {
        ++condPredicted;
        pred_taken = lookup(tid, pc.instAddr(), bp_history);
		// 根据PC查找表确定当前分支是否会被采用
        DPRINTF(Branch, "[tid:%i]: [sn:%i] Branch predictor"
                " predicted %i for PC %s\n", tid, seqNum,  pred_taken, pc);
    }

    DPRINTF(Branch, "[tid:%i]: [sn:%i] Creating prediction history "
            "for PC %s\n", tid, seqNum, pc);

    PredictorHistory predict_record(seqNum, pc.instAddr(),
                                    pred_taken, bp_history, tid);
	// 生成一个空的分支历史表项作为临时变量，记录分支指令的相关信息

    // Now lookup in the BTB or RAS.
    if (pred_taken) {
        if (inst->isReturn()) {
            ++usedRAS;
            predict_record.wasReturn = true;
			// 标记历史表项表明该指令是一个返回指令
            // If it's a function return call, then look up the address
            // in the RAS.
            TheISA::PCState rasTop = RAS[tid].top();
			// 直接从RAS中的顶部获取一个返回地址作为跳转目标地址
            
			target = TheISA::buildRetPC(pc, rasTop);
			
            // Record the top entry of the RAS, and its index.
            predict_record.usedRAS = true;
            predict_record.RASIndex = RAS[tid].topIdx();
            predict_record.RASTarget = rasTop;
			// 记录本次返回操作使用的相关信息

            RAS[tid].pop();
			// 在使用RAS获取了相应的返回地址后将对应表项删除

            DPRINTF(Branch, "[tid:%i]: Instruction %s is a return, "
                    "RAS predicted target: %s, RAS index: %i.\n",
                    tid, pc, target, predict_record.RASIndex);
        } else {
            ++BTBLookups;

            if (inst->isCall()) {
                RAS[tid].push(pc);
                predict_record.pushedRAS = true;
				// 如果当前指令是一个函数调用，那么将当前分支指令对应
				// 的下一条指令的PC压入到RAS中
				
                // Record that it was a call so that the top RAS entry can
                // be popped off if the speculation is incorrect.
                predict_record.wasCall = true;

                DPRINTF(Branch, "[tid:%i]: Instruction %s was a "
                        "call, adding %s to the RAS index: %i.\n",
                        tid, pc, pc, RAS[tid].topIdx());
            }
			// 上面的操作仅仅更新了RAS，但是并没有对调用后跳转目标地址进行预测
			// 下面的操作会将该指令作为直接跳转类型进行预测

            if (inst->isDirectCtrl() || !useIndirect) {
                // Check BTB on direct branches
                if (BTB.valid(pc.instAddr(), tid)) {
                    ++BTBHits;
					// 如果在BTB中找到了该指令PC对应的表项，那么说明BTB命中
					// 可以直接根据提取出来的表项获取分支目标PC target
					
                    // If it's not a return, use the BTB to get target addr.
                    target = BTB.lookup(pc.instAddr(), tid);

                    DPRINTF(Branch, "[tid:%i]: Instruction %s predicted"
                            " target is %s.\n", tid, pc, target);

                } else {
                    DPRINTF(Branch, "[tid:%i]: BTB doesn't have a "
                            "valid entry.\n",tid);
                    pred_taken = false;
                    // 即使通过分支指令PC查找预测分支token，如果无法找到
					// 对应的分支目标地址，那么依然认为分支不会被token
					
					// The Direction of the branch predictor is altered
                    // because the BTB did not have an entry
                    // The predictor needs to be updated accordingly
                    if (!inst->isCall() && !inst->isReturn()) {
                        btbUpdate(tid, pc.instAddr(), bp_history);
                        DPRINTF(Branch, "[tid:%i]:[sn:%i] btbUpdate"
                                " called for %s\n", tid, seqNum, pc);
						// 对于不使用RAS并且BTB表项invalid的指令，会根据
						// 实际的结果对标注需要对表项进行更新
                    } else if (inst->isCall() && !inst->isUncondCtrl()) {
                        RAS[tid].pop();
                        predict_record.pushedRAS = false;
						// 对于函数调用指令，如果发现当前指令PC的BTB不可用
						// 则取消将该PC push到RAS中？？？
                    }
                    TheISA::advancePC(target, inst);
					// 由于BTB无效，因此预测不会采用分支，
					// 此处按照顺序执行的情况进行PC的更新
                }
            } else {
				// 下面的处理针对于间接控制流变换
                predict_record.wasIndirect = true;
                ++indirectLookups;
                //Consult indirect predictor on indirect control
                if (iPred.lookup(pc.instAddr(), getGHR(tid, bp_history),
                        target, tid)) {
					// 从间接分支预测器iPred中查询该指令对应的分支目标
					// 如果发现确实存在，只需要递增计数器即可，目标地址
					// 被记录在了target中。
                    // Indirect predictor hit
                    ++indirectHits;
                    DPRINTF(Branch, "[tid:%i]: Instruction %s predicted "
                            "indirect target is %s.\n", tid, pc, target);
                } else {
					// 虽然预测命中但是并没有找到对应的分支目标地址
                    ++indirectMisses;
                    pred_taken = false;
                    DPRINTF(Branch, "[tid:%i]: Instruction %s no indirect "
                            "target.\n", tid, pc);
                    if (!inst->isCall() && !inst->isReturn()) {

                    } else if (inst->isCall() && !inst->isUncondCtrl()) {
                        RAS[tid].pop();
                        predict_record.pushedRAS = false;
                    }
                    TheISA::advancePC(target, inst);
					// 按照顺序执行的情况更新PC
                }
                iPred.recordIndirect(pc.instAddr(), target.instAddr(), seqNum,
                        tid);
				// 标注更新间接分支预测器的数据
            }
        }
    } else {
        if (inst->isReturn()) {
           predict_record.wasReturn = true;
		   // 一个return语句被预测not token？？？但是依然会被标记
        }
        TheISA::advancePC(target, inst);
		// 按照顺序执行的情况更新PC
    }

    pc = target;
	// 真正将分支预测的目标PC更新到实际的pc中

    predHist[tid].push_front(predict_record);
	// 记录本次分支预测的信息到preHist队列中

    DPRINTF(Branch, "[tid:%i]: [sn:%i]: History entry added."
            "predHist.size(): %i\n", tid, seqNum, predHist[tid].size());

    return pred_taken;
}

void
BPredUnit::update(const InstSeqNum &done_sn, ThreadID tid)
{
    DPRINTF(Branch, "[tid:%i]: Committing branches until "
            "[sn:%lli].\n", tid, done_sn);

    iPred.commit(done_sn, tid);
	// 这里首先尝试调用对间接分支预测器的更新，这里没有说明具体的更新数据
    while (!predHist[tid].empty() &&
           predHist[tid].back().seqNum <= done_sn) {
        // Update the branch predictor with the correct results.
        update(tid, predHist[tid].back().pc,
                    predHist[tid].back().predTaken,
                    predHist[tid].back().bpHistory, false);
		// 这里更新函数由具体分支单元类进行实现，更新提供了
		// 分支指令pc、线程tid、分支是否token、以及分支目标信息
		// 最后一个参数表示该指令是否被squash掉了
        predHist[tid].pop_back();
    }
	// 实际提交是通过上面的处理进行的，该操作遍历所有preHist中
	// 在该指令之前的分支指令预测信息，并对其进行更新。
}

void
BPredUnit::squash(const InstSeqNum &squashed_sn, ThreadID tid)
{
    History &pred_hist = predHist[tid];

    iPred.squash(squashed_sn, tid);
	// 首先对间接分支预测器进行squash操作
    while (!pred_hist.empty() &&
           pred_hist.front().seqNum > squashed_sn) {
		// 遍历predHist根据指令编号进行squash操作
        if (pred_hist.front().usedRAS) {
            DPRINTF(Branch, "[tid:%i]: Restoring top of RAS to: %i,"
                    " target: %s.\n", tid,
                    pred_hist.front().RASIndex, pred_hist.front().RASTarget);

            RAS[tid].restore(pred_hist.front().RASIndex,
                             pred_hist.front().RASTarget);
			// 如果是一个使用了RAS的指令，那么调用restore指令将给定线程
			// 的RAS对应表项恢复为使用前存放的数值
        } else if (pred_hist.front().wasCall && pred_hist.front().pushedRAS) {
             // Was a call but predicated false. Pop RAS here
             DPRINTF(Branch, "[tid: %i] Squashing"
                     "  Call [sn:%i] PC: %s Popping RAS\n", tid,
                     pred_hist.front().seqNum, pred_hist.front().pc);
             RAS[tid].pop();
			 // 如果是一个函数调用，那么压入RAS中的地址将会被弹出，
			 // 这里直接弹出是因为squash的一定是最近的指令，按序弹出即可
        }

        // This call should delete the bpHistory.
        squash(tid, pred_hist.front().bpHistory);
		// 该squash函数由分支单元实例实现，用来删除分支历史表项

        DPRINTF(Branch, "[tid:%i]: Removing history for [sn:%i] "
                "PC %s.\n", tid, pred_hist.front().seqNum,
                pred_hist.front().pc);

        pred_hist.pop_front();

        DPRINTF(Branch, "[tid:%i]: predHist.size(): %i\n",
                tid, predHist[tid].size());
    }
}

void
BPredUnit::squash(const InstSeqNum &squashed_sn,
                  const TheISA::PCState &corrTarget,
                  bool actually_taken, ThreadID tid)
{
    // Now that we know that a branch was mispredicted, we need to undo
    // all the branches that have been seen up until this branch and
    // fix up everything.
    // NOTE: This should be call conceivably in 2 scenarios:
	
    // (1) After an branch is executed, it updates its status in the ROB
    //     The commit stage then checks the ROB update and sends a signal to
    //     the fetch stage to squash history after the mispredict
	// 第一种场景，一个分支指令执行之后确定发生错误预测后告诉fetch阶段处理
	// 分支错误预测导致的squash操作。
	
    // (2) In the decode stage, you can find out early if a unconditional
    //     PC-relative, branch was predicted incorrectly. If so, a signal
    //     to the fetch stage is sent to squash history after the mispredict
	// 第二种场景，decode阶段发现一个基于当前PC进行间接跳转的指令分支目标地址
	// 预测发生错误（由于这里基于PC计算，因此decode阶段就可以得到分支目标地址
	// ？？？），从而通知fetch阶段进行相应的squash操作。
	
    History &pred_hist = predHist[tid];

    ++condIncorrect;
    ppMisses->notify(1);
	// 分支预测错误的Probe Point的更新通知

    DPRINTF(Branch, "[tid:%i]: Squashing from sequence number %i, "
            "setting target to %s.\n", tid, squashed_sn, corrTarget);

    // Squash All Branches AFTER this mispredicted branch
    squash(squashed_sn, tid);

    // If there's a squash due to a syscall, there may not be an entry
    // corresponding to the squash.  In that case, don't bother trying to
    // fix up the entry.
	
	// 按照正常情况下，经过上面的squash操作以后，predHist应该已经被清空
	
	// 首先如果分支预测错误发现于commit阶段，那么之前的分支指令已经提交
	// 调用update函数之后对应的predHist表项一定已经被清理，那么之后的指令
	// 通过squash之后也会被清理，因此predHist理论上来说只有发生错误预测的
	// 指令一个而已。
    
	// 如果分支发生与decode阶段，就会有所不同，因此位于decode阶段和commit
	// 阶段之间的指令将会无法清除，这一部分指令没有通过update操作提交也
	// 没有通过上面的squash清除，但是这些指令应该得到保留，因为他们不一定
	// 属于错误执行的指令。
	
	if (!pred_hist.empty()) {

        auto hist_it = pred_hist.begin();
        //HistoryIt hist_it = find(pred_hist.begin(), pred_hist.end(),
        //                       squashed_sn);

        //assert(hist_it != pred_hist.end());
        if (pred_hist.front().seqNum != squashed_sn) {
            DPRINTF(Branch, "Front sn %i != Squash sn %i\n",
                    pred_hist.front().seqNum, squashed_sn);

            assert(pred_hist.front().seqNum == squashed_sn);
        }
		// 由于squash操作处理掉了squshed_sn和之前的所有指令，此时
		// predHist的顶部一定是发生错误预测的指令本身

        if ((*hist_it).usedRAS) {
            ++RASIncorrect;
            DPRINTF(Branch, "[tid:%i]: Incorrect RAS [sn:%i]\n",
                    tid, hist_it->seqNum);
        }

        // Get the underlying Global History Register
        unsigned ghr = getGHR(tid, hist_it->bpHistory);

        // There are separate functions for in-order and out-of-order
        // branch prediction, but not for update. Therefore, this
        // call should take into account that the mispredicted branch may
        // be on the wrong path (i.e., OoO execution), and that the counter
        // counter table(s) should not be updated. Thus, this call should
        // restore the state of the underlying predictor, for instance the
        // local/global histories. The counter tables will be updated when
        // the branch actually commits.
		
		// 我们需要考虑一种特殊情况，在乱序核中发生错误预测的指令正好处于
		// 错误分支的路径上（因为乱序执行才会出现这种情况）。上面提到的
		// Counter table指的应该就是分支预测饱和计数器自动机对应的counter。

        // Remember the correct direction for the update at commit.
        pred_hist.front().predTaken = actually_taken;

        update(tid, (*hist_it).pc, actually_taken,
               pred_hist.front().bpHistory, true);
		// 这里调用分支预测实例单元的update函数进行更新操作
		
		// 下面的处理其实是在纠正该错误预测的分支指令对应的predHist表项
        if (actually_taken) {
            if (hist_it->wasReturn && !hist_it->usedRAS) {
                 DPRINTF(Branch, "[tid: %i] Incorrectly predicted"
                         "  return [sn:%i] PC: %s\n", tid, hist_it->seqNum,
                         hist_it->pc);
                 RAS[tid].pop();
                 hist_it->usedRAS = true;
				 // 实际上token但是预测没有token，因此RAS不会被使用
				 // 此处将会使用RAS的内容更新预测结果。
            }
            if (hist_it->wasIndirect) {
                ++indirectMispredicted;
                iPred.recordTarget(hist_it->seqNum, ghr, corrTarget, tid);
				// 如果是一个间接分支预测，对简介分支预测的目标进行记录
            } else {
                DPRINTF(Branch,"[tid: %i] BTB Update called for [sn:%i]"
                        " PC: %s\n", tid,hist_it->seqNum, hist_it->pc);

                BTB.update((*hist_it).pc, corrTarget, tid);
				// 其他情况更新BTB即可
            }
        } else {
           //Actually not Taken
		   // 如果实际并没有token预测token需要回复相关结构的状态
           if (hist_it->usedRAS) {
                DPRINTF(Branch,"[tid: %i] Incorrectly predicted"
                        "  return [sn:%i] PC: %s Restoring RAS\n", tid,
                        hist_it->seqNum, hist_it->pc);
                DPRINTF(Branch, "[tid:%i]: Restoring top of RAS"
                        " to: %i, target: %s.\n", tid,
                        hist_it->RASIndex, hist_it->RASTarget);
                RAS[tid].restore(hist_it->RASIndex, hist_it->RASTarget);
                hist_it->usedRAS = false;
				// 回复之前使用的RAS表项
           } else if (hist_it->wasCall && hist_it->pushedRAS) {
                 //Was a Call but predicated false. Pop RAS here
                 DPRINTF(Branch, "[tid: %i] Incorrectly predicted"
                         "  Call [sn:%i] PC: %s Popping RAS\n", tid,
                         hist_it->seqNum, hist_it->pc);
                 RAS[tid].pop();
                 hist_it->pushedRAS = false;
				// 将不该压入到RAS中的调用指令PC弹出
           }
        }
    } else {
        DPRINTF(Branch, "[tid:%i]: [sn:%i] pred_hist empty, can't "
                "update.\n", tid, squashed_sn);
		// 这种情况不应该发生，除非squash操作来自于非分支指令
		// 那么相当于来自于非更新的强制squash操作，这里处理之后
		// predHist就会变成空表项，无需进行相关内容的修正
    }
}

void
BPredUnit::dump()
{
    int i = 0;
    for (const auto& ph : predHist) {
        if (!ph.empty()) {
            auto pred_hist_it = ph.begin();

            cprintf("predHist[%i].size(): %i\n", i++, ph.size());

            while (pred_hist_it != ph.end()) {
                cprintf("[sn:%lli], PC:%#x, tid:%i, predTaken:%i, "
                        "bpHistory:%#x\n",
                        pred_hist_it->seqNum, pred_hist_it->pc,
                        pred_hist_it->tid, pred_hist_it->predTaken,
                        pred_hist_it->bpHistory);
                pred_hist_it++;
            }

            cprintf("\n");
        }
    }
}
// debug用函数

