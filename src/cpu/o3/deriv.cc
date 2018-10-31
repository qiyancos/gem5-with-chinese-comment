/*
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

#include "cpu/o3/deriv.hh"

#include <string>

#include "params/DerivO3CPU.hh"

DerivO3CPU *
DerivO3CPUParams::create()
{
    ThreadID actual_num_threads;
    if (FullSystem) {
        // Full-system only supports a single thread for the moment.
        actual_num_threads = 1;
		// FS模式不支持SMT，因此同时只有一个线程在运行
    } else {
        if (workload.size() > numThreads) {
            fatal("Workload Size (%i) > Max Supported Threads (%i) on This CPU",
                  workload.size(), numThreads);
			// numThreads默认初始化为MaxThreads的数值，实际实验证明numThreads
			// 在进入该函数调用前就已经初始化为workload的线程数目
			// 检查非FS模式下对应二进制程序的线程数量是否超过了SMT支持的最大数值
        } else if (workload.size() == 0) {
            fatal("Must specify at least one workload!");
			// 没有可以运行的内容
        }

        // In non-full-system mode, we infer the number of threads from
        // the workload if it's not explicitly specified.
        actual_num_threads =
            (numThreads >= workload.size()) ? numThreads : workload.size();
		// 设定实际CPU中的执行状态的线程数目
    }

    numThreads = actual_num_threads;
	// 初始化实际线程数量

    // Default smtFetchPolicy to "RoundRobin", if necessary.
    std::string round_robin_policy = "RoundRobin";
    std::string single_thread = "SingleThread";

    if (actual_num_threads > 1 && single_thread.compare(smtFetchPolicy) == 0)
        smtFetchPolicy = round_robin_policy;
    else
        smtFetchPolicy = smtFetchPolicy;
	// 根据实际线程数量设置取指策略，RoundRobin针对于多线程，SingleThread即单线程

    return new DerivO3CPU(this);
}
