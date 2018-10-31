/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * Copyright (c) 2016 The University of Virginia
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
 * Authors: Gabe Black
 *          Ali Saidi
 *          Korey Sewell
 *          Alec Roelke
 */
#include "arch/riscv/process.hh"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <iterator>
#include <map>
#include <string>
#include <vector>

#include "arch/riscv/isa.hh"
#include "arch/riscv/isa_traits.hh"
#include "arch/riscv/registers.hh"
#include "base/loader/elf_object.hh"
#include "base/loader/object_file.hh"
#include "base/logging.hh"
#include "base/random.hh"
#include "cpu/thread_context.hh"
#include "debug/Stack.hh"
#include "mem/page_table.hh"
#include "params/Process.hh"
#include "sim/aux_vector.hh"
#include "sim/process.hh"
#include "sim/process_impl.hh"
#include "sim/syscall_return.hh"
#include "sim/system.hh"

using namespace std;
using namespace RiscvISA;

RiscvProcess::RiscvProcess(ProcessParams *params, ObjectFile *objFile) :
        Process(params,
                new EmulationPageTable(params->name, params->pid, PageBytes),
                objFile)
{
    fatal_if(params->useArchPT, "Arch page tables not implemented.");
    const Addr stack_base = 0x7FFFFFFFFFFFFFFFL;
    const Addr max_stack_size = 8 * 1024 * 1024;
    const Addr next_thread_stack_base = stack_base - max_stack_size;
    const Addr brk_point = roundUp(objFile->bssBase() + objFile->bssSize(),
            PageBytes);
    const Addr mmap_end = 0x4000000000000000L;
    memState = make_shared<MemState>(brk_point, stack_base, max_stack_size,
            next_thread_stack_base, mmap_end);
}

void
RiscvProcess::initState()
{
    Process::initState();

    argsInit<uint64_t>(PageBytes);
    for (ContextID ctx: contextIds)
        system->getThreadContext(ctx)->setMiscRegNoEffect(MISCREG_PRV, PRV_U);
}

template<class IntType> void
RiscvProcess::argsInit(int pageSize)
{
    const int RandomBytes = 16;

    updateBias();
    objFile->loadSections(initVirtMem);
    ElfObject* elfObject = dynamic_cast<ElfObject*>(objFile);
    memState->setStackMin(memState->getStackBase());
	// 此处getStackBase获得的就是stack_base的数值0x7FFFFFFFFFFFFFFFL
	
	// 这里进行的应该是目标可执行文件的存储相关状态设定
	// 包括相应的栈缓冲区的设置，以及文件对象的变换
	// 具体需要参照mem文件夹下的源代码

    // Determine stack size and populate auxv
    Addr stack_top = memState->getStackMin();
	// 此处getStackMin获得的也是stack_base的数值0x7FFFFFFFFFFFFFFFL
    stack_top -= RandomBytes;
	// RandomBytes应该是一个16Byte的预留空间
    for (const string& arg: argv)
        stack_top -= arg.size() + 1;
	// 这里应该是将main函数的参数对应的栈空间开辟出来，包括argv[0]
	// 一个特殊的操作就是每个参数都会额外预留1Byte的空间出来
	// 这里的处理类似于脚本语言中的for in操作
    for (const string& env: envp)
        stack_top -= env.size() + 1;
	// 这里将程序所需要的环境变量空间开辟出来
	// 这里每个环境变量也额外预留了1Byte的空间
    stack_top &= -sizeof(Addr);
	// 这里对地址进行了Addr size的对齐处理，32bit ISA中就是4Byte对齐
	// 需要注意的是这里对齐是补齐，虽然地址变小了，但是遂于Stack来说是增大的

    vector<AuxVector<IntType>> auxv;
	// 这里生成了一个元素是模板类
    if (elfObject != nullptr) {
        auxv.push_back({M5_AT_ENTRY, objFile->entryPoint()});
        auxv.push_back({M5_AT_PHNUM, elfObject->programHeaderCount()});
        auxv.push_back({M5_AT_PHENT, elfObject->programHeaderSize()});
        auxv.push_back({M5_AT_PHDR, elfObject->programHeaderTable()});
        auxv.push_back({M5_AT_PAGESZ, PageBytes});
        auxv.push_back({M5_AT_SECURE, 0});
        auxv.push_back({M5_AT_RANDOM, stack_top});
        auxv.push_back({M5_AT_NULL, 0});
		// 这里的语法传递的因该是上面模板类的构造函数参数，包含了两个
    }
    stack_top -= (1 + argv.size()) * sizeof(Addr) +
                   (1 + envp.size()) * sizeof(Addr);
	// 这里存放的是之前参数对应的地址，比如对于argv，有几个参数就会存放几个地址数据
	// 这里两种参数类型都会额外增加一个Addr的空间，存放的是参数个数的信息
    stack_top -= sizeof(Addr) + 2 * sizeof(IntType) * auxv.size();
	// 这里将elf文件的头信息栈空间开辟出来
    stack_top &= -2*sizeof(Addr);
	// 地址补齐操作，这里对齐的是两倍的Addr大小，为什么？
    memState->setStackSize(memState->getStackBase() - stack_top);
	// 根据栈指针的变化重新设定栈大小
    allocateMem(roundDown(stack_top, pageSize),
            roundUp(memState->getStackSize(), pageSize));
	// 根据上面操作开辟的空间大小，执行allocateMem分配整页的内容（以页为基本单位）
	// 后面的操作才会进行真正的内容写入和栈指针变更
	// stack_top只是用来计算需要分配的栈空间的临时变量

    // Copy random bytes (for AT_RANDOM) to stack
    memState->setStackMin(memState->getStackMin() - RandomBytes);
	// getStackMin获得数值只有通过setStackMin才可以改变
    uint8_t at_random[RandomBytes];
    generate(begin(at_random), end(at_random),
             [&]{ return random_mt.random(0, 0xFF); });
	// 这里生成了一系列的0到0xFF随机数，分别赋值给at_random的每个元素
	// 也就是在开始的16Byte位置设置了16个8bit的随机数，这个操作可能是“金丝雀安全处理机制”
    initVirtMem.writeBlob(memState->getStackMin(), at_random, RandomBytes);
	// 这里才是真正的内存写入操作，将前面生成的随机数写入到栈顶16Byte位置

    // Copy argv to stack
    vector<Addr> argPointers;
    for (const string& arg: argv) {
        memState->setStackMin(memState->getStackMin() - (arg.size() + 1));
        initVirtMem.writeString(memState->getStackMin(), arg.c_str());
        // 写入字符串类型的参数到内存中
		argPointers.push_back(memState->getStackMin());
		// 记录每个参数的起始地址
        if (DTRACE(Stack)) {
            // DTRACE可能被用来查看的栈是否可以进行跟踪
			// 如果可以跟踪便进行下面的写入检查
			string wrote;
            initVirtMem.readString(wrote, argPointers.back());
			// 这里将之前写入的参数读取出来
            DPRINTFN("Wrote arg \"%s\" to address %p\n",
                    wrote, (void*)memState->getStackMin());
			// 将写入信息写入到debug的log日志中
		}
    }
    argPointers.push_back(0);
	// 上面的操作主要是将参数argv写入到栈中

    // Copy envp to stack
    vector<Addr> envPointers;
    for (const string& env: envp) {
        memState->setStackMin(memState->getStackMin() - (env.size() + 1));
        initVirtMem.writeString(memState->getStackMin(), env.c_str());
        envPointers.push_back(memState->getStackMin());
        DPRINTF(Stack, "Wrote env \"%s\" to address %p\n",
                env, (void*)memState->getStackMin());
    }
    envPointers.push_back(0);
	// 这里对环境变量envp的处理和argv是一致的，只是不需要进行DTRACE和读取检查

    // Align stack
    memState->setStackMin(memState->getStackMin() & -sizeof(Addr));
	// 对齐栈空间到Addr的大小
	
    // Calculate bottom of stack
    memState->setStackMin(memState->getStackMin() -
            ((1 + argv.size()) * sizeof(Addr) +
             (1 + envp.size()) * sizeof(Addr) +
             sizeof(Addr) + 2 * sizeof(IntType) * auxv.size()));
    memState->setStackMin(memState->getStackMin() & -2*sizeof(Addr));
	// 设置加入参数指针并对齐后的栈位置
	
    Addr sp = memState->getStackMin();
    const auto pushOntoStack =
        [this, &sp](const uint8_t* data, const size_t size) {
            initVirtMem.writeBlob(sp, data, size);
            sp += size;
        };
	// 这里定义了一个pushOntoStack的函数，这里的sp也是一个临时变量
	// 设置sp是因为参数地址是从低地址向高地址多次写入的

    // Push argc and argv pointers onto stack
    IntType argc = htog((IntType)argv.size());
    DPRINTF(Stack, "Wrote argc %d to address %p\n",
            argv.size(), (void*)sp);
    pushOntoStack((uint8_t*)&argc, sizeof(IntType));
	// 先将参数个数信息写入栈
    for (const Addr& argPointer: argPointers) {
        DPRINTF(Stack, "Wrote argv pointer %p to address %p\n",
                (void*)argPointer, (void*)sp);
        pushOntoStack((uint8_t*)&argPointer, sizeof(Addr));
    }
	// 再将参数的地址信息写入栈

    // Push env pointers onto stack
    for (const Addr& envPointer: envPointers) {
        DPRINTF(Stack, "Wrote envp pointer %p to address %p\n",
                (void*)envPointer, (void*)sp);
        pushOntoStack((uint8_t*)&envPointer, sizeof(Addr));
    }
	// 很显然envp的参数数量并没有写入到栈中

    // Push aux vector onto stack
    std::map<IntType, string> aux_keys = {
        {M5_AT_ENTRY, "M5_AT_ENTRY"},
        {M5_AT_PHNUM, "M5_AT_PHNUM"},
        {M5_AT_PHENT, "M5_AT_PHENT"},
        {M5_AT_PHDR, "M5_AT_PHDR"},
        {M5_AT_PAGESZ, "M5_AT_PAGESZ"},
        {M5_AT_SECURE, "M5_AT_SECURE"},
        {M5_AT_RANDOM, "M5_AT_RANDOM"},
        {M5_AT_NULL, "M5_AT_NULL"}
    };
	// 对上面的可执行文件信息向量进行map映射以方便进行查找
	
    for (const AuxVector<IntType>& aux: auxv) {
        DPRINTF(Stack, "Wrote aux key %s to address %p\n",
                aux_keys[aux.a_type], (void*)sp);
        pushOntoStack((uint8_t*)&aux.a_type, sizeof(IntType));
        DPRINTF(Stack, "Wrote aux value %x to address %p\n",
                aux.a_val, (void*)sp);
        pushOntoStack((uint8_t*)&aux.a_val, sizeof(IntType));
    }
	// 将auxv的内容写入到栈中，很显然每个元素包含了两个内容
	// 其中一个是a_type，另一个是a_val

    ThreadContext *tc = system->getThreadContext(contextIds[0]);
    tc->setIntReg(StackPointerReg, memState->getStackMin());
    tc->pcState(getStartPC());
	// 根据当前的stack设置情况将栈信息设置到线程的上下文中

    memState->setStackMin(roundDown(memState->getStackMin(), pageSize));
	// 对齐一个页的大小并设置栈的实际位置
}

RiscvISA::IntReg
RiscvProcess::getSyscallArg(ThreadContext *tc, int &i)
{
    // If a larger index is requested than there are syscall argument
    // registers, return 0
    RiscvISA::IntReg retval = 0;
    if (i < SyscallArgumentRegs.size())
        retval = tc->readIntReg(SyscallArgumentRegs[i]);
    i++;
    return retval;
	// 首先SyscallArgumentRegs是一个全局向量变量，其大小经过测试恒为7
	// 如果给出的编号超过了6，那么就会返回0数值表示无效索引（实际上并不会触发）
}

void
RiscvProcess::setSyscallArg(ThreadContext *tc, int i, RiscvISA::IntReg val)
{
    tc->setIntReg(SyscallArgumentRegs[i], val);
	// 进行系统调用参数的设置操作
}

void
RiscvProcess::setSyscallReturn(ThreadContext *tc, SyscallReturn sysret)
{
    if (sysret.successful()) {
        // no error
        tc->setIntReg(SyscallPseudoReturnReg, sysret.returnValue());
    } else {
        // got an error, return details
        tc->setIntReg(SyscallPseudoReturnReg, sysret.errnoValue());
    }
	// 根据系统调用的返回情况，向系统调用返回寄存器中写入返回数值或者错误标号
	// 如果成功返回，那么sysret.errnoValue()处理会出现错误！
}
