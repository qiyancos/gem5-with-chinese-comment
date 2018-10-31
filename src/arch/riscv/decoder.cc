/*
 * Copyright (c) 2012 Google
 * Copyright (c) The University of Virginia
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
 *          Alec Roelke
 */

#include "arch/riscv/decoder.hh"
#include "arch/riscv/types.hh"
#include "debug/Decode.hh"

namespace RiscvISA
{

static const MachInst LowerBitMask = (1 << sizeof(MachInst) * 4) - 1;
/* 获取对应MachInst的低一半bit的数据需要的Mask，对于32bit就是0x0000FFFF */
static const MachInst UpperBitMask = LowerBitMask << sizeof(MachInst) * 4;
/* 获取对应MachInst的高一半bit的数据需要的Mask，对于32bit就是0xFFFF0000 */

void Decoder::reset()
{
    aligned = true;
    mid = false;
    more = true;
    emi = 0;
    instDone = false;
}

void
Decoder::moreBytes(const PCState &pc, Addr fetchPC, MachInst inst)
{
	/* 
	需要注意，这里的inst对应的地址并不一定和PC对应，
	inst指令一定是按照对齐PC处理的，因此不对齐的PC指向的是inst的中间
	*/
    DPRINTF(Decode, "Requesting bytes 0x%08x from address %#x\n", inst,
            fetchPC);

    bool aligned = pc.pc() % sizeof(MachInst) == 0;
	/* 判断pc的地址是否对齐于MachInst的大小 */
    if (aligned) {
        emi = inst;
        if (compressed(emi))
            emi &= LowerBitMask;
        more = !compressed(emi);
        instDone = true;
		/* 
		如果pc对齐，则确定inst低两位是否为11；
		如果为11表示指令压缩，这里只会将指令inst的后半部分（低地址的内容）取出来给emi
		如果这里是压缩指令则more置为true表示还有指令，否则more置为false表示没有指令
		处理结束后instDone置为true 
								|fetchPC
								|inst|
								|pc
		压缩指令：  	|_|
		地址：          	|___|___|
		*/
    } else {
        if (mid) {
            assert((emi & UpperBitMask) == 0);
            emi |= (inst & LowerBitMask) << sizeof(MachInst)*4;
            mid = false;
            more = false;
            instDone = true;
			/* 
			mid默认是false状态，只有按照压缩指令处理非对齐PC出现错误的情况才会到达这里；
			（即PC是不对齐的，同时对应的指令也不是压缩指令触发）
			首先调用assert判断，如果emi的高一半内容不为0，说明并不是因为下面处理进入该步骤的，
			则报错退出，因为经过下面的处理emi的高一半一定是0
			进入这一步只需要将没有获取完全的另一部分获取到就可以了，
			根据猜测此处的inst和下面的inst应该是不一样的，但是pc一致（mid为true时产生这种效果）。
			指令
											|fetchPC
											|inst|
										|pc
			指令：  				|___|
			地址：          	|___|___|
			*/
        } else {
            emi = (inst & UpperBitMask) >> sizeof(MachInst)*4;
            mid = !compressed(emi);
            more = true;
            instDone = compressed(emi);
			/* 
			如果当前pc没有对齐，同时mid属于false，那么进行下面处理：
			首先按照压缩指令处理，取指令inst的其前半部分（实际对应于不对齐pc后的半个指令）
			作为实际指令的后半部分赋给emi；more的设置在这里没什么用处；
			如果当前指令是压缩指令，处理完后PC将会对齐，instDone就会被置为true
			若指令并非压缩，那么当前的处理就是不正确的，instDone会被置为false，进入上面的处理
									|fetchPC
									|inst|
										|pc
			指令：  				|___|
			地址：          	|___|___|
			*/
        }
    }
}

StaticInstPtr
Decoder::decode(ExtMachInst mach_inst, Addr addr)
{
    DPRINTF(Decode, "Decoding instruction 0x%08x at address %#x\n",
            mach_inst, addr);
	
    if (instMap.find(mach_inst) != instMap.end())
        return instMap[mach_inst];
		/*
		这里尝试在instMap中查找对应与mach_inst的指令相关信息
		如果没有查找查找到则会返回一个end迭代器；
		如果查找到则会直接返回vector instMap的对应元素；
		*/
    else {
        StaticInstPtr si = decodeInst(mach_inst);
        instMap[mach_inst] = si;
        return si;
		/* 
		这里是没有在instMap中查找到相应指令变换的情况
		decodeInst没有在这里实现，实际上也并不会进入到这一步。
		*/
    }
}

StaticInstPtr
Decoder::decode(RiscvISA::PCState &nextPC)
{
    if (!instDone)
        return nullptr;
    instDone = false;
	/* 
	该处理表示当上面的指令处理没有结束则该函数什么都不做
	如果上面已经完整的处理完了一个指令，
	那么初始化instDone为false并更新指令PC以便开始新的处理
	 */
    if (compressed(emi)) {
        nextPC.npc(nextPC.instAddr() + sizeof(MachInst) / 2);
    } else {
        nextPC.npc(nextPC.instAddr() + sizeof(MachInst));
    }
	/* 该处理根据上一个指令的长度更新nextPC的npc数值为新指令的起始PC */
    return decode(emi, nextPC.instAddr());
	/* 
	这里返回的是之前获得的指令对应的instMap元素
	经过测试，可以确定函数StaticInstPtr Decoder::decode(ExtMachInst mach_inst, Addr addr)
	只在这里调用了一次
	*/
}

}
