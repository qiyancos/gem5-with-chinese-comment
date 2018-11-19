/*
 * Copyright (c) 2012-2013 ARM Limited
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
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#include "cpu/o3/fu_pool.hh"

#include <sstream>

#include "cpu/func_unit.hh"

using namespace std;

////////////////////////////////////////////////////////////////////////////
//
//  A pool of function units
//

inline void
FUPool::FUIdxQueue::addFU(int fu_idx)
{
    funcUnitsIdx.push_back(fu_idx);
    ++size;
}

inline int
FUPool::FUIdxQueue::getFU()
{
    int retval = funcUnitsIdx[idx++];

    if (idx == size)
        idx = 0;

    return retval;
}

FUPool::~FUPool()
{
    fuListIterator i = funcUnits.begin();
    fuListIterator end = funcUnits.end();
    for (; i != end; ++i)
        delete *i;
}


// Constructor
FUPool::FUPool(const Params *p)
    : SimObject(p)
{
    numFU = 0;

    funcUnits.clear();

    maxOpLatencies.fill(Cycles(0));
    pipelined.fill(true);

    //
    //  Iterate through the list of FUDescData structures
    //
    const vector<FUDesc *> &paramList =  p->FUList;
    for (FUDDiterator i = paramList.begin(); i != paramList.end(); ++i) {
        //  Don't bother with this if we're not going to create any FU's
		// 遍历FUList中的所有设置的FU描述，生成并将他们加到FUIdxQueue中
        if ((*i)->number) {
			// 这里的number表示该类型FU的实例化单元个数，如果为0相当于没有
			// 支持该种类的FU
            //
            //  Create the FuncUnit object from this structure
            //   - add the capabilities listed in the FU's operation
            //     description
            //
            //  We create the first unit, then duplicate it as needed
            //
            FuncUnit *fu = new FuncUnit;
			// 生成一个新的FU

            OPDDiterator j = (*i)->opDescList.begin();
            OPDDiterator end = (*i)->opDescList.end();
            for (; j != end; ++j) {
				// 将该FU支持的操作设置到新的FU中
                // indicate that this pool has this capability
                capabilityList.set((*j)->opClass);
				// 设置对应的opclass标志位表示该类型有FU支持运算

                // Add each of the FU's that will have this capability to the
                // appropriate queue.
                for (int k = 0; k < (*i)->number; ++k)
                    fuPerCapList[(*j)->opClass].addFU(numFU + k);
				// 按照实例化数量要求增加索引到FUIdxQueue中，numFU+k操作在
				// 后面完成，此处由于需要用来生成索引，因此没有递增

                // indicate that this FU has the capability
                fu->addCapability((*j)->opClass, (*j)->opLat, (*j)->pipelined);
				// 设置新生成FU的操作类型、操作延迟以及是否流水线化三个参数
				
                if ((*j)->opLat > maxOpLatencies[(*j)->opClass])
                    maxOpLatencies[(*j)->opClass] = (*j)->opLat;
                if (!(*j)->pipelined)
                    pipelined[(*j)->opClass] = false;
				
				// 设置该操作类型的最大延迟以及是否流水化操作
            }

            numFU++;

            //  Add the appropriate number of copies of this FU to the list
            fu->name = (*i)->name() + "(0)";
            funcUnits.push_back(fu);
			// 添加生成的FU指针到funcUnits队列中

            for (int c = 1; c < (*i)->number; ++c) {
                ostringstream s;
                numFU++;
                FuncUnit *fu2 = new FuncUnit(*fu);
				
				// 复制之前设置好的fu为新的FU，按照数量生成足够的FU实例
				
                s << (*i)->name() << "(" << c << ")";
                fu2->name = s.str();
                funcUnits.push_back(fu2);
				// 设置每个FU的辨识名称并添加该FU到funcUnits中
            }
        }
    }

    unitBusy.resize(numFU);
	// 设置busy标志位大小
    for (int i = 0; i < numFU; i++) {
        unitBusy[i] = false;
		// 初始化unitBusy
    }
}

int
FUPool::getUnit(OpClass capability)
{
    //  If this pool doesn't have the specified capability,
    //  return this information to the caller
    if (!capabilityList[capability])
        return -2;
	// 此处表示没有配置属于该操作类型的FU资源

    int fu_idx = fuPerCapList[capability].getFU();
    int start_idx = fu_idx;
	// 从对应类型的FUIdxQueue获取一个空闲的FU编号

    // Iterate through the circular queue if needed, stopping if we've reached
    // the first element again.
    while (unitBusy[fu_idx]) {
        fu_idx = fuPerCapList[capability].getFU();
        if (fu_idx == start_idx) {
            // No FU available
            return -1;
			// 这里表示遍历FUIdxQueue一整遍都没有空闲的FU资源
        }
    }
	// 由于FUIdxQueue没有设置尾指针，因此并不知道到底顶部指针idx对应的
	// FU是否空闲，因此此处尝试遍历整个FUIdxQueue获取一个空闲的FU

    assert(fu_idx < numFU);

    unitBusy[fu_idx] = true;
	// 设置获取到的FU为busy状态

    return fu_idx;
}

void
FUPool::freeUnitNextCycle(int fu_idx)
{
    assert(unitBusy[fu_idx]);
    unitsToBeFreed.push_back(fu_idx);
}

void
FUPool::processFreeUnits()
{
    while (!unitsToBeFreed.empty()) {
        int fu_idx = unitsToBeFreed.back();
        unitsToBeFreed.pop_back();

        assert(unitBusy[fu_idx]);

        unitBusy[fu_idx] = false;
    }
}

void
FUPool::dump()
{
    cout << "Function Unit Pool (" << name() << ")\n";
    cout << "======================================\n";
    cout << "Free List:\n";

    for (int i = 0; i < numFU; ++i) {
        if (unitBusy[i]) {
            continue;
        }

        cout << "  [" << i << "] : ";

        cout << funcUnits[i]->name << " ";

        cout << "\n";
    }

    cout << "======================================\n";
    cout << "Busy List:\n";
    for (int i = 0; i < numFU; ++i) {
        if (!unitBusy[i]) {
            continue;
        }

        cout << "  [" << i << "] : ";

        cout << funcUnits[i]->name << " ";

        cout << "\n";
    }
}

bool
FUPool::isDrained() const
{
    bool is_drained = true;
    for (int i = 0; i < numFU; i++)
        is_drained = is_drained && !unitBusy[i];

    return is_drained;
}

//

////////////////////////////////////////////////////////////////////////////
//
//  The SimObjects we use to get the FU information into the simulator
//
////////////////////////////////////////////////////////////////////////////

//
//    FUPool - Contails a list of FUDesc objects to make available
//

//
//  The FuPool object
//
FUPool *
FUPoolParams::create()
{
    return new FUPool(this);
}
