/*
 * Copyright (c) 2012-2014,2017 ARM Limited
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
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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

#ifndef __CPU_O3_LSQ_UNIT_HH__
#define __CPU_O3_LSQ_UNIT_HH__

#include <algorithm>
#include <cstring>
#include <map>
#include <queue>

#include "arch/generic/debugfaults.hh"
#include "arch/isa_traits.hh"
#include "arch/locked_mem.hh"
#include "arch/mmapped_ipr.hh"
#include "config/the_isa.hh"
#include "cpu/inst_seq.hh"
#include "cpu/timebuf.hh"
#include "debug/LSQUnit.hh"
#include "mem/packet.hh"
#include "mem/port.hh"

struct DerivO3CPUParams;

/**
 * Class that implements the actual LQ and SQ for each specific
 * thread.  Both are circular queues; load entries are freed upon
 * committing, while store entries are freed once they writeback. The
 * LSQUnit tracks if there are memory ordering violations, and also
 * detects partial load to store forwarding cases (a store only has
 * part of a load's data) that requires the load to wait until the
 * store writes back. In the former case it holds onto the instruction
 * until the dependence unit looks at it, and in the latter it stalls
 * the LSQ until the store writes back. At that point the load is
 * replayed.
 */
template <class Impl>
class LSQUnit {
  public:
    typedef typename Impl::O3CPU O3CPU;
    typedef typename Impl::DynInstPtr DynInstPtr;
    typedef typename Impl::CPUPol::IEW IEW;
    typedef typename Impl::CPUPol::LSQ LSQ;
    typedef typename Impl::CPUPol::IssueStruct IssueStruct;

  public:
    /** Constructs an LSQ unit. init() must be called prior to use. */
    LSQUnit();

    /** Initializes the LSQ unit with the specified number of entries. */
    void init(O3CPU *cpu_ptr, IEW *iew_ptr, DerivO3CPUParams *params,
            LSQ *lsq_ptr, unsigned maxLQEntries, unsigned maxSQEntries,
            unsigned id);

    /** Returns the name of the LSQ unit. */
    std::string name() const;

    /** Registers statistics. */
    void regStats();

    /** Sets the pointer to the dcache port. */
    void setDcachePort(MasterPort *dcache_port);

    /** Perform sanity checks after a drain. */
    void drainSanityCheck() const;

    /** Takes over from another CPU's thread. */
    void takeOverFrom();

    /** Ticks the LSQ unit, which in this case only resets the number of
     * used cache ports.
     * @todo: Move the number of used ports up to the LSQ level so it can
     * be shared by all LSQ units.
     */
    void tick() { usedStorePorts = 0; }

    /** Inserts an instruction. */
    void insert(DynInstPtr &inst);
    /** Inserts a load instruction. */
    void insertLoad(DynInstPtr &load_inst);
    /** Inserts a store instruction. */
    void insertStore(DynInstPtr &store_inst);

    /** Check for ordering violations in the LSQ. For a store squash if we
     * ever find a conflicting load. For a load, only squash if we
     * an external snoop invalidate has been seen for that load address
     * @param load_idx index to start checking at
     * @param inst the instruction to check
     */
    Fault checkViolations(int load_idx, DynInstPtr &inst);

    /** Check if an incoming invalidate hits in the lsq on a load
     * that might have issued out of order wrt another load beacuse
     * of the intermediate invalidate.
     */
    void checkSnoop(PacketPtr pkt);

    /** Executes a load instruction. */
    Fault executeLoad(DynInstPtr &inst);

    Fault executeLoad(int lq_idx) { panic("Not implemented"); return NoFault;}
	
    /** Executes a store instruction. */
    Fault executeStore(DynInstPtr &inst);

    /** Commits the head load. */
    void commitLoad();
    /** Commits loads older than a specific sequence number. */
    void commitLoads(InstSeqNum &youngest_inst);

    /** Commits stores older than a specific sequence number. */
    void commitStores(InstSeqNum &youngest_inst);

    /** Writes back stores. */
    void writebackStores();

    /** Completes the data access that has been returned from the
     * memory system. */
    void completeDataAccess(PacketPtr pkt);

    /** Clears all the entries in the LQ. */
    void clearLQ();

    /** Clears all the entries in the SQ. */
    void clearSQ();

    /** Resizes the LQ to a given size. */
    void resizeLQ(unsigned size);

    /** Resizes the SQ to a given size. */
    void resizeSQ(unsigned size);

    /** Squashes all instructions younger than a specific sequence number. */
    void squash(const InstSeqNum &squashed_num);

    /** Returns if there is a memory ordering violation. Value is reset upon
     * call to getMemDepViolator().
     */
    bool violation() { return memDepViolator; }

    /** Returns the memory ordering violator. */
    DynInstPtr getMemDepViolator();

    /** Returns the number of free LQ entries. */
    unsigned numFreeLoadEntries();

    /** Returns the number of free SQ entries. */
    unsigned numFreeStoreEntries();

    /** Returns the number of loads in the LQ. */
    int numLoads() { return loads; }

    /** Returns the number of stores in the SQ. */
    int numStores() { return stores; }

    /** Returns if either the LQ or SQ is full. */
    bool isFull() { return lqFull() || sqFull(); }

    /** Returns if both the LQ and SQ are empty. */
    bool isEmpty() const { return lqEmpty() && sqEmpty(); }

    /** Returns if the LQ is full. */
    bool lqFull() { return loads >= (LQEntries - 1); }

    /** Returns if the SQ is full. */
    bool sqFull() { return stores >= (SQEntries - 1); }

    /** Returns if the LQ is empty. */
    bool lqEmpty() const { return loads == 0; }

    /** Returns if the SQ is empty. */
    bool sqEmpty() const { return stores == 0; }

    /** Returns the number of instructions in the LSQ. */
    unsigned getCount() { return loads + stores; }

    /** Returns if there are any stores to writeback. */
    bool hasStoresToWB() { return storesToWB; }

    /** Returns the number of stores to writeback. */
    int numStoresToWB() { return storesToWB; }

    /** Returns if the LSQ unit will writeback on this cycle. */
    bool willWB() { return storeQueue[storeWBIdx].canWB &&
                        !storeQueue[storeWBIdx].completed &&
                        !isStoreBlocked; }

    /** Handles doing the retry. */
    void recvRetry();

  private:
    /** Reset the LSQ state */
    void resetState();

    /** Writes back the instruction, sending it to IEW. */
    void writeback(DynInstPtr &inst, PacketPtr pkt);

    /** Writes back a store that couldn't be completed the previous cycle. */
    void writebackPendingStore();

    /** Handles completing the send of a store to memory. */
    void storePostSend(PacketPtr pkt);

    /** Completes the store at the specified index. */
    void completeStore(int store_idx);

    /** Attempts to send a store to the cache. */
    bool sendStore(PacketPtr data_pkt);

    /** Increments the given store index (circular queue). */
    inline void incrStIdx(int &store_idx) const;
    /** Decrements the given store index (circular queue). */
    inline void decrStIdx(int &store_idx) const;
    /** Increments the given load index (circular queue). */
    inline void incrLdIdx(int &load_idx) const;
    /** Decrements the given load index (circular queue). */
    inline void decrLdIdx(int &load_idx) const;

  public:
    /** Debugging function to dump instructions in the LSQ. */
    void dumpInsts() const;

  private:
    /** Pointer to the CPU. */
    O3CPU *cpu;

    /** Pointer to the IEW stage. */
    IEW *iewStage;

    /** Pointer to the LSQ. */
    LSQ *lsq;

    /** Pointer to the dcache port.  Used only for sending. */
    MasterPort *dcachePort;

    /** Derived class to hold any sender state the LSQ needs. */
    class LSQSenderState : public Packet::SenderState
    {
      public:
        /** Default constructor. */
        LSQSenderState()
            : mainPkt(NULL), pendingPacket(NULL), idx(0), outstanding(1),
              isLoad(false), noWB(false), isSplit(false),
              pktToSend(false), cacheBlocked(false)
          { }

        /** Instruction who initiated the access to memory. */
        DynInstPtr inst;
        /** The main packet from a split load, used during writeback. */
        PacketPtr mainPkt;
        /** A second packet from a split store that needs sending. */
        PacketPtr pendingPacket;
        /** The LQ/SQ index of the instruction. */
        uint8_t idx;
        /** Number of outstanding packets to complete. */
        uint8_t outstanding;
        /** Whether or not it is a load. */
        bool isLoad;
        /** Whether or not the instruction will need to writeback. */
        bool noWB;
        /** Whether or not this access is split in two. */
        bool isSplit;
        /** Whether or not there is a packet that needs sending. */
        bool pktToSend;
        /** Whether or not the second packet of this split load was blocked */
        bool cacheBlocked;

        /** Completes a packet and returns whether the access is finished. */
        inline bool complete() { return --outstanding == 0; }
    };

    /** Writeback event, specifically for when stores forward data to loads. */
    class WritebackEvent : public Event {
      public:
        /** Constructs a writeback event. */
        WritebackEvent(DynInstPtr &_inst, PacketPtr pkt, LSQUnit *lsq_ptr);

        /** Processes the writeback event. */
        void process();

        /** Returns the description of this event. */
        const char *description() const;

      private:
        /** Instruction whose results are being written back. */
        DynInstPtr inst;

        /** The packet that would have been sent to memory. */
        PacketPtr pkt;

        /** The pointer to the LSQ unit that issued the store. */
        LSQUnit<Impl> *lsqPtr;
    };

  public:
    struct SQEntry {
        /** Constructs an empty store queue entry. */
        SQEntry()
            : inst(NULL), req(NULL), size(0),
              canWB(0), committed(0), completed(0)
        {
            std::memset(data, 0, sizeof(data));
        }

        ~SQEntry()
        {
            inst = NULL;
        }

        /** Constructs a store queue entry for a given instruction. */
        SQEntry(DynInstPtr &_inst)
            : inst(_inst), req(NULL), sreqLow(NULL), sreqHigh(NULL), size(0),
              isSplit(0), canWB(0), committed(0), completed(0), isAllZeros(0)
        {
            std::memset(data, 0, sizeof(data));
        }
        /** The store data. */
        char data[16];
        /** The store instruction. */
        DynInstPtr inst;
        /** The request for the store. */
        RequestPtr req;
        /** The split requests for the store. */
        RequestPtr sreqLow;
        RequestPtr sreqHigh;
        /** The size of the store. */
        uint8_t size;
        /** Whether or not the store is split into two requests. */
        bool isSplit;
        /** Whether or not the store can writeback. */
        bool canWB;
        /** Whether or not the store is committed. */
        bool committed;
        /** Whether or not the store is completed. */
        bool completed;
        /** Does this request write all zeros and thus doesn't
         * have any data attached to it. Used for cache block zero
         * style instructs (ARM DC ZVA; ALPHA WH64)
         */
        bool isAllZeros;
    };

  private:
    /** The LSQUnit thread id. */
    ThreadID lsqID;

    /** The store queue. */
    std::vector<SQEntry> storeQueue;

    /** The load queue. */
    std::vector<DynInstPtr> loadQueue;

    /** The number of LQ entries, plus a sentinel entry (circular queue).
     *  @todo: Consider having var that records the true number of LQ entries.
     */
    unsigned LQEntries;
    /** The number of SQ entries, plus a sentinel entry (circular queue).
     *  @todo: Consider having var that records the true number of SQ entries.
     */
    unsigned SQEntries;

    /** The number of places to shift addresses in the LSQ before checking
     * for dependency violations
     */
    unsigned depCheckShift;

    /** Should loads be checked for dependency issues */
    bool checkLoads;

    /** The number of load instructions in the LQ. */
    int loads;
    /** The number of store instructions in the SQ. */
    int stores;
    /** The number of store instructions in the SQ waiting to writeback. */
    int storesToWB;

    /** The index of the head instruction in the LQ. */
    int loadHead;
    /** The index of the tail instruction in the LQ. */
    int loadTail;

    /** The index of the head instruction in the SQ. */
    int storeHead;
    /** The index of the first instruction that may be ready to be
     * written back, and has not yet been written back.
     */
    int storeWBIdx;
    /** The index of the tail instruction in the SQ. */
    int storeTail;

    /// @todo Consider moving to a more advanced model with write vs read ports
    /** The number of cache ports available each cycle (stores only). */
    int cacheStorePorts;

    /** The number of used cache ports in this cycle by stores. */
    int usedStorePorts;

    //list<InstSeqNum> mshrSeqNums;

    /** Address Mask for a cache block (e.g. ~(cache_block_size-1)) */
    Addr cacheBlockMask;

    /** Wire to read information from the issue stage time queue. */
    typename TimeBuffer<IssueStruct>::wire fromIssue;

    /** Whether or not the LSQ is stalled. */
    bool stalled;
    /** The store that causes the stall due to partial store to load
     * forwarding.
     */
    InstSeqNum stallingStoreIsn;
    /** The index of the above store. */
    int stallingLoadIdx;

    /** The packet that needs to be retried. */
    PacketPtr retryPkt;

    /** Whehter or not a store is blocked due to the memory system. */
    bool isStoreBlocked;

    /** Whether or not a store is in flight. */
    bool storeInFlight;

    /** The oldest load that caused a memory ordering violation. */
    DynInstPtr memDepViolator;

    /** Whether or not there is a packet that couldn't be sent because of
     * a lack of cache ports. */
    bool hasPendingPkt;

    /** The packet that is pending free cache ports. */
    PacketPtr pendingPkt;

    /** Flag for memory model. */
    bool needsTSO;

    // Will also need how many read/write ports the Dcache has.  Or keep track
    // of that in stage that is one level up, and only call executeLoad/Store
    // the appropriate number of times.
    /** Total number of loads forwaded from LSQ stores. */
    Stats::Scalar lsqForwLoads;

    /** Total number of loads ignored due to invalid addresses. */
    Stats::Scalar invAddrLoads;

    /** Total number of squashed loads. */
    Stats::Scalar lsqSquashedLoads;

    /** Total number of responses from the memory system that are
     * ignored due to the instruction already being squashed. */
    Stats::Scalar lsqIgnoredResponses;

    /** Total number of memory ordering violations. */
    Stats::Scalar lsqMemOrderViolation;

    /** Total number of squashed stores. */
    Stats::Scalar lsqSquashedStores;

    /** Total number of software prefetches ignored due to invalid addresses. */
    Stats::Scalar invAddrSwpfs;

    /** Ready loads blocked due to partial store-forwarding. */
    Stats::Scalar lsqBlockedLoads;

    /** Number of loads that were rescheduled. */
    Stats::Scalar lsqRescheduledLoads;

    /** Number of times the LSQ is blocked due to the cache. */
    Stats::Scalar lsqCacheBlocked;

  public:
    /** Executes the load at the given index. */
    Fault read(Request *req, Request *sreqLow, Request *sreqHigh,
               int load_idx);

    /** Executes the store at the given index. */
    Fault write(Request *req, Request *sreqLow, Request *sreqHigh,
                uint8_t *data, int store_idx);

    /** Returns the index of the head load instruction. */
    int getLoadHead() { return loadHead; }
    /** Returns the sequence number of the head load instruction. */
    InstSeqNum getLoadHeadSeqNum()
    {
        if (loadQueue[loadHead]) {
            return loadQueue[loadHead]->seqNum;
        } else {
            return 0;
        }

    }

    /** Returns the index of the head store instruction. */
    int getStoreHead() { return storeHead; }
    /** Returns the sequence number of the head store instruction. */
    InstSeqNum getStoreHeadSeqNum()
    {
        if (storeQueue[storeHead].inst) {
            return storeQueue[storeHead].inst->seqNum;
        } else {
            return 0;
        }

    }

    /** Returns whether or not the LSQ unit is stalled. */
    bool isStalled()  { return stalled; }
};

template <class Impl>
Fault
LSQUnit<Impl>::read(Request *req, Request *sreqLow, Request *sreqHigh,
                    int load_idx)
{
    DynInstPtr load_inst = loadQueue[load_idx];
    assert(load_inst);
    assert(!load_inst->isExecuted());
	// 获取一个有效的指令并且保证该指令没有被执行过

    // Make sure this isn't a strictly ordered load
    // A bit of a hackish way to get strictly ordered accesses to work
    // only if they're at the head of the LSQ and are ready to commit
    // (at the head of the ROB too).
	
    if (req->isStrictlyOrdered() &&
        (load_idx != loadHead || !load_inst->isAtCommit())) {
		// 这里要求对应请求是一个按序处理的请求，但是不位于LQ顶部或者？？？
        iewStage->rescheduleMemInst(load_inst);
		// 由于违背按序规则执行，因此会被重新调度
        ++lsqRescheduledLoads;
        DPRINTF(LSQUnit, "Strictly ordered load [sn:%lli] PC %s\n",
                load_inst->seqNum, load_inst->pcState());

        // Must delete request now that it wasn't handed off to
        // memory.  This is quite ugly.  @todo: Figure out the proper
        // place to really handle request deletes.
        delete req;
        if (TheISA::HasUnalignedMemAcc && sreqLow) {
            delete sreqLow;
            delete sreqHigh;
        }
		// 相应的请求将被删除，再度按序执行的时候会重新生成
        return std::make_shared<GenericISA::M5PanicFault>(
            "Strictly ordered load [sn:%llx] PC %s\n",
            load_inst->seqNum, load_inst->pcState());
		// 返回一个尝试将严格按序执行指令乱序执行的错误
    }
	// 上面的处理保证要求严格按序执行的指令被按序执行，否则需要重新调度该指令

    // Check the SQ for any previous stores that might lead to forwarding
    int store_idx = load_inst->sqIdx;
	// 如果这里获取到的是-1，下面将不会进行SQ数据前递的扫描
	// 该sqIdx表明本load指令只会和SQ中在比sqIdx更老的store指令
	// 存在相关关系
    int store_size = 0;

    DPRINTF(LSQUnit, "Read called, load idx: %i, store idx: %i, "
            "storeHead: %i addr: %#x%s\n",
            load_idx, store_idx, storeHead, req->getPaddr(),
            sreqLow ? " split" : "");

    if (req->isLLSC()) {
        assert(!sreqLow);
        // Disable recording the result temporarily.  Writing to misc
        // regs normally updates the result, but this is not the
        // desired behavior when handling store conditionals.
		
        load_inst->recordResult(false);
        TheISA::handleLockedRead(load_inst.get(), req);
		// 处理一个带锁的读操作，这里除了读取数据
		// 还会记录一个flag和读取数据的地址
        load_inst->recordResult(true);
		// 标记指令表示该指令已经处理过了
    }
	// 在该load指令是LLSC中的LL部分进行的处理

    if (req->isMmappedIpr()) {
		// 这里表示请求处理的地址是一个映射到寄存器的地址
		// 一般来说这种映射的寄存器针对的是IO外设的控制寄存器
		// 这种地址一般是non-cacheable的！
        assert(!load_inst->memData);
        load_inst->memData = new uint8_t[64];
		// 这里生成了一个512bit/64Byte的元数据，对应了一个Cache Block

        ThreadContext *thread = cpu->tcBase(lsqID);
        Cycles delay(0);
        PacketPtr data_pkt = new Packet(req, MemCmd::ReadReq);
		// 生成一个用于传递数据请求的空Packet

        data_pkt->dataStatic(load_inst->memData);
		// 设置接收传输数据的容器指针
        if (!TheISA::HasUnalignedMemAcc || !sreqLow) {
			// 如果访存操作没有被拆分或者没有进行非对齐的内存访问
            delay = TheISA::handleIprRead(thread, data_pkt);
			// 由该函数处理Packet，返回对应操作模拟的延迟
        } else {
            assert(sreqLow->isMmappedIpr() && sreqHigh->isMmappedIpr());
			
            PacketPtr fst_data_pkt = new Packet(sreqLow, MemCmd::ReadReq);
            PacketPtr snd_data_pkt = new Packet(sreqHigh, MemCmd::ReadReq);
			// 进入这里只能说明非对齐的内存访问一定会被拆分成两部分

            fst_data_pkt->dataStatic(load_inst->memData);
            snd_data_pkt->dataStatic(load_inst->memData + sreqLow->getSize());
			// 很明显第一次访问填充的是load_inst->memData低一部分数据
			// 第二次访问load_inst->memData则填充其剩余部分的数据
			// 需要注意的是，这里两次load获得数据可能不是对半分的
			
            delay = TheISA::handleIprRead(thread, fst_data_pkt);
            Cycles delay2 = TheISA::handleIprRead(thread, snd_data_pkt);
			// 分别处理两个Packet并记录他们的延迟
			
            if (delay2 > delay)
                delay = delay2;
			// 两个请求我们认为是无关并行的，因此最后会取两个delay中更大
			// 的作为最后的实际延迟，即变量delay；delay2只是一个临时变量

            delete sreqLow;
            delete sreqHigh;
            delete fst_data_pkt;
            delete snd_data_pkt;
			// 在处理结束后删除Packet和Request
        }
        WritebackEvent *wb = new WritebackEvent(load_inst, data_pkt, this);
		// 生成一个新的WritebackEvent来记录发出的Packet，以便在对应延迟之后
		// 完成该Packet的写回操作
        cpu->schedule(wb, cpu->clockEdge(delay));
		// 由CPU在对应的delay周期后定时处理该Packet返回数据的写回操作
        return NoFault;
		// 对于这一类寄存器固定映射的内存，处理已经完毕了，立即返回
    }

	// 对于一般的load指令，在下面的操作中尝试进行数据前递
    while (store_idx != -1) {
        // End once we've reached the top of the LSQ
        if (store_idx == storeWBIdx) {
            break;
        }
		// 此处的限制是因为已经完成写回的store指令不可用来进行数据前递
		// 因此遍历的范围是SQ中尚未进行写回的指令

        // Move the index to one younger
        if (--store_idx < 0)
            store_idx += SQEntries;
		// 循环队列指针到达顶部以后的处理

        assert(storeQueue[store_idx].inst);

        store_size = storeQueue[store_idx].size;
		// 这里获得的应该是store指令需要存放的数据大小

        if (!store_size || storeQueue[store_idx].inst->strictlyOrdered() ||
            (storeQueue[store_idx].req &&
             storeQueue[store_idx].req->isCacheMaintenance())) {
            // Cache maintenance instructions go down via the store
            // path but they carry no data and they shouldn't be
            // considered for forwarding
            continue;
        }
		// 对于store没有存放数据，或者该store指令要求严格按序执行，
		// 或者该store指令是一个内存维护指令，不带有有效数据，则
		// 不进行前递操作

        assert(storeQueue[store_idx].inst->effAddrValid());
		// 检查确认store指令的目标地址有效

        // Check if the store data is within the lower and upper bounds of
        // addresses that the request needs.
        bool store_has_lower_limit =
            req->getVaddr() >= storeQueue[store_idx].inst->effAddr;
		// store起始地址比load更小
        bool store_has_upper_limit =
            (req->getVaddr() + req->getSize()) <=
            (storeQueue[store_idx].inst->effAddr + store_size);
		// store结束地址比load更大
        bool lower_load_has_store_part =
            req->getVaddr() < (storeQueue[store_idx].inst->effAddr + store_size);
		// load起始地址是否小于store结束地址
        bool upper_load_has_store_part =
            (req->getVaddr() + req->getSize()) >
            storeQueue[store_idx].inst->effAddr;
		// load结束地址大于store的起始地址
		// 检查store指令和当前load指令处理的地址范围是否存在交叠

        // If the store's data has all of the data needed and the load isn't
        // LLSC, we can forward.
        if (store_has_lower_limit && store_has_upper_limit && !req->isLLSC()){
			// 这里表示store指令存储数据包含了load需要的全部数据，且load不是LLSC
            // Get shift amount for offset into the store's data.
            int shift_amt = req->getVaddr() - storeQueue[store_idx].inst->effAddr;
			// 计算两个数据起始地址的差值
			
            // Allocate memory if this is the first time a load is issued.
            if (!load_inst->memData) {
                load_inst->memData = new uint8_t[req->getSize()];
            }
            if (storeQueue[store_idx].isAllZeros)
                memset(load_inst->memData, 0, req->getSize());
            else
                memcpy(load_inst->memData,
                    storeQueue[store_idx].data + shift_amt, req->getSize());
			// 拷贝store的数据到load数据容器中

            DPRINTF(LSQUnit, "Forwarding from store idx %i to load to "
                    "addr %#x\n", store_idx, req->getVaddr());

            PacketPtr data_pkt = new Packet(req, MemCmd::ReadReq);
            data_pkt->dataStatic(load_inst->memData);
			// 这里生成Packet仅仅是用于生成load指令需要的WritebackEvent
			
            WritebackEvent *wb = new WritebackEvent(load_inst, data_pkt, this);

            // We'll say this has a 1 cycle load-store forwarding latency
            // for now.
            // @todo: Need to make this a parameter.
            cpu->schedule(wb, curTick());
			// 由于load通过前递获得了数据，因此可以在本周期立即进行写回

            // Don't need to do anything special for split loads.
            if (TheISA::HasUnalignedMemAcc && sreqLow) {
                delete sreqLow;
                delete sreqHigh;
				// 删除拆分请求对应的Request
            }

            ++lsqForwLoads;
            return NoFault;
			// load的read操作通过前递处理完毕
        } else if (
                (!req->isLLSC() &&
                 ((store_has_lower_limit && lower_load_has_store_part) ||
				// load和store数据范围交叉，且store数据地址更低
                  (store_has_upper_limit && upper_load_has_store_part) ||
				// load和store数据范围交叉，且store数据地址更高
                  (lower_load_has_store_part && upper_load_has_store_part))) ||
				// load和store之间的数据范围存在交叉或者包含，其中包含
				// 是load包含store数据范围
				
				// 这里存在一个问题：最后一个条件包含了前两个条件的可能性
				// 前两个条件判断似乎是多余的？？？
                (req->isLLSC() &&
                 ((store_has_lower_limit || upper_load_has_store_part) &&
                  (store_has_upper_limit || lower_load_has_store_part)))) {
				// load是一个LLSC，而且store和load的数据范围存在包含或者交叉
				// 其中包含是load包含store
				
				// 其实这里也可以用前面非LLSC判断中的最后一行条件代替
            // This is the partial store-load forwarding case where a store
            // has only part of the load's data and the load isn't LLSC or
            // the load is LLSC and the store has all or part of the load's
            // data

            // If it's already been written back, then don't worry about
            // stalling on it.
            if (storeQueue[store_idx].completed) {
                panic("Should not check one of these");
                continue;
            }
			// 不应该处理到已经完成的指令

            // Must stall load and force it to retry, so long as it's the oldest
            // load that needs to do so.
            if (!stalled || (stalled && load_inst->seqNum <
                 loadQueue[stallingLoadIdx]->seqNum)) {
				// 当前LSQ没有stall或者
				// LSQ发生了stall但是当前处理的load指令比stall的load指令更老
                stalled = true;
                stallingStoreIsn = storeQueue[store_idx].inst->seqNum;
                stallingLoadIdx = load_idx;
            }
			// 这里会对LSQ进行stall，同时更新stall指令索引，这两个索引
			// 是专门针对于partial forwarding处理过程设置的
			// 但是为什么需要stall呢？？？

            // Tell IQ/mem dep unit that this instruction will need to be
            // rescheduled eventually
            iewStage->rescheduleMemInst(load_inst);
            load_inst->clearIssued();
			// 清除以发射状态以便在reschedule时进行第二次发射
            ++lsqRescheduledLoads;
			// 本次处理到前递已经说明该load指令之前从LSQ发射了，但是对于
			// partial forwarding来说，本次处理仅能获得部分数据其他内容需要
			// 重新发射直接从Mem获取，而该操作是通过reschedule实现的

            // Do not generate a writeback event as this instruction is not
            // complete.
            DPRINTF(LSQUnit, "Load-store forwarding mis-match. "
                    "Store idx %i to load addr %#x\n",
                    store_idx, req->getVaddr());

            // Must delete request now that it wasn't handed off to
            // memory.  This is quite ugly.  @todo: Figure out the
            // proper place to really handle request deletes.
            delete req;
            if (TheISA::HasUnalignedMemAcc && sreqLow) {
                delete sreqLow;
                delete sreqHigh;
            }
            return NoFault;
			// 删除拆分访存请求并返回
        }
    }
	// 走到这里说明该load指令没有可以前递的数据，需要直接访问Mem获取数据
    // If there's no forwarding case, then go access memory
    DPRINTF(LSQUnit, "Doing memory access for inst [sn:%lli] PC %s\n",
            load_inst->seqNum, load_inst->pcState());

    // Allocate memory if this is the first time a load is issued.
    if (!load_inst->memData) {
        load_inst->memData = new uint8_t[req->getSize()];
    }

    // if we the cache is not blocked, do cache access
    bool completedFirst = false;
    PacketPtr data_pkt = Packet::createRead(req);
    PacketPtr fst_data_pkt = NULL;
    PacketPtr snd_data_pkt = NULL;
	// 生成数据读取需要的Packet

    data_pkt->dataStatic(load_inst->memData);
	// 设置接收load获取数据的容器

    LSQSenderState *state = new LSQSenderState;
    state->isLoad = true;
    state->idx = load_idx;
    state->inst = load_inst;
    data_pkt->senderState = state;
	// 设置Packet的内容

    if (!TheISA::HasUnalignedMemAcc || !sreqLow) {
        // Point the first packet at the main data packet.
        fst_data_pkt = data_pkt;
    } else {
        // Create the split packets.
        fst_data_pkt = Packet::createRead(sreqLow);
        snd_data_pkt = Packet::createRead(sreqHigh);

        fst_data_pkt->dataStatic(load_inst->memData);
        snd_data_pkt->dataStatic(load_inst->memData + sreqLow->getSize());

        fst_data_pkt->senderState = state;
        snd_data_pkt->senderState = state;

        state->isSplit = true;
        state->outstanding = 2;
        state->mainPkt = data_pkt;
		// 访问请求拆分的信息会被记录到state中
    }
	// 对于非对齐的内存访问，会生成两个不同的访问请求Packet

    // For now, load throughput is constrained by the number of
    // load FUs only, and loads do not consume a cache port (only
    // stores do).
	// 在这里，load的唯一约束是load操作执行单元的数目，而非Cache接口
	// 的数目，因为Cache接口数目仅用于限制store操作
	
    // @todo We should account for cache port contention
    // and arbitrate between loads and stores.
    bool successful_load = true;
	// sendTimingReq似乎在处理成功的时候返回一个非0值，失败则返回0
    if (!dcachePort->sendTimingReq(fst_data_pkt)) {
        successful_load = false;
		// 从dachePort发送第一个请求，如果返回0则successful_load置为false
		// 返回非0值才能够处理拆分时的第二部分Packet数据请求，因为这里0
		// 返回值表示处理Packet时遇到了Cache Blocked的情况
    } else if (TheISA::HasUnalignedMemAcc && sreqLow) {
        // 对于对齐内存访问来说，不存在第二个访问Packet，进入这里说明
		// 进行了非对齐的内存访问，并且第一个Packet访问成功处理
		completedFirst = true;
		
        // The first packet was sent without problems, so send this one
        // too. If there is a problem with this packet then the whole
        // load will be squashed, so indicate this to the state object.
        // The first packet will return in completeDataAccess and be
        // handled there.
        // @todo We should also account for cache port contention
        // here.
        if (!dcachePort->sendTimingReq(snd_data_pkt)) {
			// 第二次Packet处理失败则会返回0
            // The main packet will be deleted in completeDataAccess.
            state->complete();
			// 递减state的outstanding数值，表示第一个Packet已经处理完毕了
            // Signify to 1st half that the 2nd half was blocked via state
            state->cacheBlocked = true;
            successful_load = false;
			// 设置相关状态表明Packet处理失败
        }
    }

    // If the cache was blocked, or has become blocked due to the access,
    // handle it.
	// 如果发送Packet处理失败的话，进行下面的squash处理
    if (!successful_load) {
        if (!sreqLow) {
            // Packet wasn't split, just delete main packet info
            delete state;
            delete req;
            delete data_pkt;
        }
		// 删除总的Request、Packet以及State

        if (TheISA::HasUnalignedMemAcc && sreqLow) {
            if (!completedFirst) {
                // Split packet, but first failed.  Delete all state.
                delete state;
                delete req;
                delete data_pkt;
                delete fst_data_pkt;
                delete snd_data_pkt;
                delete sreqLow;
                delete sreqHigh;
                sreqLow = NULL;
                sreqHigh = NULL;
            } else {
                // Can't delete main packet data or state because first packet
                // was sent to the memory system
                delete data_pkt;
                delete req;
                delete sreqHigh;
                delete snd_data_pkt;
                sreqHigh = NULL;
            }
        }
		// 上面对于拆分访问情况处理比较特殊，如果仅仅第一个访问Packet
		// 完成的话，则只会保留第一次访问生成的内容和state

        ++lsqCacheBlocked;
		// 很明显如果出现了错误，原因只可能是Cache Blocked，即使
		// 是TLB miss也可以当作Cache Blocked来看

        iewStage->blockMemInst(load_inst);
		// 记录因为Cache Blocked没有全部完成的指令，在Cache unblocked的时候
		// 这些指令会被放到replay List中准备重新执行

        // No fault occurred, even though the interface is blocked.
        return NoFault;
    }

    return NoFault;
}

template <class Impl>
Fault
LSQUnit<Impl>::write(Request *req, Request *sreqLow, Request *sreqHigh,
                     uint8_t *data, int store_idx)
{
    assert(storeQueue[store_idx].inst);

    DPRINTF(LSQUnit, "Doing write to store idx %i, addr %#x"
            " | storeHead:%i [sn:%i]\n",
            store_idx, req->getPaddr(), storeHead,
            storeQueue[store_idx].inst->seqNum);

    storeQueue[store_idx].req = req;
    storeQueue[store_idx].sreqLow = sreqLow;
    storeQueue[store_idx].sreqHigh = sreqHigh;
    unsigned size = req->getSize();
    storeQueue[store_idx].size = size;
    bool store_no_data = req->getFlags() & Request::STORE_NO_DATA;
    storeQueue[store_idx].isAllZeros = store_no_data;
	// 设置SQ中对应表项的相关信息
	
    assert(size <= sizeof(storeQueue[store_idx].data) || store_no_data);

    // Split stores can only occur in ISAs with unaligned memory accesses.  If
    // a store request has been split, sreqLow and sreqHigh will be non-null.
    if (TheISA::HasUnalignedMemAcc && sreqLow) {
        storeQueue[store_idx].isSplit = true;
    }
	// 设置是否是需要拆分的非对齐内存访问

    if (!(req->getFlags() & Request::CACHE_BLOCK_ZERO) && \
        !req->isCacheMaintenance())
        memcpy(storeQueue[store_idx].data, data, size);
	// 如果该store指令确实会存放一些有效数据，这些数据将会被记录到SQ的表项中

    // This function only writes the data to the store queue, so no fault
    // can happen here.
    return NoFault;
}

#endif // __CPU_O3_LSQ_UNIT_HH__
