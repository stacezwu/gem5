/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
 * Copyright (c) 2020 Barkhausen Institut
 * Copyright (c) 2021 Huawei International
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
 */

#ifndef __ARCH_STRAIGHT_TLB_HH__
#define __ARCH_STRAIGHT_TLB_HH__

#include <list>

#include "arch/generic/tlb.hh"
#include "arch/straight/isa.hh"
#include "arch/straight/pagetable.hh"
#include "arch/straight/pma_checker.hh"

#include "arch/straight/regs/misc.hh"
#include "arch/straight/utility.hh"
#include "base/statistics.hh"
#include "mem/request.hh"
#include "params/StraightTLB.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class ThreadContext;

/* To maintain compatibility with other architectures, we'll
   simply create an ITLB and DTLB that will point to the real TLB */
namespace StraightISA {

class Walker;

class TLB : public BaseTLB
{
    typedef std::list<TlbEntry *> EntryList;

  protected:
    size_t size;
    std::vector<TlbEntry> tlb;  // our TLB
    TlbEntryTrie trie;          // for quick access
    EntryList freeList;         // free entries
    uint64_t lruSeq;

    Walker *walker;

    struct TlbStats : public statistics::Group
    {
        TlbStats(statistics::Group *parent);

        statistics::Scalar readHits;
        statistics::Scalar readMisses;
        statistics::Scalar readAccesses;
        statistics::Scalar writeHits;
        statistics::Scalar writeMisses;
        statistics::Scalar writeAccesses;

        statistics::Formula hits;
        statistics::Formula misses;
        statistics::Formula accesses;
    } stats;

  public:
    PMAChecker *pma;
    PMP *pmp;

  public:
    typedef StraightTLBParams Params;
    TLB(const Params &p);

    Walker *getWalker();

    void takeOverFrom(BaseTLB *old) override {}

    TlbEntry *insert(Addr vpn, const TlbEntry &entry);
    void flushAll() override;
    void demapPage(Addr vaddr, uint64_t asn) override;

    Fault checkPermissions(STATUS status, PrivilegeMode pmode, Addr vaddr,
                           BaseMMU::Mode mode, PTESv39 pte);
    Fault createPagefault(Addr vaddr, BaseMMU::Mode mode);

    PrivilegeMode getMemPriv(ThreadContext *tc, BaseMMU::Mode mode);

    // Checkpointing
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /**
     * Get the table walker port. This is used for
     * migrating port connections during a CPU takeOverFrom()
     * call. For architectures that do not have a table walker,
     * NULL is returned, hence the use of a pointer rather than a
     * reference. For RISC-V this method will always return a valid
     * port pointer.
     *
     * @return A pointer to the walker port
     */
    Port *getTableWalkerPort() override;

    Addr translateWithTLB(Addr vaddr, uint16_t asid, BaseMMU::Mode mode);

    Fault translateAtomic(const RequestPtr &req,
                          ThreadContext *tc, BaseMMU::Mode mode) override;
    void translateTiming(const RequestPtr &req, ThreadContext *tc,
                         BaseMMU::Translation *translation,
                         BaseMMU::Mode mode) override;
    Fault translateFunctional(const RequestPtr &req, ThreadContext *tc,
                              BaseMMU::Mode mode) override;
    Fault finalizePhysical(const RequestPtr &req, ThreadContext *tc,
                           BaseMMU::Mode mode) const override;

  private:
    uint64_t nextSeq() { return ++lruSeq; }

    TlbEntry *lookup(Addr vpn, uint16_t asid, BaseMMU::Mode mode, bool hidden);

    void evictLRU();
    void remove(size_t idx);

    Fault translate(const RequestPtr &req, ThreadContext *tc,
                    BaseMMU::Translation *translation, BaseMMU::Mode mode,
                    bool &delayed);
    Fault doTranslate(const RequestPtr &req, ThreadContext *tc,
                      BaseMMU::Translation *translation, BaseMMU::Mode mode,
                      bool &delayed);
};

} // namespace StraightISA
} // namespace gem5

#endif // __STRAIGHT_MEMORY_HH__
