
/*
 * Copyright (c) 2015 RISC-V Foundation
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
 */

#ifndef __ARCH_STRAIGHT_STATIC_INST_HH__
#define __ARCH_STRAIGHT_STATIC_INST_HH__

#include <string>

#include "arch/straight/pcstate.hh"
#include "arch/straight/types.hh"
#include "arch/straight/rpstate.hh"
#include "cpu/exec_context.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
// #include "cpu/simple_thread.hh"
#include "mem/packet.hh"

namespace gem5
{

namespace StraightISA
{

/**
 * Base class for all RISC-V static instructions.
 */
class StraightStaticInst : public StaticInst
{
  protected:
    // RPState *_rp_cache = 0; 

    StraightStaticInst(const char *_mnemonic, ExtMachInst _machInst,
            OpClass __opClass) :
        StaticInst(_mnemonic, __opClass), machInst(_machInst)
    {}

  public:
    ExtMachInst machInst;

    void 
    advanceRP(RPStateBase &rp) {

        RPState rp_cache = rp.as<RPState>();
        // translateDestReg(rp_cache);
        // translateSrcReg(rp_cache);

        rp.advance();

        // RPState rp_cache = rp.as<RPState>();
        // translateDestReg(rp_cache);
        // translateSrcReg(rp_cache);
    }

    void
    advanceRP(ThreadContext *tc) override
    {
        std::cout << "opclass" << _opClass << std::endl;
        std::cout  << "StraightStaticInst::advanceRP" << std::endl;

        RPState rp = tc->rpState().as<RPState>();
        // translateDestReg(rp);
        // translateSrcReg(rp);
        
        rp.advance();
        tc->rpState(rp);
    }

    // const RegId &destRegIdx(int i) const {
    //     return destRegIdx(i);
    // }
    
    // virtual void 
    // translateReg(RPStateBase &RP){
    //     ;
    // }

    virtual void 
    translateDestReg(RPStateBase &RP) override{;}

    // const RegId &srcRegIdx(int i) const {
    //     return srcRegIdx(i);
    // }

    virtual void 
    translateSrcReg(RPStateBase &RP) override {;}

    void
    advancePC(PCStateBase &pc) const override
    {
        pc.as<PCState>().advance();
    }

    void
    advancePC(ThreadContext *tc) const override
    {
        PCState pc = tc->pcState().as<PCState>();
        pc.advance();
        tc->pcState(pc);
    }

    std::unique_ptr<PCStateBase>
    buildRetPC(const PCStateBase &cur_pc,
            const PCStateBase &call_pc) const override
    {
        PCStateBase *ret_pc_ptr = call_pc.clone();
        auto &ret_pc = ret_pc_ptr->as<PCState>();
        ret_pc.advance();
        ret_pc.pc(cur_pc.as<PCState>().npc());
        return std::unique_ptr<PCStateBase>{ret_pc_ptr};
    }

    size_t
    asBytes(void *buf, size_t size) override
    {
        return simpleAsBytes(buf, size, machInst);
    }
};

/**
 * Base class for all RISC-V Macroops
 */
class StraightMacroInst : public StraightStaticInst
{
  protected:
    std::vector<StaticInstPtr> microops;

    StraightMacroInst(const char *mnem, ExtMachInst _machInst,
                   OpClass __opClass) :
            StraightStaticInst(mnem, _machInst, __opClass)
    {
        flags[IsMacroop] = true;
    }

    ~StraightMacroInst() { microops.clear(); }

    StaticInstPtr
    fetchMicroop(MicroPC upc) const override
    {
        return microops[upc];
    }

    Fault
    initiateAcc(ExecContext *xc, Trace::InstRecord *traceData) const override
    {
        panic("Tried to execute a macroop directly!\n");
    }

    Fault
    completeAcc(PacketPtr pkt, ExecContext *xc,
                Trace::InstRecord *traceData) const override
    {
        panic("Tried to execute a macroop directly!\n");
    }

    Fault
    execute(ExecContext *xc, Trace::InstRecord *traceData) const override
    {
        panic("Tried to execute a macroop directly!\n");
    }

  public: 
    // virtual void 
    // translateSrcReg() override{
    //     ;
    // } 
};

/**
 * Base class for all RISC-V Microops
 */
class StraightMicroInst : public StraightStaticInst
{
  protected:
    StraightMicroInst(const char *mnem, ExtMachInst _machInst,
                   OpClass __opClass) :
            StraightStaticInst(mnem, _machInst, __opClass)
    {
        flags[IsMicroop] = true;
    }

    void advancePC(PCStateBase &pcState) const override;
    void advancePC(ThreadContext *tc) const override;
  public: 
    // virtual void 
    // translateSrcReg() override{
    //     ;
    // } 
};

} // namespace StraightISA
} // namespace gem5

#endif // __ARCH_STRAIGHT_STATIC_INST_HH__

