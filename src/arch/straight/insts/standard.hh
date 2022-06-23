/*
 * Copyright (c) 2015 RISC-V Foundation
 * Copyright (c) 2017 The University of Virginia
 * Copyright (c) 2020 Barkhausen Institut
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

#ifndef __ARCH_STRAIGHT_STANDARD_INST_HH__
#define __ARCH_STRAIGHT_STANDARD_INST_HH__

#include <string>

#include "arch/straight/insts/bitfields.hh"
#include "arch/straight/insts/static_inst.hh"
#include "arch/straight/regs/misc.hh"
#include "cpu/exec_context.hh"
#include "cpu/static_inst.hh"

namespace gem5
{

namespace StraightISA
{

/**
 * Base class for operations that work only on registers
 */
class RegOp : public StraightStaticInst
{
  protected:
    // RegId _rp_tmp;
    RegIndex _RS1_cache;
    RegIndex _RS2_cache;

    // using StraightStaticInst::StraightStaticInst;
    RegOp(const char *mnem, MachInst _machInst, OpClass __opClass)
        : StraightStaticInst(mnem, _machInst, __opClass), _RS1_cache(0), _RS2_cache(0)
    {}

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;

  public:
    // using StraightStaticInst::advanceRP;
    virtual void 
    translateDestReg(RPStateBase &RP) override {
        std::cout << "RegOp::translateDestReg()" << std::endl;
        setDestRegIdx(_numDestRegs++, RegId(IntRegClass, RP.rp()));
    }

    virtual void 
    translateSrcReg(RPStateBase &RP) override {
        std::cout << "RegOp::translateSrcReg()" << std::endl;
        std::cout << "RP.rp() - _RS1_cache" << (RP.rp() - _RS1_cache) << std::endl;
        std::cout << "RP.rp() - _RS2_cache" << (RP.rp() - _RS2_cache) << std::endl;
        setSrcRegIdx(_numSrcRegs++, RegId(IntRegClass, RP.rp() - _RS1_cache));
        setSrcRegIdx(_numSrcRegs++, RegId(IntRegClass, RP.rp() - _RS2_cache));
        // setSrcRegIdx(_numSrcRegs++, RegId(IntRegClass, RP.rp() - _RS1_cache));
        // setSrcRegIdx(_numSrcRegs++, RegId(IntRegClass, RP.rp() - _RS2_cache));
    } 
    
    // virtual void 
    // translateReg(RPState &RP) override {

    //     setDestRegIdx(_numDestRegs++, RegId(IntRegClass, RP.rp()));
    //     std::cout << "RP.rp() - _RS1_cache" << (RP.rp() - _RS1_cache) << std::endl;
    //     std::cout << "RP.rp() - _RS2_cache" << (RP.rp() - _RS2_cache) << std::endl;
    //     setSrcRegIdx(_numSrcRegs++, RegId(IntRegClass, 0));
    //     setSrcRegIdx(_numSrcRegs++, RegId(IntRegClass, 0));
    // }
};

/**
 * Base class for operations with immediates (I is the type of immediate)
 */
template<typename I>
class ImmOp : public StraightStaticInst
{
  protected:
    I imm;
    RegIndex _RS1_cache;
    RegIndex _RS2_cache;

    ImmOp(const char *mnem, MachInst _machInst, OpClass __opClass)
        : StraightStaticInst(mnem, _machInst, __opClass), imm(0)
    {}
  public: 
    virtual void 
    translateDestReg(RPState &RP){
        std::cout << "RegOp::translateDestReg()" << std::endl;
        setDestRegIdx(_numDestRegs++, RegId(IntRegClass, RP.rp()));
    }

    virtual void 
    translateSrcReg(RPState &RP) {
        std::cout << "RegOp::translateSrcReg()" << std::endl;
        std::cout << "RP.rp() - _RS1_cache" << (RP.rp() - _RS1_cache) << std::endl;
        setSrcRegIdx(_numSrcRegs++, RegId(IntRegClass, RP.rp() - _RS1_cache));

    } 
};

/**
 * Base class for system operations
 */
class SystemOp : public StraightStaticInst
{
  protected:
    using StraightStaticInst::StraightStaticInst;

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
  public: 
    // virtual void 
    // translateSrcReg() override{
    //     ;
    // } 
};

/**
 * Base class for CSR operations
 */
class CSROp : public StraightStaticInst
{
  protected:
    uint64_t csr;
    uint64_t uimm;

    bool valid = false;
    RegIndex midx = 0;
    std::string csrName;
    uint64_t maskVal = 0;

    /// Constructor
    CSROp(const char *mnem, MachInst _machInst, OpClass __opClass)
        : StraightStaticInst(mnem, _machInst, __opClass),
            csr(FUNCT12), uimm(CSRIMM)
    {
        auto csr_data_it = CSRData.find(csr);
        if (csr_data_it == CSRData.end()) {
            valid = false;
        } else {
            valid = true;
            midx = csr_data_it->second.physIndex;
            csrName = csr_data_it->second.name;
            auto mask_it = CSRMasks.find(csr);
            if (mask_it == CSRMasks.end())
                maskVal = mask(64);
            else
                maskVal = mask_it->second;
        }

        if (csr == CSR_SATP) {
            flags[IsSquashAfter] = true;
        }
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
  public: 
    // virtual void 
    // translateSrcReg() override{
    //     ;
    // } 
};

} // namespace StraightISA
} // namespace gem5

#endif // __ARCH_STRAIGHT_STANDARD_INST_HH__