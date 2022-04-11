/*
 * Copyright (c) 2015 RISC-V Foundation
 * Copyright (c) 2017 The University of Virginia
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

#define __ARCH_STRAIGHT_INSTS_AMO_HH__
#ifndef __ARCH_STRAIGHT_INSTS_AMO_HH__

#include <string>

#include "arch/straight/insts/mem.hh"
#include "arch/straight/insts/static_inst.hh"
#include "cpu/static_inst.hh"

namespace gem5
{

namespace StraightISA
{

// memfence micro instruction
class MemFenceMicro : public StraightMicroInst
{
  public:
    MemFenceMicro(ExtMachInst _machInst, OpClass __opClass)
        : StraightMicroInst("fence", _machInst, __opClass)
    { }
  protected:
    using StraightMicroInst::StraightMicroInst;

    Fault execute(ExecContext *, Trace::InstRecord *) const override;
    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

// load-reserved
class LoadReserved : public StraightMacroInst
{
  protected:
    using StraightMacroInst::StraightMacroInst;

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class LoadReservedMicro : public StraightMicroInst
{
  protected:
    Request::Flags memAccessFlags;
    using StraightMicroInst::StraightMicroInst;

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

// store-cond
class StoreCond : public StraightMacroInst
{
  protected:
    using StraightMacroInst::StraightMacroInst;

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class StoreCondMicro : public StraightMicroInst
{
  protected:
    Request::Flags memAccessFlags;
    using StraightMicroInst::StraightMicroInst;

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

// AMOs
class AtomicMemOp : public StraightMacroInst
{
  protected:
    using StraightMacroInst::StraightMacroInst;

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

class AtomicMemOpMicro : public StraightMicroInst
{
  protected:
    Request::Flags memAccessFlags;
    using StraightMicroInst::StraightMicroInst;

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;
};

/**
 * A generic atomic op class
 */

template<typename T>
class AtomicGenericOp : public TypedAtomicOpFunctor<T>
{
  public:
    AtomicGenericOp(T _a, std::function<void(T*,T)> _op)
      : a(_a), op(_op) { }
    AtomicOpFunctor* clone() { return new AtomicGenericOp<T>(*this); }
    void execute(T *b) { op(b, a); }
  private:
    T a;
    std::function<void(T*,T)> op;
};

} // namespace StraightISA
} // namespace gem5

#endif // __ARCH_STRAIGHT_INSTS_AMO_HH__
