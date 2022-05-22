/*
 * Copyright (c) 2012 Google
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

#ifndef __ARCH_STRAIGHT_DECODER_HH__
#define __ARCH_STRAIGHT_DECODER_HH__

#include "arch/generic/decode_cache.hh"
#include "arch/generic/decoder.hh"
#include "arch/straight/types.hh"
#include "base/logging.hh"
#include "base/types.hh"
#include "cpu/static_inst.hh"
#include "debug/Decode.hh"
#include "params/StraightDecoder.hh"
#include <vector>

namespace gem5
{

namespace StraightISA
{

class ISA;
class Decoder : public InstDecoder
{
  private:
    decode_cache::InstMap<ExtMachInst> instMap;

    std::vector<ExtMachInst> instHandcode = {
      0x0000000000100013,
      0x0000000000200013,
      0x0000000000208033,
      0x0000000000208033,
      0x0000000000208033,
      0x0000000000208033,
      0x0000000000208033
    };

    bool aligned;
    bool mid;

  protected:
    //The extended machine instruction being generated
    ExtMachInst emi;
    // STRAIGHT instructions are 64 bits
    uint64_t machInst;
    //uint32_t machInst;

    StaticInstPtr decodeInst(ExtMachInst mach_inst);

    /// Decode a machine instruction.
    /// @param mach_inst The binary instruction to decode.
    /// @retval A pointer to the corresponding StaticInst object.
    StaticInstPtr decode(ExtMachInst mach_inst, Addr addr);

    StaticInstPtr decode(ExtMachInst mach_inst, Addr addr, RPStateBase &_next_rp);

  public:
    Decoder(const StraightDecoderParams &p) : InstDecoder(p, &machInst)
    {
        reset();
    }

    void reset() override;

    inline bool compressed(ExtMachInst inst) { return (inst & 0x3) < 0x3; }

    //Use this to give data to the decoder. This should be used
    //when there is control flow.
    void moreBytes(const PCStateBase &pc, Addr fetchPC) override;

    StaticInstPtr decode(PCStateBase &nextPC) override;
    StaticInstPtr decode(PCStateBase &nextPC, Counter numInst) override;

    StaticInstPtr decode(PCStateBase &_next_pc, RPStateBase &_next_rp) override;
};

} // namespace StraightISA
} // namespace gem5

#endif // __ARCH_STRAIGHT_DECODER_HH__
