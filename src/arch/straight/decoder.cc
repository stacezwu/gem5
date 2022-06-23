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
 */

#include "arch/straight/decoder.hh"
#include "arch/straight/types.hh"
#include "base/bitfield.hh"
#include "debug/Decode.hh"

namespace gem5
{

namespace StraightISA
{

void Decoder::reset()
{
    aligned = true;
    mid = false;
    emi = 0;
}

void
Decoder::moreBytes(const PCStateBase &pc, Addr fetchPC)
{
    // The MSB of the upper and lower halves of a machine instruction.
    // constexpr size_t max_bit = sizeof(machInst) * 8 - 1;
    // constexpr size_t mid_bit = sizeof(machInst) * 4 - 1;

    std::cout << " sizeof(machInst): " <<  sizeof(machInst) << std::endl;
    printf("moreBytes machInst: 0x%16x", machInst);
    printf("moreBytes emi: 0x%16x", emi);
    auto inst = letoh(machInst);
    emi = inst;
    instDone = true;
    DPRINTF(Decode, "Requesting bytes 0x%08x from address %#x\n", inst,
            fetchPC);
    printf("Instruction fetched: %#x", emi);

    // bool aligned = pc.instAddr() % sizeof(machInst) == 0;
    // if (aligned) {
    //     emi = inst;
    //     if (compressed(emi))
    //         emi = bits(emi, mid_bit, 0);
    //     outOfBytes = !compressed(emi);
    //     instDone = true;
    // } else {
    //     if (mid) {
    //         assert(bits(emi, max_bit, mid_bit + 1) == 0);
    //         replaceBits(emi, max_bit, mid_bit + 1, inst);
    //         mid = false;
    //         outOfBytes = false;
    //         instDone = true;
    //     } else {
    //         emi = bits(inst, max_bit, mid_bit + 1);
    //         mid = !compressed(emi);
    //         outOfBytes = true;
    //         instDone = compressed(emi);
    //     }
    // }
}

StaticInstPtr
Decoder::decode(ExtMachInst mach_inst, Addr addr)
{
    DPRINTF(Decode, "Decoding instruction 0x%llx at address %#x\n",
            mach_inst, addr);

    // StaticInstPtr &si = instMap[mach_inst];
    // if (!si)
    //     si = decodeInst(mach_inst);
    StaticInstPtr si = decodeInst(mach_inst);

    DPRINTF(Decode, "Decode: Decoded %s instruction: %#x\n",
            si->getName(), mach_inst);
    return si;
}

StaticInstPtr
Decoder::decode(PCStateBase &_next_pc)
{
    if (!instDone)
        return nullptr;
    instDone = false;

    auto &next_pc = _next_pc.as<PCState>();

    if (compressed(emi)) {
        std::cout << "is compressed(emi): maybe this is where the problem is" << std::endl;
        next_pc.npc(next_pc.instAddr() + sizeof(machInst) / 2);
        next_pc.compressed(true);
    } else {
        std::cout << "not compressed probably correct" << std::endl;
        next_pc.npc(next_pc.instAddr() + sizeof(machInst));
        next_pc.compressed(false);
    }

    return decode(emi, next_pc.instAddr());
}

StaticInstPtr
Decoder::decode(ExtMachInst mach_inst, Addr addr, RPStateBase &_next_rp)
{
    DPRINTF(Decode, "Decoding instruction 0x%llx at address %#x\n",
            mach_inst, addr);

    // StaticInstPtr &si = instMap[mach_inst];
    // if (!si)
    //     si = decodeInst(mach_inst);
    StaticInstPtr si = decodeInst(mach_inst);

    // si->translateSrcReg(_next_rp);
    // si->translateDestReg(_next_rp);

    DPRINTF(Decode, "Decode: Decoded %s instruction: %#x\n",
            si->getName(), mach_inst);
    return si;
}

StaticInstPtr
Decoder::decode(PCStateBase &_next_pc, RPStateBase &_next_rp)
{
    if (!instDone)
        return nullptr;
    instDone = false;

    auto &next_pc = _next_pc.as<PCState>();

    if (compressed(emi)) {
        next_pc.npc(next_pc.instAddr() + sizeof(machInst) / 2);
        next_pc.compressed(true);
    } else {
        next_pc.npc(next_pc.instAddr() + sizeof(machInst));
        next_pc.compressed(false);
    }

    return decode(emi, next_pc.instAddr(), _next_rp);
}

StaticInstPtr
Decoder::decode(PCStateBase &_next_pc, Counter numInst)
{
    if (!instDone)
        return nullptr;
    instDone = false;

    auto &next_pc = _next_pc.as<PCState>();

    if (compressed(emi)) {
        next_pc.npc(next_pc.instAddr() + sizeof(machInst) / 2);
        next_pc.compressed(true);
    } else {
        next_pc.npc(next_pc.instAddr() + sizeof(machInst));
        next_pc.compressed(false);
    }
    // emi = instHandcode[numInst];

    return decode(emi, next_pc.instAddr());
}

} // namespace StraightISA
} // namespace gem5
