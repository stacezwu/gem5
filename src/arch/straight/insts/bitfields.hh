#ifndef __ARCH_STRAIGHT_BITFIELDS_HH__
#define __ARCH_STRAIGHT_BITFIELDS_HH__

#include "base/bitfield.hh"

#define CSRIMM  bits(machInst, 19, 15)
#define FUNCT12 bits(machInst, 31, 20)
#define IMM5    bits(machInst, 11, 7)
#define IMM7    bits(machInst, 31, 25)
#define IMMSIGN bits(machInst, 31)
#define OPCODE  bits(machInst, 6, 0)

#define AQ      bits(machInst, 26)
#define RP      bits(machInst, 11, 7)
#define RL      bits(machInst, 25)
#define RS1     bits(machInst, 36, 32, 19, 15)
#define RS2     bits(machInst, 41, 37, 24, 20)


#endif // __ARCH_STRAIGHT_BITFIELDS_HH__
