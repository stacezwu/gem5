/*
 * Copyright (c) 2013 ARM Limited
 * Copyright (c) 2014 Sven Karlsson
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

#ifndef __ARCH_GENERIC_RPSTATE_HH__
#define __ARCH_GENERIC_RPSTATE_HH__

#include <iostream>
#include <memory>
#include <type_traits>

#include "base/compiler.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "sim/serialize.hh"

namespace gem5
{

class RPStateBase : public Serializable
{
  protected:
    RegIndex _rp = 0;

    void set(RegIndex val)
    {
      this->rp(val);
    }
    
    RPStateBase(const RPStateBase &other) : _rp(other._rp) {}

    
    

  public:
    explicit RPStateBase(RegIndex val) { set(val); }
    RPStateBase() {}
    ~RPStateBase() = default;

    template<class Target>
    Target &
    as()
    {
        return static_cast<Target &>(*this);
    }

    template<class Target>
    const Target &
    as() const
    {
        return static_cast<const Target &>(*this);
    }

    RPStateBase &operator=(const RPStateBase &other) = default;

    virtual RPStateBase *clone() const
    {
      return new RPStateBase(*this);
    };

    void
    output(std::ostream &os) const 
    {
        ccprintf(os, "(%#x)", this->rp());
    }

    virtual void
    update(const RPStateBase &other)
    {
        _rp = other._rp;
    }
    void update(const RPStateBase *ptr) { update(*ptr); }

    bool
    equals(const RPStateBase &other) const
    {
        auto &ps = other.as<RPStateBase>();
        return RPStateBase::equals(other) &&
            _rp == ps._rp;
    }

    void
    serialize(CheckpointOut &cp) const override
    {
        SERIALIZE_SCALAR(_rp);
    }

    void
    unserialize(CheckpointIn &cp) override
    {
        UNSERIALIZE_SCALAR(_rp);
    }

    RegIndex rp() const { return _rp; }
    void rp(RegIndex val) { _rp = val; }

    void advance() {
      this->_rp += 1;
    }
    RegIndex translate(RegIndex rs) {
        return (_rp - rs);
    }


};

static inline std::ostream &
operator<<(std::ostream & os, const RPStateBase &pc)
{
    pc.output(os);
    return os;
}

static inline bool
operator==(const RPStateBase &a, const RPStateBase &b)
{
    return a.equals(b);
}

static inline bool
operator!=(const RPStateBase &a, const RPStateBase &b)
{
    return !a.equals(b);
}

namespace
{

inline void
set(RPStateBase *&dest, const RPStateBase *src)
{
    if (GEM5_LIKELY(dest)) {
        if (GEM5_LIKELY(src)) {
            // Both src and dest already have storage, so just copy contents.
            dest->update(src);
        } else {
            // src is empty, so clear out dest.
            dest = nullptr;
        }
    } else {
        if (GEM5_LIKELY(src)) {
            // dest doesn't have storage, so create some as a copy of src.
            dest = src->clone();
        } else {
            // dest is already nullptr, so nothing to do.
        }
    }
}

inline void
set(std::unique_ptr<RPStateBase> &dest, const RPStateBase *src)
{
    RPStateBase *dest_ptr = dest.get();
    set(dest_ptr, src);
    if (dest.get() != dest_ptr)
        dest.reset(dest_ptr);
}

inline void
set(std::unique_ptr<RPStateBase> &dest,
        const std::unique_ptr<RPStateBase> &src)
{
    RPStateBase *dest_ptr = dest.get();
    const RPStateBase *src_ptr = src.get();
    set(dest_ptr, src_ptr);
    if (dest.get() != dest_ptr)
        dest.reset(dest_ptr);
}


inline void
set(RPStateBase *&dest, const RPStateBase &src)
{
    if (GEM5_LIKELY(dest)) {
        // Update dest with the contents of src.
        dest->update(src);
    } else {
        // Clone src over to dest.
        dest = src.clone();
    }
}

inline void
set(std::unique_ptr<RPStateBase> &dest, const RPStateBase &src)
{
    RPStateBase *dest_ptr = dest.get();
    set(dest_ptr, src);
    if (dest.get() != dest_ptr)
        dest.reset(dest_ptr);
}

// inline void
// set(RPStateBase *&dest, const RPStateBase &src)
// {
//     if (GEM5_LIKELY(dest)) {
//         // Update dest with the contents of src.
//         dest->update(src);
//     } else {
//         // Clone src over to dest.
//         dest = src.clone();
//     }
// }

}

} // namespace gem5

#endif 
