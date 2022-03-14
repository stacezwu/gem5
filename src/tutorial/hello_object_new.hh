#ifndef __LEARNING_GEM5_HELLO_OBJECT_HH__
#define __LEARNING_GEM5_HELLO_OBJECT_HH__

#include "params/HelloObjectNew.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class HelloObjectNew : public SimObject
{
  public:
    HelloObjectNew(const HelloObjectNewParams &p);
};


} // namespace gem5

#endif // __LEARNING_GEM5_HELLO_OBJECT_HH__