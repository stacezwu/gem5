#include "tutorial/hello_object_new.hh"

#include <iostream>

namespace gem5
{

HelloObjectNew::HelloObjectNew(const HelloObjectNewParams &params) :
    SimObject(params)
{
    std::cout << "Hello World! From a new SimObject!" << std::endl;
}

} // namespace gem5
