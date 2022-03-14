from m5.params import *
from m5.SimObject import SimObject

class HelloObjectNew(SimObject):
    type = 'HelloObjectNew'
    cxx_header = "tutorial/hello_object_new.hh"
    cxx_class = 'gem5::HelloObjectNew'
