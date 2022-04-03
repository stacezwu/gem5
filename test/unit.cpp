#define BOOST_TEST_MODULE example
#include <boost/test/included/unit_test.hpp>
#include <boost/test/test_case_template.hpp>
#include <boost/mpl/list.hpp>
#include <iostream>


inline uint64_t
mask(int nbits)
{
    return (nbits == 64) ? (uint64_t)-1LL : (1ULL << nbits) - 1;
}

template <class T>
inline
T
bits(T val, int first_front, int last_front, int first_back, int last_back)
{
    int nbits_front = first_front - last_front + 1;
    int nbits_back = first_back - last_back + 1;
    return (val >> last_front) & mask(nbits_front) << nbits_front | (val >> last_back & mask(nbits_back));
}


typedef boost::mpl::list<uint64_t> test_types;

BOOST_AUTO_TEST_CASE_TEMPLATE(bits_test, T, test_types)
{
    uint64_t machInst = 0x89ABCDEF01234567;
    int first_front = 36;
    int last_front = 32;
    int first_back = 19;
    int last_back = 15;
    
    T result = bits(machInst, first_front, last_front, first_back, last_back);

    std::cout << result << std::endl;
    BOOST_CHECK_EQUAL(486, result);
}
