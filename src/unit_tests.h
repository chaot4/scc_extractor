#ifndef _UNIT_TESTS_H
#define _UNIT_TESTS_H

namespace scc
{

#define Test(x) \
	do { \
		if (!(x)) { \
			std::cout << "TEST_FAILED: " << #x << std::endl; \
			std::abort(); \
		} \
	} while (0)

namespace unit_tests
{
	void testAll();
}

}

#endif
