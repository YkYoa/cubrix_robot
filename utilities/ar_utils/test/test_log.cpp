#include "ar_utils/ar_logging.h"
#include <iostream>

// const std::string datasetPathPrefix = ros::package::getPath("ar_logging") + "/../../../dataset/";

bool waitTest(int a)
{
	usleep(1000000);
	return false;
}

int waitTest2(int a)
{
	usleep(500000);
	return a;
}
class test
{
private:
	/* data */
public:
	test(/* args */);
	int testa(int a);
	~test();
};

test::test(/* args */)
{
}

test::~test()
{
}

int test::testa(int a)
{
	usleep(a * 1000000);
	return 1;
}

int main(int argc, char* argv[])
{
	ar_utils::ArLogger logging("test");
	test my_test;

	LOG_CALL(my_test.testa(1), logging, "test1");

	LOG_CALL(
		{
			my_test.testa(1);
			my_test.testa(2);
		},
		logging, "test2");
}