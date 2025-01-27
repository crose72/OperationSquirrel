#include "CppUTest/TestHarness.h"
#include "pid_controller.h"

/********************************************************************************
 * Test group name
 ********************************************************************************/

TEST_GROUP(ExampleGroup){};

TEST(ExampleGroup, FirstTest)
{
    PID vc;

    float control = 0.0;

    control = vc.pid3(0.1, 0.0, 0.0,
                                   5, 0.0, 0.0,
                                   1.0, 0.0, 0.0, ControlDim::X);
    CHECK_EQUAL(control, 0.7);
}

/********************************************************************************
 * Test details
 ********************************************************************************/

TEST(ExampleGroup, SecondTest)
{
    int x = 5;
    int y = 10;
    CHECK(x < y);
}
