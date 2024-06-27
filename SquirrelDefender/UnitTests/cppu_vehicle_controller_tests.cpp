#include "CppUTest/TestHarness.h"
#include "pid_controller.h"

TEST_GROUP(ExampleGroup)
{
};

TEST(ExampleGroup, FirstTest)
{
    PID vc;

    float control = 0.0;

    control = vc.pid_controller_3d(0.1, 0.0, 0.0, 
                                   5, 0.0, 0.0, 
                                   1.0, 0.0, 0.0, CONTROL_DIM::X);
    CHECK_EQUAL(control, 0.7);
}

TEST(ExampleGroup, SecondTest)
{
    int x = 5;
    int y = 10;
    CHECK(x < y);
}
