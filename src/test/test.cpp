#include "unittest.hpp"
#include "test_planner.hpp"
#include "test_qp.hpp"

int main() {
    Test_Planner testPlanner;
    testPlanner.run();

    Test_QP testQP;
    testQP.run();
}