#include "unittest.hpp"
#include "controller_mpc/mpc.h"

class Test_QP : public UnitTest {
public:
    void run() {
        testInfo("Testing QP...");
        MPC::QPProblem<c_float> qp;

        double inf = 1e8;

        qp.initialize(2, 3);
        qp.A_.addElement(0, 0, 2);
        qp.A_.addElement(0, 1, 3);
        qp.A_.addElement(1, 0, 1);
        qp.A_.addElement(2, 1, 1);
        qp.l_ = {4, 0, 0};
        qp.u_ = {inf, inf, inf};

        qp.P_.addElement(0, 0, 3);
        qp.P_.addElement(1, 1, 1);
        qp.P_.addElement(0, 1, 2);

        qp.q_ = {1, 6};

        int errcode;
        OSQPSolution *sol = qp.solve(&errcode);

        testAssert(errcode == 0, "QP solver fail");

        testPass("QP");
    }
};