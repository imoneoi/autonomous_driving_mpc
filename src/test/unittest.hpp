#ifndef __CAR_MODEL_SRC_TEST_TEST_H__
#define __CAR_MODEL_SRC_TEST_TEST_H__

#include <string>
#include <iostream>
#include <cassert>
#include <random>

class UnitTest {
protected:
    void testAssert(bool x, std::string error_msg) {
        if(!x) {
            std::cerr << "!!![TEST ERROR]!!! " << error_msg << std::endl;

            exit(-1);
        }
    }

    void testInfo(std::string info_msg) {
        std::cerr << "[INFO] " << info_msg << std::endl;
    }

    void testPass(std::string name) {
        std::cerr << "[PASS] " << name << std::endl;
    }

    double randomFloat(double l = 0., double r = 1.) {
        static std::random_device rd;
        static std::mt19937 e(rd());
        static std::uniform_real_distribution<> dist(0, 1);

        assert(l <= r);

        return l + dist(e) * (r - l);
    }
public:
    void run() {}
};

#endif