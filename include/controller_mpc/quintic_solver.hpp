#ifndef __CAR_MODEL_CONTROLLER_MPC_QUINTIC_SOLVER_H__
#define __CAR_MODEL_CONTROLLER_MPC_QUINTIC_SOLVER_H__

#include <cmath>
#include <vector>
#include <cassert>

namespace QuinticSolver {
    //a[0] + a[1]*x + a[2]*x^2 + a[3]*x^3 + a[4]*x^4 + a[5]*x^5
    typedef double QuinticPolynomial[6];

    inline double evaluatePolynomial(QuinticPolynomial a, double x) {
        return a[0] + x * (a[1] + x * (a[2] + x * (a[3] + x * (a[4] + x * a[5]))));
    }

    //descartes sign changes on f(x + t)
    /*
        Calculated using sagemath

        a, b, c, d, e, f, x, t = var("a, b, c, d, e, f, x, t")

        (a+b*(x+t)+c*(x+t)^2+d*(x+t)^3+e*(x+t)^4+f*(x+t)^5).expand().collect(x)
    */
    inline int descartesSignChanges(QuinticPolynomial a, double t, double eps = 1e-12) {
        double t2 = t  * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;

        QuinticPolynomial g = {
            a[5]*t5 + a[4]*t4 + a[3]*t3 + a[2]*t2 + a[1]*t + a[0], //0
            5*a[5]*t4 + 4*a[4]*t3 + 3*a[3]*t2 + 2*a[2]*t + a[1],   //1
            10*a[5]*t3 + 6*a[4]*t2 + 3*a[3]*t + a[2],              //2
            10*a[5]*t2 + 4*a[4]*t + a[3],                          //3
            5*a[5]*t + a[4],                                       //4
            a[5]                                                   //5
        };

        int result = 0;
        int last_sign = 0;
        for(int i = 5; i >= 0; i--) if(std::fabs(g[i]) > eps) {
            int sign = (g[i] > 0) ? 1 : -1;

            if((last_sign != 0) && (sign != last_sign)) result++;
            last_sign = sign;
        }
        return result;
    }


    // max number of real roots in (l, r]
    /*
        Budan's theorem
        https://en.wikipedia.org/wiki/Budan%27s_theorem
    */
    inline int maxNumberOfRealRoots(QuinticPolynomial a, double l, double r, double eps = 1e-12) {
        int result = descartesSignChanges(a, l, eps) - descartesSignChanges(a, r, eps);

        assert(result >= 0);
        return result;
    }

    // solves all real roots in (l, r]
    /*
        Budan's theorem for isolation
        Bisection for solve roots

        Special: checks whether (f(l) < 0) && (f(r) > 0)

        Reference: Improved Algebraic Algorithm On Point Projection For Bézier Curves https://hal.inria.fr/file/index/docid/518379/filename/Xiao-DiaoChen2007c.pdf
    */
    inline void solveRealRootsSpecialForCubicSplineDerivative(QuinticPolynomial a, double l, double r, std::vector<double> *roots, double eps = 1e-8, double numerical_eps = 1e-12) {
        roots->clear();

        struct rootInterval
        {
            double l, r;
            int max_roots;
        };

        std::vector<rootInterval> intervals;
        intervals.reserve(10);
        intervals.push_back({l, r, maxNumberOfRealRoots(a, l, r, numerical_eps)});
        while (intervals.size()) {
            rootInterval interval = *intervals.rbegin();
            intervals.pop_back();

            //end condition
            if((interval.max_roots <= 1) || ((interval.r - interval.l) < eps)) {
                if(interval.max_roots > 0) {
                    //special here
                    if((evaluatePolynomial(a, l) < 0) && (evaluatePolynomial(a, r) > 0)) {
                        //bisection root solver
                        double bi_l = interval.l, bi_r = interval.r;
                        while((bi_r - bi_l) > eps) {
                            double bi_m = (bi_l + bi_r) / 2;

                            if( evaluatePolynomial(a, bi_m) > 0 ) bi_r = bi_m;
                            else bi_l = bi_m;
                        }

                        roots->push_back( (bi_l + bi_r) / 2 );
                    }
                }
            }
            else {
                //bisection
                double mid = (interval.l + interval.r) / 2;
                intervals.push_back({interval.l, mid, maxNumberOfRealRoots(a, interval.l, mid, numerical_eps)});
                intervals.push_back({mid, interval.r, maxNumberOfRealRoots(a, mid, interval.r, numerical_eps)});
            }
        }
    }
}

#endif