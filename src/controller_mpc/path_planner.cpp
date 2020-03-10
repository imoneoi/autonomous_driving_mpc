#include "controller_mpc/path_planner.h"
#include "controller_mpc/quintic_solver.hpp"

PathPlanner::Planner::PathPlanner() {

}

bool PathPlanner::Planner::loadPath(std::string filename, double scale) {
    waypoints_.clear();

    FILE *file = fopen(filename.c_str(), "rt");
    if(file != nullptr) {
        double x, y;
        while(fscanf(file, "%lf,%lf", &x, &y) != EOF) {
            geometry_msgs::Point point;

            point.x = scale * x;
            point.y = scale * y;
            point.z = 0;

            waypoints_.push_back(point);
        }
        fclose(file);

        computeSplineWithPoints();

        return true;
    }

    return false;
}

void PathPlanner::Planner::computeSplineWithPoints() {
    int num_waypoints = int(waypoints_.size());
    if(num_waypoints <= 0) {
        return;
    }

    //create x and y line array
    static ecl::Array<double> x_list, y_list, t_list;
    x_list.resize(num_waypoints);
    y_list.resize(num_waypoints);
    t_list.resize(num_waypoints);

    double t = 0;
    for(int i = 0; i < num_waypoints; i++) {
        x_list[i] = waypoints_[i].x;
        y_list[i] = waypoints_[i].y;
        t_list[i] = t;

        t += 1;
    }

    spline_max_t_ = t_list[num_waypoints - 1];

    //interpolate parametric cubic spline
    spline_x_ = CubicSpline::Natural(t_list, x_list);
    spline_y_ = CubicSpline::Natural(t_list, y_list);
}

/*
    Assume that a, b >= 0
*/
double PathPlanner::Planner::lengthOfSpline(double a, double b) {
    const double gauss_quadrature_coeff[5][2] = {
        {  0.0,                0.5688888888888889 },
        { -0.5384693101056831, 0.47862867049936647 },
        {  0.5384693101056831, 0.47862867049936647 },
        { -0.906179845938664,  0.23692688505618908 },
        {  0.906179845938664,  0.23692688505618908 }
    };

    double length = 0;
    int max_segments = int(spline_x_.polynomials().size()) - 1;
    for(int segment = int(a); segment <= std::min(int(b), max_segments); segment++) {
        double start = std::max(a, double(segment));
        double end   = std::min(b, double(segment + 1));

        //get coefficients
        const ecl::CubicPolynomial::Coefficients& coeff_x = spline_x_.polynomials()[segment].coefficients();
        const ecl::CubicPolynomial::Coefficients& coeff_y = spline_y_.polynomials()[segment].coefficients();

        //integrate this segment
        double integral = 0;
        for(int k = 0; k < 5; k++) {
            double x = ((end - start) / 2) * gauss_quadrature_coeff[k][0] + ((start + end) / 2);

            double A_x = coeff_x[1] + 2 * coeff_x[2] * x + 3 * coeff_x[3] * x * x;
            double B_x = coeff_y[1] + 2 * coeff_y[2] * x + 3 * coeff_y[3] * x * x;
            double F_x = std::sqrt( A_x * A_x + B_x * B_x );

            integral += gauss_quadrature_coeff[k][1] * F_x;
        }
        integral *= (end - start) / 2;

        length += integral;
    }

    return length;
}

double PathPlanner::Planner::nearestPointOnSpline(double x, double y, double eps) {
    double min_dist_2 = 1.0 / 0.0;
    double min_t = 0;

    for(int segment = 0; segment < spline_max_t_; segment++) {
        //get 5th polynomial
        /*
            evaluated using sagemath

            a, b, c, d = var("a, b, c, d")
            e, f, g, h = var("e, f, g, h")
            x, y = var("x, y")

            f_x(t) = a + b * t + c * t^2 + d * t^3
            f_y(t) = e + f * t + g * t^2 + h * t^3

            df_x(t) = b + 2*c*t + 3*d*t^2
            df_y(t) = f + 2*g*t + 3*h*t^2

            -(
                (x - f_x(t)) * df_x(t) + (y - f_y(t)) * df_y(t)
            ).expand().collect(t)
        */

        const ecl::CubicPolynomial::Coefficients& coeff_x = spline_x_.polynomials()[segment].coefficients();
        const ecl::CubicPolynomial::Coefficients& coeff_y = spline_y_.polynomials()[segment].coefficients();

        double a = coeff_x[0];
        double b = coeff_x[1];
        double c = coeff_x[2];
        double d = coeff_x[3];

        double e = coeff_y[0];
        double f = coeff_y[1];
        double g = coeff_y[2];
        double h = coeff_y[3];

        QuinticSolver::QuinticPolynomial poly = {
            a*b + e*f - b*x - f*y, //0
            (b*b + 2*a*c + f*f + 2*e*g - 2*c*x - 2*g*y), //1
            3*(b*c + a*d + f*g + e*h - d*x - h*y), //2
            2*(c*c + 2*b*d + g*g + 2*f*h), //3
            5*(c*d + g*h), //4
            3*(d*d + h*h) //5
        };

        //solve 5th polynomial
        std::vector<double> sol_t;
        QuinticSolver::solveRealRootsSpecialForCubicSplineDerivative(poly, double(segment), double(segment + 1), &sol_t, eps);

        //get nearest
        for(auto it = sol_t.begin(); it != sol_t.end(); it++) {
            double t  = *it;
            double dx = x - spline_x_(t);
            double dy = y - spline_y_(t);
            double dist_2 = dx * dx + dy * dy;

            if(dist_2 < min_dist_2) {
                min_dist_2 = dist_2;
                min_t = t;
            }
        }
    }

    return min_t;
}

double PathPlanner::Planner::pointWithFixedDistance(double t, double d, double eps) {
    if(d < eps) return t;

    //bi-section
    double l = t;
    double r = spline_max_t_;

    while((r - l) > eps) {
        double m = (l + r) / 2;

        if(lengthOfSpline(t, m) <= d) l = m;
        else r = m;
    }

    return (l + r) / 2;
}

void PathPlanner::Planner::getPath(const PathPlanner::PoseStamped &cur_pose, double v, double dt, int n, std::vector<PathPlanner::PoseStamped> *path) {
    path->clear();

    //find nearest point on trajectory
    double t_start = nearestPointOnSpline(cur_pose.x, cur_pose.y);
    double time    = 0;

    for(int i = 0; i < n; i++) {
        double t = pointWithFixedDistance(t_start, v * dt * i);

        path->push_back(PathPlanner::PoseStamped({
            spline_x_(t), spline_y_(t), std::atan2(spline_y_.derivative(t), spline_x_.derivative(t)),
            time
        }));

        time += dt;
    }
}

double PathPlanner::Planner::randomFloat() {
    static std::random_device rd;
    static std::mt19937 e(rd());
    static std::uniform_real_distribution<> dist(0, 1);

    return dist(e);
}

void PathPlanner::Planner::runTests() {
    fprintf(stderr, "PathPlanner Tests begin.\n");

    clock_t t_start, t_end;
    double t_total;
    int n_compute;

    //spline length
    t_start = clock();
    n_compute = 0;

    for(int i = 0; i < 100; i++) {
        double start_point = randomFloat() * spline_max_t_;

        double dt = spline_max_t_ / 10000;
        double last_len = -1;

        for(double t = start_point; t < spline_max_t_; t += dt) {
            double len = lengthOfSpline(0, t);
            n_compute++;

            assert(len > last_len);

            last_len = len;
        }
    }

    t_end = clock();
    t_total = double(t_end - t_start) * 1000 / CLOCKS_PER_SEC / n_compute;

    fprintf(stderr, "[OK] Spline length, %lf msec/call.\n", t_total);

    //point on spline
    t_start = clock();
    n_compute = 0;

    double max_error = 0;
    for(int i = 0; i < 10000; i++) {
        double t = randomFloat() * spline_max_t_;
        double x = spline_x_(t);
        double y = spline_y_(t);

        double t_estimated = nearestPointOnSpline(x, y);
        double x_estimated = spline_x_(t_estimated);
        double y_estimated = spline_y_(t_estimated);

        double error = std::sqrt((x - x_estimated) * (x - x_estimated) + (y - y_estimated) * (y - y_estimated));
        max_error = std::max(max_error, error);

        n_compute++;
    }

    t_end = clock();
    t_total = double(t_end - t_start) * 1000 / CLOCKS_PER_SEC / n_compute;

    fprintf(stderr, "[OK] Spline point, max error = %lf, %lf msec/call.\n", max_error, t_total);

    //point with fixed distance
    t_start = clock();
    n_compute = 0;

    max_error = 0;
    for(int i = 0; i < 10000; i++) {
        double t = randomFloat() * spline_max_t_;
        double d = randomFloat() * lengthOfSpline(t, spline_max_t_);

        double t_next = pointWithFixedDistance(t, d);
        assert(t_next >= t);

        double error = std::fabs(lengthOfSpline(t, t_next) - d);

        max_error = std::max(max_error, error);

        n_compute++;
    }

    t_end = clock();
    t_total = double(t_end - t_start) * 1000 / CLOCKS_PER_SEC / n_compute;

    fprintf(stderr, "[OK] Point with distance, max error = %lf, %lf msec/call.\n", max_error, t_total);
}