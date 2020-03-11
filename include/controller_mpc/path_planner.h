#ifndef __CAR_MODEL_CONTROLLER_MPC_PATH_PLANNER_H__
#define __CAR_MODEL_CONTROLLER_MPC_PATH_PLANNER_H__

#include <string>
#include <cmath>
#include <algorithm>

#include "ros/ros.h"
#include <ecl/geometry.hpp>
#include <ecl/geometry/polynomial.hpp>
#include <nav_msgs/Path.h>

using ecl::CubicSpline;

namespace PathPlanner {
    struct PoseStamped {
        double x, y, yaw;
        double v;
        double t;
    };

    class Planner {
    private:
        std::vector<geometry_msgs::Point> waypoints_;
        CubicSpline spline_x_;
        CubicSpline spline_y_;
        double spline_max_t_;

        void computeSplineWithPoints();
        double nearestPointOnSpline(double x, double y, double eps = 1e-8);
        double lengthOfSpline(double a, double b);
        double pointWithFixedDistance(double t, double d, double eps = 1e-8);
        double maxCurvature(double l, double r, double numerical_eps = 1e-8);

        double randomFloat();
    public:
        Planner();

        bool loadPath(std::string filename, double scale = 1);
        void getPath(const PathPlanner::PoseStamped &cur_pose, double dt, double v_ref, double v_min, double k, double max_brake_accel, int n, std::vector<PathPlanner::PoseStamped> *path);

        void runTests();
    };
}

#endif