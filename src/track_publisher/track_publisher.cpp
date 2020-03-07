#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <ecl/geometry.hpp>
#include <tf/tf.h>

using ecl::CubicSpline;

std::vector<geometry_msgs::Point> gWaypoints;
nav_msgs::Path gPath;

void loadWaypoints(std::string filename, double scale, std::vector<geometry_msgs::Point> *waypoints) {
    FILE *file = fopen(filename.c_str(), "rt");

    double x, y;
    while(fscanf(file, "%lf,%lf", &x, &y) != EOF) {
        geometry_msgs::Point point;

        point.x = scale * x;
        point.y = scale * y;
        point.z = 0;

        waypoints->push_back(point);
    }
    fclose(file);
}

void interpolatePathNaive(const std::vector<geometry_msgs::Point> &waypoints, nav_msgs::Path *path) {
    path->header.frame_id = "map";

    path->poses.clear();
    for(int i = 0; i < waypoints.size(); i++) {
        geometry_msgs::PoseStamped pose;

        pose.header.stamp = ros::Time(0);
        //pose.header.frame_id = "base_link";

        pose.pose.position = waypoints[i];

        pose.pose.orientation.w = 1;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;

        path->poses.push_back(pose);
    }
}

void interpolatePathCubic(const std::vector<geometry_msgs::Point> &waypoints, double density, nav_msgs::Path *path) {
    int num_waypoints = waypoints.size();

    //create x and y splines
    static ecl::Array<double> x_list, y_list, t_list;
    x_list.resize(num_waypoints);
    y_list.resize(num_waypoints);
    t_list.resize(num_waypoints);

    double t = 0;
    for(int i = 0; i < num_waypoints; i++) {
        x_list[i] = waypoints[i].x;
        y_list[i] = waypoints[i].y;
        t_list[i] = t;

        t += 1;
    }

    CubicSpline spline_x = CubicSpline::Natural(t_list, x_list);
    CubicSpline spline_y = CubicSpline::Natural(t_list, y_list);

    //generate path
    path->header.frame_id = "map";

    path->poses.clear();

    //for every segment of spline
    ecl::Array<ecl::CubicPolynomial> segments_x = spline_x.polynomials();
    ecl::Array<ecl::CubicPolynomial> segments_y = spline_y.polynomials();

    double gauss_quadrature_coeff[5][2] = {
        {  0.0,                0.5688888888888889 },
        { -0.5384693101056831, 0.47862867049936647 },
        {  0.5384693101056831, 0.47862867049936647 },
        { -0.906179845938664,  0.23692688505618908 },
        {  0.906179845938664,  0.23692688505618908 }
    };

    double last_remain_len = 0.0;
    for(int i = 0; i < segments_x.size(); i++) {
        //intergrate arc length using gauss quadraute
        double arc_len = 0;
        for(int k = 0; k < 5; k++) {
            double x = i + ((gauss_quadrature_coeff[k][0] + 1) / 2);
            double A_x = segments_x[i].coefficients()[1] + 2 * segments_x[i].coefficients()[2] * x + 3 * segments_x[i].coefficients()[3] * x * x;
            double B_x = segments_y[i].coefficients()[1] + 2 * segments_y[i].coefficients()[2] * x + 3 * segments_y[i].coefficients()[3] * x * x;
            double F_x = std::sqrt( A_x * A_x + B_x * B_x );

            arc_len += gauss_quadrature_coeff[k][1] * F_x;
        }
        arc_len /= 2;

        /*printf("x: %lf ", segments_x[i](0 + i));
        printf("y: %lf ", segments_x[i](0 + i));
        printf("len: %lf\n", arc_len);*/

        //calculate points on segment
        double delta = density / arc_len;
        double v_t;
        for(v_t = (density - last_remain_len) / arc_len; v_t < 1; v_t += delta) {
            geometry_msgs::PoseStamped pose;

            pose.header.stamp = ros::Time(0);

            pose.pose.position.x = segments_x[i](i + v_t);
            pose.pose.position.y = segments_y[i](i + v_t);
            pose.pose.position.z = 0;

            tf::Quaternion q;
            q.setRPY(0, 0, atan2(segments_y[i].derivative(i + v_t), segments_x[i].derivative(i + v_t)));
            tf::quaternionTFToMsg(q, pose.pose.orientation);

            path->poses.push_back(pose);
        }

        v_t -= delta;
        last_remain_len = (1 - v_t) * arc_len;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "track_publisher");

    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("track", 1);

    std::string param_waypoints_filename;
    double param_scale;
    double param_density;

    ros::NodeHandle npriv("~");
    npriv.param<std::string>("waypoints_file", param_waypoints_filename, "/home/one/Documents/ROS_Workspace/src/car_model/tracks/racetrack01.txt");
    npriv.param<double>("scale", param_scale, 0.001);
    npriv.param<double>("density", param_density, 0.2);

    loadWaypoints(param_waypoints_filename, param_scale, &gWaypoints);
    interpolatePathCubic(gWaypoints, param_density, &gPath);

    ros::Rate loop_rate(1.0 / 5.0);
    while (ros::ok()) {
        path_pub.publish(gPath);

        //sleep and spin
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}