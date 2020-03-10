#include <random>

#include "ros/ros.h"
#include "controller_pid/pid.hpp"

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>

#include <car_model/simulator_reset.h>
#include <car_model/PIDControllerConfig.h>

struct State {
    double x, y, yaw;

    double t;
    double v_x;

    bool ready;

    State() { reset(); }

    void reset() {
        x = y = yaw = 0;
        v_x = 0;
        t = 0;

        ready = false;
    }
};

State gState, gLastState;
PID gPIDLongtitudal;
PID gPIDLateral;
nav_msgs::Path gPath;

double gMaxAccel = 3.0;
double gMaxSteer = 45.0 / 180.0 * M_PI;

double gNoiseAccel = 0.0;
double gNoiseSteer = 0.0;

double gSpeedSetpoint = 0.7;

double quaternionToYaw(tf::Quaternion q) {
    double angle = q.getAngle();
    double zAxis = q.getAxis().z();

    //must be quaternions around z axis
    const double eps = 1e-5;
    assert((std::fabs(angle) < eps) || (std::fabs(std::fabs(zAxis) - 1) < eps));

    if(zAxis < 0) angle = -angle;
    return std::fmod(angle + 2 * M_PI, 2 * M_PI);
}

double AngularMinus(double a, double b)
{
    a = fmod(a, 2.0 * M_PI);
    b = fmod(b, 2.0 * M_PI);

    double res1 = a - b;
    double res2 = (a < b) ? (a + 2 * M_PI - b) : (a - 2 * M_PI - b);

    return (std::abs(res1) < std::abs(res2)) ? res1 : res2;
}

double randomFloat() {
    static std::random_device rd;
    static std::mt19937 e(rd());
    static std::uniform_real_distribution<> dist(-1, 1);

    return dist(e);
}

void pathCallback(const nav_msgs::Path::ConstPtr &path) {
    gPath = *path;
}

double findNearestWaypoint(double x, double y, double yaw, geometry_msgs::Pose *pose) {
    const double inf = 1.0 / 0.0;

    if(gPath.poses.empty()) {
        return inf;
    }

    double min_dist_2 = inf; //inf
    int min_id = -1;

    for(int i = 0; i < gPath.poses.size(); i++) {
        double dx = x - gPath.poses[i].pose.position.x;
        double dy = y - gPath.poses[i].pose.position.y;
        double dist_2 = dx * dx + dy * dy;

        if(dist_2 < min_dist_2) {
            min_dist_2 = dist_2;
            min_id = i;
        }
    }

    *pose = gPath.poses[min_id].pose;
    return std::sqrt(min_dist_2);
}

void resetSimulator() {
    while(ros::ok()) {
        if(!gPath.poses.empty()) {
            car_model::simulator_reset simulator_reset_srv;

            simulator_reset_srv.request.x = gPath.poses[0].pose.position.x;
            simulator_reset_srv.request.y = gPath.poses[0].pose.position.y;

            tf::Quaternion q;
            tf::quaternionMsgToTF(gPath.poses[0].pose.orientation, q);
            simulator_reset_srv.request.yaw = quaternionToYaw(q);

            if(ros::service::call("simulator_reset", simulator_reset_srv)) {
                break;
            }
        }

        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    gState.reset();
    gLastState.reset();

    ros::Duration(0.1).sleep();
}

void parameterReconfigureCallback(car_model::PIDControllerConfig &config, uint32_t level) {
    gMaxAccel = config.max_accel;
    gMaxSteer = config.max_steer * M_PI / 180.0;

    gNoiseAccel = config.noise_accel;
    gNoiseSteer = config.noise_steer;

    gSpeedSetpoint = config.v;

    gPIDLongtitudal.setCoefficients(config.v_kp, config.v_ki, config.v_kd, config.v_maxi, gMaxAccel);
    gPIDLateral.setCoefficients(config.s_kp, config.s_ki, config.s_kd, config.s_maxi, gMaxSteer);

    if(config.reset_simulator) resetSimulator();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller_pid");

    ros::NodeHandle nh;
    ros::Publisher  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    ros::Publisher  debug_pub   = nh.advertise<std_msgs::Float64MultiArray>("pid_controller_debug", 1);
    ros::Subscriber path_sub    = nh.subscribe<nav_msgs::Path>("track", 1, pathCallback);
    tf::TransformListener tf_listener;

    dynamic_reconfigure::Server<car_model::PIDControllerConfig> server;
    server.setCallback(boost::bind(&parameterReconfigureCallback, _1, _2));

    //simulator reset
    resetSimulator();

    ROS_INFO("Controller online!");

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        //get pose
        tf::StampedTransform map2base_tf;
        try {
            tf_listener.lookupTransform("map", "base_link", ros::Time(0), map2base_tf);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            continue;
        }

        //write state
        gState.t = map2base_tf.stamp_.toSec();
        gState.x = map2base_tf.getOrigin().x();
        gState.y = map2base_tf.getOrigin().y();

        //get yaw from quaternion
        gState.yaw = quaternionToYaw(map2base_tf.getRotation());

        //estimate velocity
        if(gLastState.ready) {
            if(std::fabs(gState.t - gLastState.t) < 1e-8) {
                gState.v_x = gLastState.v_x;
            }
            else {
                gState.v_x = ((gState.x - gLastState.x) * std::cos(gLastState.yaw) + (gState.y - gLastState.y) * std::sin(gLastState.yaw)) / (gState.t - gLastState.t);
            }
        }

        gState.ready = true;

        gLastState = gState;

        //control loop
        geometry_msgs::Twist twist;

        //lateral Stanley controller
        geometry_msgs::Pose waypoint;
        double waypoint_dist = findNearestWaypoint(gState.x, gState.y, gState.yaw, &waypoint);

        tf::Quaternion q;
        tf::quaternionMsgToTF(waypoint.orientation, q);
        
        double waypoint_x = waypoint.position.x;
        double waypoint_y = waypoint.position.y;
        double waypoint_yaw = quaternionToYaw(q);

        double waypoint_cross_product = cos(waypoint_yaw) * (gState.y - waypoint.position.y) - (gState.x - waypoint.position.x) * sin(waypoint_yaw);
        double waypoint_direction = (waypoint_cross_product < 0) ? 1 : -1;

        //double steer_angle = gMaxSteer * waypoint_direction;

        //double angle_error = AngularMinus(waypoint_yaw, gState.yaw);
        //double steer_angle = angle_error + atan2(gStanleyK * waypoint_dist * waypoint_direction, gState.v_x);
        double steer_angle = gPIDLateral.update(-waypoint_direction * waypoint_dist, 0);

        twist.angular.z = steer_angle;

        //longtitudal PID (velocity)
        twist.linear.x = gPIDLongtitudal.update(gState.v_x, gSpeedSetpoint);

        //interfere controlling commands with +-20%
        twist.linear.x +=  randomFloat() * gMaxAccel * gNoiseAccel;
        twist.angular.z += randomFloat() * gMaxSteer * gNoiseSteer;

        //clamp control commands
        twist.linear.x  = std::min(gMaxAccel, std::max(0.0, twist.linear.x));
        twist.angular.z = std::min(gMaxSteer, std::max(-gMaxSteer, twist.angular.z));

        //send control command
        cmd_vel_pub.publish(twist);

        //send debug info
        std_msgs::Float64MultiArray debug_info;

        debug_info.data.clear();
        debug_info.data.push_back(gState.v_x);
        debug_info.data.push_back(waypoint_dist);
        debug_pub.publish(debug_info);

        //sleep and spin
        loop_rate.sleep();
        ros::spinOnce();
    }
}