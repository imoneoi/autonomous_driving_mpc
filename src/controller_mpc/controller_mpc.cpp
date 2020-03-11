#include "controller_mpc/path_planner.h"
#include "controller_mpc/mpc.h"

#include "ros/ros.h"
#include <tf/tf.h>

#include <geometry_msgs/Twist.h>
#include <car_model/simulator_reset.h>
#include <car_model/car_state.h>

double gVelocity = 1.0;

car_model::car_state gState;
PathPlanner::Planner gPlanner;
MPC::Controller gController;

void initializeMPC() {
    MPC::Parameters parameters;
    parameters.dt = 0.1;
    parameters.pred_horizon = 20;
    parameters.control_horizon = 10;

    MPC::Model model;
    model.l_f = 0.125;
    model.l_r = 0.125;
    model.m   = 1.98;

    MPC::HardConstraint constraint;
    constraint.max_accel = 2;
    constraint.min_accel = 0;
    constraint.max_steer = 30.0 / 180.0 * M_PI;

    MPC::CostFunctionWeights weights;
    weights.w_position = 1.0;
    weights.w_angle    = 1.0;

    weights.w_velocity = 1.0;

    weights.w_steer = 0;
    weights.w_accel = 0;

    weights.w_dsteer = 0;
    weights.w_jerk   = 0;

    gController.initialize(parameters, model, constraint, weights);
}

void stateCallback(const car_model::car_state::ConstPtr &state) {
    gState = *state;
}

void resetSimulator() {
    while(ros::ok()) {
        car_model::simulator_reset simulator_reset_srv;
        std::vector<PathPlanner::PoseStamped> poses;
        PathPlanner::PoseStamped initial_state;

        initial_state.x = 0;
        initial_state.y = 0;
        initial_state.yaw = 0;
        initial_state.v = 0;
        initial_state.t = 0;
        gPlanner.getPath(initial_state, 0, 0, 1, &poses);

        simulator_reset_srv.request.x = poses[0].x;
        simulator_reset_srv.request.y = poses[0].y;
        simulator_reset_srv.request.yaw = poses[0].yaw;

        if(ros::service::call("simulator_reset", simulator_reset_srv)) {
            break;
        }

        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }

    ros::Duration(0.1).sleep();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller_mpc");

    //setup communication
    ros::NodeHandle nh;
    ros::Publisher  cmd_vel_pub  = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
    ros::Subscriber state_sub    = nh.subscribe<car_model::car_state>("car_state", 1, stateCallback);

    //load tracks
    ros::NodeHandle nh_priv("~");
    std::string track_filename = nh_priv.param<std::string>("track_filename", "/home/one/Documents/ROS_Workspace/src/car_model/tracks/racetrack01.txt");
    double track_scale = nh_priv.param<double>("track_scale", 0.0025);
    gPlanner.loadPath(track_filename, track_scale);

    //reset simulator
    resetSimulator();

    //initialize controller
    initializeMPC();

    ROS_INFO("MPC controller online.");

    double last_steer_angle = 0.0;

    ros::Rate rate(50);
    while (ros::ok()) {
        //get state
        MPC::State state;
        state.x = gState.x;
        state.y = gState.y;
        state.yaw = gState.phi;
        state.v = gState.v_x;
        state.steer_angle = last_steer_angle;

        //plan track
        PathPlanner::PoseStamped start_pose;
        std::vector<PathPlanner::PoseStamped> track;

        start_pose.x = gState.x;
        start_pose.y = gState.y;
        start_pose.yaw = gState.phi;
        start_pose.v = gState.v_x;
        start_pose.t = 0;

        gPlanner.getPath(start_pose, gController.parameters_.dt, gVelocity, gController.parameters_.pred_horizon - 1, &track);

        //calculate mpc
        MPC::ControlOutput control_output;
        gController.update(state, track, &control_output);

        //send control commands
        geometry_msgs::Twist cmd_vel;

        cmd_vel.linear.x = control_output.accel;
        cmd_vel.angular.z = control_output.steer;

        cmd_vel_pub.publish(cmd_vel);

        last_steer_angle = control_output.steer;

        //sleep and spin
        rate.sleep();
        ros::spinOnce();
    }
}