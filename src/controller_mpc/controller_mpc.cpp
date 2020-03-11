#include "controller_mpc/path_planner.h"
#include "controller_mpc/mpc.h"

#include "ros/ros.h"
#include <tf/tf.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <car_model/simulator_reset.h>
#include <car_model/car_state.h>

double gVelocity = 5.0;
double gMinVelocity = 0.5;
double gCurvatureK = 150.0;

car_model::car_state gState;
PathPlanner::Planner gPlanner;
MPC::IterativeController gController;

void initializeMPC() {
    MPC::Parameters parameters;
    parameters.dt = 0.025;
    parameters.pred_horizon = 40;
    parameters.control_horizon = 20;

    MPC::Model model;
    model.l_f = 0.125;
    model.l_r = 0.125;
    model.m   = 1.98;

    MPC::HardConstraint constraint;
    constraint.max_accel = 2.0;
    constraint.min_accel = -2.0;
    constraint.max_steer = 30.0 / 180.0 * M_PI;

    constraint.max_steer_rate = constraint.max_steer / 1.0;
    constraint.max_jerk   = constraint.max_accel / 0.1;

    MPC::CostFunctionWeights weights;
    weights.w_position = 1.0;
    weights.w_angle    = 0.1;

    weights.w_velocity = 0.025;

    weights.w_steer = 0.01;
    weights.w_accel = 0.01;

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
        gPlanner.getPath(initial_state, 0, 0, 0, 0, 1, 1, &poses);

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

double normalizeAngle(double a) {
    return fmod(fmod(a + M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI) - M_PI;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller_mpc");

    //setup communication
    ros::NodeHandle nh;
    ros::Publisher  cmd_vel_pub     = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Publisher  trajectory_pub  = nh.advertise<geometry_msgs::PoseArray>("planned_trajectory", 1);
    ros::Publisher  prediction_pub  = nh.advertise<geometry_msgs::PoseArray>("predicted_trajectory", 1);
    ros::Subscriber state_sub       = nh.subscribe<car_model::car_state>("car_state", 1, stateCallback);

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
    double last_accel = 0;

    ros::Rate rate(50);
    while (ros::ok()) {
        //get state
        MPC::State state;
        state.x = gState.x;
        state.y = gState.y;
        state.yaw = normalizeAngle(gState.phi);
        state.v = sqrt(gState.v_x * gState.v_x + gState.v_y * gState.v_y);
        state.steer_angle = last_steer_angle;
        state.accel = last_accel;

        //plan track
        PathPlanner::PoseStamped start_pose;
        std::vector<PathPlanner::PoseStamped> track;

        start_pose.x = gState.x;
        start_pose.y = gState.y;
        start_pose.yaw = gState.phi;
        start_pose.v = gState.v_x;
        start_pose.t = 0;

        gPlanner.getPath(start_pose, gController.parameters_.dt, gVelocity, gMinVelocity, gCurvatureK, 0.25, gController.parameters_.pred_horizon - 1, &track);

        //calculate mpc
        MPC::ControlOutput control_output;
        std::vector<MPC::State> prediction_output;
        gController.update(state, track, 10, 0.01, &control_output, &prediction_output);

        //send control commands
        geometry_msgs::Twist cmd_vel;

        cmd_vel.linear.x = control_output.accel;
        cmd_vel.angular.z = control_output.steer;

        cmd_vel_pub.publish(cmd_vel);

        last_steer_angle = control_output.steer;
        last_accel       = control_output.accel;

        //visualize track
        geometry_msgs::PoseArray planned_trajectory;
        planned_trajectory.header.frame_id = "map";

        for(int i = 0; i < track.size(); i++) {
            geometry_msgs::Pose pose;
            pose.position.x = track[i].x;
            pose.position.y = track[i].y;
            pose.position.z = 0;
            pose.orientation = tf::createQuaternionMsgFromYaw(track[i].yaw);

            planned_trajectory.poses.push_back(pose);
        }
        trajectory_pub.publish(planned_trajectory);

        //visualize predicted track
        geometry_msgs::PoseArray predicted_trajectory;
        predicted_trajectory.header.frame_id = "map";

        for(int i = 0; i < prediction_output.size(); i++) {
            geometry_msgs::Pose pose;
            pose.position.x = prediction_output[i].x;
            pose.position.y = prediction_output[i].y;
            pose.position.z = 0;
            pose.orientation = tf::createQuaternionMsgFromYaw(prediction_output[i].yaw);

            predicted_trajectory.poses.push_back(pose);
        }
        prediction_pub.publish(predicted_trajectory);

        //sleep and spin
        rate.sleep();
        ros::spinOnce();
    }
}