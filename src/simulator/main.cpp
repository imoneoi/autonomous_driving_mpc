#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <car_model/simulator_reset.h>
#include <car_model/car_state.h>

#include <mutex>
#include <thread>

#include "simulator/simulator.h"
#define SIMULATOR_DATA_TYPE double

Simulator<SIMULATOR_DATA_TYPE> gSimulator;
SimulatorInput<SIMULATOR_DATA_TYPE> gInput;
SimulatorState<SIMULATOR_DATA_TYPE> gState;
SimulatorParameters<SIMULATOR_DATA_TYPE> gParam;
std::mutex gSimulateMutex;

car_model::simulator_reset::Request gResetRequest;
bool gResetRequestFlag = false;

void loadParameters() {
    ros::NodeHandle nh_priv("~");

    nh_priv.param<SIMULATOR_DATA_TYPE>("simulation/dt", gParam.dt_, 0.0001);
    nh_priv.param<SIMULATOR_DATA_TYPE>("simulation/display_dt", gParam.display_dt_, 0.01);

    //default parameter setting from BARC https://github.com/MPC-Berkeley/barc
    nh_priv.param<SIMULATOR_DATA_TYPE>("vehicle/m", gParam.vehicle_parameters.m, 1.98);
    nh_priv.param<SIMULATOR_DATA_TYPE>("vehicle/I_z", gParam.vehicle_parameters.I_z, 0.24);
    nh_priv.param<SIMULATOR_DATA_TYPE>("vehicle/a", gParam.vehicle_parameters.a, 0.125);
    nh_priv.param<SIMULATOR_DATA_TYPE>("vehicle/b", gParam.vehicle_parameters.b, 0.125);

    nh_priv.param<SIMULATOR_DATA_TYPE>("vehicle/max_steer", gParam.vehicle_parameters.max_steer, M_PI / 2);
    nh_priv.param<SIMULATOR_DATA_TYPE>("vehicle/max_accel", gParam.vehicle_parameters.max_accel, 2.0);

    nh_priv.param<SIMULATOR_DATA_TYPE>("tire/B", gParam.tire_parameters.B, 0.3);
    nh_priv.param<SIMULATOR_DATA_TYPE>("tire/C", gParam.tire_parameters.C, 1.25);
    nh_priv.param<SIMULATOR_DATA_TYPE>("tire/mu", gParam.tire_parameters.mu, 0.234);

    nh_priv.param<SIMULATOR_DATA_TYPE>("extforce/a0", gParam.extforce_parameters.a0, 0.1308);
    nh_priv.param<SIMULATOR_DATA_TYPE>("extforce/Kf", gParam.extforce_parameters.Ff, 0.1711);
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    gSimulateMutex.lock();

    gInput.force_ = msg->linear.x;
    gInput.steer_ = msg->angular.z;

    gSimulateMutex.unlock();
}

bool simulatorResetServiceCallback(car_model::simulator_reset::Request &request, car_model::simulator_reset::Response &response) {
    gSimulateMutex.lock();
    gResetRequest = request;
    gResetRequestFlag = true;
    gSimulateMutex.unlock();

    return true;
}

void simulatorThread() {
    ros::Rate loop_rate(1.0 / gParam.display_dt_);

    //time synchorization
    gSimulateMutex.lock();
    gState.t = ros::Time::now().toSec();
    gSimulateMutex.unlock();

    SimulatorInput<SIMULATOR_DATA_TYPE> input;
    SimulatorState<SIMULATOR_DATA_TYPE> state;
    while (ros::ok()) {
        gSimulateMutex.lock();

        //Reset state request
        if(gResetRequestFlag) {
            gResetRequestFlag = false;

            gSimulator.initializeStateAndInput(&gState, &gInput);

            gState.x = gResetRequest.x;
            gState.y = gResetRequest.y;
            gState.phi = gResetRequest.yaw;

            gState.t = ros::Time::now().toSec();
        }

        input = gInput;
        state = gState;

        gSimulateMutex.unlock();

        int num_steps = int(std::lround((ros::Time::now().toSec() - state.t) / gParam.dt_));
        gSimulator.step(&state, input, gParam, num_steps);

        gSimulateMutex.lock();
        gState = state;
        gSimulateMutex.unlock();

        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "car_simulator");

    ros::NodeHandle nh;
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmdVelCallback);
    ros::ServiceServer service  = nh.advertiseService("simulator_reset", simulatorResetServiceCallback);
    ros::Publisher state_pub    = nh.advertise<car_model::car_state>("car_state", 10);
    tf::TransformBroadcaster pos_pub_tf; 

    loadParameters();
    gSimulator.initializeStateAndInput(&gState, &gInput);

    //create simulator thread
    std::thread simulator_thread(simulatorThread);

    //main loop
    ros::Rate loop_rate(1.0 / gParam.display_dt_);
    while (ros::ok()) {
        tf::Transform transform;
        tf::Quaternion q;

        //read state
        SimulatorState<SIMULATOR_DATA_TYPE> state;
        gSimulateMutex.lock();
        state = gState;
        gSimulateMutex.unlock();

        //broadcast state
        car_model::car_state state_msg;
        state_msg.x = state.x;
        state_msg.y = state.y;
        state_msg.v_x = state.v_x;
        state_msg.v_y = state.v_y;
        state_msg.phi = state.phi;
        state_msg.r = state.r;
        state_pub.publish(state_msg);

        //broadcast CoM tf
        ros::Time state_stamp(state.t);

        transform.setOrigin( tf::Vector3(state.x, state.y, 0.0) );
        q.setRPY(0, 0, state.phi);
        transform.setRotation(q);

        pos_pub_tf.sendTransform(tf::StampedTransform(transform, state_stamp, "map", "base_link"));

        //broadcast wheel tf
        transform.setOrigin( tf::Vector3(state.x + gParam.vehicle_parameters.a * std::cos(state.phi), state.y + gParam.vehicle_parameters.a * std::sin(state.phi), 0.0) );
        q.setRPY(0, 0, state.phi + gInput.steer_);
        transform.setRotation(q);

        pos_pub_tf.sendTransform(tf::StampedTransform(transform, state_stamp, "map", "wheel_front"));

        transform.setOrigin( tf::Vector3(state.x - gParam.vehicle_parameters.a * std::cos(state.phi), state.y - gParam.vehicle_parameters.a * std::sin(state.phi), 0.0) );
        q.setRPY(0, 0, state.phi);
        transform.setRotation(q);

        pos_pub_tf.sendTransform(tf::StampedTransform(transform, state_stamp, "map", "wheel_back"));

        //sleep and spin
        loop_rate.sleep();
        ros::spinOnce();
    }

    //wait for simulator thread
    simulator_thread.join();

    return 0;
}