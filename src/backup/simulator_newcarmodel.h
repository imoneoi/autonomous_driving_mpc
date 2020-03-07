#ifndef __CAR_MODEL_SIMULATOR_SIMULATOR_H__
#define __CAR_MODEL_SIMULATOR_SIMULATOR_H__

#include "vec2.h"

template<typename T>
//Parameters are SI
struct VehicleParameters {
    //T mT;     // Mass of the car (tractor) [kg]
    T IT = 10000;     // Moment of inertia the car (tractor) [kg * m2]
    //T a;      // Distance from front axle of the car (tractor) to the center of mass of the car (tractor) [m]
    //T b;      // Distance from center of mass of the car (tractor) to the front axle of the car (tractor) [m]
    T mF0 = 700;    // Mass over the front axle [kg]
    T mR0 = 600;    // Mass over the rear axle [kg]
    T lT = 3.5;     // Wheelbase [m]
    T nF = 2;     // Number of front tires
    T nR = 2;     // Number of rear tires
    T wT = 2;     // Track of the car (tractor)  [m]
    T muy = .8;    // Operational friction coefficient
};

template<typename T>
struct TireParameters {
    T k = 40000;
};

template<typename T>
struct ExtForceParameters {
    T a0; //Air drag coefficient Fd = a0 * v ^ 2
    T Kf; //Ground friction coefficient
};

template<typename T>
struct SimulatorParameters {
    T dt_;
    T display_dt_;

    VehicleParameters<T> vehicle_parameters;
    TireParameters<T> tire_parameters;
    ExtForceParameters<T> extforce_parameters;
};

template<typename T>
struct SimulatorInput {
    T force_;
    T steer_;
};

template<typename T>
struct SimulatorState {
    T x, y;
    T psi;
    T v;
    T alpha_t;
    T d_psi;
    T t;
};

template<typename T>
class Simulator {
private:
    T tireCharacteristic(TireParameters<T> param, T alpha, T Fz, T muy);

public:
    Simulator();

    void initializeStateAndInput(SimulatorState<T> *state, SimulatorInput<T> *input);
    void step(SimulatorState<T> *state, const SimulatorInput<T> &input, const SimulatorParameters<T> &param, int num_steps);
};

#endif