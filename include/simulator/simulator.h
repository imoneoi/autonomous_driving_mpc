#ifndef __CAR_MODEL_SIMULATOR_SIMULATOR_H__
#define __CAR_MODEL_SIMULATOR_SIMULATOR_H__

#include "vec2.h"

template<typename T>
//Parameters are SI
struct VehicleParameters {
    T a;
    T b;
    T m;
    T I_z;

    T max_steer;
    T max_accel;
};

template<typename T>
struct TireParameters {
    T B;
    T C;
    T mu;
};

template<typename T>
struct ExtForceParameters {
    T a0; //Air drag coefficient Fd = a0 * v ^ 2
    T Ff; //Ground friction coefficient
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
    T x;
    T y;
    T v_x;
    T v_y;
    T phi;
    T r;

    T t;
};

template<typename T>
class Simulator {
private:
    T f_pajecka(T b, T c, T d, T alpha);
public:
    Simulator();

    void initializeStateAndInput(SimulatorState<T> *state, SimulatorInput<T> *input);
    void step(SimulatorState<T> *state, const SimulatorInput<T> &input, const SimulatorParameters<T> &param, int num_steps);
};

#endif