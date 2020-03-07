#include "simulator/simulator.h"

template
class Simulator<float>;
template
class Simulator<double>;

template<typename T>
Simulator<T>::Simulator() {

}

template<typename T>
void Simulator<T>::initializeStateAndInput(SimulatorState<T> *state, SimulatorInput<T> *input) {
    state->x = 0;
    state->y = 0;

    state->v = 0;
    state->alpha_t = 0;

    state->psi = 0;
    state->d_psi = 0;

    state->t = 0;

    input->steer_ = 0;
    input->force_ = 0;
}

template<typename T>
T Simulator<T>::tireCharacteristic(TireParameters<T> param, T alpha, T Fz, T muy) {
    return -param.k * alpha;
}

template<typename T>
void Simulator<T>::step(SimulatorState<T> *state, const SimulatorInput<T> &input, const SimulatorParameters<T> &param, int num_steps) {
    T dt = param.dt_;
    T eps = 1e-15;

    T I = param.vehicle_parameters.IT;
    T nF = param.vehicle_parameters.nF;
    T nR = param.vehicle_parameters.nR;
    T muy = param.vehicle_parameters.muy;

    T m = param.vehicle_parameters.mF0 + param.vehicle_parameters.mR0;
    T a = param.vehicle_parameters.mR0 / m * param.vehicle_parameters.lT;
    T b = param.vehicle_parameters.lT - a;

    T g = 9.81;                 // Gravity [m/s^2]

    T FzF = param.vehicle_parameters.mF0 * g;       // Vertical load @ F [N]
    T FzR = param.vehicle_parameters.mR0 * g;       // Vertical load @ R [N]

    for (int step = 0; step < num_steps; step++) {
        //States
        T X = state->x;
        T Y = state->y;
        T PSI = state->psi;
        T v = state->v;
        T ALPHAT = state->alpha_t;
        T dPSI = state->d_psi;

        // 
        T DELTA = /*input.steer_*/ 10.0 * M_PI / 180.0;

        // Slip angles
        T ALPHAF = std::atan2((v * std::sin(ALPHAT) + a * dPSI), (v * std::cos(ALPHAT))) - DELTA; // Dianteiro
        T ALPHAR = std::atan2((v * std::sin(ALPHAT) - b * dPSI), (v * std::cos(ALPHAT)));         // Traseiro

        // Longitudinal forces
        T FxF = 0;
        T FxR = 0/*input.force_*/;

        // Characteristic curve
        T FyF = nF * tireCharacteristic(param.tire_parameters, ALPHAF, FzF/nF, muy);
        T FyR = nR * tireCharacteristic(param.tire_parameters, ALPHAR, FzR/nR, muy);

        // Equations of motion
        T next_X = X + dt * v * std::cos(ALPHAT + PSI); // X
        T next_Y = Y + dt * v * std::sin(ALPHAT + PSI); // Y
        T next_PSI = PSI + dt * dPSI; // PSI
        T next_v = v + dt * (FxF * std::cos(ALPHAT - DELTA) + FxR * std::cos(ALPHAT) + FyF * std::sin(ALPHAT - DELTA) + FyR * std::sin(ALPHAT))/(m);
        T next_ALPHAT = ALPHAT + dt * ( - FxF * std::sin(ALPHAT - DELTA) - FxR * std::sin(ALPHAT) + FyF * std::cos(ALPHAT - DELTA) + FyR * std::cos(ALPHAT) - m * v * dPSI) / (m * v + eps);
        T next_dPSI = dPSI + dt * (FxF * a * std::sin(DELTA) + FyF * a * std::cos(DELTA) - FyR * b) / I;

        //Update
        state->x = next_X;
        state->y = next_Y;
        state->psi = next_PSI;
        state->v = next_v;
        state->alpha_t = next_ALPHAT;
        state->d_psi = next_dPSI;

        state->t += dt;
    }
}