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

    state->v_x = 0;
    state->v_y = 0;

    state->phi = 0;
    state->r = 0;

    state->t = 0;

    input->steer_ = 0;
    input->force_ = 0;
}

template<typename T>
T Simulator<T>::f_pajecka(T b, T c, T d, T alpha) {
    /*
        f_pajecka = d*sin(c*atan(b*alpha))    
        
        inputs :
            * trMdl := tire model, a list or tuple of parameters (b,c,d)
            * alpha := tire slip angle [radians]
        outputs :
            * Fy := lateral force from tire [Newtons]
    */

    return d*std::sin(c*std::atan(b*alpha));
}

template<typename T>
void Simulator<T>::step(SimulatorState<T> *state, const SimulatorInput<T> &input, const SimulatorParameters<T> &param, int num_steps) {
    T dt = param.dt_;

    //Reference: https://github.com/MPC-Berkeley/barc/blob/master/workspace/src/barc/src/estimation/system_models.py

    //T eps = 1e-8;

    //extract parameters
    T max_d_f = param.vehicle_parameters.max_steer;
    T d_f = std::min(max_d_f, std::max(-max_d_f, input.steer_));
    T FxR = std::min(std::max(T(0), input.force_), param.vehicle_parameters.max_accel);

    //T d_f = input.steer_;
    //T FxR = input.force_;

    T a = param.vehicle_parameters.a;
    T b = param.vehicle_parameters.b;
    T m = param.vehicle_parameters.m;
    T I_z = param.vehicle_parameters.I_z;

    T a0 = param.extforce_parameters.a0;
    T Ff = param.extforce_parameters.Ff;

    T B = param.tire_parameters.B;
    T C = param.tire_parameters.C;
    T mu = param.tire_parameters.mu;

    T g = 9.81;
    T Fn = m * g / 2.0;

    for (int step = 0; step < num_steps; step++) {
        // limit force to tire friction circle
        if(FxR >= mu * Fn) {
            FxR = mu * Fn;
        }

        // compute the front/rear slip  [rad/s]
        // ref: Hindiyeh Thesis, p58
        T a_F     = std::atan2(state->v_y + a * state->r, state->v_x) - d_f;
        T a_R     = std::atan2(state->v_y - b * state->r, state->v_x);

        // compute lateral tire force at the front
        T FyF         = -f_pajecka(B, C, mu*Fn, a_F);

        // compute lateral tire force at the rear
        // ensure that magnitude of longitudinal/lateral force lie within friction circle
        //FIXME: Sign error?
        T FyR_paj     = -f_pajecka(B, C, mu*Fn, a_R);
        T FyR_max     = std::sqrt((mu*Fn)*(mu*Fn) - FxR * FxR);
        T FyR         = std::max(-FyR_max, std::min(FyR_max, FyR_paj));

        //compute force
        T x_force = FxR - FyF*std::sin(d_f);
        T y_force = FyF * std::cos(d_f) + FyR;

        //compute friction
        x_force += ( -((state->v_x > 0) ? 1 : -1) * (a0 * state->v_x * state->v_x ) ) + (-((state->v_x > 0) ? 1 : -1) * Ff);

        // compute next state
        T x_next      = state->x   + dt * (state->v_x*std::cos(state->phi) - state->v_y * std::sin(state->phi));
        T y_next      = state->y   + dt * (state->v_x*std::sin(state->phi) + state->v_y * std::cos(state->phi));
        T phi_next    = state->phi + dt * state->r;
        T v_x_next    = state->v_x + dt * (state->r * state->v_y + (1/m) * x_force);
        T v_y_next    = state->v_y + dt * (-state->r * state->v_x + (1/m) * y_force);
        T r_next      = state->r   + dt / I_z * (a*FyF*std::cos(d_f) - b * FyR);

        state->x = x_next;
        state->y = y_next;
        state->phi = phi_next;
        state->v_x = v_x_next;
        state->v_y = v_y_next;
        state->r = r_next;

        state->t += dt;
    }
}