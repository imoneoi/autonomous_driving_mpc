#include "controller_mpc/mpc.h"

template
class MPC::SparseMatrix<c_float>;

template
class MPC::QPProblem<c_float>;

template<typename T>
MPC::SparseMatrix<T>::SparseMatrix() {
    m_ = n_ = 0;
}

template<typename T>
MPC::SparseMatrix<T>::~SparseMatrix() {
    elements_.clear();
    freeOSQPCSCInstance();
}

template<typename T>
void MPC::SparseMatrix<T>::freeOSQPCSCInstance() {
    osqp_csc_data_.clear();
    osqp_csc_row_idx_.clear();
    osqp_csc_col_start_.clear();

    if(osqp_csc_instance != nullptr) {
        c_free(osqp_csc_instance);
        osqp_csc_instance = nullptr;
    }
}

template<typename T>
void MPC::SparseMatrix<T>::initialize(int m, int n) {
    m_ = m;
    n_ = n;
    elements_.clear();
}

template<typename T>
void MPC::SparseMatrix<T>::addElement(int r, int c, T v) {
    elements_.push_back({r, c, v});
}

template<typename T>
csc* MPC::SparseMatrix<T>::toOSQPCSC() {
    freeOSQPCSCInstance();

    sort(elements_.begin(), elements_.end());

    int idx = 0;
    int n_elem = elements_.size();

    osqp_csc_col_start_.push_back(0);
    for(int c = 0; c < n_; c++) {
        while((idx < n_elem) && elements_[idx].c == c) {
            osqp_csc_data_.push_back(elements_[idx].v);
            osqp_csc_row_idx_.push_back(elements_[idx].r);
            idx++;
        }

        osqp_csc_col_start_.push_back(osqp_csc_data_.size());
    }

    osqp_csc_instance = csc_matrix(m_, n_, osqp_csc_data_.size(), osqp_csc_data_.data(), osqp_csc_row_idx_.data(), osqp_csc_col_start_.data());
    return osqp_csc_instance;
}

template<typename T>
MPC::QPProblem<T>::~QPProblem() {
    if(osqp_workspace_ != nullptr) {
        //cleanup workspace
        osqp_cleanup(osqp_workspace_);
    }
}

template<typename T>
void MPC::QPProblem<T>::initialize(int n, int m) {
    n_ = n;
    m_ = m;

    A_.initialize(m_, n_);
    l_.resize(m_);
    u_.resize(m_);

    std::fill(l_.begin(), l_.end(), 0);
    std::fill(u_.begin(), u_.end(), 0);

    P_.initialize(n_, n_);
    q_.resize(n_);

    std::fill(q_.begin(), q_.end(), 0);
}

template<typename T>
OSQPSolution* MPC::QPProblem<T>::solve(int *error_code) {
    //set up workspace
    if(osqp_workspace_ == nullptr) {
        osqp_settings_ = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
        osqp_data_     = (OSQPData *)    c_malloc(sizeof(OSQPData));

        //populate data
        osqp_data_->n = n_;
        osqp_data_->m = m_;

        osqp_data_->A = A_.toOSQPCSC();
        osqp_data_->l = l_.data();
        osqp_data_->u = u_.data();

        osqp_data_->P = P_.toOSQPCSC();
        osqp_data_->q = q_.data();

        osqp_set_default_settings(osqp_settings_);
        osqp_setup(&osqp_workspace_, osqp_data_, osqp_settings_);
    }
    else {
        csc *A_csc = A_.toOSQPCSC();
        osqp_update_A(osqp_workspace_, A_csc->x, NULL, A_csc->nzmax);
        osqp_update_bounds(osqp_workspace_, l_.data(), u_.data());

        csc *P_csc = P_.toOSQPCSC();
        osqp_update_P(osqp_workspace_, P_csc->x, NULL, P_csc->nzmax);
        osqp_update_lin_cost(osqp_workspace_, q_.data());
    }

    *error_code = osqp_solve(osqp_workspace_);

    return osqp_workspace_->solution;
}

MPC::Controller::Controller() {

}

void MPC::Controller::initialize(const Parameters &parameters, const Model &model, const HardConstraint &constraint, const CostFunctionWeights &weights) {
    parameters_ = parameters;
    model_ = model;

    constraint_ = constraint;
    weights_    = weights;
}

void MPC::Controller::update(const State &state, const State &linearize_point, const std::vector<PathPlanner::PoseStamped> &track_input, ControlOutput *out, std::vector<State> *pred_out) {
    /*
        Variable:
            [s0 s1 s2 s3 ...](t=0) [s0 s1 s2 s3 ...](t=1) ... [c0 c1 ...](t=0)
    */
    const int dim_state = 4;
    const int dim_control = 2;

    /*
        Sub-functions
    */
    auto state_var_idx = [&] (int t, int id) {
        return t * dim_state + id;
    };
    auto control_var_idx = [&] (int t, int id) {
        return parameters_.pred_horizon * dim_state + std::min(t, parameters_.control_horizon - 1) * dim_control + id;
    };
    auto control_derivative_row_idx = [&] (int t, int id) {
        return dim_state * parameters_.pred_horizon + dim_control * parameters_.control_horizon + std::min(t, parameters_.control_horizon - 1) * dim_control + id;
    };
    /*auto normalize_angle = [&] (double a) {
        return fmod(fmod(a + M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI) - M_PI;
    };*/
    auto get_beta_from_steer = [&] (double steer) {
        return std::atan(model_.l_r / (model_.l_f + model_.l_r) * std::tan(steer));
    };
    auto get_steer_from_beta = [&] (double beta) {
        return std::atan((model_.l_f + model_.l_r) / model_.l_r * std::tan(beta));
    };

    /*
        Setup and solve QP
    */

    int n = dim_state * parameters_.pred_horizon + dim_control * parameters_.control_horizon; //number of variables
    int m = n + dim_control * parameters_.control_horizon; //number of constraints

    c_float dt = parameters_.dt;

    qp_.initialize(n, m);

    /*
        initialize constraint matrix
    */

    //create initial state constraints
    for(int i = 0; i < dim_state; i++) qp_.A_.addElement(state_var_idx(0, i), state_var_idx(0, i), 1);
    qp_.l_[0] = qp_.u_[0] = state.x;
    qp_.l_[1] = qp_.u_[1] = state.y;
    qp_.l_[2] = qp_.u_[2] = state.v;
    qp_.l_[3] = qp_.u_[3] = state.yaw;

    c_float linearize_beta = get_beta_from_steer(linearize_point.steer_angle);

    //create state model constraints
    for(int t = 1; t < parameters_.pred_horizon; t++) {
        //adaptive MPC

        //x
        qp_.A_.addElement(state_var_idx(t, 0), state_var_idx(t, 0), -1);
        qp_.A_.addElement(state_var_idx(t, 0), state_var_idx(t - 1, 0), 1);

        qp_.A_.addElement(state_var_idx(t, 0), state_var_idx(t - 1, 2), dt * std::cos(linearize_point.yaw + linearize_beta));
        qp_.A_.addElement(state_var_idx(t, 0), state_var_idx(t - 1, 3), dt * -linearize_point.v * std::sin(linearize_point.yaw + linearize_beta));
        qp_.l_[state_var_idx(t, 0)] = qp_.u_[state_var_idx(t, 0)] =   -(dt *  linearize_point.v * std::sin(linearize_point.yaw + linearize_beta) * (linearize_point.yaw + linearize_beta));

        //y
        qp_.A_.addElement(state_var_idx(t, 1), state_var_idx(t, 1), -1);
        qp_.A_.addElement(state_var_idx(t, 1), state_var_idx(t - 1, 1), 1);

        qp_.A_.addElement(state_var_idx(t, 1), state_var_idx(t - 1, 2), dt * std::sin(linearize_point.yaw + linearize_beta));
        qp_.A_.addElement(state_var_idx(t, 1), state_var_idx(t - 1, 3), dt * linearize_point.v * std::cos(linearize_point.yaw + linearize_beta));
        qp_.l_[state_var_idx(t, 1)] = qp_.u_[state_var_idx(t, 1)] =  -(-dt * linearize_point.v * std::cos(linearize_point.yaw + linearize_beta) * (linearize_point.yaw + linearize_beta));

        //v
        qp_.A_.addElement(state_var_idx(t, 2), state_var_idx(t, 2), -1);
        qp_.A_.addElement(state_var_idx(t, 2), state_var_idx(t - 1, 2), 1);
        qp_.A_.addElement(state_var_idx(t, 2), control_var_idx(t - 1, 1), dt);
        qp_.l_[state_var_idx(t, 2)] = qp_.u_[state_var_idx(t, 2)] = 0;

        //yaw
        qp_.A_.addElement(state_var_idx(t, 3), state_var_idx(t, 3), -1);
        qp_.A_.addElement(state_var_idx(t, 3), state_var_idx(t - 1, 3), 1);
        qp_.A_.addElement(state_var_idx(t, 3), state_var_idx(t - 1, 2), dt * std::sin(linearize_beta) / model_.l_r);
        qp_.A_.addElement(state_var_idx(t, 3), control_var_idx(t - 1, 0), dt * linearize_point.v / model_.l_r * std::cos(linearize_beta));
        qp_.l_[state_var_idx(t, 3)] = qp_.u_[state_var_idx(t, 3)] =    -(-dt * linearize_point.v / model_.l_r * std::cos(linearize_beta) * linearize_beta);
    }

    //create control output constraints
    for(int t = 0; t < parameters_.control_horizon; t++) {
        for(int i = 0; i < dim_control; i++) qp_.A_.addElement(control_var_idx(t, i), control_var_idx(t, i), 1);
        
        qp_.l_[control_var_idx(t, 0)] = get_beta_from_steer(-constraint_.max_steer);
        qp_.u_[control_var_idx(t, 0)] = get_beta_from_steer( constraint_.max_steer);

        qp_.l_[control_var_idx(t, 1)] = constraint_.min_accel;
        qp_.u_[control_var_idx(t, 1)] = constraint_.max_accel;
    }

    //create control derivative constraints
    //integral average approximation
    double max_delta_beta  = dt * constraint_.max_steer_rate * ((get_beta_from_steer(constraint_.max_steer) - get_beta_from_steer(0)) / (constraint_.max_steer - 0));
    double max_delta_accel = dt * constraint_.max_jerk;

    for(int i = 0; i < dim_control; i++) {
        qp_.A_.addElement(control_derivative_row_idx(0, i), control_var_idx(0, i), 1);
    }

    c_float state_beta = get_beta_from_steer(state.steer_angle);
    qp_.l_[control_derivative_row_idx(0, 0)] = state_beta - max_delta_beta;
    qp_.u_[control_derivative_row_idx(0, 0)] = state_beta + max_delta_beta;

    qp_.l_[control_derivative_row_idx(0, 1)] = state.accel - max_delta_accel;
    qp_.u_[control_derivative_row_idx(0, 1)] = state.accel + max_delta_accel;

    for(int t = 1; t < parameters_.control_horizon; t++) {
        for(int i = 0; i < dim_control; i++) {
            qp_.A_.addElement(control_derivative_row_idx(t, i), control_var_idx(t, i),      1);
            qp_.A_.addElement(control_derivative_row_idx(t, i), control_var_idx(t - 1, i), -1);
        }

        qp_.l_[control_derivative_row_idx(t, 0)] = -max_delta_beta;
        qp_.u_[control_derivative_row_idx(t, 0)] =  max_delta_beta;

        qp_.l_[control_derivative_row_idx(t, 1)] = -max_delta_accel;
        qp_.u_[control_derivative_row_idx(t, 1)] =  max_delta_accel;
    }

    //create cost function
    //pred horizon
    for(int t = 1; t < parameters_.pred_horizon; t++) {
        PathPlanner::PoseStamped pose = track_input[t - 1];

        /*
            Fix yaw +-2pi for QP
        */
        if(std::fabs((pose.yaw + M_PI * 2.0) - state.yaw) < std::fabs(pose.yaw - state.yaw))
            pose.yaw += M_PI * 2.0;

        if(std::fabs((pose.yaw - M_PI * 2.0) - state.yaw) < std::fabs(pose.yaw - state.yaw))
            pose.yaw -= M_PI * 2.0;

        //position
        qp_.P_.addElement(state_var_idx(t, 0), state_var_idx(t, 0), weights_.w_position);
        qp_.q_[state_var_idx(t, 0)] = -weights_.w_position * pose.x;

        qp_.P_.addElement(state_var_idx(t, 1), state_var_idx(t, 1), weights_.w_position);
        qp_.q_[state_var_idx(t, 1)] = -weights_.w_position * pose.y;

        //angle
        qp_.P_.addElement(state_var_idx(t, 3), state_var_idx(t, 3), weights_.w_angle);
        qp_.q_[state_var_idx(t, 3)] = -weights_.w_angle * pose.yaw;

        //velocity
        qp_.P_.addElement(state_var_idx(t, 2), state_var_idx(t, 2), weights_.w_velocity);
        qp_.q_[state_var_idx(t, 2)] = -weights_.w_velocity * pose.v;
    }

    //control horizon
    for(int t = 0; t < parameters_.control_horizon; t++) {
        //accel
        qp_.P_.addElement(control_var_idx(t, 0), control_var_idx(t, 0), weights_.w_accel);

        //steer
        qp_.P_.addElement(control_var_idx(t, 1), control_var_idx(t, 1), weights_.w_steer);
    }

    //solve qp
    OSQPSolution *solution = qp_.solve(&out->error_code);

    out->steer = get_steer_from_beta(solution->x[control_var_idx(0, 0)]);
    out->accel = solution->x[control_var_idx(0, 1)];

    //output predictive result
    if(pred_out != nullptr) {
        pred_out->clear();

        for(int t = 0; t < parameters_.pred_horizon; t++) {
            State pred_state;
            pred_state.x = solution->x[state_var_idx(t, 0)];
            pred_state.y = solution->x[state_var_idx(t, 1)];
            pred_state.v = solution->x[state_var_idx(t, 2)];
            pred_state.yaw = solution->x[state_var_idx(t, 3)];

            pred_out->push_back(pred_state);
        }
    }
}

void MPC::IterativeController::simulate(const State &state, State *next) {
    double dt = parameters_.dt;
    double beta = std::atan(model_.l_r / (model_.l_f + model_.l_r) * std::tan(state.steer_angle));

    next->x   = state.x + dt * state.v * std::cos(state.yaw + beta);
    next->y   = state.y + dt * state.v * std::sin(state.yaw + beta);
    next->yaw = state.yaw + dt * state.v / model_.l_r * std::sin(beta);
    next->v   = state.v + dt * state.accel;

    next->accel = state.accel;
    next->steer_angle = state.steer_angle;
}

void MPC::IterativeController::update(const State &state, const std::vector<PathPlanner::PoseStamped> &track_input, int iterations, double threshold, ControlOutput *out, std::vector<State> *pred_out) {
    State state_with_new_control_input = state;

    int num_iterations = 0;
    for(int i = 0; i < iterations; i++) {
        num_iterations++;

        State linearize_point;
        simulate(state_with_new_control_input, &linearize_point);

        Controller::update(state, linearize_point, track_input, out, pred_out);

        double delta_output = 
            std::fabs(state_with_new_control_input.accel - out->accel) + 
            std::fabs(state_with_new_control_input.steer_angle - out->steer);

        if(delta_output < threshold) break;

        state_with_new_control_input.accel       = out->accel;
        state_with_new_control_input.steer_angle = out->steer;
    }

    printf("%d iterations\n", num_iterations);
}