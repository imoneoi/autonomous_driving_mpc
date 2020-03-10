#include "controller_mpc/mpc.h"

MPC::Controller::Controller(const Parameters &parameters, const Model &model, const HardConstraint &constraint, const CostFunctionWeights &weights) {
    parameters_ = parameters;
    model_ = model;

    constraint_ = constraint;
    weights_    = weights;
}

MPC::Controller::~Controller() {
    if(osqp_workspace_ != nullptr) osqp_cleanup(osqp_workspace_);
}

void MPC::Controller::update(const State &state, const std::vector<PathPlanner::PoseStamped> &track_input, ControlOutput *out) {
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
    auto get_beta_from_steer = [&] (c_float steer) {
        return std::atan(model_.l_r / (model_.l_f + model_.l_r) * std::tan(steer));
    };
    auto get_steer_from_beta = [&] (c_float steer) {
        return std::atan(model_.l_r / (model_.l_f + model_.l_r) * std::tan(steer));
    };

    /*
        fix the track yaw +-2pi for QP
    */
    std::vector<PathPlanner::PoseStamped> track_fixed = track_input;
    for(int i = 0; i < track_fixed.size(); i++) {
        if(std::fabs((track_fixed[i].yaw + M_PI * 2.0) - state.yaw) < std::fabs(track_fixed[i].yaw - state.yaw))
            track_fixed[i].yaw += M_PI * 2.0;

        if(std::fabs((track_fixed[i].yaw - M_PI * 2.0) - state.yaw) < std::fabs(track_fixed[i].yaw - state.yaw))
            track_fixed[i].yaw -= M_PI * 2.0;
    }

    /*
        Setup and solve QP
    */

    int n = dim_state * parameters_.pred_horizon + dim_control * parameters_.control_horizon; //number of variables
    int m = n; //number of constraints

    c_float dt = parameters_.dt;

    /*
        create constraint matrix
    */
    std::vector< std::vector<c_float> > A_csc_2d_data;
    std::vector< std::vector<int> >     A_csc_2d_col_index;

    std::vector<c_float> l;
    std::vector<c_float> u;

    A_csc_2d_data.resize(m);
    A_csc_2d_col_index.resize(m);
    l.resize(m);
    u.resize(m);

    auto add_A_item = [&] (int r, int c, c_float v) {
        A_csc_2d_data[c].push_back(v);
        A_csc_2d_col_index[c].push_back(r);
    };

    //create initial state constraints
    for(int i = 0; i < dim_state; i++) add_A_item(state_var_idx(0, i), state_var_idx(0, i), 1);
    l[0] = u[0] = state.x;
    l[1] = u[1] = state.y;
    l[2] = u[2] = state.v;
    l[3] = u[3] = state.yaw;

    c_float state_beta = get_beta_from_steer(state.steer_angle);

    //create state model constraints
    for(int t = 1; t < parameters_.pred_horizon; t++) {
        //adaptive MPC

        //x
        add_A_item(state_var_idx(t, 0), state_var_idx(t, 0), -1);
        add_A_item(state_var_idx(t, 0), state_var_idx(t - 1, 0), 1);
        add_A_item(state_var_idx(t, 0), state_var_idx(t - 1, 3),   dt * state.v * -std::sin(state.yaw + state_beta));
        add_A_item(state_var_idx(t, 0), control_var_idx(t - 1, 0), dt * state.v * -std::sin(state.yaw + state_beta));

        //y
        add_A_item(state_var_idx(t, 1), state_var_idx(t, 1), -1);
        add_A_item(state_var_idx(t, 1), state_var_idx(t - 1, 1), 1);
        add_A_item(state_var_idx(t, 1), state_var_idx(t - 1, 3),   dt * state.v * std::cos(state.yaw + state_beta));
        add_A_item(state_var_idx(t, 1), control_var_idx(t - 1, 0), dt * state.v * std::cos(state.yaw + state_beta));

        //v
        add_A_item(state_var_idx(t, 2), state_var_idx(t, 2), -1);
        add_A_item(state_var_idx(t, 2), state_var_idx(t - 1, 2), 1);
        add_A_item(state_var_idx(t, 2), control_var_idx(t - 1, 1), dt);

        //yaw
        add_A_item(state_var_idx(t, 3), state_var_idx(t, 3), -1);
        add_A_item(state_var_idx(t, 3), state_var_idx(t - 1, 3), 1);
        add_A_item(state_var_idx(t, 3), control_var_idx(t - 1, 0), dt * state.v / model_.l_r * std::cos(state_beta));

        //l, u set to zero
        for(int i = 0; i < dim_state; i++) {
            int row = state_var_idx(t, i);
            l[row] = u[row] = 0;
        }
    }

    //create control output constraints
    for(int t = 0; t < parameters_.control_horizon; t++) {
        for(int i = 0; i < dim_control; i++) add_A_item(control_var_idx(t, i), control_var_idx(t, i), 1);
        
        l[control_var_idx(t, 0)] = get_beta_from_steer(-constraint_.max_steer);
        u[control_var_idx(t, 0)] = get_beta_from_steer( constraint_.max_steer);

        l[control_var_idx(t, 1)] = constraint_.min_accel;
        u[control_var_idx(t, 1)] = constraint_.max_accel;
    }

    //convert to csc matrix
    std::vector<c_float> A_csc_data;
    std::vector<c_int> A_csc_row_idx;
    std::vector<c_int> A_csc_col_start;

    A_csc_col_start.push_back(0);
    for(int c = 0; c < m; c++) {
        for(int i = 0; i < A_csc_2d_data[c].size(); i++) {
            A_csc_data.push_back(A_csc_2d_data[c][i]);
            A_csc_row_idx.push_back(A_csc_2d_col_index[c][i]);

            if(i != 0) {
                assert(A_csc_2d_col_index[c][i] > A_csc_2d_col_index[c][i - 1]);
            }
        }

        A_csc_col_start.push_back(A_csc_row_idx.size());
    }

    csc* A_csc = csc_matrix(m, n, A_csc_data.size(), A_csc_data.data(), A_csc_row_idx.data(), A_csc_col_start.data());

    //create cost matrix
    std::vector<c_float> q;
    q.resize(n);

    //current impl: cost on state 
    std::vector<c_float> P_csc_data;
    std::vector<c_int>   P_csc_row_idx;
    std::vector<c_int>   P_csc_col_start;

    csc* P_csc = csc_matrix(n, n, P_csc_data.size(), P_csc_data.data(), P_csc_row_idx.data(), P_csc_col_start.data());

    //set up workspace
    if(osqp_workspace_ == nullptr) {
        osqp_settings_ = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
        osqp_data_     = (OSQPData *)c_malloc(sizeof(OSQPData));
        
        //populate data
        osqp_data_->n = n;
        osqp_data_->m = m;
        osqp_data_->P = P_csc;
        osqp_data_->q = q.data();
        osqp_data_->A = A_csc;
        osqp_data_->l = l.data();
        osqp_data_->u = u.data();

        osqp_set_default_settings(osqp_settings_);
        osqp_setup(&osqp_workspace_, osqp_data_, osqp_settings_);
    }
    else {
        osqp_update_bounds(osqp_workspace_, l.data(), u.data());
        osqp_update_A(osqp_workspace_, A_csc_data.data(), NULL, A_csc_data.size());

        osqp_update_P(osqp_workspace_, P_csc_data.data(), NULL, P_csc_data.size());
        osqp_update_lin_cost(osqp_workspace_, q.data());
    }

    out->error_code = osqp_solve(osqp_workspace_);
    out->steer = osqp_workspace_->solution->x[control_var_idx(0, 0)];
    out->accel = osqp_workspace_->solution->x[control_var_idx(0, 1)];

    c_free(A_csc);
    c_free(P_csc);
}