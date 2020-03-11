#ifndef __CAR_MODEL_CONTROLLER_MPC_MPC_H__
#define __CAR_MODEL_CONTROLLER_MPC_MPC_H__

#include "path_planner.h"
#include "osqp.h"
#include <vector>

namespace MPC {
    struct HardConstraint {
        double max_steer;

        double max_accel;
        double min_accel;
    };

    struct CostFunctionWeights {
        double w_position;
        double w_angle;

        double w_velocity;

        double w_accel;
        double w_jerk;

        double w_steer;
        double w_dsteer;
    };

    struct State {
        double x;
        double y;
        double yaw;
        double v;

        double steer_angle;
        double accel;
    };

    struct Parameters {
        int pred_horizon;
        int control_horizon;

        double dt;
    };

    struct Model {
        double l_f;
        double l_r;
        double m;
    };

    struct ControlOutput {
        int error_code;
        double steer;
        double accel;
    };

    template<typename T>
    struct SparseMatrixElement {
        int r, c;
        T v;

        inline bool operator<(const SparseMatrixElement &rhs) {
            return (c == rhs.c) ? (r < rhs.r) : (c < rhs.c);
        }
    };

    template<typename T>
    class SparseMatrix {
    private:
        int m_, n_;

        std::vector< SparseMatrixElement<T> > elements_;

        std::vector<T> osqp_csc_data_;
        std::vector<c_int> osqp_csc_row_idx_;
        std::vector<c_int> osqp_csc_col_start_;
        csc *osqp_csc_instance = nullptr;

        void freeOSQPCSCInstance();

    public:
        SparseMatrix();
        ~SparseMatrix();

        void initialize(int m, int n);
        void addElement(int r, int c, T v);
        csc *toOSQPCSC();
    };

    template<typename T>
    class QPProblem {
    private:
        OSQPWorkspace *osqp_workspace_ = nullptr;
        OSQPSettings  *osqp_settings_= nullptr;
        OSQPData      *osqp_data_ = nullptr;

    public:
        //number of variables and constraints
        int n_, m_;

        //constraints
        SparseMatrix<T> A_;
        std::vector<T> l_, u_;

        //cost function
        SparseMatrix<T> P_;
        std::vector<T> q_;

        ~QPProblem();
        void initialize(int n, int m);
        OSQPSolution* solve(int *error_code);
    };

    class Controller {
    private:
        QPProblem<c_float> qp_;

    public:
        /* controller parameters */
        HardConstraint constraint_;
        CostFunctionWeights weights_;
        Parameters parameters_;
        Model model_;

        Controller();

        void initialize(const Parameters &parameters, const Model &model, const HardConstraint &constraint, const CostFunctionWeights &weights);
        void update(const State &state, const State &linearize_point, const std::vector<PathPlanner::PoseStamped> &track_input, ControlOutput *out, std::vector<State> *pred_out);
    };

    class IterativeController : public Controller {
    public:
        void simulate(const State &state, State *next);
        void update(const State &state, const std::vector<PathPlanner::PoseStamped> &track_input, int iterations, double threshold, ControlOutput *out, std::vector<State> *pred_out);
    };
}

#endif