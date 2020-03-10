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

    class Controller {
        private:
            OSQPWorkspace *osqp_workspace_ = nullptr;
            OSQPSettings  *osqp_settings_= nullptr;
            OSQPData      *osqp_data_ = nullptr;

            HardConstraint constraint_;
            CostFunctionWeights weights_;
            Parameters parameters_;
            Model model_;

        public:
            Controller(const Parameters &parameters, const Model &model, const HardConstraint &constraint, const CostFunctionWeights &weights);
            void update(const State &state, const std::vector<PathPlanner::PoseStamped> &track_input, ControlOutput *out);
    };
}

#endif