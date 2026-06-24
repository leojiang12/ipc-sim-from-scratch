#pragma once

#include "Mesh.h"

#include <Eigen/Dense>

struct State {
    Eigen::VectorXd x;      // current positions, size 2n
    Eigen::VectorXd x_prev; // previous positions, optional

    Eigen::VectorXd v;      // velocities, size 2n
    Eigen::VectorXd v_prev; // previous velocities, optional

    void update_position(Eigen::VectorXd& x);
    void update_velocity(Eigen::VectorXd& v);
};

State build_initial_state(Mesh& mesh);