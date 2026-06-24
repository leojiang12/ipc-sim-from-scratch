#include "State.h"
#include "Mesh.h"

#include <Eigen/Dense>
#include <iostream>

void State::update_position(Eigen::VectorXd& x) {
    this->x_prev = this->x;
    this->x = x;
}

void State::update_velocity(Eigen::VectorXd& v) {
    this->v_prev = this->v;
    this->v = v;
}

State build_initial_state(Mesh& mesh) {

    State s;

    int n = mesh.initial_positions.size();
    int d = mesh.initial_positions[0].size();

    // Allocate/Initialize position/velocities as 0s vectors of length n*d
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(n * d);
    Eigen::VectorXd v0 = Eigen::VectorXd::Zero(n * d);

    Eigen::VectorXd x_prev = Eigen::VectorXd::Zero(n * d);
    Eigen::VectorXd v_prev = Eigen::VectorXd::Zero(n * d);

    // interleaved [x0,y0,x1,y1,...]
    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < d; j++) {
            x0(2*i + j) = mesh.initial_positions[i](j);
        }
    }

    s.x = x0;
    s.v = v0;
    s.x_prev = x_prev;
    s.v_prev = v_prev;

    return s;
}