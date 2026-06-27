#include "State.h"
#include "Mesh.h"

#include <Eigen/Dense>
#include <iostream>

void State::update_position(Eigen::VectorXd& x_new) {
    this->x_prev = this->x;
    this->x = x_new;
}

void State::update_velocity(Eigen::VectorXd& v_new) {
    this->v_prev = this->v;
    this->v = v_new;
}

State build_initial_state(Mesh& mesh) {

    State s;

    size_t n = mesh.initial_positions.rows();
    size_t d = mesh.initial_positions.cols();

    // Allocate/Initialize position/velocities as 0s vectors of length n*d
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(n * d);
    Eigen::VectorXd v0 = Eigen::VectorXd::Zero(n * d);

    Eigen::VectorXd x_prev = Eigen::VectorXd::Zero(n * d);
    Eigen::VectorXd v_prev = Eigen::VectorXd::Zero(n * d);

    // interleaved [x0,y0,x1,y1,...] and loop as column major (Eigen default)
    for (size_t i = 0; i < d; i++) {
        for (size_t j = 0; j < n; j++) {
            x0(d*i + j) = mesh.initial_positions(j, i);
        }
    }

    s.x = x0;
    s.v = v0;
    s.x_prev = x_prev;
    s.v_prev = v_prev;

    return s;
}