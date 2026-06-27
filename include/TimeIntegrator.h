#pragma once

#include "State.h"
#include "Mesh.h"
#include "DOFMap.h"

// include energies in the future

#include <Eigen/Dense>
#include <Eigen/Sparse>

/** Newton iteration loop */
void step_forward(State& current_state, Mesh& mesh, DOFMap& dofmap, double h, double tol);

/** IP Energy (value) computation */
double IP_val(Eigen::VectorXd& x_tilde, State& current_state, Mesh& mesh, double h);  

/** IP gradient computation */
Eigen::VectorXd IP_grad(Eigen::VectorXd& x_tilde, State& current_state, Mesh& mesh, double h);  

/** IP Hessian computation */
Eigen::SparseMatrix<double> IP_hess(Eigen::VectorXd& x_tilde, State& current_state, Mesh& mesh, double h);

/** Search direction computation per newton iteration */
Eigen::VectorXd search_dir(Eigen::VectorXd& x_tilde, State& current_state, Mesh& mesh, DOFMap &dofmap, double h);