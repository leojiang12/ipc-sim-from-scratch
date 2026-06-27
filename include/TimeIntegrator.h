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
double IP_val(const Eigen::VectorXd& x, Eigen::VectorXd& x_tilde, Mesh& mesh, double h);  

/** IP gradient computation */
Eigen::VectorXd IP_grad(const Eigen::VectorXd& x, Eigen::VectorXd& x_tilde, Mesh& mesh, double h);  

/** IP Hessian computation */
Eigen::SparseMatrix<double> IP_hess(const Eigen::VectorXd& x, Eigen::VectorXd& x_tilde, Mesh& mesh, double h);

/** Search direction computation per newton iteration */
Eigen::VectorXd search_dir(const Eigen::VectorXd& x, Eigen::VectorXd& x_tilde, Mesh& mesh, DOFMap &dofmap, double h);