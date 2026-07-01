#pragma once

#include "Mesh.h"
#include "DOFMap.h"
#include "State.h"

// include energies in the future

#include <Eigen/Dense>
#include <Eigen/Sparse>

/** Newton iteration loop */
int step_forward(State& current_state, const Mesh& mesh, const DOFMap& dofmap, double h, double tol);

/** IP Energy (value) computation */
double IP_val(const Eigen::VectorXd& x, const Eigen::VectorXd& x_tilde, const Mesh& mesh, double h);  

/** IP gradient computation */
Eigen::VectorXd IP_grad(const Eigen::VectorXd& x, const Eigen::VectorXd& x_tilde, const Mesh& mesh, double h);  

/** IP Hessian computation */
Eigen::SparseMatrix<double> IP_hess(const Eigen::VectorXd& x, const Eigen::VectorXd& x_tilde, const Mesh& mesh, double h);

/** Search direction computation per newton iteration */
Eigen::VectorXd search_dir(const Eigen::VectorXd& x, const Eigen::VectorXd& x_tilde, const Mesh& mesh, const DOFMap &dofmap, double h);