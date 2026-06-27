#include "TimeIntegrator.h"

#include "Mesh.h"
#include "State.h"
#include "DOFMap.h"
#include "PCGSolver.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <memory>

/** Forward declarations to tell the compiler these 
    functions exist but defined after step_foward() */
double IP_val(Eigen::VectorXd& x_tilde, State& current_state, Mesh& mesh, double h);  
Eigen::VectorXd IP_grad(Eigen::VectorXd& x_tilde, State& current_state, Mesh& mesh, double h);  
Eigen::SparseMatrix<double> IP_hess(Eigen::VectorXd& x_tilde, State& current_state, Mesh& mesh, double h);  
Eigen::VectorXd search_dir(Eigen::VectorXd& x_tilde, State& current_state, Mesh& mesh, DOFMap& dofmap, double h);


/** Newton iteration loop */
void step_forward(State& current_state, Mesh& mesh, DOFMap& dofmap, double h, double tol) {

    Eigen::VectorXd x_tilde = current_state.x + h * current_state.v;
    Eigen::VectorXd x_n = current_state.x;      // deep copy of x

    // Newton loop
    size_t i = 0;
    //double E_last = IP_val(current_state, mesh, h);
    // p = search_dir(current_state, mesh, dofmap, h)

    // while ((p.lpNorm<Eigen::Infinity>() / h) > tol )

}

/** IP Energy (value) computation */
double IP_val(Eigen::VectorXd& x_tilde, State& current_state, Mesh& mesh, double h) {
    return 1.0;
}  

/** IP gradient computation */
Eigen::VectorXd IP_grad(Eigen::VectorXd& x_tilde, State& current_state, Mesh& mesh, double h) {
    return Eigen::VectorXd::Ones(10);
}  

/** IP Hessian computation */
Eigen::SparseMatrix<double> IP_hess(Eigen::VectorXd& x_tilde, State& current_state, 
                                        Mesh& mesh, double h) {

    return Eigen::MatrixXd::Ones(10, 10).sparseView();
}

/** Search direction computation per newton iteration */
Eigen::VectorXd search_dir(Eigen::VectorXd& x_tilde, State& current_state, 
                                Mesh& mesh, DOFMap& dofmap, double h) {
    Eigen::SparseMatrix<double> projected_hess = IP_hess(x_tilde, current_state, mesh, h);
    Eigen::VectorXd grad = IP_grad(x_tilde, current_state, mesh, h);

    dofmap.apply_dirichlet(projected_hess, grad);
}