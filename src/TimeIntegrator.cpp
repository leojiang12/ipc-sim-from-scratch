#include "TimeIntegrator.h"

#include "Mesh.h"
#include "State.h"
#include "DOFMap.h"
#include "PCGSolver.h"

#include "InertiaEnergy.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <memory>


/** Newton iteration loop */
int step_forward(State& current_state, const Mesh& mesh, const DOFMap& dofmap, double h, double tol) {

    Eigen::VectorXd x_tilde = current_state.x + h * current_state.v;
    Eigen::VectorXd x_n = current_state.x;      // deep copy of x
    Eigen::VectorXd x(current_state.x);

    // Newton loop
    size_t iter = 0;
    double E_last = IP_val(x, x_tilde, mesh, h);
    Eigen::VectorXd p = search_dir(x, x_tilde, mesh, dofmap, h);

    while ((p.lpNorm<Eigen::Infinity>() / h) > tol ) {
        printf("Iteration %lu :", iter);
        printf("residual = %lf", p.lpNorm<Eigen::Infinity>() / h);

        // backtracking line search
        double alpha = 1.0;

        while (IP_val(x + alpha * p, x_tilde, mesh, h) > E_last) {
            alpha /= 2.0;
        }
        printf("step size = %lf", alpha);

        x += alpha * p;
        E_last = IP_val(x, x_tilde, mesh, h);
        p = search_dir(x, x_tilde, mesh, dofmap, h);
        iter++;
    }
    current_state.v_prev = current_state.v;
    current_state.v = (x - x_n) / h;
    current_state.x = x;

    return iter;
}

/** IP Energy (value) computation */
double IP_val(const Eigen::VectorXd& x, const Eigen::VectorXd& x_tilde, const Mesh& mesh, double h) {
    return InertiaEnergy::val(x, x_tilde, mesh);
}  

/** IP gradient computation */
Eigen::VectorXd IP_grad(const Eigen::VectorXd& x, const Eigen::VectorXd& x_tilde, const Mesh& mesh, double h) {
    return InertiaEnergy::grad(x, x_tilde, mesh);
}  

/** IP Hessian computation */
Eigen::SparseMatrix<double> IP_hess(const Eigen::VectorXd& x, const Eigen::VectorXd& x_tilde, const Mesh& mesh, double h) {
    return InertiaEnergy::hess(x, x_tilde, mesh);
}

/** Search direction computation per newton iteration */
Eigen::VectorXd search_dir(const Eigen::VectorXd& x, const Eigen::VectorXd& x_tilde,  
                                const Mesh& mesh, const DOFMap& dofmap, double h) {
    Eigen::SparseMatrix<double> projected_hess = IP_hess(x, x_tilde, mesh, h);
    Eigen::VectorXd grad = IP_grad(x, x_tilde, mesh, h);//.reshaped<Eigen::RowMajor>(x.size(), 1);

    // Eliminate Dirichlet BC DOFs by 0-ing corresponding gradient and Hessian entries.
    dofmap.apply_dirichlet(projected_hess, grad);

    // PCG solver options
    PCGOptions opts;        // tol=1e-6, maxit=1000, x0=0, no precond

    // Matrix-Vector operator
    auto matvec = [&projected_hess](const Eigen::VectorXd& y) { 
        return projected_hess * y; 
    };

    // Solves a Ax = b: shapes (dn x dn), (dn x 1), and (dn x 1)
    PCGResult r = pcg(matvec, -grad, opts);   // A x = b  -->  P p = -g : P is proj hess, g is grad, p is our search direction

    if (r.flag != 0) {
        puts("pcg did not converge!!!");
        exit(1);
    } else {
        return r.x;//.reshaped<Eigen::RowMajor>(mesh.initial_positions.rows(), 
                 //                           mesh.initial_positions.cols());    // to n x d
    }
}