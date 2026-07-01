#include "InertiaEnergy.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

/** 

    E_inertia = 1/2 * ||x - (x_n + hv_n)||_M,   x_tilde = x_n + hv_n 

    - M is an (2n x 2n) matrix. 
    - x and x_tilde are (2n x 1) interleaved position vectors.

*/
double InertiaEnergy::val(const Eigen::VectorXd& x, const Eigen::VectorXd& x_tilde, const Mesh& mesh) {
    Eigen::VectorXd diff = x - x_tilde;
    return 0.5 * (diff.transpose() * mesh.M * diff).value();
}

/** 

    grad E_inertia = M (x - x_tilde) 

    - M is an (2n x 2n) matrix. 
    - x and x_tilde are (2n x 1) interleaved position vectors.

*/
Eigen::VectorXd InertiaEnergy::grad(const Eigen::VectorXd& x, const Eigen::VectorXd& x_tilde, const Mesh& mesh) {
    return mesh.M * (x - x_tilde);
}

/** 

    hess E_inertia = M 

    - M is an (2n x 2n) matrix. 
    - x and x_tilde are (2n x 1) interleaved position vectors.

*/
Eigen::SparseMatrix<double> InertiaEnergy::hess(const Eigen::VectorXd& x, const Eigen::VectorXd& x_tilde, const Mesh& mesh) {
    
    // x and x_tilde are unused here.
    (void)x;
    (void)x_tilde;
    
    return mesh.M;
}
