#include <Eigen/Dense>
#include <iostream>

/** Positive Semi-Definite Projection */

Eigen::MatrixXd make_PSD(Eigen::MatrixXd& hess) {
    /*
        Neutralizes negative eigenvalues of the LOCAL Hessian corresponding 
        to each edge.

        This is done prior to incorporating these local Hessians into the global matrix.

        This process involves setting negative eigenvalues to zero, thus ensuring that the 
        resulting global Hessian matrix adheres more closely to the desired SPD properties.

        This modification is crucial for Newton's method.
    */

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(hess);
    Eigen::VectorXd eig = es.eigenvalues();
    size_t n = eig.size();
    for (size_t i = 0; i < n; i++) {
        eig(i) = std::max(0.0, eig(i));
    }
    
    // U E_clamped V^T
    return es.eigenvectors() * eig.asDiagonal() * es.eigenvectors().transpose();
}

/** Finite difference checker using central differences */