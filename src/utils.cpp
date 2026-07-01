#include "utils.h"

#include <algorithm>
#include <iostream>

/** Positive Semi-Definite Projection */
Eigen::MatrixXd make_PSD(Eigen::MatrixXd& hess) {
    /*
        Neutralizes negative eigenvalues of the LOCAL Hessian corresponding
        to each edge, prior to assembling into the global matrix, so the
        global Hessian stays SPD for Newton / PCG.
    */
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(hess);
    Eigen::VectorXd eig = es.eigenvalues();
    for (int i = 0; i < eig.size(); ++i) {
        eig(i) = std::max(0.0, eig(i));
    }
    // U * clamp(E) * U^T
    return es.eigenvectors() * eig.asDiagonal() * es.eigenvectors().transpose();
}

/** Finite-difference gradient check (central differences). */
bool fd_check_grad(const std::function<double(const Eigen::VectorXd&)>& f,
                   const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& g,
                   const Eigen::VectorXd& x,
                   double eps, double tol) {
    const int n = static_cast<int>(x.size());

    // Build the FD gradient one coordinate at a time.
    Eigen::VectorXd grad_fd(n);
    for (int i = 0; i < n; ++i) {
        Eigen::VectorXd xp = x, xm = x;
        xp(i) += eps;
        xm(i) -= eps;
        grad_fd(i) = (f(xp) - f(xm)) / (2.0 * eps);   // central difference
    }

    const Eigen::VectorXd grad_an = g(x);             // analytical gradient

    // Relative error, with a floor so a near-zero gradient can't divide by 0.
    const double denom   = std::max(grad_an.norm(), 1e-12);
    const double rel_err = (grad_fd - grad_an).norm() / denom;

    std::cout << "fd_check_grad: relative error = " << rel_err
              << (rel_err < tol ? "  [PASS]" : "  [FAIL]") << "\n";
    std::cerr << "fd_check_grad: relative error = " << rel_err
              << (rel_err < tol ? "  [PASS]" : "  [FAIL]") << "\n";
    return rel_err < tol;
}
