#pragma once

#include <Eigen/Dense>
#include <functional>

/** Positive Semi-Definite Projection (clamps negative eigenvalues to 0). */
Eigen::MatrixXd make_PSD(Eigen::MatrixXd& hess);

/** Central-difference gradient check: verifies  g(x) ≈ ∂f/∂x  at x.
 *
 *  f : R^n -> R    (e.g. an energy val, with all other params curried in)
 *  g : R^n -> R^n  (the claimed gradient)
 *
 *  Builds the finite-difference gradient one coordinate at a time
 *  (central difference, O(eps^2) accurate) and compares it to g(x).
 *  Returns true if the RELATIVE error is below tol; prints the error.
 *
 *  eps ~ 1e-6 is the sweet spot for doubles (truncation vs roundoff).
 *  Evaluate at an x where g(x) != 0 (a generic point) for a meaningful test.
 */
bool fd_check_grad(const std::function<double(const Eigen::VectorXd&)>& f,
                   const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& g,
                   const Eigen::VectorXd& x,
                   double eps = 1e-6, double tol = 1e-5);
