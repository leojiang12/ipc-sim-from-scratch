#include <Eigen/Dense>
#include <iostream>

/** Positive Semi-Definite Projection */
Eigen::MatrixXd make_PSD(Eigen::MatrixXd& hess);

/** Finite difference checker using central differences */
bool check_FD(const Eigen::VectorXd& x, const Eigen::VectorXd& v);