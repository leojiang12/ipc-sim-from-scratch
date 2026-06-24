#pragma once

#include <Eigen/Dense>
#include <vector>
#include <functional>

using MatVec = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;

struct PCGOptions {
    double tol = 1e-6;
    int maxit = 1000;
    Eigen::VectorXd x0;         // empty defaults to 0s
    MatVec apply_precond;       // empty defaults to identity
};

struct PCGResult {
    Eigen::VectorXd x;
    int flag = 1;        // 0 converged, 1 maxit, (2 breakdown?)
    double relres = -1.0;
    int iters  = 0;
    std::vector<double> resvec;   // ‖b − A x_k‖ per iteration, resvec[0] = initial abs residual norm
};

PCGResult pcg(const MatVec& A, const Eigen::VectorXd& b, const PCGOptions& opts);
