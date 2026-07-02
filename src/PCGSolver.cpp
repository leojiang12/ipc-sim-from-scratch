#include "PCGSolver.h"

// #include <vector>

using MatVec = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;

/** Preconditioned conjugate gradients CPU implementation. */
PCGResult pcg(const MatVec& A, const Eigen::VectorXd& b, const PCGOptions& opts)
{
    /*
     * Implements Jonathan Shewchuck's PCG Algorithm B3.
    */

    // Load options
    double tol = opts.tol;
    size_t maxit = opts.maxit;
    Eigen::VectorXd x0 = opts.x0;
    MatVec apply_precond;

    // Empty x0 -> zero vector
    if (x0.size() == 0) {
        x0 = Eigen::VectorXd::Zero(b.size());
    }

    // Empty preconditioner -> identity operator
    if (opts.apply_precond) {
        apply_precond = opts.apply_precond;
    } else {    // Identity operator
        apply_precond = [&](const Eigen::VectorXd &x) {return x;}; 
    }

    // Store abs residual norms per iteration
    std::vector<double> resvec;

    size_t i = 0;
    Eigen::VectorXd x = x0;
    Eigen::VectorXd r = b - A(x0);

    Eigen::VectorXd d = apply_precond(r);          
    double delta_new = r.transpose().dot(d);        // r_0^T M_inv r_0
    double delta_0 = delta_new;

    double rr_0 = r.squaredNorm();

    // Trivially converged 
    if (rr_0 == 0.0) {
        PCGResult result;
        result.x = x;           // x0
        result.flag = 0;        
        result.iters = 0;
        result.relres = 0.0;    
        result.resvec = {0.0};
        return result;
    }

    resvec.push_back(std::sqrt(rr_0 / rr_0));      // store relative residual in unpreconditioned space

    // Loops while less than max iterations and relative residual norm less than tol.
    while ((i < maxit) && (delta_new > (tol*tol*delta_0))) {
        Eigen::VectorXd q = A(d);
        double alpha = delta_new / (d.transpose().dot(q));
        x += alpha * d;

        if (i % 50 == 0) {    // Refresh every 50 iterations
            r = b - A(x);
        }
        else {               // Use recurrrence relation
            r = r - alpha * q;
        }

        Eigen::VectorXd s = apply_precond(r);

        double delta_old = delta_new;
        delta_new = r.transpose().dot(s);
        resvec.push_back(std::sqrt(r.squaredNorm() / rr_0));

        double beta = delta_new / delta_old;
        d = s + beta * d;
        i++;
    }

    // Package results
    int flag = 1;

    // Convergence check (0 converged, 1 not converged)
    if ((i < maxit) && (delta_new < (tol*tol*delta_0)))
        flag = 0;

    PCGResult result;
    result.x = x;
    result.flag = flag;
    result.relres = std::sqrt(r.squaredNorm() / rr_0);
    result.iters = i;
    result.resvec = resvec;

    return result;
}
