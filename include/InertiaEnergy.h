#include "Mesh.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

/** const is used to promise the compiler these functions will only read from
    these argument references. */

struct InertiaEnergy {
    static double val(const Eigen::VectorXd& x, const Eigen::VectorXd& x_tilde, const Mesh& mesh);

    static Eigen::VectorXd grad(const Eigen::VectorXd& x, const Eigen::VectorXd& x_tilde, const Mesh& mesh);

    static Eigen::SparseMatrix<double> hess(const Eigen::VectorXd& x, const Eigen::VectorXd& x_tilde, const Mesh& mesh);
};