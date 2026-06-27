#include "Mesh.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

struct InertiaEnergy {
    double val(Eigen::VectorXd x, Eigen::VectorXd x_tilde, Mesh& mesh);

    Eigen::VectorXd grad(Eigen::VectorXd x, Eigen::VectorXd x_tilde, Mesh& mesh);

    Eigen::SparseMatrix<double> hess(Eigen::VectorXd x, Eigen::VectorXd x_tilde, Mesh& mesh);
};