#include "InertiaEnergy.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

/** E_inertia = 1/2 * ||x - (x_n + hv_n)||_M,   x_tilde = x_n + hv_n */
double InertiaEnergy::val(Eigen::VectorXd x, Eigen::VectorXd x_tilde, Mesh& mesh) {
    double sum = 0.0;
    size_t nd = x.rows();
    for (size_t i = 0; i < nd; i++) {       // loops over nodes
        Eigen::VectorXd diff = x(i) - x_tilde(i);
        sum += 0.5 * mesh.m(i) * diff.dot(diff);
    }
    return sum;
}

/** grad E_inertia = M (x - x_tilde) */
Eigen::VectorXd InertiaEnergy::grad(Eigen::VectorXd x, Eigen::VectorXd x_tilde, Mesh& mesh) {
    // Allocate memory for gradient
    Eigen::MatrixXd g = Eigen::MatrixXd::Zeros(mesh.initial_positions.rows(), 
                                                    mesh.initial_positions.cols());

    // Loop over edges
    size_t n = mesh.initial_positions.rows();
    for (size_t i = 0; i < n; i++) {
        g(i) = mesh.m(i) * (x(i) - x_tilde(i));
    }
    return g;
}

/** hess E_inertia = M */
Eigen::SparseMatrix<double> InertiaEnergy::hess(Eigen::VectorXd x, Eigen::VectorXd x_tilde, Mesh& mesh) {
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(mesh.initial_positions.rows() * mesh.initial_positions.cols());       // 3 2n-lengthed arrays

    // Loop over edges
    size_t E = mesh.edges.size();
    size_t d = mesh.initial_positions.cols();
    for (size_t i = 0; i < E; i++) {
        for (size_t j = 0; j < d; j++) {
            triplets.push_back(Eigen::Triplet<double>(i * 2 + d, i * 2 + d, mesh.m[i]));
        }
    }

    // Initialize the sparse matrix (Size: 3x3)
    Eigen::SparseMatrix<double> mat(3, 3);
    
    // Populate the matrix
    mat.setFromTriplets(tripletList.begin(), tripletList.end());

    return mat;
}
