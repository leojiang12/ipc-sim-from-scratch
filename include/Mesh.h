#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

struct Mesh {
    Eigen::MatrixXd initial_positions;              // n x d
    std::vector<Eigen::Vector3i> triangles;

    // useful for mass-spring
    std::vector<std::pair<int, int>> edges;
    Eigen::SparseMatrix<double> M;          // mass matrix                  dn x dn
    std::vector<double> k;          // stiffness of edge                    length e
    std::vector<double> l2;         // rest length squared of edge          length e

    int nx = 0;
    int ny = 0;

    void assign_masses(double rho);
};

Mesh make_square_grid_edges(int nx, int ny, double width, double height);

Mesh make_square_grid_triangles(int nx, int ny, double width, double height);
