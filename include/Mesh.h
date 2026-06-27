#pragma once

#include "Mesh.h"

#include <Eigen/Dense>
#include <vector>

struct Mesh {
    std::vector<Eigen::Vector2d> initial_positions;
    std::vector<Eigen::Vector3i> triangles;

    // useful for mass-spring
    std::vector<std::pair<int, int>> edges;
    std::vector<double> m;          // mass of node (point particle model)
    std::vector<double> k;          // stiffness
    std::vector<double> l2;         // rest length squared of edge

    int nx = 0;
    int ny = 0;
};

Mesh make_square_grid_edges(int nx, int ny, double width, double height);

Mesh make_square_grid_triangles(int nx, int ny, double width, double height);
