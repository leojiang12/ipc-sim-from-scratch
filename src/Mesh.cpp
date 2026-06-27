#include "Mesh.h"

#include <Eigen/Dense>
#include <iostream>

Mesh make_square_grid_edges(int nx, int ny, double width, double height) {

    /* 
        Unit square with (n+1)^2 nodes with center at 0,0. 
            - Stores only edge elements.
    */
    Mesh mesh;
    mesh.initial_positions = Eigen::MatrixXd((nx + 1) * (ny + 1), 2);
    mesh.nx = nx;
    mesh.ny = ny;

    double square_center_x = 0.0;
    double square_center_y = 0.0;

    for (int i = 0; i <= ny; i++) {
        for (int j = 0; j <= nx; j++) {
            double x = width * (static_cast<double>(j) / nx - 0.5);
            double y = height * (static_cast<double>(i) / ny - 0.5);
            mesh.initial_positions.row(i*(nx + 1) + j) = Eigen::Vector2d(x + square_center_x, y + square_center_y);
        }
    }

    auto id = [nx](int i, int j) {
        return i * (nx + 1) + j;
    };

    /** EDGES (Mass Spring Model) */

    // 2 * n(n+1) (vertical + horizontal) + 2*n^2 (diagonal) edges

    // horizontal
    for (int i = 0; i <= ny; i++) {     // ny + 1 row nodes
        for (int j = 0; j < nx; j++) { // nx horizontal segments per row of nodes
            mesh.edges.push_back({id(i, j), id(i, j + 1)});
        }
    }

    // vertical
    for (int j = 0; j <= nx; j++) {     // nx + 1 col nodes
        for (int i = 0; i < ny; i++) { // ny horizontal segments per row of nodes
            mesh.edges.push_back({id(i, j), id(i + 1, j)});
        }
    }

    // diagonal (cross)
    for (int i = 0; i < ny; i++) {     // ny row boxes
        for (int j = 0; j < nx; j++) { // nx col boxes
            mesh.edges.push_back({id(i, j), id(i + 1, j + 1)});
            mesh.edges.push_back({id(i + 1, j), id(i, j + 1)});
        }
    }

    return mesh;
}

Mesh make_square_grid_triangles(int nx, int ny, double width, double height) {

    /* 
        Unit square with (n+1)^2 nodes with center at 0,0.
            - Stores only triangle elements
    */

    Mesh mesh;
    mesh.initial_positions = Eigen::MatrixXd((nx + 1) * (ny + 1), 2);
    mesh.nx = nx;
    mesh.ny = ny;

    double square_center_x = 0.0;
    double square_center_y = 0.0;

    for (int i = 0; i <= ny; i++) {
        for (int j = 0; j <= nx; j++) {
            double x = width * static_cast<double>(j) / nx;
            double y = height * static_cast<double>(i) / ny;
            mesh.initial_positions.row(i*(nx + 1) + j) = Eigen::Vector2d(x + square_center_x, y + square_center_y);
        }
    }

    auto id = [nx](int i, int j) {
        return i * (nx + 1) + j;
    };

    // Triangles for deformation

    for (int i = 0; i < ny; i++) {
        for (int j = 0; j < nx; j++) {
            int v00 = id(i, j);
            int v01 = id(i, j + 1);
            int v10 = id(i + 1, j);
            int v11 = id(i + 1, j + 1);

            // If i xor j is even
            if ((i % 2) ^ (j % 2)) {
                // upper right / lower left triangles
                mesh.triangles.push_back({v00, v10, v01});  // bl, br, tl
                mesh.triangles.push_back({v10, v11, v01});  // br, tr, tl
            } else {
                // upper left / lower right triangles 
                mesh.triangles.push_back({v00, v10, v11});  // bl, br, tr
                mesh.triangles.push_back({v00, v11, v01});  // bl, tr, t1
            }
        }
    }

    return mesh;
}
