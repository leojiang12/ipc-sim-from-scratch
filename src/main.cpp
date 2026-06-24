#include "Mesh.h"
#include "State.h"
#include "PCGSolver.h"
#include "Exporter.h"

#include <Eigen/Dense>
#include <iostream>

int main() {

    // Square mesh (centered at 0,0)
    int n = 4;
    double length = 1.0;
    Mesh mesh = make_square_grid_edges(n, n, length, length);
    std::cout << "vertices: " << mesh.initial_positions.size() << "\n";

    // Build initial state
    State state = build_initial_state(mesh);
    //Eigen::VectorXd initial_velocities = Eigen::VectorXd::Zero(state.x.size());     // v0 = 0 by default on initialization, use for arbitrary velocity

    std::string output_dir = "output";
    int frame = 0;
    write_obj(output_dir, frame, state.x, mesh);
    std::cout << "wrote frame 0 to " << "output_dir" << "/" << frame << ".obj" << std::endl;


    return 0;
}