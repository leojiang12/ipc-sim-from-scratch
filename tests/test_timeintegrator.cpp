#include "TimeIntegrator.h"
#include "testing_utils.h"

#include <Eigen/Dense>

#include <iostream>
#include <string>


void test1_check_time_integrator()
{
    // setup
    Mesh mesh = make_square_grid_edges(2, 2, 1.0, 1.0);
    mesh.assign_masses(1.0);
    int nd = mesh.initial_positions.size();

    State s = build_initial_state(mesh);
    s.v = Eigen::VectorXd::Ones(nd);        // uniform constant velocity
    Eigen::VectorXd x_n = s.x, v_n = s.v;
    double h = 0.01;

    DOFMap dofmap = make_dofmap_fixed_dofs(nd, {});   // NO fixed DOFs -> pure inertia

    // act
    step_forward(s, mesh, dofmap, h, 1e-8);

    // assert: free particles coast at constant velocity
    Eigen::VectorXd x_tilde = x_n + h * v_n;
   
    CHECK((s.x - x_tilde).norm() < 1e-6);     // moved exactly to x_tilde
    CHECK((s.v - v_n).norm()     < 1e-6);     // velocity unchanged
    CHECK(IP_grad(s.x, x_tilde, mesh, h).norm() < 1e-8);  // Check stationarity
}

int main(int argc, char *argv[]) {

    /*
        strcmp(*str1, *str2) returns 0 for equal strings and not 0 otherwise
    */

    if (argc == 1) {
        test1_check_time_integrator();
    
        // --- T1: time_integrator ---
    } else if (!strcmp(argv[1], "time_integrator")) {
        test1_check_time_integrator();
    }

    if (failures) { 
        std::cerr << failures << " checks failed\n"; 
        return 1; 
    } else {
        std::cout << "check time_integrator passed\n";
        return 0;
    }
}