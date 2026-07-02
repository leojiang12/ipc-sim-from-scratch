#include "InertiaEnergy.h"
#include "Mesh.h"
#include "utils.h"
#include "testing_utils.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <string>



void test1_check_inertia_energy()
{

    // 2D Square mesh (centered at 0,0)
    int nseg = 4;
    int d = 2;
    double length = 1.0;
    double rho = 1000;      // kg / m^2 for 2d
    Mesh mesh = make_square_grid_edges(nseg, nseg, length, length);
    mesh.assign_masses(rho);
    
    int n = mesh.initial_positions.rows();      // num nodes
    Eigen::VectorXd x_tilde = Eigen::VectorXd::Random(n * d);

    auto f = [&](const Eigen::VectorXd& z){ return InertiaEnergy::val(z, x_tilde, mesh); };
    auto g = [&](const Eigen::VectorXd& z){ return InertiaEnergy::grad(z, x_tilde, mesh); };
        
    Eigen::VectorXd x = Eigen::VectorXd::Random(n * d);

    CHECK(fd_check_grad(f, g, x));  // eps = 1e-6, tol = 1e-5 by default

}

int main(int argc, char *argv[]) {

    /*
        strcmp(*str1, *str2) returns 0 for equal strings and not 0 otherwise
    */

    if (argc == 1) {
        test1_check_inertia_energy();
    
        // --- T1: inertia energy ---
    } else if (!strcmp(argv[1], "inertia")) {
        test1_check_inertia_energy();
    }

    if (failures) { 
        std::cerr << failures << " checks failed\n"; 
        return 1; 
    } else {
        std::cout << "check inertia_energy passed\n";
        return 0;
    }
}