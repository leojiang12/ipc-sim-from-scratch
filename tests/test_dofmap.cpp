#include "DOFMap.h"      // match your actual layout (see note below)
#include "Mesh.h"
#include "test_utils.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <string>


void print_apply_dirichlet_results(Eigen::SparseMatrix<double>& A_sp, Eigen::VectorXd& b)
{
    std::cout << "A_sp:" << std::endl;
    std::cout << A_sp << std::endl;
    std::cout << "b:" << std::endl;
    std::cout << b << std::endl;
}


void test1_check_apply_dirichlet()
{
    // 2D Square mesh (centered at 0,0)
    int n = 4;
    int d = 2;
    double length = 1.0;
    Mesh mesh = make_square_grid_edges(n, n, length, length);


    // Random Matrix / Vector
    Eigen::MatrixXd A = Eigen::MatrixXd::Ones(n*d, n*d);
    Eigen::VectorXd b = Eigen::VectorXd::Random(n*d);
    Eigen::SparseMatrix<double> A_sp = A.sparseView();

    print_apply_dirichlet_results(A_sp, b);

    // DOFMap Instantiation
    std::vector<int> fixed_nodes = {0, 3};

    std::cout << "\n***Zeroing out nodes:";
    for (int fixed_node : fixed_nodes)
        std::cout << fixed_node << " ";
    std::cout << "***\n"<< std::endl;

    DOFMap dofmap = make_dofmap_fixed_nodes(n, fixed_nodes);

    // Solutions
    Eigen::MatrixXd A_sol(n*d, n*d);
    A_sol << 1, 0, 0, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0, 0, 0,
             0, 0, 1, 1, 1, 1, 0, 0,
             0, 0, 1, 1, 1, 1, 0, 0,
             0, 0, 1, 1, 1, 1, 0, 0,
             0, 0, 1, 1, 1, 1, 0, 0,
             0, 0, 0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 0, 0, 1;

    Eigen::VectorXd b_sol = b;
    for (int node : fixed_nodes) {
        b_sol(2*node) = 0;
        b_sol(2*node + 1) = 0;
    }

    // DOFMap Test
    dofmap.apply_dirichlet(A_sp, b);

    print_apply_dirichlet_results(A_sp, b);

    CHECK(Eigen::MatrixXd(A_sp).isApprox(A_sol, 1e-10));  
    CHECK(b.isApprox(b_sol, 1e-10));  
}

int main(int argc, char *argv[]) {

    /*
        strcmp(*str1, *str2) returns 0 for equal strings and not 0 otherwise
    */

    if (argc == 1) {
        test1_check_apply_dirichlet();
    
        // --- T1: apply_dirichlet ---
    } else if (!strcmp(argv[1], "apply_dirichlet")) {
        test1_check_apply_dirichlet();
    }

    if (failures) { 
        std::cerr << failures << " checks failed\n"; 
        return 1; 
    }
    std::cout << "check applyDirichlet passed\n";
    return 0;
}