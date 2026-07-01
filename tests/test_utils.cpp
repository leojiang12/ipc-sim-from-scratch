#include "utils.h"
#include "test_utils.h"

#include <Eigen/Dense>

#include <iostream>
#include <string>


void print_mat(Eigen::MatrixXd& A)
{
    std::cout << A << std::endl;
}


void test1_check_make_PSD()
{
    /* 
        symmetric 3 x 3 matrix
        
        1 2 3
        2 5 11
        3 1 0

        with eigenvalues:
        2 - sqrt(21)    ~ -2.58258
        2               ~ 2
        2 + sqrt(21)    ~ 6.582258
    
        should get new eigenvalues of 
        0
        2
        6.582258

    */
    Eigen::MatrixXd A(3, 3);
    A << 1, 2, 3,
         2, 5, 1,
         3, 1, 0;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigsolver_A(A);
    Eigen::VectorXd eigval_A = eigsolver_A.eigenvalues();
    
    print_mat(A);
    std::cout << "Eigenvalues:" << std::endl;
    std::cout << eigval_A << std::endl;

    // Run make_PSD
    Eigen::MatrixXd B = make_PSD(A);

    // Extract eigenvalues to check
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigsolver_B(B);
    Eigen::VectorXd s = eigsolver_B.eigenvalues();

    std::cout << "positive semi-definite projected matrix:" << std::endl;
    print_mat(B);
    std::cout << "New eigenvalues:" << std::endl;
    std::cout << s << std::endl;

    // Solutions
    Eigen::VectorXd sol(3);
    sol << 0.0, 2.0, 2.0 + std::sqrt(21);

    std::cout << "sol:" << std::endl;
    std::cout << sol << std::endl;


    CHECK(s.isApprox(sol, 1e-10));  
}

int main(int argc, char *argv[]) {

    /*
        strcmp(*str1, *str2) returns 0 for equal strings and not 0 otherwise
    */

    if (argc == 1) {
        test1_check_make_PSD();
    
        // --- T1: make_PSD ---
    } else if (!strcmp(argv[1], "make_PSD")) {
        test1_check_make_PSD();
    }

    if (failures) { 
        std::cerr << failures << " checks failed\n"; 
        return 1; 
    }
    std::cout << "check make_PSD passed\n";
    return 0;
}