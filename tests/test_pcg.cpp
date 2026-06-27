#include "PCGSolver.h"

#include <Eigen/Dense>
#include <iostream>
#include <string>

static int failures = 0;
#define CHECK(cond) \
    do { if (!(cond)) { \
    std::cerr << "FAIL: " #cond "  (line " << __LINE__ << ")\n"; ++failures; \
    } } while (0)

void print_pcg_results(PCGResult& r)
{
    std::cerr << "PCG Result Summary " << r.flag << std::endl;
    std::cerr << "flag: " << r.flag << std::endl;
    std::cerr << "relres: " << r.relres << std::endl;
    std::cerr << "iters: " << r.iters << std::endl;
}

void test1_identity()
{
    // Initialize A, x*, and b
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd x_star(3); 
    x_star << 1, 2, 3;
    Eigen::VectorXd b = A * x_star;

    // Matrix-Vector operator
    auto matvec = [&A](const Eigen::VectorXd& x) { 
        return A * x; 
    };

    // PCG options
    PCGOptions opts; 
    opts.tol = 1e-10; 
    opts.maxit = 100;

    // Solve
    PCGResult r = pcg(matvec, b, opts);
    print_pcg_results(r);
    CHECK(r.flag == 0);                       // converged
    CHECK((r.x - x_star).norm() / b.norm() < 1e-10);
    CHECK(r.iters == 1);                      // I = trivial, one step
}

void test2_small_spd()
{
    // Initialize A, x*, b
    Eigen::MatrixXd A(2, 2); 
    A << 4, 1, 1, 3;
    Eigen::VectorXd x_star(2); 
    x_star << 1, 2;
    Eigen::VectorXd b = A * x_star;

    // Matrix-Vector operator
    auto matvec = [&A](const Eigen::VectorXd& x) { 
        return A * x; 
    };

    // PCG Options
    PCGOptions opts; 
    opts.tol = 1e-10; 
    opts.maxit = 100;

    // Solver
    PCGResult r = pcg(matvec, b, opts);
    print_pcg_results(r);
    CHECK((r.x - x_star).norm() / b.norm() < 1e-8);      // error loose vs residual tol (conditioning!)
    CHECK(r.iters <= 2);                      // finite termination, n = 2
}

void test3_random_50x50_spd()
{
    int n = 50;

    // Initialize A, x*, b
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(n, n);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    A = A * A.transpose() + 1e-4 * I;
    Eigen::VectorXd x_star = Eigen::VectorXd::Random(n); 
    Eigen::VectorXd b = A * x_star;


    // Matrix-Vector operator
    auto matvec = [&A](const Eigen::VectorXd& x) { 
        return A * x; 
    };

    // PCG Options
    PCGOptions opts; 
    opts.tol = 1e-10; 
    opts.maxit = 100;

    // Solver
    PCGResult r = pcg(matvec, b, opts);
    print_pcg_results(r);
    CHECK((r.x - x_star).norm() / b.norm() < 1e-10);  
    CHECK(r.iters <= opts.maxit);                      
}

void test4_random_500x500_spd()
{
    int n = 500;

    // Initialize A, x*, b
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(n, n);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    A = A * A.transpose() + 1e-4 * I;
    Eigen::VectorXd x_star = Eigen::VectorXd::Random(n); 
    Eigen::VectorXd b = A * x_star;

    // Matrix-Vector operator
    auto matvec = [&A](const Eigen::VectorXd& x) { 
        return A * x; 
    };

    // PCG Options
    PCGOptions opts; 
    opts.tol = 1e-10; 
    opts.maxit = 1000;

    // Solver
    PCGResult r = pcg(matvec, b, opts);
    print_pcg_results(r);
    CHECK((r.x - x_star).norm() / b.norm() < 1e-10);  
    CHECK(r.iters <= opts.maxit);                      
}

int main(int argc, char *argv[]) {

    /*
        strcmp(*str1, *str2) returns 0 for equal strings and not 0 otherwise
    */

    if (argc == 1) {
        test1_identity();
        test2_small_spd();
        test3_random_50x50_spd();
        test4_random_500x500_spd();

    // --- T0: identity, plumbing ---
    } else if (!strcmp(argv[1], "identity")) {
        test1_identity();
    
    // --- T1: tiny known SPD ---
    } else if (!strcmp(argv[1], "small_spd")) {
        test2_small_spd();

    // --- T2: 50 x 50 random SPD---
    } else if (!strcmp(argv[1], "random_50x50_spd")) {
        test3_random_50x50_spd();
    
    // --- T3: 500 x 500 random SPD ---
    } else if (!strcmp(argv[1], "random_500x500_spd")) {
        test4_random_500x500_spd();
    
    }



    if (failures) { 
        std::cerr << failures << " checks failed\n"; 
        return 1; 
    }
    std::cout << "all PCG tests passed\n";
    return 0;
}