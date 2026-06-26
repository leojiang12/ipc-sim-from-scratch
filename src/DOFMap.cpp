#include "DOFMap.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <iostream>

// True if this global DOF index is Dirichlet-fixed.
bool DOFMap::fixed(int dof) const {
    return (this->is_fixed[dof] == 1);
}

void DOFMap::apply_dirichlet(Eigen::SparseMatrix<double>& A, Eigen::VectorXd& b, 
                            double diag_fill) const {        // check header default value
                   
    /** Fill diagonals with fill_diag if provide one greater than 0, 
        else compute mean of trace of A to use as filler. */
    double fill_val = diag_fill;

    if (diag_fill < 0) {
        // Compute trace of A
        double trace_mean = 0.0;
        for (size_t i = 0; i < (size_t)A.outerSize(); ++i) {
            for (Eigen::SparseMatrix<double>::InnerIterator it(A, i); it; ++it) {
                if (it.row() == it.col()) { // Check if it's on the main diagonal
                    trace_mean += it.value();
                    break;      // go to next col
                }
            }
        }
        trace_mean /= b.size();
        fill_val = trace_mean;
    }

    // Loop over all dirichlet dofs and nnz entries to 0-out cols/rows

    for (size_t i = 0; i < this->is_fixed.size(); i++) {

        // Do nothing if dof is not fixed
        if (!this->fixed(i))
            continue;

        int fixed_dof = i;

        // Assuming A is a Column-Major SparseMatrix<double>

        // Zero out the row of A 
        for (size_t j = 0; j < (size_t)A.outerSize(); j++) {
            for (Eigen::SparseMatrix<double>::InnerIterator it(A, j); it; ++it) {   // it(A, j) gets col j iterator
                if ((it.row() == fixed_dof) && (it.col() != fixed_dof)) {
                    it.valueRef() = 0.0;
                }
            }
        }
        // Zero out column and fill diagonals
        for (Eigen::SparseMatrix<double>::InnerIterator it(A, fixed_dof); it; ++it) {
            if (it.row() != fixed_dof) {
                it.valueRef() = 0.0;
            } else {
                it.valueRef() = fill_val;
            }
        }

        // Set b to 0 for fixed dofs
        b(fixed_dof) = 0.0;
    }

    // Remove any zero'd entries in A from sparse matrix
    A.prune(0.0);
    
}


// Pin entire nodes: fixes BOTH the x (2k) and y (2k+1) DOFs of each listed node.
DOFMap make_dofmap_fixed_nodes(int num_nodes, const std::vector<int>& fixed_nodes) {

    // Initialize DOFMap instance
    DOFMap dofmap;
    dofmap.num_dofs = 2 * num_nodes;
    dofmap.n_fixed = 0;
    dofmap.n_free = 2 * num_nodes;
    dofmap.is_fixed = std::vector<char> (2 * num_nodes, 0);
    
    for (size_t i = 0; i < fixed_nodes.size(); i++) {
        int node = fixed_nodes[i];
        dofmap.n_fixed += 2;
        dofmap.n_free -= 2;
        dofmap.is_fixed[2*node] = 1;       // x
        dofmap.is_fixed[2*node + 1] = 1;    // y
    }

    return dofmap;
}

// Per-DOF control: fixes exactly the listed global DOF indices 
DOFMap make_dofmap_fixed_dofs(int num_dofs, const std::vector<int>& fixed_dofs) {

    // Initialize DOFMap instance
    DOFMap dofmap;
    dofmap.num_dofs = num_dofs;
    dofmap.n_fixed = 0;
    dofmap.n_free = num_dofs;
    dofmap.is_fixed = std::vector<char> (num_dofs, 0);

    for (size_t i = 0; i < (size_t)fixed_dofs.size(); i++) {
        int dof = fixed_dofs[i];
        dofmap.n_fixed += 1;
        dofmap.n_free -= 1;
        dofmap.is_fixed[dof] = 1;
    }
    return dofmap;
}
