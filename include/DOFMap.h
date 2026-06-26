#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

// Dirichlet BC bookkeeping over a flattened, interleaved DOF vector
// (node k -> DOFs 2k, 2k+1). Fixing is per-DOF (supports roller BCs).
// Strategy: zeroing, not reduction — system stays full size (num_dofs) so
// exported matrices are uniformly sized and mesh-aligned across BC configs.
struct DOFMap {
    int num_dofs = 0;            // = 2 * num_nodes
    int n_fixed  = 0;            // counts dofs
    int n_free   = 0;            // counts dofs
    std::vector<char> is_fixed;  // size num_dofs; 1 = fixed, 0 = free

    bool fixed(int dof) const;

    // Neutralize each fixed DOF i in place, preserving symmetry:
    //   zero row+col i; A(i,i)=diag_fill;
    //   b(i)=0
    // diag_fill : <=0 uses mean(trace(A)) from the original diagonal.
    // Pre: A SPD, size num_dofs. Post: A symmetric; A·x unchanged on free DOFs.
    void apply_dirichlet(Eigen::SparseMatrix<double>& A,
                         Eigen::VectorXd& b,
                         double diag_fill = -1.0) const;
};

// Pin whole nodes
DOFMap make_dofmap_fixed_nodes(int num_nodes, const std::vector<int>& fixed_nodes);

// Pin specific global DOF indices 
DOFMap make_dofmap_fixed_dofs(int num_dofs, const std::vector<int>& fixed_dofs);
