#pragma once

#include "Mesh.h"
#include <Eigen/Dense>
#include <string>

// Writes one OBJ frame to <output_dir>/<frame>.obj.
//
// x    : current positions, flattened size 2n, interleaved [x0,y0,x1,y1,...].
// mesh : provides connectivity (mesh.triangles) — the single source of truth
//        for topology, so the writer never re-derives the triangulation.
//
// OBJ is 1-based, so triangle indices are written as (v+1). The mesh is 2D,
// so each vertex is emitted with z = 0.
void write_obj(const std::string& output_dir, int frame,
               const Eigen::VectorXd& x, const Mesh& mesh);
