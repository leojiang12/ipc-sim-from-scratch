#include "Exporter.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>

void write_obj(const std::string& output_dir, int frame,
               const Eigen::VectorXd& x, const Mesh& mesh)
{
    std::filesystem::create_directories(output_dir);

    const std::string filename = output_dir + "/" + std::to_string(frame) + ".obj";
    std::ofstream f(filename);
    if (!f) {
        throw std::runtime_error("write_obj: cannot open " + filename);
    }
    f << std::fixed << std::setprecision(6);

    // Vertices: x is flattened 2n, interleaved [x0,y0,x1,y1,...]; 2D -> z = 0.
    const int n = static_cast<int>(x.size() / 2);
    for (int k = 0; k < n; ++k) {
        f << "v " << x[2 * k] << " " << x[2 * k + 1] << " 0.0\n";
    }

    // Use if triangles are used (for deformations)
    if (mesh.triangles.size() != 0) {
        // Faces: serialize the mesh's own triangle list, +1 for OBJ's 1-based indexing.
        for (const Eigen::Vector3i& tri : mesh.triangles) {
            f << "f " << tri[0] + 1 << " " << tri[1] + 1 << " " << tri[2] + 1 << "\n";
        }

    } else {    // mass spring

        int nx = mesh.nx;
        auto id = [nx](int i, int j) {
            return i * (nx + 1) + j;
        };
        
        for (size_t i = 0; i < mesh.ny; i++) {
            for (size_t j = 0; j < mesh.nx; j++) {

                int v00 = id(i, j);
                int v01 = id(i, j + 1);
                int v10 = id(i + 1, j);
                int v11 = id(i + 1, j + 1);

                // bl br tr
                // bl tr tl
                f << "f " << v01 + 1 << " " << v10 + 1 << " " << v11 + 1 << "\n";
                f << "f " << v00 + 1 << " " << v11 + 1 << " " << v01 + 1 << "\n";
            }
        }

    }
}
