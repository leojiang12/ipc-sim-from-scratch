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

                int v0, v1, v2, v3, v4, v5;

                int v00 = id(i, j);
                int v01 = id(i, j + 1);
                int v10 = id(i + 1, j);
                int v11 = id(i + 1, j + 1);

                // If i xor j is even
                if ((i % 2) ^ (j % 2)) {
                    v0 = v00;
                    v1 = v10;
                    v2 = v01;

                    v3 = v10;
                    v4 = v11;
                    v5 = v01;
                //    mesh.triangles.push_back({v00, v10, v01});  // bl, br, tl
                //    mesh.triangles.push_back({v10, v11, v01});  // br, tr, tl
                } else {
                    v0 = v00;
                    v1 = v10;
                    v2 = v11;

                    v3 = v00;
                    v4 = v11;
                    v5 = v01;
                //  mesh.triangles.push_back({v00, v10, v11});  // bl, br, tr
                //  mesh.triangles.push_back({v00, v11, v01});  // bl, tr, t1
                }

                f << "f " << v0 + 1 << " " << v1 + 1 << " " << v2 + 1 << "\n";
                f << "f " << v3 + 1 << " " << v4 + 1 << " " << v5 + 1 << "\n";
            }
        }

    }
}
