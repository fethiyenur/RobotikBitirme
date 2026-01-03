#define _CRT_SECURE_NO_WARNINGS

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/Real_timer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = K::Point_3;
using Mesh = CGAL::Surface_mesh<Point_3>;

using PCL_Point = pcl::PointXYZ;
using PCL_Cloud = pcl::PointCloud<PCL_Point>;

int main(int argc, char** argv)
{
    // --- PCD DOSYASINI OKU ---
    const std::string filename =
        (argc > 1) ? argv[1] :
        "C:/Users/fetik/OneDrive/Masaüstü/4_güz/bitirme/MO_Bitirme_Hospital/Location2/Location2_merged_voxel_sor2.pcd";

    std::cout << "Reading PCD: " << filename << std::endl;

    PCL_Cloud::Ptr cloud(new PCL_Cloud);
    if (pcl::io::loadPCDFile<PCL_Point>(filename, *cloud) == -1 || cloud->empty())
    {
        std::cerr << "Error: Could not read PCD file.\n";
        return EXIT_FAILURE;
    }

    std::cout << cloud->size() << " points loaded\n";

    // --- PCL → CGAL POINT CONTAINER ---
    std::vector<Point_3> points;
    points.reserve(cloud->size());

    for (const auto& p : cloud->points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;

        points.emplace_back(p.x, p.y, p.z);
    }

    std::cout << points.size() << " valid CGAL points\n";

    // --- OFFSET HESABI (AYNI ŞEKİLDE KORUNDU) ---
    const double relative_offset = (argc > 3) ? std::stod(argv[3]) : 600.0;

    CGAL::Bbox_3 bbox = CGAL::bbox_3(points.begin(), points.end());

    const double diag_length =
        std::sqrt(CGAL::square(bbox.xmax() - bbox.xmin()) +
            CGAL::square(bbox.ymax() - bbox.ymin()) +
            CGAL::square(bbox.zmax() - bbox.zmin()));

    const double alpha = 0.6;                 // SABİT: 0.6 METRE (1.2 m ÇAP)
    const double offset = .2;

    std::cout << "alpha = " << alpha << ", offset = " << offset << std::endl;

    // --- ALPHA WRAP (MESH OLUŞTURMA) ---
    CGAL::Real_timer timer;
    timer.start();

    Mesh wrap;
    CGAL::alpha_wrap_3(points, alpha, offset, wrap);

    timer.stop();

    std::cout << "Mesh created:\n";
    std::cout << "  Vertices: " << num_vertices(wrap) << "\n";
    std::cout << "  Faces:    " << num_faces(wrap) << "\n";
    std::cout << "  Time:     " << timer.time() << " s\n";

    // --- ÇIKTI DOSYASI (.off) ---
    std::string base = filename.substr(0, filename.find_last_of('.'));
    std::string output =
        base + std::to_string((int)relative_offset) +
        ".off";

    std::cout << "Writing mesh to " << output << std::endl;
    CGAL::IO::write_polygon_mesh(output, wrap, CGAL::parameters::stream_precision(17));

    return EXIT_SUCCESS;
}
