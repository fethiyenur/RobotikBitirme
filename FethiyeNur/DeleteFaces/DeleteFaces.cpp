#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/boost/graph/Euler_operations.h>
#include <CGAL/IO/polygon_mesh_io.h>

#include <boost/property_map/property_map.hpp>

#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;
namespace PMP = CGAL::Polygon_mesh_processing;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3   Point;
typedef K::Vector_3  Vector;

typedef CGAL::Polyhedron_3<K> Mesh;
typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;
typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;

// --------------------------------------------------
Vector normalize(const Vector& v)
{
    double len = CGAL::sqrt(v.squared_length());
    if (len == 0) return Vector(0, 0, 0);
    return v / len;
}

// --------------------------------------------------
// base_link translation okuma
// --------------------------------------------------
bool read_base_link_translation(
    const std::string& filename,
    double& x, double& y, double& z)
{
    std::ifstream in(filename);
    if (!in.is_open())
        return false;

    std::string line;
    bool in_base_link = false;
    bool in_translation = false;

    while (std::getline(in, line))
    {
        if (line.find("child_frame_id: base_link") != std::string::npos)
        {
            in_base_link = true;
            continue;
        }

        if (!in_base_link) continue;

        if (line.find("translation:") != std::string::npos)
        {
            in_translation = true;
            continue;
        }

        if (in_translation)
        {
            if (line.find("x:") != std::string::npos)
                x = std::stod(line.substr(line.find(":") + 1));
            else if (line.find("y:") != std::string::npos)
                y = std::stod(line.substr(line.find(":") + 1));
            else if (line.find("z:") != std::string::npos)
            {
                z = std::stod(line.substr(line.find(":") + 1));
                return true;
            }
        }
    }
    return false;
}

// --------------------------------------------------
// Klasörden tüm TXT dosyalarını topla
// --------------------------------------------------
std::vector<std::string> collect_txt_files(const std::string& folder)
{
    std::vector<std::string> files;

    for (const auto& entry : fs::directory_iterator(folder))
    {
        if (entry.path().extension() == ".txt")
            files.push_back(entry.path().string());
    }
    return files;
}

// --------------------------------------------------
// Drone konumunu TXT'lerden hesapla
// --------------------------------------------------
Point compute_drone_position_from_txt(
    const std::vector<std::string>& txt_files)
{
    double sx = 0, sy = 0, sz = 0;
    int count = 0;

    for (const auto& file : txt_files)
    {
        double x, y, z;
        if (read_base_link_translation(file, x, y, z))
        {
            sx += x; sy += y; sz += z;
            count++;
        }
    }

    if (count == 0)
    {
        std::cerr << "HATA: base_link verisi bulunamadi!\n";
        return Point(0, 0, 0);
    }

    return Point(sx / count, sy / count, sz / count);
}

// --------------------------------------------------
// STL Writer
// --------------------------------------------------
void write_STL(const Mesh& mesh,
    const std::map<face_descriptor, Vector>& fnormals,
    const std::string& outname)
{
    std::ofstream out(outname);
    out << "solid mesh\n";

    for (face_descriptor f : faces(mesh))
    {
        auto h0 = halfedge(f, mesh);
        auto h1 = next(h0, mesh);
        auto h2 = next(h1, mesh);

        Point p0 = target(h0, mesh)->point();
        Point p1 = target(h1, mesh)->point();
        Point p2 = target(h2, mesh)->point();

        Vector n = normalize(-fnormals.at(f));

        out << "facet normal " << n << "\n";
        out << "  outer loop\n";
        out << "    vertex " << p0 << "\n";
        out << "    vertex " << p1 << "\n";
        out << "    vertex " << p2 << "\n";
        out << "  endloop\nendfacet\n";
    }
    out << "endsolid mesh\n";
}

// --------------------------------------------------
// MAIN
// --------------------------------------------------
int main()
{
    Mesh mesh;
    if (!CGAL::IO::read_polygon_mesh(
        "C:/Users/fetik/OneDrive/Masaüstü/4_güz/bitirme/github/FethiyeNur/MergeandMesh/Location2_merged_voxel_sor2600.off",
        mesh))
    {
        std::cerr << "Mesh okunamadi.\n";
        return 1;
    }

    // TXT klasörü (TEK SATIR)
    std::string txt_folder =
        "C:/Users/fetik/OneDrive/Masaüstü/4_güz/bitirme/MO_Bitirme_Hospital/Location2";

    std::vector<std::string> txt_files = collect_txt_files(txt_folder);
    Point drone_origin = compute_drone_position_from_txt(txt_files);

    std::cout << "Drone konumu: " << drone_origin << std::endl;

    PMP::orient(mesh);

    std::map<face_descriptor, Vector> fnormals;
    std::map<vertex_descriptor, Vector> vnormals;

    PMP::compute_normals(
        mesh,
        boost::make_assoc_property_map(vnormals),
        boost::make_assoc_property_map(fnormals)
    );

    std::vector<face_descriptor> faces_to_remove;

    for (face_descriptor f : faces(mesh))
    {
        auto h = halfedge(f, mesh);
        Point p0 = target(h, mesh)->point();
        Point p1 = target(next(h, mesh), mesh)->point();
        Point p2 = target(next(next(h, mesh), mesh), mesh)->point();

        Point centroid(
            (p0.x() + p1.x() + p2.x()) / 3.0,
            (p0.y() + p1.y() + p2.y()) / 3.0,
            (p0.z() + p1.z() + p2.z()) / 3.0
        );

        Vector view_vec = drone_origin - centroid;

        if (CGAL::scalar_product(fnormals[f], view_vec) <= 0)
            faces_to_remove.push_back(f);
    }

    for (face_descriptor f : faces_to_remove)
        CGAL::Euler::remove_face(halfedge(f, mesh), mesh);

    mesh.normalize_border();

    fnormals.clear();
    vnormals.clear();

    PMP::compute_normals(
        mesh,
        boost::make_assoc_property_map(vnormals),
        boost::make_assoc_property_map(fnormals)
    );

    write_STL(mesh, fnormals, "location2_drone_visible_mesh_FINAL.stl");

    std::cout << "Bitti.\n";
    return 0;
}
