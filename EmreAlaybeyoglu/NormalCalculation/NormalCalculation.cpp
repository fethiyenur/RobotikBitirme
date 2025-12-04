#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/IO/polygon_soup_io.h>
#include <CGAL/IO/PLY.h>
#include <CGAL/IO/OBJ.h>
#include <CGAL/IO/OFF.h>

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/orient.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>

#include <boost/foreach.hpp>
#include <boost/property_map/property_map.hpp>

namespace PMP = CGAL::Polygon_mesh_processing;

using Kernel = CGAL::Simple_cartesian<double>;
using Point  = Kernel::Point_3;
using Vector = Kernel::Vector_3;
using Mesh   = CGAL::Surface_mesh<Point>;

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " input_mesh.(ply|off|obj) output_prefix\n";
        return EXIT_FAILURE;
    }

    std::string input_path = argv[1];
    std::string out_prefix = argv[2];

    Mesh mesh;
    // Try to read various formats
    std::string ext;
    {
        auto pos = input_path.find_last_of('.');
        if (pos == std::string::npos) {
            std::cerr << "Input file has no extension.\n";
            return EXIT_FAILURE;
        }
        ext = input_path.substr(pos+1);
    }

    bool ok = false;
    if (ext == "ply" || ext == "PLY") {
        ok = CGAL::IO::read_PLY(input_path, mesh);
    } else if (ext == "off" || ext == "OFF") {
        ok = CGAL::IO::read_OFF(input_path, mesh);
    } else if (ext == "obj" || ext == "OBJ") {
        ok = CGAL::IO::read_OBJ(input_path, mesh);
    } else {
        std::cerr << "Unsupported input format: " << ext << "\nSupported: ply, off, obj\n";
        return EXIT_FAILURE;
    }

    if (!ok) {
        std::cerr << "Failed to read mesh from " << input_path << "\n";
        return EXIT_FAILURE;
    }

    if(num_vertices(mesh) == 0) {
        std::cerr << "Mesh is empty.\n";
        return EXIT_FAILURE;
    }

    std::cout << "Loaded mesh: vertices=" << num_vertices(mesh)
              << " faces=" << num_faces(mesh) << "\n";

    // Ensure mesh has a consistent orientation: orient connected components outward
    try {
        PMP::orient(mesh);
        std::cout << "Mesh oriented (PMP::orient).\n";
    } catch (const std::exception& e) {
        std::cerr << "PMP::orient threw: " << e.what() << "\n";
    }

    // Prepare property maps to store normals
    typedef boost::property_map<Mesh, boost::vertex_index_t>::type VertexIndexMap;
    VertexIndexMap vim = get(boost::vertex_index, mesh);

    std::vector<Vector> face_normals(num_faces(mesh));
    auto face_normals_map = boost::make_iterator_property_map(face_normals.begin(), get(boost::face_index, mesh));

    // compute face normals
    PMP::compute_face_normals(mesh, face_normals_map);
    std::cout << "Computed face normals.\n";

    // compute vertex normals (optional/useful)
    std::vector<Vector> vertex_normals(num_vertices(mesh));
    auto vnormals_map = boost::make_iterator_property_map(vertex_normals.begin(), vim);
    PMP::compute_vertex_normals(mesh,
                                vnormals_map,
                                boost::parameter::face_normals_map(face_normals_map));
    std::cout << "Computed vertex normals.\n";

    // compute centroids for faces
    std::vector<Point> face_centroids;
    face_centroids.reserve(num_faces(mesh));

    for (auto f : mesh.faces()) {
        Vector sum(0,0,0);
        int count = 0;
        // iterate halfedges around face to get vertices
        for (auto h : CGAL::halfedges_around_face(mesh.halfedge(f), mesh)) {
            auto v = mesh.target(h);
            Point p = mesh.point(v);
            // convert point to vector
            sum = sum + (p - CGAL::ORIGIN); 
            ++count;
        }
        if (count > 0) {
            Vector avg = sum / static_cast<double>(count);
            Point centroid = CGAL::ORIGIN + avg;
            face_centroids.push_back(centroid);
        } else {
            face_centroids.push_back(Point(0,0,0));
        }
    }
    std::cout << "Computed face centroids.\n";

    // Write face normals + centroids to CSV
    std::string face_csv = out_prefix + "_face_normals_centroids.csv";
    std::ofstream ofs(face_csv);
    ofs << std::fixed << std::setprecision(6);
    ofs << "face_index,centroid_x,centroid_y,centroid_z,normal_x,normal_y,normal_z\n";

    std::size_t fi = 0;
    for (auto f : mesh.faces()) {
        const Vector& n = face_normals[fi];
        const Point& c = face_centroids[fi];
        ofs << fi << "," << c.x() << "," << c.y() << "," << c.z() << "," << n.x() << "," << n.y() << "," << n.z() << "\n";
        ++fi;
    }
    ofs.close();
    std::cout << "Wrote " << face_csv << "\n";

    // Optionally write vertex normals too
    std::string vert_csv = out_prefix + "_vertex_normals.csv";
    std::ofstream vofs(vert_csv);
    vofs << "vertex_index,x,y,z,nx,ny,nz\n";
    for (auto v : mesh.vertices()) {
        Point p = mesh.point(v);
        Vector vn = vertex_normals[get(vim, v)];
        vofs << get(vim, v) << "," << p.x() << "," << p.y() << "," << p.z()
             << "," << vn.x() << "," << vn.y() << "," << vn.z() << "\n";
    }
    vofs.close();
    std::cout << "Wrote " << vert_csv << "\n";

    // Save oriented mesh
    std::string out_mesh = out_prefix + "_oriented.ply";
    bool write_ok = CGAL::IO::write_PLY(out_mesh, mesh);
    if (write_ok) {
        std::cout << "Wrote oriented mesh to " << out_mesh << "\n";
    } else {
        std::cerr << "Failed to write oriented mesh to " << out_mesh << "\n";
    }

    std::cout << "Done.\n";
    return EXIT_SUCCESS;
}
