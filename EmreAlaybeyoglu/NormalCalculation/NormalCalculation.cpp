#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>
#include <cmath>
#include <set>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/IO/PLY.h>
#include <CGAL/IO/OBJ.h>
#include <CGAL/IO/OFF.h>
#include <CGAL/IO/STL.h>

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>

#include <boost/property_map/property_map.hpp>

namespace PMP = CGAL::Polygon_mesh_processing;

using Kernel = CGAL::Simple_cartesian<double>;
using Point = Kernel::Point_3;
using Vector = Kernel::Vector_3;
using Mesh = CGAL::Surface_mesh<Point>;

// Camera position (origin)
const Point CAMERA_POSITION(0.0, 0.0, 0.0);

// Check if a face is visible from camera
bool is_face_visible_from_camera(const Point& centroid, const Vector& normal)
{
    Vector camera_to_face = centroid - CAMERA_POSITION;

    double camera_to_face_length = std::sqrt(camera_to_face.squared_length());
    double normal_length = std::sqrt(normal.squared_length());

    if (camera_to_face_length < 1e-10 || normal_length < 1e-10) {
        return false;
    }

    double dot_product = (camera_to_face * normal) / (camera_to_face_length * normal_length);

    return dot_product < 0; // Keep faces pointing towards camera
}

// Create new mesh with only visible faces
Mesh create_frontface_mesh(const Mesh& original_mesh,
    const std::vector<Point>& centroids,
    const std::vector<Vector>& normals,
    std::size_t& removed_count)
{
    Mesh new_mesh;

    // Create a map from old vertex indices to new vertex indices
    std::map<typename Mesh::Vertex_index, typename Mesh::Vertex_index> vertex_map;

    // Determine which faces to keep
    std::vector<bool> keep_face(original_mesh.number_of_faces(), false);
    std::size_t fi = 0;
    std::size_t kept_count = 0;

    for (auto f : original_mesh.faces()) {
        if (fi < centroids.size() && fi < normals.size()) {
            if (is_face_visible_from_camera(centroids[fi], normals[fi])) {
                keep_face[fi] = true;
                kept_count++;
            }
        }
        ++fi;
    }

    removed_count = original_mesh.number_of_faces() - kept_count;

    // If all faces would be removed, return original mesh
    if (kept_count == 0) {
        return original_mesh;
    }

    // Copy vertices that are used by kept faces
    fi = 0;
    for (auto f : original_mesh.faces()) {
        if (fi < keep_face.size() && keep_face[fi]) {
            for (auto v : CGAL::vertices_around_face(original_mesh.halfedge(f), original_mesh)) {
                if (vertex_map.find(v) == vertex_map.end()) {
                    Point p = original_mesh.point(v);
                    auto new_v = new_mesh.add_vertex(p);
                    vertex_map[v] = new_v;
                }
            }
        }
        ++fi;
    }

    // Copy kept faces
    fi = 0;
    for (auto f : original_mesh.faces()) {
        if (fi < keep_face.size() && keep_face[fi]) {
            std::vector<typename Mesh::Vertex_index> face_vertices;
            for (auto v : CGAL::vertices_around_face(original_mesh.halfedge(f), original_mesh)) {
                face_vertices.push_back(vertex_map[v]);
            }
            new_mesh.add_face(face_vertices);
        }
        ++fi;
    }

    return new_mesh;
}

// Save normal vectors as lines in OBJ format
void write_obj_with_normals(const std::string& filename,
    const Mesh& mesh,
    const std::vector<Point>& centroids,
    const std::vector<Vector>& normals,
    double scale)
{
    std::ofstream out(filename);
    if (!out) {
        std::cerr << "Cannot open OBJ file: " << filename << std::endl;
        return;
    }

    out << std::fixed << std::setprecision(6);
    out << "# Mesh with normals visualization\n";

    // Mesh vertices
    for (auto v : mesh.vertices()) {
        Point p = mesh.point(v);
        out << "v " << p.x() << " " << p.y() << " " << p.z() << "\n";
    }

    // Mesh faces
    for (auto f : mesh.faces()) {
        out << "f";
        for (auto v : CGAL::vertices_around_face(mesh.halfedge(f), mesh)) {
            out << " " << (v.idx() + 1);
        }
        out << "\n";
    }

    // Normal vectors as lines
    out << "\n# Normal vectors\n";
    std::size_t base_idx = mesh.number_of_vertices() + 1;

    std::size_t num_to_draw = std::min(centroids.size(), normals.size());

    for (std::size_t fi = 0; fi < num_to_draw; ++fi) {
        const Point& c = centroids[fi];
        const Vector& n = normals[fi];
        Point arrow_end = c + (n * scale);

        out << "v " << c.x() << " " << c.y() << " " << c.z() << "\n";
        out << "v " << arrow_end.x() << " " << arrow_end.y() << " " << arrow_end.z() << "\n";
        out << "l " << base_idx << " " << (base_idx + 1) << "\n";
        base_idx += 2;
    }

    out.close();
    std::cout << "OBJ file saved (with normals): " << filename << std::endl;
}

int main(int argc, char** argv)
{
    // ============================================
    // ENTER FILE PATH HERE
    // ============================================
    std::string input_path = "input_mesh.stl";

    double normal_scale = 0.05;
    bool remove_backfaces_enabled = true;
    // ============================================

    std::string out_prefix = input_path;
    auto last_dot = out_prefix.find_last_of('.');
    if (last_dot != std::string::npos) {
        out_prefix = out_prefix.substr(0, last_dot);
    }
    out_prefix += "_processed";

    std::cout << "=======================================" << std::endl;
    std::cout << "  Mesh Orientation and Normal Computation" << std::endl;
    std::cout << "=======================================" << std::endl;
    std::cout << "Input: " << input_path << std::endl;
    std::cout << "Output prefix: " << out_prefix << std::endl;
    std::cout << "Normal scale: " << normal_scale << std::endl;
    std::cout << "Backface removal: " << (remove_backfaces_enabled ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "Camera position: (0, 0, 0)" << std::endl;
    std::cout << std::endl;

    Mesh mesh;

    // Find file extension
    std::string ext;
    {
        auto pos = input_path.find_last_of('.');
        if (pos == std::string::npos) {
            std::cerr << "ERROR: Input file has no extension.\n";
            std::cout << "\nPress any key to continue..." << std::endl;
            std::cin.get();
            return EXIT_FAILURE;
        }
        ext = input_path.substr(pos + 1);
    }

    // Read mesh
    std::cout << "Loading mesh..." << std::endl;
    bool ok = false;

    if (ext == "ply" || ext == "PLY") {
        ok = CGAL::IO::read_PLY(input_path, mesh);
    }
    else if (ext == "off" || ext == "OFF") {
        ok = CGAL::IO::read_OFF(input_path, mesh);
    }
    else if (ext == "obj" || ext == "OBJ") {
        ok = CGAL::IO::read_OBJ(input_path, mesh);
    }
    else if (ext == "stl" || ext == "STL") {
        std::vector<Point> points;
        std::vector<std::vector<std::size_t>> faces;

        std::ifstream in(input_path, std::ios::binary);
        if (!in || !CGAL::IO::read_STL(in, points, faces)) {
            std::cerr << "ERROR: Cannot read STL: " << input_path << "\n";
            std::cout << "\nPress any key to continue..." << std::endl;
            std::cin.get();
            return EXIT_FAILURE;
        }
        in.close();

        PMP::polygon_soup_to_polygon_mesh(points, faces, mesh);
        ok = true;
    }
    else {
        std::cerr << "ERROR: Unsupported format: " << ext << std::endl;
        std::cout << "\nPress any key to continue..." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    if (!ok || num_vertices(mesh) == 0) {
        std::cerr << "ERROR: Cannot read mesh or mesh is empty.\n";
        std::cout << "\nPress any key to continue..." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    std::cout << "Success! Vertices: " << num_vertices(mesh)
        << ", Faces: " << num_faces(mesh) << "\n" << std::endl;

    // Orient mesh
    std::cout << "Orienting mesh..." << std::endl;
    try {
        PMP::orient_to_bound_a_volume(mesh);
        std::cout << "Mesh successfully oriented.\n" << std::endl;
    }
    catch (...) {
        std::cout << "Using default orientation.\n" << std::endl;
    }

    // Compute face normals
    std::cout << "Computing face normals..." << std::endl;
    std::vector<Vector> face_normals(num_faces(mesh));
    auto face_normals_map = boost::make_iterator_property_map(
        face_normals.begin(), get(boost::face_index, mesh));
    PMP::compute_face_normals(mesh, face_normals_map);
    std::cout << "Success!\n" << std::endl;

    // Compute face centroids
    std::cout << "Computing face centroids..." << std::endl;
    std::vector<Point> face_centroids;
    for (auto f : mesh.faces()) {
        Vector sum(0, 0, 0);
        int count = 0;
        for (auto h : CGAL::halfedges_around_face(mesh.halfedge(f), mesh)) {
            Point p = mesh.point(mesh.target(h));
            sum = sum + (p - CAMERA_POSITION);
            ++count;
        }
        if (count > 0) {
            face_centroids.push_back(CAMERA_POSITION + sum / static_cast<double>(count));
        }
        else {
            face_centroids.push_back(CAMERA_POSITION);
        }
    }
    std::cout << "Success!\n" << std::endl;

    // Remove backfaces if enabled
    if (remove_backfaces_enabled) {
        std::cout << "Analyzing face visibility..." << std::endl;
        std::size_t removed_count = 0;

        Mesh new_mesh = create_frontface_mesh(mesh, face_centroids, face_normals, removed_count);

        if (removed_count == num_faces(mesh)) {
            std::cerr << "WARNING: All faces would be removed! Keeping original mesh." << std::endl;
        }
        else if (removed_count > 0) {
            std::cout << "Removed " << removed_count << " backfaces." << std::endl;
            std::cout << "Remaining faces: " << new_mesh.number_of_faces() << "\n" << std::endl;

            mesh = new_mesh;

            // Recompute normals and centroids
            std::cout << "Recomputing normals and centroids..." << std::endl;
            face_normals.clear();
            face_normals.resize(num_faces(mesh));
            face_normals_map = boost::make_iterator_property_map(
                face_normals.begin(), get(boost::face_index, mesh));
            PMP::compute_face_normals(mesh, face_normals_map);

            face_centroids.clear();
            for (auto f : mesh.faces()) {
                Vector sum(0, 0, 0);
                int count = 0;
                for (auto h : CGAL::halfedges_around_face(mesh.halfedge(f), mesh)) {
                    Point p = mesh.point(mesh.target(h));
                    sum = sum + (p - CAMERA_POSITION);
                    ++count;
                }
                if (count > 0) {
                    face_centroids.push_back(CAMERA_POSITION + sum / static_cast<double>(count));
                }
                else {
                    face_centroids.push_back(CAMERA_POSITION);
                }
            }
            std::cout << "Success!\n" << std::endl;
        }
        else {
            std::cout << "No backfaces to remove.\n" << std::endl;
        }
    }

    // Compute vertex normals
    std::cout << "Computing vertex normals..." << std::endl;
    std::vector<Vector> vertex_normals(num_vertices(mesh));
    auto vim = get(boost::vertex_index, mesh);
    auto vnormals_map = boost::make_iterator_property_map(vertex_normals.begin(), vim);
    PMP::compute_vertex_normals(mesh, vnormals_map,
        CGAL::parameters::face_normal_map(face_normals_map));
    std::cout << "Success!\n" << std::endl;

    // Write CSV files
    std::cout << "Creating CSV files..." << std::endl;

    std::string face_csv = out_prefix + "_face_normals_centroids.csv";
    std::ofstream ofs(face_csv);
    ofs << std::fixed << std::setprecision(6);
    ofs << "face_index,centroid_x,centroid_y,centroid_z,normal_x,normal_y,normal_z\n";
    for (std::size_t fi = 0; fi < face_centroids.size() && fi < face_normals.size(); ++fi) {
        const Point& c = face_centroids[fi];
        const Vector& n = face_normals[fi];
        ofs << fi << "," << c.x() << "," << c.y() << "," << c.z() << ","
            << n.x() << "," << n.y() << "," << n.z() << "\n";
    }
    ofs.close();
    std::cout << "  - " << face_csv << std::endl;

    std::string vert_csv = out_prefix + "_vertex_normals.csv";
    std::ofstream vofs(vert_csv);
    vofs << std::fixed << std::setprecision(6);
    vofs << "vertex_index,x,y,z,nx,ny,nz\n";
    for (auto v : mesh.vertices()) {
        Point p = mesh.point(v);
        Vector vn = vertex_normals[get(vim, v)];
        vofs << get(vim, v) << "," << p.x() << "," << p.y() << "," << p.z()
            << "," << vn.x() << "," << vn.y() << "," << vn.z() << "\n";
    }
    vofs.close();
    std::cout << "  - " << vert_csv << std::endl << std::endl;

    // Save mesh
    std::cout << "Saving oriented mesh..." << std::endl;
    std::string out_mesh = out_prefix + "_oriented.ply";
    if (CGAL::IO::write_PLY(out_mesh, mesh)) {
        std::cout << "  - " << out_mesh << std::endl;
    }
    std::cout << std::endl;

    // Create visualization
    std::cout << "Creating visualization file..." << std::endl;
    std::string vis_obj = out_prefix + "_with_normals.obj";
    write_obj_with_normals(vis_obj, mesh, face_centroids, face_normals, normal_scale);
    std::cout << std::endl;

    std::cout << "=======================================" << std::endl;
    std::cout << "           PROCESSING COMPLETE!           " << std::endl;
    std::cout << "=======================================" << std::endl;
    std::cout << "\nFinal statistics:" << std::endl;
    std::cout << "- Vertices: " << num_vertices(mesh) << std::endl;
    std::cout << "- Faces: " << num_faces(mesh) << std::endl;
    std::cout << "\nPress any key to continue..." << std::endl;
    std::cin.get();

    return EXIT_SUCCESS;
}