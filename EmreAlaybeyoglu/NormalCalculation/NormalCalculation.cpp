#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>

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
    out << "# Original mesh vertices and faces\n";

    // Original mesh vertices
    std::size_t vertex_offset = 1; // OBJ 1-indexed
    for (auto v : mesh.vertices()) {
        Point p = mesh.point(v);
        out << "v " << p.x() << " " << p.y() << " " << p.z() << "\n";
    }

    // Original mesh faces
    for (auto f : mesh.faces()) {
        out << "f";
        for (auto v : CGAL::vertices_around_face(mesh.halfedge(f), mesh)) {
            out << " " << (v.idx() + vertex_offset);
        }
        out << "\n";
    }

    // Add normal vectors as line segments
    out << "\n# Normal vectors as line segments\n";
    std::size_t base_idx = num_vertices(mesh) + 1;

    for (std::size_t fi = 0; fi < centroids.size() && fi < normals.size(); ++fi) {
        const Point& c = centroids[fi];
        const Vector& n = normals[fi];
        Point arrow_end = c + (n * scale);

        // Two points for the line
        out << "v " << c.x() << " " << c.y() << " " << c.z() << "\n";
        out << "v " << arrow_end.x() << " " << arrow_end.y() << " " << arrow_end.z() << "\n";

        // Line segment
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

    // Scale of normal vectors (adjust based on mesh size)
    // Larger value = longer normal arrows
    double normal_scale = 0.05;
    // ============================================

    // Auto-generate output prefix
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
        // Read STL
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

        // Convert polygon soup to mesh
        PMP::polygon_soup_to_polygon_mesh(points, faces, mesh);
        ok = true;
    }
    else {
        std::cerr << "ERROR: Unsupported format: " << ext << std::endl;
        std::cerr << "Supported formats: ply, off, obj, stl\n";
        std::cout << "\nPress any key to continue..." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    if (!ok) {
        std::cerr << "ERROR: Cannot read mesh: " << input_path << "\n";
        std::cout << "\nPress any key to continue..." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    if (num_vertices(mesh) == 0) {
        std::cerr << "ERROR: Mesh is empty.\n";
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
    catch (const std::exception& e) {
        std::cerr << "Orientation error: " << e.what() << "\n";
        std::cerr << "Trying alternative method...\n";
        try {
            PMP::reverse_face_orientations(mesh);
            std::cout << "Face orientations reversed.\n" << std::endl;
        }
        catch (...) {
            std::cerr << "Orientation failed.\n" << std::endl;
        }
    }

    // Prepare normal property maps
    typedef boost::property_map<Mesh, boost::vertex_index_t>::type VertexIndexMap;
    VertexIndexMap vim = get(boost::vertex_index, mesh);

    std::vector<Vector> face_normals(num_faces(mesh));
    auto face_normals_map = boost::make_iterator_property_map(face_normals.begin(), get(boost::face_index, mesh));

    // Compute face normals
    std::cout << "Computing face normals..." << std::endl;
    PMP::compute_face_normals(mesh, face_normals_map);
    std::cout << "Success!\n" << std::endl;

    // Compute vertex normals
    std::cout << "Computing vertex normals..." << std::endl;
    std::vector<Vector> vertex_normals(num_vertices(mesh));
    auto vnormals_map = boost::make_iterator_property_map(vertex_normals.begin(), vim);
    PMP::compute_vertex_normals(mesh,
        vnormals_map,
        CGAL::parameters::face_normal_map(face_normals_map));
    std::cout << "Success!\n" << std::endl;

    // Compute face centroids
    std::cout << "Computing face centroids..." << std::endl;
    std::vector<Point> face_centroids;
    face_centroids.reserve(num_faces(mesh));

    for (auto f : mesh.faces()) {
        Vector sum(0, 0, 0);
        int count = 0;
        for (auto h : CGAL::halfedges_around_face(mesh.halfedge(f), mesh)) {
            auto v = mesh.target(h);
            Point p = mesh.point(v);
            sum = sum + (p - CGAL::ORIGIN);
            ++count;
        }
        if (count > 0) {
            Vector avg = sum / static_cast<double>(count);
            Point centroid = CGAL::ORIGIN + avg;
            face_centroids.push_back(centroid);
        }
        else {
            face_centroids.push_back(Point(0, 0, 0));
        }
    }
    std::cout << "Success!\n" << std::endl;

    // Write face normals + centroids to CSV
    std::cout << "Creating CSV files..." << std::endl;
    std::string face_csv = out_prefix + "_face_normals_centroids.csv";
    std::ofstream ofs(face_csv);
    ofs << std::fixed << std::setprecision(6);
    ofs << "face_index,centroid_x,centroid_y,centroid_z,normal_x,normal_y,normal_z\n";

    std::size_t fi = 0;
    for (auto f : mesh.faces()) {
        const Vector& n = face_normals[fi];
        const Point& c = face_centroids[fi];
        ofs << fi << "," << c.x() << "," << c.y() << "," << c.z() << ","
            << n.x() << "," << n.y() << "," << n.z() << "\n";
        ++fi;
    }
    ofs.close();
    std::cout << "  - " << face_csv << std::endl;

    // Write vertex normals to CSV
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
    std::cout << "  - " << vert_csv << std::endl;
    std::cout << std::endl;

    // Save oriented mesh
    std::cout << "Saving oriented mesh..." << std::endl;
    std::string out_mesh = out_prefix + "_oriented.ply";
    bool write_ok = CGAL::IO::write_PLY(out_mesh, mesh);
    if (write_ok) {
        std::cout << "  - " << out_mesh << std::endl;
    }
    else {
        std::cerr << "ERROR: Cannot save mesh: " << out_mesh << "\n";
    }
    std::cout << std::endl;

    // Create OBJ file with normal vectors for visualization
    std::cout << "Creating visualization file..." << std::endl;
    std::string vis_obj = out_prefix + "_with_normals.obj";
    write_obj_with_normals(vis_obj, mesh, face_centroids, face_normals, normal_scale);
    std::cout << std::endl;

    std::cout << "=======================================" << std::endl;
    std::cout << "           PROCESSING COMPLETE!           " << std::endl;
    std::cout << "=======================================" << std::endl;
    std::cout << "\nGenerated files:" << std::endl;
    std::cout << "1. " << out_mesh << std::endl;
    std::cout << "   (Oriented mesh)" << std::endl;
    std::cout << "\n2. " << face_csv << std::endl;
    std::cout << "   (Face normals and centroids)" << std::endl;
    std::cout << "\n3. " << vert_csv << std::endl;
    std::cout << "   (Vertex normals)" << std::endl;
    std::cout << "\n4. " << vis_obj << std::endl;
    std::cout << "   (Visual with normals - open in MeshLab)" << std::endl;
    std::cout << "\n---------------------------------------" << std::endl;
    std::cout << "FOR VISUALIZATION:" << std::endl;
    std::cout << "- Open with MeshLab, Blender or CloudCompare" << std::endl;
    std::cout << "- Normal vectors will appear as red lines" << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "\nPress any key to continue..." << std::endl;
    std::cin.get();

    return EXIT_SUCCESS;
}