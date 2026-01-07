#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/polygon_mesh_io.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <set>
#include <map>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point;
typedef K::Vector_3 Vector;
typedef CGAL::Polyhedron_3<K> Mesh;

// =====================
// NORMALIZE VECTOR
// =====================
Vector normalize(const Vector& v)
{
    double len = std::sqrt(v.squared_length());
    if (len == 0.0)
        return Vector(0, 0, 0);
    return v / len;
}

// =====================
// EXTRACT BOUNDARY LOOPS
// =====================
std::vector<std::vector<Mesh::Halfedge_handle>> extract_boundary_loops(Mesh& mesh)
{
    std::vector<std::vector<Mesh::Halfedge_handle>> loops;
    std::set<Mesh::Halfedge_handle> visited;

    // Find all boundary halfedges
    for (auto h = mesh.halfedges_begin(); h != mesh.halfedges_end(); ++h)
    {
        if (h->is_border() && visited.find(h) == visited.end())
        {
            // Start a new loop
            std::vector<Mesh::Halfedge_handle> loop;
            Mesh::Halfedge_handle current = h;

            // Follow the loop
            do {
                loop.push_back(current);
                visited.insert(current);
                current = current->next();
            } while (current != h && current->is_border());

            loops.push_back(loop);
        }
    }

    return loops;
}

// =====================
// COMPUTE LOOP CENTROID
// =====================
Point compute_loop_centroid(const std::vector<Mesh::Halfedge_handle>& loop)
{
    if (loop.empty())
        return Point(0, 0, 0);

    // Collect unique vertices in this loop
    std::set<Mesh::Vertex_handle> vertices;
    for (auto h : loop)
    {
        vertices.insert(h->vertex());
    }

    if (vertices.empty())
        return Point(0, 0, 0);

    // Calculate average
    double sum_x = 0, sum_y = 0, sum_z = 0;
    for (auto v : vertices)
    {
        Point p = v->point();
        sum_x += p.x();
        sum_y += p.y();
        sum_z += p.z();
    }

    int count = vertices.size();
    return Point(sum_x / count, sum_y / count, sum_z / count);
}

// =====================
// WRITE CENTROIDS TO CSV
// =====================
void write_centroids_csv(
    const std::vector<Point>& centroids,
    const std::string& filename)
{
    std::ofstream out(filename);

    // Write header
    out << "Loop_ID   ,   X   ,   Y   ,   Z\n";

    // Write each centroid
    for (size_t i = 0; i < centroids.size(); ++i)
    {
        out << (i + 1) << ", x = "
            << centroids[i].x() << ", y = "
            << centroids[i].y() << ", z = "
            << centroids[i].z() << "\n";
    }

    out.close();
}

// =====================
// WRITE CENTROIDS TO STL (as small spheres)
// =====================
void write_centroids_stl(
    const std::vector<Point>& centroids,
    const std::string& filename,
    double sphere_radius = 0.05)
{
    std::ofstream out(filename);
    out << "solid centroids\n";

    for (const auto& center : centroids)
    {
        // Create a simple octahedron at each centroid point
        // 6 vertices: top, bottom, and 4 around middle
        Point top(center.x(), center.y(), center.z() + sphere_radius);
        Point bottom(center.x(), center.y(), center.z() - sphere_radius);
        Point front(center.x(), center.y() + sphere_radius, center.z());
        Point back(center.x(), center.y() - sphere_radius, center.z());
        Point right(center.x() + sphere_radius, center.y(), center.z());
        Point left(center.x() - sphere_radius, center.y(), center.z());

        // Create 8 triangular faces
        // Top pyramid
        std::vector<std::vector<Point>> faces = {
            {top, front, right},
            {top, right, back},
            {top, back, left},
            {top, left, front},
            // Bottom pyramid
            {bottom, right, front},
            {bottom, back, right},
            {bottom, left, back},
            {bottom, front, left}
        };

        for (const auto& face : faces)
        {
            Vector v1 = face[1] - face[0];
            Vector v2 = face[2] - face[0];
            Vector n = CGAL::cross_product(v1, v2);
            n = normalize(n);

            out << "facet normal " << n.x() << " " << n.y() << " " << n.z() << "\n";
            out << "  outer loop\n";
            out << "    vertex " << face[0].x() << " " << face[0].y() << " " << face[0].z() << "\n";
            out << "    vertex " << face[1].x() << " " << face[1].y() << " " << face[1].z() << "\n";
            out << "    vertex " << face[2].x() << " " << face[2].y() << " " << face[2].z() << "\n";
            out << "  endloop\n";
            out << "endfacet\n";
        }
    }

    out << "endsolid centroids\n";
    out.close();
}

// =====================
// WRITE BOUNDARY STL
// =====================
void write_boundary_stl(
    const std::vector<std::vector<Mesh::Halfedge_handle>>& loops,
    const std::string& filename,
    double thickness = 0.001)
{
    std::ofstream out(filename);
    out << "solid boundary\n";

    for (const auto& loop : loops)
    {
        for (auto h : loop)
        {
            Point p0 = h->vertex()->point();
            Point p1 = h->opposite()->vertex()->point();

            // Edge direction
            Vector dir = p1 - p0;

            // Create a perpendicular offset
            Vector perp = CGAL::cross_product(dir, Vector(0, 0, 1));
            if (perp.squared_length() == 0)
                perp = CGAL::cross_product(dir, Vector(0, 1, 0));
            perp = normalize(perp) * thickness;

            // Triangle vertices
            Point a = p0;
            Point b = p1;
            Point c = p0 + perp;

            Vector n = CGAL::cross_product(b - a, c - a);
            n = normalize(n);

            out << "facet normal " << n.x() << " " << n.y() << " " << n.z() << "\n";
            out << "  outer loop\n";
            out << "    vertex " << a.x() << " " << a.y() << " " << a.z() << "\n";
            out << "    vertex " << b.x() << " " << b.y() << " " << b.z() << "\n";
            out << "    vertex " << c.x() << " " << c.y() << " " << c.z() << "\n";
            out << "  endloop\n";
            out << "endfacet\n";
        }
    }

    out << "endsolid boundary\n";
    out.close();
}

// =====================
// MAIN
// =====================
int main()
{
    const std::string input_mesh =
        "inputMesh.stl";
    const std::string output_boundary =
        "finalBoundries.stl";
    const std::string output_csv =
        "boundaryCentroids.csv";
    const std::string output_centroids_stl =
        "centroidPoints.stl";

    Mesh mesh;
    if (!CGAL::IO::read_polygon_mesh(input_mesh, mesh))
    {
        std::cerr << "Failed to read mesh!\n";
        return EXIT_FAILURE;
    }

    // Extract separate boundary loops
    std::vector<std::vector<Mesh::Halfedge_handle>> boundary_loops = extract_boundary_loops(mesh);

    std::cout << "Number of boundary loops found: " << boundary_loops.size() << "\n\n";

    // Store centroids
    std::vector<Point> centroids;

    // Calculate centroid for each loop
    for (size_t i = 0; i < boundary_loops.size(); ++i)
    {
        Point centroid = compute_loop_centroid(boundary_loops[i]);
        centroids.push_back(centroid);

        std::cout << "=== BOUNDARY LOOP " << (i + 1) << " ===\n";
        std::cout << "Edges in loop: " << boundary_loops[i].size() << "\n";
        std::cout << "Centroid X: " << centroid.x() << "\n";
        std::cout << "Centroid Y: " << centroid.y() << "\n";
        std::cout << "Centroid Z: " << centroid.z() << "\n\n";
    }

    // Write centroids to CSV
    write_centroids_csv(centroids, output_csv);
    std::cout << "Centroids written to CSV:\n" << output_csv << "\n\n";

    // Write centroids to STL (as small spheres)
    write_centroids_stl(centroids, output_centroids_stl);
    std::cout << "Centroids written to STL:\n" << output_centroids_stl << "\n\n";

    // Write boundary STL
    write_boundary_stl(boundary_loops, output_boundary);
    std::cout << "Boundary STL written:\n" << output_boundary << std::endl;

    return EXIT_SUCCESS;
}