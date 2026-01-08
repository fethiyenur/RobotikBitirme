#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/polygon_mesh_io.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <cmath>

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

    for (auto h = mesh.halfedges_begin(); h != mesh.halfedges_end(); ++h)
    {
        if (h->is_border() && visited.find(h) == visited.end())
        {
            std::vector<Mesh::Halfedge_handle> loop;
            Mesh::Halfedge_handle current = h;

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

    std::set<Mesh::Vertex_handle> vertices;
    for (auto h : loop)
    {
        vertices.insert(h->vertex());
    }

    if (vertices.empty())
        return Point(0, 0, 0);

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
// COMPUTE LOOP RADIUS (average distance from centroid)
// =====================
double compute_loop_radius(
    const std::vector<Mesh::Halfedge_handle>& loop,
    const Point& centroid)
{
    if (loop.empty())
        return 0.0;

    std::set<Mesh::Vertex_handle> vertices;
    for (auto h : loop)
    {
        vertices.insert(h->vertex());
    }

    if (vertices.empty())
        return 0.0;

    double sum_distance = 0.0;
    for (auto v : vertices)
    {
        Point p = v->point();
        double dist = std::sqrt(CGAL::squared_distance(p, centroid));
        sum_distance += dist;
    }

    return sum_distance / vertices.size();
}

// =====================
// COMPUTE LOOP MAX RADIUS
// =====================
double compute_loop_max_radius(
    const std::vector<Mesh::Halfedge_handle>& loop,
    const Point& centroid)
{
    if (loop.empty())
        return 0.0;

    std::set<Mesh::Vertex_handle> vertices;
    for (auto h : loop)
    {
        vertices.insert(h->vertex());
    }

    if (vertices.empty())
        return 0.0;

    double max_distance = 0.0;
    for (auto v : vertices)
    {
        Point p = v->point();
        double dist = std::sqrt(CGAL::squared_distance(p, centroid));
        if (dist > max_distance)
            max_distance = dist;
    }

    return max_distance;
}

// =====================
// WRITE CENTROIDS TO CSV
// =====================
void write_centroids_csv(
    const std::vector<Point>& centroids,
    const std::vector<double>& radii,
    const std::string& filename)
{
    std::ofstream out(filename);

    out << "Loop_ID,X,Y,Z,Radius\n";

    for (size_t i = 0; i < centroids.size(); ++i)
    {
        out << (i + 1) << ","
            << centroids[i].x() << ","
            << centroids[i].y() << ","
            << centroids[i].z() << ","
            << radii[i] << "\n";
    }

    out.close();
}

// =====================
// WRITE CENTROIDS TO STL
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
        Point top(center.x(), center.y(), center.z() + sphere_radius);
        Point bottom(center.x(), center.y(), center.z() - sphere_radius);
        Point front(center.x(), center.y() + sphere_radius, center.z());
        Point back(center.x(), center.y() - sphere_radius, center.z());
        Point right(center.x() + sphere_radius, center.y(), center.z());
        Point left(center.x() - sphere_radius, center.y(), center.z());

        std::vector<std::vector<Point>> faces = {
            {top, front, right},
            {top, right, back},
            {top, back, left},
            {top, left, front},
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
// WRITE BOUNDARY STL (only filtered loops)
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

            Vector dir = p1 - p0;

            Vector perp = CGAL::cross_product(dir, Vector(0, 0, 1));
            if (perp.squared_length() == 0)
                perp = CGAL::cross_product(dir, Vector(0, 1, 0));
            perp = normalize(perp) * thickness;

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
    const std::string input_mesh = "inputMesh.stl";
    const std::string output_boundary = "finalBoundries.stl";
    const std::string output_csv = "boundaryCentroids.csv";
    const std::string output_centroids_stl = "centroidPoints.stl";

    // Minimum radius threshold in meters
    const double MIN_RADIUS = 1.0;

    Mesh mesh;
    if (!CGAL::IO::read_polygon_mesh(input_mesh, mesh))
    {
        std::cerr << "Failed to read mesh!\n";
        return EXIT_FAILURE;
    }

    std::vector<std::vector<Mesh::Halfedge_handle>> boundary_loops = extract_boundary_loops(mesh);

    std::cout << "Total boundary loops found: " << boundary_loops.size() << "\n\n";

    std::vector<Point> filtered_centroids;
    std::vector<double> filtered_radii;
    std::vector<std::vector<Mesh::Halfedge_handle>> filtered_loops;

    int filtered_count = 0;

    for (size_t i = 0; i < boundary_loops.size(); ++i)
    {
        Point centroid = compute_loop_centroid(boundary_loops[i]);
        double avg_radius = compute_loop_radius(boundary_loops[i], centroid);
        double max_radius = compute_loop_max_radius(boundary_loops[i], centroid);

        std::cout << "=== BOUNDARY LOOP " << (i + 1) << " ===\n";
        std::cout << "Edges in loop: " << boundary_loops[i].size() << "\n";
        std::cout << "Centroid: ("
            << centroid.x() << ", "
            << centroid.y() << ", "
            << centroid.z() << ")\n";
        std::cout << "Average Radius: " << avg_radius << " m\n";
        std::cout << "Max Radius: " << max_radius << " m\n";

        // Filter by radius
        if (avg_radius >= MIN_RADIUS)
        {
            filtered_centroids.push_back(centroid);
            filtered_radii.push_back(avg_radius);
            filtered_loops.push_back(boundary_loops[i]);
            std::cout << "✓ INCLUDED (radius >= " << MIN_RADIUS << " m)\n\n";
        }
        else
        {
            filtered_count++;
            std::cout << "✗ FILTERED OUT (radius < " << MIN_RADIUS << " m)\n\n";
        }
    }

    std::cout << "========================================\n";
    std::cout << "Total loops: " << boundary_loops.size() << "\n";
    std::cout << "Filtered out: " << filtered_count << "\n";
    std::cout << "Remaining loops: " << filtered_centroids.size() << "\n";
    std::cout << "========================================\n\n";

    if (filtered_centroids.empty())
    {
        std::cout << "No boundary loops meet the radius criteria!\n";
        return EXIT_SUCCESS;
    }

    write_centroids_csv(filtered_centroids, filtered_radii, output_csv);
    std::cout << "Filtered centroids written to CSV:\n" << output_csv << "\n\n";

    write_centroids_stl(filtered_centroids, output_centroids_stl);
    std::cout << "Filtered centroids written to STL:\n" << output_centroids_stl << "\n\n";

    write_boundary_stl(filtered_loops, output_boundary);
    std::cout << "Filtered boundary STL written:\n" << output_boundary << std::endl;

    return EXIT_SUCCESS;
}
