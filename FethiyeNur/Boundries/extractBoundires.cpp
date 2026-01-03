#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/polygon_mesh_io.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <set>

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
// WRITE BOUNDARY STL
// =====================
void write_boundary_stl(
    const std::vector<Mesh::Halfedge_handle>& boundary_edges,
    const std::string& filename,
    double thickness = 0.001)
{
    std::ofstream out(filename);
    out << "solid boundary\n";

    for (auto h : boundary_edges)
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

    out << "endsolid boundary\n";
    out.close();
}

// =====================
// EXTRACT BOUNDARY
// =====================
void extract_boundary(
    Mesh& mesh,
    std::vector<Mesh::Halfedge_handle>& boundary_edges)
{
    for (auto h = mesh.halfedges_begin(); h != mesh.halfedges_end(); ++h)
    {
        if (h->is_border())
            boundary_edges.push_back(h);
    }
}

// =====================
// MAIN
// =====================
int main()
{
    const std::string input_mesh =
        "C:/Users/fetik/OneDrive/Masaüstü/4_güz/bitirme/meshDeleteFaces/location2_drone_visible_mesh_FINAL.stl";

    const std::string output_boundary =
        "C:/Users/fetik/OneDrive/Masaüstü/4_güz/bitirme/Boundires/finalboundries.stl";

    Mesh mesh;
    if (!CGAL::IO::read_polygon_mesh(input_mesh, mesh))
    {
        std::cerr << "Mesh okunamadi!\n";
        return EXIT_FAILURE;
    }

    std::vector<Mesh::Halfedge_handle> boundary_edges;
    extract_boundary(mesh, boundary_edges);

    std::cout << "Boundary edge sayisi: " << boundary_edges.size() << "\n";

    write_boundary_stl(boundary_edges, output_boundary);

    std::cout << "Boundary STL yazildi:\n" << output_boundary << std::endl;
    return EXIT_SUCCESS;
}
