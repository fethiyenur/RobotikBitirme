#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>

#include <iostream>
#include <string>
#include <vector>
#include <fstream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef CGAL::Point_set_3<Point_3> Point_set;

int main(int argc, char** argv)
{
    // Parameters
    std::string input_file = "4380_filtered.ply";
    std::string output_file = "output_mesh.stl";

    // Alpha and offset parameters
    // alpha: mesh detail level (smaller = more detailed)
    // offset: distance from surface (smaller = tighter wrapping)
    double alpha = 0.02;  // Adjust according to your point cloud size
    double offset = 0.01; // Adjust according to your point cloud size

    // Command line arguments
    if (argc > 1) input_file = argv[1];
    if (argc > 2) output_file = argv[2];
    if (argc > 3) alpha = std::stod(argv[3]);
    if (argc > 4) offset = std::stod(argv[4]);

    std::cout << "CGAL Alpha Wrapping" << std::endl;
    std::cout << "===================" << std::endl;
    std::cout << "Input file: " << input_file << std::endl;
    std::cout << "Output file: " << output_file << std::endl;
    std::cout << "Alpha: " << alpha << std::endl;
    std::cout << "Offset: " << offset << std::endl << std::endl;

    // Check if file exists
    std::ifstream test_file(input_file);
    if (!test_file.good())
    {
        std::cerr << "Error: File not found: " << input_file << std::endl;
        std::cerr << "Please check the file path." << std::endl;
        std::cout << "\nPress any key to continue..." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }
    test_file.close();

    // Load point cloud
    Point_set point_set;
    std::cout << "Loading point cloud..." << std::endl;

    std::ifstream in(input_file, std::ios::binary);
    if (!in || !CGAL::IO::read_PLY(in, point_set))
    {
        std::cerr << "Error: Cannot read file: " << input_file << std::endl;
        std::cout << "\nPress any key to continue..." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }
    in.close();

    // Convert Point_set to vector
    std::vector<Point_3> points;
    points.reserve(point_set.size());
    for (const auto& p : point_set.points())
    {
        points.push_back(p);
    }

    std::cout << "Loaded points: " << points.size() << std::endl;

    if (points.empty())
    {
        std::cerr << "Error: No points loaded" << std::endl;
        std::cout << "\nPress any key to continue..." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    // Apply alpha wrapping
    std::cout << "\nApplying alpha wrapping..." << std::endl;
    std::cout << "This process may take some time..." << std::endl;
    Mesh mesh;

    try
    {
        CGAL::alpha_wrap_3(points, alpha, offset, mesh);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cout << "\nPress any key to continue..." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    std::cout << "\nGenerated mesh:" << std::endl;
    std::cout << "  - Vertices: " << mesh.number_of_vertices() << std::endl;
    std::cout << "  - Faces: " << mesh.number_of_faces() << std::endl;

    // Save mesh
    std::cout << "\nSaving mesh..." << std::endl;

    std::ofstream out(output_file);
    if (!out || !CGAL::IO::write_STL(out, mesh))
    {
        std::cerr << "Error: Cannot write file: " << output_file << std::endl;
        std::cout << "\nPress any key to continue..." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }
    out.close();

    std::cout << "\nSuccess! Mesh saved: " << output_file << std::endl;
    std::cout << "\nPress any key to continue..." << std::endl;
    std::cin.get();

    return EXIT_SUCCESS;
}