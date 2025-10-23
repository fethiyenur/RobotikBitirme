#include <iostream>
#include <string>
#include <filesystem>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

namespace fs = std::filesystem;

int main()
{
    std::string input_file = "../Downsampling/Downsampled_PointCloudData/4380-pointcloud_downsampled-(0.01f, 0.01f, 0.01f).ply";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ror(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPLYFile<pcl::PointXYZ>(input_file, *cloud) == -1) {
        PCL_ERROR("Unable to read PLY file!\n");
        return -1;
    }
    std::cout << "Original point cloud size: " << cloud->size() << std::endl;

    // ----------------- Statistical Outlier Removal -----------------
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(70);
    sor.setStddevMulThresh(1);
    sor.filter(*cloud_sor);

    std::cout << "Size after StatisticalOutlierRemoval: " << cloud_sor->size() << std::endl;

    fs::path sor_dir = "StatisticalOutlierRemoval";
    fs::create_directories(sor_dir); // create directory if it does not exist
    fs::path sor_output = sor_dir / "filtered_SOR.ply";
    pcl::io::savePLYFileBinary(sor_output.string(), *cloud_sor);

    // ----------------- Radius Outlier Removal -----------------
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud_sor);
    ror.setRadiusSearch(0.03);
    ror.setMinNeighborsInRadius(5);
    ror.filter(*cloud_ror);

    std::cout << "Size after RadiusOutlierRemoval: " << cloud_ror->size() << std::endl;

    fs::path ror_dir = "RadiusOutlierRemoval";
    fs::create_directories(ror_dir); // create directory if it does not exist
    fs::path ror_output = ror_dir / "filtered_ROR.ply";
    pcl::io::savePLYFileBinary(ror_output.string(), *cloud_ror);

    std::cout << "Filtering completed. Files have been saved.\n";
    return 0;
}
