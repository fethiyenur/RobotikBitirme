#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main()
{
    // Path to the unprocessed point cloud file
    std::string input_path = "Raw_PointCloudData/4380-pointcloud.ply";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Load the unprocessed point cloud file
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(input_path, *cloud) == -1)
    {
        PCL_ERROR("Could not open file: %s\n", input_path.c_str());
        return -1;
    }

    std::cout << "PointCloud before filtering: " << cloud->size() << " points.\n";

    // Create the VoxelGrid filter object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);

    std::cout << "PointCloud after filtering: " << cloud_filtered->size() << " points.\n";

    // Save the processed point cloud to Downsampled_PointCloudData folder
    std::string output_path = "Downsampled_PointCloudData/4380-pointcloud_downsampled.ply";
    pcl::io::savePLYFile(output_path, *cloud_filtered);

    std::cout << "Filtered point cloud saved to: " << output_path << std::endl;

    return 0;
}
