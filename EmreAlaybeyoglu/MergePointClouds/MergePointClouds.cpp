#include <iostream>
#include <vector>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

int main()
{
    std::vector<std::string> ply_files = {
        "3574-pointcloud.ply",
        "3577-pointcloud.ply",
        "3580-pointcloud.ply",
        "3583-pointcloud.ply",
        "3586-pointcloud.ply",
        "3589-pointcloud.ply",
        "3592-pointcloud.ply",
        "3595-pointcloud.ply",
        "3598-pointcloud.ply",
        "3601-pointcloud.ply",
        "3604-pointcloud.ply",
        "3607-pointcloud.ply",
        "3610-pointcloud.ply",
        "3613-pointcloud.ply",
        "3616-pointcloud.ply",
        "3619-pointcloud.ply"
    };

    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(
        new pcl::PointCloud<pcl::PointXYZ>
    );

    for (const auto& path : ply_files)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(
            new pcl::PointCloud<pcl::PointXYZ>
        );

        if (pcl::io::loadPLYFile<pcl::PointXYZ>(path, *temp_cloud) == -1)
            continue;

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*temp_cloud, *temp_cloud, indices);

        *merged_cloud += *temp_cloud;
    }

    pcl::io::savePLYFileASCII("merged.ply", *merged_cloud);

    return 0;
}
