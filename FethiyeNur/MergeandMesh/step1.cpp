#define _CRT_SECURE_NO_WARNINGS
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <iostream>
#include <vector>
#include <sstream>
#include <iomanip>

int main()
{
    using PointT = pcl::PointXYZ;
    pcl::PointCloud<PointT>::Ptr merged_cloud(new pcl::PointCloud<PointT>);

    std::vector<int> rotations = { 0, 45, 90, 135, 180, 225, 270, 315 };

    std::string base_path =
        "C:/Users/fetik/OneDrive/Masaüstü/4_güz/bitirme/MO_Bitirme_Hospital/Location2/";

    // ----------------------------
    // MERGE
    // ----------------------------
    for (int index = 1; index <= 16; ++index)
    {
        int rotation = rotations[(index - 1) % 8];
        std::string height = (index <= 8) ? "075" : "220";

        std::stringstream ss;
        ss << base_path
            << std::setw(2) << std::setfill('0') << index
            << "_location_2_rotation_"
            << std::setw(3) << std::setfill('0') << rotation
            << "_height_" << height << ".pcd";

        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        if (pcl::io::loadPCDFile<PointT>(ss.str(), *cloud) == -1)
            continue;

        *merged_cloud += *cloud;
    }

    if (merged_cloud->empty())
    {
        std::cerr << "Hic nokta okunamadi." << std::endl;
        return -1;
    }

    // ----------------------------
    // VOXEL DOWNSAMPLE
    // ----------------------------
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(merged_cloud);
    voxel.setLeafSize(0.03f, 0.03f, 0.03f);

    pcl::PointCloud<PointT>::Ptr voxel_cloud(new pcl::PointCloud<PointT>);
    voxel.filter(*voxel_cloud);

    std::cout << "Voxel sonrasi: " << voxel_cloud->size() << std::endl;

    // ----------------------------
    // SOR
    // ----------------------------
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(voxel_cloud);
    sor.setMeanK(30);
    sor.setStddevMulThresh(0.8);

    pcl::PointCloud<PointT>::Ptr sor_cloud(new pcl::PointCloud<PointT>);
    sor.filter(*sor_cloud);

    std::cout << "SOR sonrasi: " << sor_cloud->size() << std::endl;

    // ----------------------------
    // ROR
    // ----------------------------
    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(sor_cloud);
    ror.setRadiusSearch(0.1);
    ror.setMinNeighborsInRadius(2);

    pcl::PointCloud<PointT>::Ptr final_cloud(new pcl::PointCloud<PointT>);
    ror.filter(*final_cloud);

    std::cout << "ROR sonrasi: " << final_cloud->size() << std::endl;

    // ----------------------------
    // KAYDET
    // ----------------------------
    pcl::io::savePCDFileBinary(
        base_path + "Location2_merged_voxel_sor2.pcd",
        *final_cloud
    );

    std::cout << "Tum islemler tamamlandi." << std::endl;
    std::cin.get();
    return 0;
}