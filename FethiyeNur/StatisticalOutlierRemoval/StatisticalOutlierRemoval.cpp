#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
//30,0.8 Number of points before filtering: 86004 Number of points after filtering : 75559 4491
//      Number of points before filtering: 56512 Number of points after filtering : 51714 4443

//50,1.0 Number of points before filtering: 86004 Number of points after filtering : 77626
//      Number of points before filtering: 56512 Number of points after filtering : 52640

//80,1.5 Number of points before filtering: 86004 Number of points after filtering : 81303
//      Number of points before filtering: 56512 Number of points after filtering : 53991

int main()
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    // Nokta bulutlarý
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Girdi dosyasýný oku (.ply)
    pcl::PLYReader reader;
    if (reader.read<pcl::PointXYZ>("C:/Users/fetik/OneDrive/Masaüstü/4_güz/4443_pointcloud_downsampled1.ply", *cloud) == -1)
    {
        PCL_ERROR("Error: Could not read the file. Please check the file path.\n");
        return -1;
    }

    std::cout << "Number of points before filtering: " << cloud->points.size() << std::endl;

    // Filtreleme nesnesi
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(80);              // Ortalama alýnacak komþu sayýsý
    sor.setStddevMulThresh(1.5);   // Standart sapma çarpaný
    sor.filter(*cloud_filtered);

    std::cout << "Number of points after filtering: " << cloud_filtered->points.size() << std::endl;

    // Sonucu kaydet (.ply)
    pcl::PLYWriter writer;
    writer.write<pcl::PointXYZ>("4443_filtered3.ply", *cloud_filtered, false);

    std::cout << "Filtered point cloud saved as '4491_filtered.ply'." << std::endl;
    return 0;
}
