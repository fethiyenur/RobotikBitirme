#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

//Leaf size = sahnenin toplam boyutuna göre %0.1–%1 arası mantıklıdır.

// leaf 0.01f iken once: 217088 nokta Sonra: 2919 nokta

//leaf 0.05 iken Once: 217088 nokta Sonra: 8746 nokta

//leaf 0.001 iken Leaf size is too small for the input dataset. Integer indices would overflow.

// leaf 0.005 iken Once: 217088 nokta Sonra: 86004 nokta

//leaf 0.008 iken Once: 217088 nokta Sonra: 69367 nokta %68 azalma

// LeafSize(0.008, 0.008, 0.004) iken Once: 217088 nokta Sonra: 78280 nokta %64 azalma

// LeafSize(0.005, 0.005, 0.004) iken Once: 217088 nokta Sonra: 89237 nokta %59 azalma


int main()
{
    // 🔹 Girdi ve çıktı dosya yolları (kendi yolunu yaz)
    std::string input_file = "C:/Users/fetik/OneDrive/Masaüstü/4_güz/Dataset1_Square_withoutDelay/4491-pointcloud.ply";
    std::string output_file = "C:/Users/fetik/OneDrive/Masaüstü/4_güz/4491_pointcloud_downsampled7.ply";

    //float leaf = 0.006f; // voxel boyutu (metre cinsinden)

    // Nokta bulutlarını tanımla
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 🔹 PLY dosyasını oku
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(input_file, *cloud) == -1)
    {
        std::cerr << "Dosya okunamadı: " << input_file << std::endl;
        return -1;
    }

    std::cout << "Once: " << cloud->size() << " nokta" << std::endl;

    // 🔹 VoxelGrid filtresi
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.005, 0.005, 0.004);
    sor.filter(*cloud_filtered);

    std::cout << "Sonra: " << cloud_filtered->size() << " nokta" << std::endl;

    // 🔹 Downsample edilmiş bulutu kaydet
    pcl::io::savePLYFileBinary(output_file, *cloud_filtered);

    std::cout << "Kaydedildi: " << output_file << std::endl;
    return 0;
}
