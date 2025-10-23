#define _CRT_SECURE_NO_WARNINGS // G�venlik uyar�lar�n� kapatmak i�in, C4996 hatas�n� ��zmek i�in eklendi

#include <iostream>
#include <pcl/io/pcd_io.h> 
#include <pcl/io/ply_io.h> // PLY dosyas� okuma/yazma i�in
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <string>
#include <stdexcept>

// Downsampled nokta bulutunuzun dosya yolu
// L�tfen T�rk�e karakter i�ermeyen bir dizine ta��may� d���n�n.
const std::string INPUT_FILENAME = "C:/Users/fetik/OneDrive/Masa�st�/4_g�z/4491_pointcloud_downsampled4.ply";
const std::string OUTPUT_PREFIX = "filtered_cloud_";

// <<<<<<<<<<<<<<<< BURADAN SE��M YAPIYORSUNUZ >>>>>>>>>>>>>>>>>
// Kullanmak istedi�iniz filtreyi buraya yaz�n: 
// 'r' -> RadiusOutlierRemoval
// 'c' -> ConditionalRemoval
const char FILTER_MODE = 'r'; // Varsay�lan olarak RadiusOutlierRemoval se�ildi.
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

// PARAMETRE DE�ERLER� - HER DENEME ���N BURAYI DE���T�R�N
// Nokta bulutu metre �l�e�inde (yakla��k 7x3x5 m) oldu�u i�in 0.05 ba�ar�l� olmu�tur.
// -r (RadiusOutlierRemoval) i�in
const double RADIUS_SEARCH = 0.05;
const int MIN_NEIGHBORS = 5;

// -c (ConditionalRemoval) i�in
// Z aral��� (0.48m - 5.95m) i�inde oldu�undan, bu parametreler kesme yapacakt�r.
const double Z_MIN_CONDITION = 1.5;
const double Z_MAX_CONDITION = 4.0;

int main(int argc, char** argv)
{
    // Komut sat�r� kontrol� kald�r�ld� (argc != 2 kontrol� yok)

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Nokta Bulutunu Dosyadan Y�kle (PLY)
    std::cerr << "Yukleniyor: " << INPUT_FILENAME << " (PLY)" << std::endl;
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(INPUT_FILENAME, *cloud) == -1)
    {
        PCL_ERROR("PLY dosyasini okunamadi: %s \n", INPUT_FILENAME.c_str());
        std::cerr << "Lutfen INPUT_FILENAME degiskenindeki dosya yolunu kontrol edin." << std::endl;
        return (-1);
    }

    std::cerr << "Nokta bulutu yuklendi. Baslangic nokta sayisi: " << cloud->width * cloud->height << std::endl;

    std::string output_filename;

    // Komut sat�r� arg�man� yerine, FILTER_MODE de�i�keni kontrol ediliyor
    if (FILTER_MODE == 'r')
    {
        // PARAMETRELER� KONSOLA YAZDIR
        std::cerr << "\n--- RadiusOutlierRemoval Uygulaniyor ---" << std::endl;
        std::cerr << "Parametreler: " << std::endl;
        std::cerr << "  Radius Search: " << RADIUS_SEARCH << std::endl;
        std::cerr << "  Min Neighbors: " << MIN_NEIGHBORS << std::endl;

        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(RADIUS_SEARCH);
        outrem.setMinNeighborsInRadius(MIN_NEIGHBORS);
        outrem.setKeepOrganized(false); // Sadece ge�erli noktalar� d�nd�rmek i�in

        outrem.filter(*cloud_filtered);

        // ��kt� dosya ad�n� olu�tur
        output_filename = OUTPUT_PREFIX + "r_" +
            std::to_string(RADIUS_SEARCH) + "_" +
            std::to_string(MIN_NEIGHBORS) + ".ply";
    }
    else if (FILTER_MODE == 'c')
    {
        // PARAMETRELER� KONSOLA YAZDIR
        std::cerr << "\n--- ConditionalRemoval Uygulaniyor ---" << std::endl;
        std::cerr << "Parametreler: " << std::endl;
        std::cerr << "  Z Min (>): " << Z_MIN_CONDITION << std::endl;
        std::cerr << "  Z Max (<): " << Z_MAX_CONDITION << std::endl;

        // build the condition
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new
            pcl::ConditionAnd<pcl::PointXYZ>());

        // Z > Z_MIN_CONDITION
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
            pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, Z_MIN_CONDITION)));

        // Z < Z_MAX_CONDITION
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
            pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, Z_MAX_CONDITION)));

        // build the filter
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setCondition(range_cond);
        condrem.setInputCloud(cloud);
        condrem.setKeepOrganized(false); // Sadece ge�erli noktalar� d�nd�rmek i�in

        // apply filter
        condrem.filter(*cloud_filtered);

        // ��kt� dosya ad�n� olu�tur
        output_filename = OUTPUT_PREFIX + "c_" +
            std::to_string(Z_MIN_CONDITION) + "_" +
            std::to_string(Z_MAX_CONDITION) + ".ply";
    }
    else
    {
        std::cerr << "HATA: Gecersiz FILTER_MODE degeri ('" << FILTER_MODE << "'). Lutfen 'r' veya 'c' kullanin." << std::endl;
        return (1);
    }

    std::cerr << "\nFiltreleme SONUCU:" << std::endl;
    std::cerr << "Baslangic Nokta Sayisi: " << cloud->width * cloud->height << std::endl;
    std::cerr << "Filtrelenmis Nokta Sayisi: " << cloud_filtered->width * cloud_filtered->height << std::endl;

    // Filtrelenmi� Nokta Bulutunu Dosyaya Kaydet (PLY)
    std::cerr << "Kaydediliyor: " << output_filename << " (PLY ASCII)" << std::endl;
    pcl::io::savePLYFileASCII(output_filename, *cloud_filtered);

    std::cerr << "Islem Tamamlandi. Tablonuz icin gerekli veriler yukarida." << std::endl;

    return (0);
}
