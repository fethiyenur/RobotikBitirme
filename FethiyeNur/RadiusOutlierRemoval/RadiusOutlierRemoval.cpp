#define _CRT_SECURE_NO_WARNINGS // Güvenlik uyarýlarýný kapatmak için, C4996 hatasýný çözmek için eklendi

#include <iostream>
#include <pcl/io/pcd_io.h> 
#include <pcl/io/ply_io.h> // PLY dosyasý okuma/yazma için
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <string>
#include <stdexcept>

// Downsampled nokta bulutunuzun dosya yolu
// Lütfen Türkçe karakter içermeyen bir dizine taþýmayý düþünün.
const std::string INPUT_FILENAME = "C:/Users/fetik/OneDrive/Masaüstü/4_güz/4491_pointcloud_downsampled4.ply";
const std::string OUTPUT_PREFIX = "filtered_cloud_";

// <<<<<<<<<<<<<<<< BURADAN SEÇÝM YAPIYORSUNUZ >>>>>>>>>>>>>>>>>
// Kullanmak istediðiniz filtreyi buraya yazýn: 
// 'r' -> RadiusOutlierRemoval
// 'c' -> ConditionalRemoval
const char FILTER_MODE = 'r'; // Varsayýlan olarak RadiusOutlierRemoval seçildi.
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

// PARAMETRE DEÐERLERÝ - HER DENEME ÝÇÝN BURAYI DEÐÝÞTÝRÝN
// Nokta bulutu metre ölçeðinde (yaklaþýk 7x3x5 m) olduðu için 0.05 baþarýlý olmuþtur.
// -r (RadiusOutlierRemoval) için
const double RADIUS_SEARCH = 0.05;
const int MIN_NEIGHBORS = 5;

// -c (ConditionalRemoval) için
// Z aralýðý (0.48m - 5.95m) içinde olduðundan, bu parametreler kesme yapacaktýr.
const double Z_MIN_CONDITION = 1.5;
const double Z_MAX_CONDITION = 4.0;

int main(int argc, char** argv)
{
    // Komut satýrý kontrolü kaldýrýldý (argc != 2 kontrolü yok)

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Nokta Bulutunu Dosyadan Yükle (PLY)
    std::cerr << "Yukleniyor: " << INPUT_FILENAME << " (PLY)" << std::endl;
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(INPUT_FILENAME, *cloud) == -1)
    {
        PCL_ERROR("PLY dosyasini okunamadi: %s \n", INPUT_FILENAME.c_str());
        std::cerr << "Lutfen INPUT_FILENAME degiskenindeki dosya yolunu kontrol edin." << std::endl;
        return (-1);
    }

    std::cerr << "Nokta bulutu yuklendi. Baslangic nokta sayisi: " << cloud->width * cloud->height << std::endl;

    std::string output_filename;

    // Komut satýrý argümaný yerine, FILTER_MODE deðiþkeni kontrol ediliyor
    if (FILTER_MODE == 'r')
    {
        // PARAMETRELERÝ KONSOLA YAZDIR
        std::cerr << "\n--- RadiusOutlierRemoval Uygulaniyor ---" << std::endl;
        std::cerr << "Parametreler: " << std::endl;
        std::cerr << "  Radius Search: " << RADIUS_SEARCH << std::endl;
        std::cerr << "  Min Neighbors: " << MIN_NEIGHBORS << std::endl;

        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(RADIUS_SEARCH);
        outrem.setMinNeighborsInRadius(MIN_NEIGHBORS);
        outrem.setKeepOrganized(false); // Sadece geçerli noktalarý döndürmek için

        outrem.filter(*cloud_filtered);

        // Çýktý dosya adýný oluþtur
        output_filename = OUTPUT_PREFIX + "r_" +
            std::to_string(RADIUS_SEARCH) + "_" +
            std::to_string(MIN_NEIGHBORS) + ".ply";
    }
    else if (FILTER_MODE == 'c')
    {
        // PARAMETRELERÝ KONSOLA YAZDIR
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
        condrem.setKeepOrganized(false); // Sadece geçerli noktalarý döndürmek için

        // apply filter
        condrem.filter(*cloud_filtered);

        // Çýktý dosya adýný oluþtur
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

    // Filtrelenmiþ Nokta Bulutunu Dosyaya Kaydet (PLY)
    std::cerr << "Kaydediliyor: " << output_filename << " (PLY ASCII)" << std::endl;
    pcl::io::savePLYFileASCII(output_filename, *cloud_filtered);

    std::cerr << "Islem Tamamlandi. Tablonuz icin gerekli veriler yukarida." << std::endl;

    return (0);
}
