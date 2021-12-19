#include <iostream>
#include <ctime>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2), cloud2_filtered  (new pcl::PCLPointCloud2);

    string filename = "/home/kangrui/Data/FP_point/GZB_3_Cloud_6";
    clock_t start, end;
    start=clock();
    pcl::PCDReader reader;
    if(reader.read(filename + ".pcd", *cloud) == -1){
        PCL_ERROR ("Couldn't read file  \n");
        return(-1);
    }
    end=clock();
    cout << filename << "  Successfully Loaded" << endl
                <<"Reading time: "<<(double)(end-start)/CLOCKS_PER_SEC << "s" << endl
                <<"point cloud shape: "<< cloud->width * cloud->height
                <<endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    cout <<"Downsample start ......" << endl;
    start = clock();
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud2_filtered);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2 (*cloud2_filtered, *cloud_filtered);
    end = clock();
    cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl
        <<"Time cost: "<<(double)(end-start)/CLOCKS_PER_SEC << "s" << endl<<"Writing starts......"<<endl;
    
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> (filename+"_downsampled.pcd", *cloud_filtered, false);
    
    return(0);
}