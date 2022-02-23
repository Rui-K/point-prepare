#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d_omp.h>
#include <ctime>
#include <fstream>
#include <iostream>

typedef pcl::PointXYZ PointT;//rename PointXYZ PointT
using namespace std;

int
main ()
{
  // All the objects needed
  pcl::PCDReader reader;
  //pcl::PassThrough<PointT> pass;
  pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  clock_t start, end;

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>), cloud_f(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  //pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  //pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
  ofstream logfile;
  logfile.open("log.txt",ios::out | ios::trunc);

  // Read in the cloud data
  string data_root = "/home/kangrui/Data/FP_point/";
  string data_name = "GZB_3_Cloud_4_downsampled";
  reader.read (data_root + data_name + ".pcd", *cloud);
  cerr << "PointCloud has: " << cloud->size () << " data points." << endl;
  logfile<<"PointCloud has: " << cloud->size () << " data points." << endl;
  // Estimate point normals
  cerr << "Computing normals......"<<endl;
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setKSearch (20);
  ne.compute (*cloud_normals);

  //Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.2);
  seg.setRadiusLimits (0.4, 0.6);

  //Extract in a loop
  //int i = 0, nr_points = (int) cloud->size ();
  //cerr << "Start loop......"<<endl;
  
  volatile bool finish_flag=false;


  //#pragma omp parallel for
  for(int i = 0; i<=15; i++){
    // if(finish_flag==true){
    //   continue;
    // }
    logfile<<"column_"<< i <<endl;
    start = clock();
    seg.setInputCloud (cloud);
    seg.setInputNormals (cloud_normals);
    // Obtain the cylinder inliers and coefficients
    cerr << "Computing cylinder......"<<endl;
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    if (inliers_cylinder->indices.size()==0){
      cerr << "Could not estimate a cylinder model for the given dataset." << endl;
      //finish_flag==true;
      break;
    }
    else
      cerr << "Cylinder coefficients: " << *coefficients_cylinder << endl;
      cerr << "Radius: " << coefficients_cylinder->values[6] << endl;
      logfile << "Cylinder coefficients: " << *coefficients_cylinder << endl;
    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    extract.filter (*cloud_cylinder);

    cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->size () << " data points." << endl;
    logfile << "PointCloud representing the cylindrical component: " << cloud_cylinder->size () << " data points." << endl;
    stringstream ss;
    ss << data_root << data_name << "_cylinder_" << i << ".pcd";
    writer.write (ss.str(), *cloud_cylinder, false);

    // Create the remaining point cloud
    extract.setNegative (true); 
    extract.filter (*cloud_f);
    cloud.swap (cloud_f);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_cylinder);
    extract_normals.filter (*cloud_normals2);
    cloud_normals.swap(cloud_normals2);
    end = clock();
    cerr << "This Loop using time: "<<(double)(end-start)/CLOCKS_PER_SEC << "s" << endl;
    logfile<< "This Loop using time: "<<(double)(end-start)/CLOCKS_PER_SEC << "s" << endl;
  }
  cerr << "Remaining PointCloud: " << cloud->size () << " data points." << endl
            << "Writing to a new file......." << endl;
  writer.write (data_root + data_name + "_without_cylinder.pcd", *cloud, false);
  return (0);
}
