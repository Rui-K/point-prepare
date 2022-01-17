#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

using namespace std;

int
 main ()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),cloud_p (new pcl::PointCloud<pcl::PointXYZ>),cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  string fileroot = "/home/kangrui/Data/Guanglianda/";
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  cerr << "Start reading......"<<endl;
  reader.read(fileroot + "3.pcd", *cloud);
  std::cerr << "Point cloud data: " << cloud->size () << " points" << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (3000);
  seg.setDistanceThreshold (0.1);

  // //single extract
  // seg.setInputCloud (cloud);
  // seg.segment (*inliers, *coefficients);
  // pcl::ExtractIndices<pcl::PointXYZ> extract;
  // // Extract the inliers
  // extract.setInputCloud (cloud);
  // extract.setIndices (inliers);
  // extract.setNegative (false);
  // extract.filter (*cloud_p);
  // std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
  // writer.write<pcl::PointXYZ> (fileroot + "3_ground_1.pcd", *cloud_p, false);
  // extract.setNegative (true);
  // extract.filter (*cloud_f);
  // std::cerr << "Remain PointCloud: "<< cloud_f->width * cloud_f->height << " data points." << std::endl;
  // writer.write<pcl::PointXYZ> (fileroot + "3_remain.pcd", *cloud_f, false);
  //loop
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  int i = 0, nr_points = (int) cloud->size ();
  // While 70% of the original cloud is still there
  while (cloud->size () > 0.7 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    // std::stringstream ss;
    // ss << fileroot + "3_ground_" << i << ".pcd";
    // writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

    // Create the filtering object
    extract.setNegative (true); //正常提取还是反向提取, 设定为true应是反向提取
    extract.filter (*cloud_f);
    cloud.swap (cloud_f);//swap, 交换
    i++;
    if (i==10){
      writer.write<pcl::PointXYZ> (fileroot + "3_ground.pcd", *cloud, false);
      break;
    }
    
  }

  return (0);
}
