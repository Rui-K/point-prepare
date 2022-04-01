#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

using namespace std;
typedef pcl::PointXYZRGB PointT;
int
 main ()
{
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>),cloud_p (new pcl::PointCloud<PointT>),cloud_f (new pcl::PointCloud<PointT>);
  string fileroot = "/home/kangrui/Data/FP_point/";
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  cerr << "Start reading......"<<endl;
  reader.read(fileroot + "3_3_R.pcd", *cloud);
  std::cerr << "Point cloud data: " << cloud->size () << " points" << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
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
  // pcl::ExtractIndices<PointT> extract;
  // // Extract the inliers
  // extract.setInputCloud (cloud);
  // extract.setIndices (inliers);
  // extract.setNegative (false);
  // extract.filter (*cloud_p);
  // std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
  // writer.write<PointT> (fileroot + "3_ground_1.pcd", *cloud_p, false);
  // extract.setNegative (true);
  // extract.filter (*cloud_f);
  // std::cerr << "Remain PointCloud: "<< cloud_f->width * cloud_f->height << " data points." << std::endl;
  // writer.write<PointT> (fileroot + "3_remain.pcd", *cloud_f, false);

  //loop
  pcl::ExtractIndices<PointT> extract;
  int i = 0, nr_points = (int) cloud->size ();
  // While 70% of the original cloud is still there
  while (cloud->size () > 0.5 * nr_points)
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

    std::stringstream ss;
    ss << fileroot + "3_3_ground_" << i << ".pcd";
    writer.write<PointT> (ss.str (), *cloud_p, false);

    // Create the filtering object
    extract.setNegative (true); //正常提取还是反向提取, 设定为true应是反向提取
    extract.filter (*cloud_f);
    cloud.swap (cloud_f);//swap, 交换
    i++;
    if (i>=5){
      //writer.write<PointT> (fileroot + "3_ground.pcd", *cloud, false);
      break;
    }
  }
  writer.write<PointT> (fileroot + "3_Remain.pcd", *cloud, false);
  return (0);
}
