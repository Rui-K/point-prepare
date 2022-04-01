#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
 
#include <boost/thread/thread.hpp>
#include <pcl/features/boundary.h>
#include <math.h>
#include <boost/make_shared.hpp>
#include <ctime>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
 
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/kdtree/kdtree_flann.h>
 
#include <pcl/io/ply_io.h>
 
#include <pcl/filters/statistical_outlier_removal.h>

std::string data_root = "/home/kangrui/Data/FP_point/";
std::string data_name = "内支撑";

int estimateBorders(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,float re,float reforn) 
{ 
    pcl::PCDWriter writer;
	pcl::PointCloud<pcl::Boundary> boundaries; //保存边界估计结果
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; //定义一个进行边界特征估计的对象
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normEst; //定义一个法线估计的对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    
    std::cout<<"Normal computing......"<<std::endl; 
	normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud)); 
    normEst.setSearchMethod (tree);
	normEst.setKSearch(reforn);; //设置法线估计的半径
	normEst.compute(*normals); //将法线估计结果保存至normals
	//输出法线的个数
	std::cout<<"reforn: "<<reforn<<std::endl;
	std::cerr << "normals: " << normals->size() << std::endl;
 
	boundEst.setInputCloud(cloud); //设置输入的点云
	boundEst.setInputNormals(normals); //设置边界估计的法线，因为边界估计依赖于法线
	boundEst.setRadiusSearch(re); //设置边界估计所需要的半径
	boundEst.setAngleThreshold(M_PI/4); //边界估计时的角度阈值
	boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); //设置搜索方式KdTree
	boundEst.compute(boundaries); //将边界估计结果保存在boundaries
 
 
	//存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
	for(int i = 0; i < cloud->points.size(); i++) 
	{ 
		
		if(boundaries[i].boundary_point > 0) 
		{ 
			cloud_boundary->push_back(cloud->points[i]); 
		} 
	} 
    writer.write (data_root + data_name + "_boundary.pcd", *cloud_boundary, false);
	return 0; 
} 
int
	main()
{
    clock_t start, end;
 
	float re,reforn;
	re=0.4;
	reforn=50;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src (new pcl::PointCloud<pcl::PointXYZ>); 

 
	//Laden der PCD-Files 
	pcl::io::loadPLYFile (data_root + data_name + ".ply", *cloud_src);
    std::cout<<data_name<<" loaded"<<std::endl;
    start = clock();
	estimateBorders(cloud_src,re,reforn);
    end = clock();
    std::cout << "This Loop using time: "<<(double)(end-start)/CLOCKS_PER_SEC << "s" << std::endl;
	return 0;
}