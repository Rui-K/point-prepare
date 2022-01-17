#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/exceptions.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/features/features.hpp>
#include <pcl/search/search.h>

#include <pcl/io/pcd_io.h>

#include <boost/filesystem.hpp>

#include <iostream>

int main(int argc, char** argv)
{
  // selecting GPU and prining info
    int device = 0;
    pcl::gpu::setDevice (device);
    pcl::gpu::printShortCudaDeviceInfo (device);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read ("/home/kangrui/Data/rabbit.pcd", *cloud);
    std::cerr << "PointCloud Readed: " << cloud->width * cloud->height << " data points." << std::endl;

    pcl::gpu::NormalEstimation::PointCloud gpuCloud;
	
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	kdtree->setInputCloud(cloud);

	size_t cloud_size = cloud->points.size();
	//std::cout << "cloud->points.size()/cloud_size: "<< cloud_size <<std::endl;
	std::vector<float> dists;
	std::vector<std::vector<int>> neighbors_all;
	std::vector<int> sizes;
	neighbors_all.resize(cloud_size);
	sizes.resize(cloud_size);
	int k = 10;
#pragma omp parallel for
	for (int i = 0; i < cloud_size; ++i)
	{
		kdtree->nearestKSearch(cloud->points[i], k, neighbors_all[i], dists);
		sizes[i]=(int)neighbors_all[i].size();//每个点的邻居点的数量
	}
      // convert to single array format
	int max_nn_size = *max_element(sizes.begin(), sizes.end());//从size范围里查找最多邻居数量
	std::vector<int> temp_neighbors_all(max_nn_size * cloud->size());
	pcl::gpu::PtrStep<int> ps(&temp_neighbors_all[0], max_nn_size * pcl::gpu::PtrStep<int>::elem_size);
	for (size_t i = 0; i < cloud->size(); ++i)
		std::copy(neighbors_all[i].begin(), neighbors_all[i].end(), ps.ptr(i));

	pcl::gpu::NeighborIndices indices;
	gpuCloud.upload(cloud->points);
	indices.upload(temp_neighbors_all, sizes, max_nn_size);

	pcl::gpu::NormalEstimation::Normals normals;
	pcl::gpu::NormalEstimation::computeNormals(gpuCloud, indices, normals);
	pcl::gpu::NormalEstimation::flipNormalTowardsViewpoint(gpuCloud, 0.f, 0.f, 0.f, normals);
	
	std::vector<pcl::PointXYZ> downloaded;
	normals.download(downloaded);

  return 0;
}