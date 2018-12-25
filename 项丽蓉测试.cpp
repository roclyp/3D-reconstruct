#include "tchar.h"
#include <iostream>
#include <vector>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>


//int main_0()
//{
//
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::PCDReader reader;
//	reader.read("C:\\Users\\Zhihong MA\\Desktop\\cloud_filtered.pcd", *cloud_filtered); 
//	//pcl::PLYReader reader;
//	//reader.read("C:\\Users\\Zhihong MA\\Desktop\\final3.ply", *cloud_filtered); 
//	//pcl::io::loadPCDFile("C:\\Users\\Zhihong MA\\Desktop\\After_Test1.pcd", *cloud_filtered);
//	if (cloud_filtered->empty())
//	{
//		cout << "wrong" << endl;
//		return -1;
//	}
//	cout << "loaded" << endl;
//	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);// Create a KD-Tree
//	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;	// Init object (second point type is for the normals, even if unused)
//	pcl::PointCloud<pcl::PointNormal> mls_points;
//	mls.setComputeNormals(true);
//	mls.setInputCloud(cloud_filtered);
//	mls.setPolynomialFit(true);
//	mls.setSearchMethod(tree);
//	mls.setSearchRadius(0.03);
//	cout << "开始处理" << endl;
//	mls.process(mls_points);// Reconstruct
//	cout << "chuli" << endl;
//	// 如果加上这一段，把mls_points先写出再读入，分两次运行，就没有问题，就可以提取出中间的植株
//	pcl::PCDWriter writer;
//	writer.write<pcl::PointNormal>("C:\\Users\\Zhihong MA\\Desktop\\mls_points.pcd", mls_points, false);
//
//	pcl::PointCloud<pcl::PointNormal> mls_points1;
//	pcl::PCDReader reader2;
//	reader2.read("C:\\Users\\Zhihong MA\\Desktop\\mls_points.pcd", mls_points);
//	
//
//	std::vector<int> indices;
//	pcl::PointCloud<pcl::PointNormal> mls_points2;
//	//pcl::removeNaNFromPointCloud(mls_points1, mls_points2, indices);
//	//std::cout << "size: " << mls_points2.points.size() << std::endl;
//	std::cout << "size: " << mls_points1.points.size() << std::endl;
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud0(&mls_points2);
//	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
//	pcl::copyPointCloud(*cloud0, *normals);
//
//	pcl::search::Search<pcl::PointNormal>::Ptr tree2 = boost::shared_ptr<pcl::search::Search<pcl::PointNormal> >(new pcl::search::KdTree<pcl::PointNormal>);
//
//	pcl::RegionGrowing<pcl::PointNormal, pcl::Normal> reg;
//	reg.setMinClusterSize(50);
//	reg.setMaxClusterSize(1000000);
//	reg.setSearchMethod(tree2);
//	reg.setNumberOfNeighbours(30);
//	reg.setInputCloud(cloud0);
//	//reg.setIndices (indices);
//	reg.setInputNormals(normals);
//	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
//	reg.setCurvatureThreshold(1.0);
//
//	std::vector <pcl::PointIndices> clusters;
//	reg.extract(clusters);
//	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
//
//	int temp = 0;
//	for (int i = 0; i < clusters.size(); ++i) {
//		if (clusters[temp].indices.size() < clusters[i].indices.size()) {
//			temp = i;
//		}
//	}
//
//	std::cout << "the biggest cluster has  " << clusters[temp].indices.size() << " points" << std::endl;
//
//	pcl::copyPointCloud(*cloud0, clusters[temp], *cloud0);
//	pcl::PCDWriter writer2;
//	writer2.write<pcl::PointNormal>("C:\\Users\\Zhihong MA\\Desktop\\cloud00.pcd", *cloud0, false);
//
//	getchar();
//	return 0;
//}


int main()
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PCDReader reader;
	reader.read("C:\\Users\\Zhihong MA\\Desktop\\cloud_filtered.pcd", *cloud_filtered); 
	//pcl::PLYReader reader;
	//reader.read("C:\\Users\\Zhihong MA\\Desktop\\final3.ply", *cloud_filtered);
	//pcl::io::loadPCDFile("C:\\Users\\Zhihong MA\\Desktop\\After_Test1.pcd", *cloud_filtered);
	if (cloud_filtered->empty())
	{
		cout << "wrong" << endl;
		return -1;
	}
	cout << "loaded" << endl;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);// Create a KD-Tree
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;	// Init object (second point type is for the normals, even if unused)
	pcl::PointCloud<pcl::PointNormal> mls_points;
	mls.setComputeNormals(true);
	mls.setInputCloud(cloud_filtered);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);	cout << "开始处理" << endl;
	mls.process(mls_points);// Reconstruct
	cout << "chuli" << endl;
	//// 如果加上这一段，把mls_points先写出再读入，分两次运行，就没有问题，就可以提取出中间的植株
	//pcl::PCDWriter writer;
	//writer.write<pcl::PointNormal>("C:\\Users\\Zhihong MA\\Desktop\\mls_points.pcd", mls_points, false);

	//pcl::PointCloud<pcl::PointNormal> mls_points1;
	//pcl::PCDReader reader2;
	//reader2.read("C:\\Users\\Zhihong MA\\Desktop\\mls_points.pcd", mls_points);


	//std::vector<int> indices;
	//pcl::removeNaNFromPointCloud(mls_points, mls_points, indices);
	std::cout << "size: " << mls_points.points.size() << std::endl;

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud0(&mls_points);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::copyPointCloud(*cloud0, *normals);

	pcl::search::Search<pcl::PointNormal>::Ptr tree2 = boost::shared_ptr<pcl::search::Search<pcl::PointNormal> >(new pcl::search::KdTree<pcl::PointNormal>);

	pcl::RegionGrowing<pcl::PointNormal, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree2);
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(cloud0);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);
	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;

	int temp = 0;
	for (int i = 0; i < clusters.size(); ++i) {
		if (clusters[temp].indices.size() < clusters[i].indices.size()) {
			temp = i;
		}
	}

	//std::cout << "the biggest cluster has  " << clusters[temp].indices.size() << " points" << std::endl;

	pcl::copyPointCloud(*cloud0, clusters[temp], *cloud0);
	pcl::PCDWriter writer;
	writer.write<pcl::PointNormal>("C:\\Users\\Zhihong MA\\Desktop\\cloud00.pcd", *cloud0, false);

	getchar();
	return 0;
}


//int main()
//{
//
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
//	//pcl::PCDReader reader;
//	//reader.read("C:\\Users\\Zhihong MA\\Desktop\\After_Test1.pcd", *cloud_filtered); 
//	pcl::PLYReader reader;
//	reader.read("C:\\Users\\Zhihong MA\\Desktop\\final3.ply", *cloud_filtered);
//	//pcl::io::loadPCDFile("C:\\Users\\Zhihong MA\\Desktop\\After_Test1.pcd", *cloud_filtered);
//	if (cloud_filtered->empty())
//	{
//		cout << "wrong" << endl;
//		return -1;
//	}
//	cout << "loaded" << endl;
//	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);// Create a KD-Tree
//	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;	// Init object (second point type is for the normals, even if unused)
//	pcl::PointCloud<pcl::PointNormal> mls_points;
//	mls.setComputeNormals(true);
//	mls.setInputCloud(cloud_filtered);
//	mls.setPolynomialFit(true);
//	mls.setSearchMethod(tree);
//	mls.setSearchRadius(0.03);
//	cout << "开始处理" << endl;
//	mls.process(mls_points);// Reconstruct
//	cout << "chuli" << endl;
//	//// 如果加上这一段，把mls_points先写出再读入，分两次运行，就没有问题，就可以提取出中间的植株
//	pcl::PCDWriter writer;
//	writer.write<pcl::PointNormal>("C:\\Users\\Zhihong MA\\Desktop\\mls_points.pcd", mls_points, false);
//
//	pcl::PointCloud<pcl::PointNormal> mls_points;
//	pcl::PCDReader reader;
//	reader.read("C:\\Users\\Zhihong MA\\Desktop\\mls_points.pcd", mls_points);
//
//
//	std::vector<int> indices;
//	pcl::removeNaNFromPointCloud(mls_points, mls_points, indices);
//	std::cout << "size: " << mls_points.points.size() << std::endl;
//
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud0(&mls_points);
//	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
//	pcl::copyPointCloud(*cloud0, *normals);
//
//	pcl::search::Search<pcl::PointNormal>::Ptr tree2 = boost::shared_ptr<pcl::search::Search<pcl::PointNormal> >(new pcl::search::KdTree<pcl::PointNormal>);
//
//	pcl::RegionGrowing<pcl::PointNormal, pcl::Normal> reg;
//	reg.setMinClusterSize(50);
//	reg.setMaxClusterSize(1000000);
//	reg.setSearchMethod(tree2);
//	reg.setNumberOfNeighbours(30);
//	reg.setInputCloud(cloud0);
//	//reg.setIndices (indices);
//	reg.setInputNormals(normals);
//	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
//	reg.setCurvatureThreshold(1.0);
//
//	std::vector <pcl::PointIndices> clusters;
//	reg.extract(clusters);
//	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
//
//	int temp = 0;
//	for (int i = 0; i < clusters.size(); ++i) {
//		if (clusters[temp].indices.size() < clusters[i].indices.size()) {
//			temp = i;
//		}
//	}
//
//	//std::cout << "the biggest cluster has  " << clusters[temp].indices.size() << " points" << std::endl;
//
//	pcl::copyPointCloud(*cloud0, clusters[temp], *cloud0);
//	pcl::PCDWriter writer;
//	writer.write<pcl::PointNormal>("C:\\Users\\Zhihong MA\\Desktop\\cloud00.pcd", *cloud0, false);
//
//	getchar();
//	return 0;
//}
