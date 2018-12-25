#include <stdio.h>
#include <Kinect.h>
#include <Windows.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>  
#include <opencv2/opencv.hpp>
#include <cv.h>
#include <iostream>
#include "PCllibrary.h"
#include <GL/glut.h>
#include "GLFW/glfw3.h"
#include <time.h>
#include <thread>


using namespace std;
using namespace pcl;
//using namespace pcl::io;
using namespace io;
//可视化
void view(shared_ptr<visualization::PCLVisualizer> viewer, PointCloud<PointXYZRGBA>::Ptr &cloud_a)
{
	//建立可视化窗口，并命名，可定义为全局指针，保证可全局使用
	viewer->initCameraParameters();//设置相机参数，用户从默认的角度和方向观察点云
	//创建不同viewport可以进行对比,参数为：x轴的最小值、最大值，y轴最小值、最大值，标识符
	viewer->setBackgroundColor(0, 0, 0);
	//设置背景颜色
	viewer->addCoordinateSystem(1.0);
	//添加坐标系,坐标轴——Z：蓝，Y：绿，X：红
	//addPointCloud(const pcl::PointCloud<pcl::PointXYZRGBARGBA>::ConstPtr &cloud,
	//	const std::string &id = "cloud", int viewport = 0)
	viewer->addPointCloud(cloud_a, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);//设置点云特征
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
};

int main()
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Incloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	int errors = pcl::io::loadPCDFile<pcl::PointXYZRGBA>("C:\\Users\\Zhihong MA\\Desktop\\test2017.9.22\\AfterFilter\\After_Test1.pcd", *Incloud);
	if (errors == -1)
	{
		cout << "Can not find the file!" << endl;
		return -1;
	}
	cout << "Loaded" << endl;
	shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer("原始点云数据"));
	view(viewer, Incloud);

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
	vg.setInputCloud(Incloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl; //*

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02);

	int i = 0, nr_points = (int)cloud_filtered->points.size();
	while (cloud_filtered->points.size() > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_filtered);
	
	}

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
	ec.setClusterTolerance(0.02); // 2cm
	ec.setMinClusterSize(15);//100
	ec.setMaxClusterSize(300);//25000
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);


	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		std::stringstream ss;
		ss << "C:/Users/Zhihong MA/Desktop/test2017.9.22/AfterFilter/cloud_cluster_" << j << ".pcd";
		savePCDFileBinary(ss.str(), *cloud_cluster);
		//writer.write<pcl::PointXYZRGBA>(ss.str(), *cloud_cluster); //*
		j++;
		shared_ptr<visualization::PCLVisualizer> viewer2(new visualization::PCLVisualizer("原始点云数据"));
		view(viewer2, cloud_cluster);
	}

	return 0;
}