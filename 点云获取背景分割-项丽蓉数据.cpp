#include <stdio.h>
#include <Kinect.h>
#include <Windows.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>  
#include <opencv2/opencv.hpp>
#include <pcl/filters/impl/bilateral.hpp>
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
void view(shared_ptr<visualization::PCLVisualizer> viewer, PointCloud<PointXYZRGB>::Ptr &cloud_a)
{
	//建立可视化窗口，并命名，可定义为全局指针，保证可全局使用
	viewer->initCameraParameters();//设置相机参数，用户从默认的角度和方向观察点云
	//创建不同viewport可以进行对比,参数为：x轴的最小值、最大值，y轴最小值、最大值，标识符
	viewer->setBackgroundColor(0, 0, 0);
	//设置背景颜色
	viewer->addCoordinateSystem(1.0);
	//添加坐标系,坐标轴——X：蓝，Y：绿，Z：红
	//addPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
	//	const std::string &id = "cloud", int viewport = 0)
	viewer->addPointCloud(cloud_a, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);//设置点云特征
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
};

//滤波处理
//直通滤波
pcl::PointCloud<pcl::PointXYZRGB>::Ptr  PassThroughway(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr incloud, string A, float limitDown, float LimitUp)
//输入参数分别，点云，滤波字段，滤波范围下限，滤波范围上限
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(incloud);
	pass.setFilterFieldName(A);//滤波字段，选择xyz
	pass.setFilterLimits(limitDown, LimitUp);
	pass.filter(*outcloud);
	cout << "PassThrough Done！" << endl;
	return outcloud;
}
//

pcl::PointCloud<pcl::PointXYZRGB>::Ptr StatistCloudway(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr incloud, int searchnum, double threshold)
//输入参数分为为：点云，kmeans聚类点数量，设置判断阈值
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	// 创建滤波器
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(incloud);
	sor.setMeanK(searchnum);//搜索范围内点个数
	sor.setStddevMulThresh(threshold);//设置判断是否为离群点的阈值
	sor.filter(*outcloud);
	cout << "StatisticalOutlierRemoval Done！" << endl;
	return outcloud;
}

//BilateralFilter只能对PointXYZI类型的点云数据处理，FastBilateralFilter则只能处理有序的点云，本实验点与数据无法使用
pcl::PointCloud<pcl::PointXYZRGB>::Ptr FastBilaCloudway(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr incloud, float sigma_s, float sigma_r)
//输入参数分为为：点云，对于空间邻域/窗口使用的高斯标准偏差（空间）,用于控制相邻像素下降多少的高斯的标准偏差(深度)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	// 创建滤波器
	/*pcl::BilateralFilter<pcl::PointXYZRGB> bf;*/
	pcl::FastBilateralFilter<pcl::PointXYZRGB> fbf;
	//pcl::search::Search<pcl::PointXYZRGB>::Ptr kdtree;
	//创建kd树，快速双边滤波只能对有序点云进行处理，kd书可以处理无序点云
	//pcl::FastBilateralFilter<pcl::PointXYZRGB> fbf;
	fbf.setInputCloud(incloud);
	//bf.setSearchMethod(kdtree);
	fbf.setSigmaS(sigma_s);
	fbf.setSigmaR(sigma_r);//高斯方差
	//fbf.filter(*outcloud);
	fbf.applyFilter(*outcloud);
	return outcloud;
}

////平面模型分割
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr SegCloudway(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr incloud, double threshold, int max_iterations, double probability,
//	int modeType = SACMODEL_PLANE, int methodType = SAC_RANSAC)
//{
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//	// 创建平面分割对象
//	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
//	// Optional
//	seg.setOptimizeCoefficients(true);
//	// Mandatory
//	seg.setModelType(pcl::SACMODEL_PLANE);//提取目标模型属性
//	seg.setMethodType(pcl::SAC_RANSAC);//采样方法，有Ransac，Lmeds
//	seg.setDistanceThreshold(threshold);//差选的到目标模型的距离，若大于该阈值，则不再目标模型上，默认值为0
//	seg.setMaxIterations(threshold);//最大迭代次数，默认值为50
//	seg.setProbability(probability);//至少一个样本不包含离群点的概率，默认值为0.99
//	seg.setInputCloud(incloud);
//	seg.segment(*inliers, *coefficients);//输出提取点的索引和目标模型的参数
//	if (inliers->indices.size() == 0)
//	{
//		PCL_ERROR("Could not estimate a planar model for the given dataset.");
//	}
//	//提取特定对象
//
//	pcl::ExtractIndices<pcl::PointXYZRGB> Ei;
//	Ei.setIndices(inliers);
//	Ei.setInputCloud(incloud);
//	Ei.filter(*outcloud);
//	return outcloud;
//}
//
////局部增长分割
//void RegionGrowingSeg(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr incloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr outcloud)
//{
//	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> >(new pcl::search::KdTree<pcl::PointXYZRGB>);
//	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
//	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
//	normal_estimator.setSearchMethod(tree);
//	normal_estimator.setInputCloud(incloud);
//	normal_estimator.setKSearch(50);
//	normal_estimator.compute(*normals);
//
//	//pcl::IndicesPtr indices(new std::vector <int>);
//	//pcl::PassThrough<pcl::PointXYZRGB> pass;
//	//pass.setInputCloud(incloud);
//	//pass.setFilterFieldName("z");
//	//pass.setFilterLimits(0.0, 1.0);
//	//pass.filter(*indices);
//
//	pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
//	reg.setMinClusterSize(50);
//	reg.setMaxClusterSize(1000000);
//	reg.setSearchMethod(tree);
//	reg.setNumberOfNeighbours(30);
//	reg.setInputCloud(incloud);
//	//reg.setIndices (indices);
//	reg.setInputNormals(normals);
//	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
//	reg.setCurvatureThreshold(1.0);
//
//	std::vector <pcl::PointIndices> clusters;
//	reg.extract(clusters);
//	outcloud = reg.getColoredCloudRGBA();
//	//outcloud = reg.getColoredCloud();
//
//	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
//	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
//	std::cout << "These are the indices of the points of the initial" <<
//		std::endl << "cloud that belong to the first cluster:" << std::endl;
//	int counter = 0;
//	while (counter < clusters[0].indices.size())
//	{
//		std::cout << clusters[0].indices[counter] << ", ";
//		counter++;
//		if (counter % 10 == 0)
//			std::cout << std::endl;
//	}
//	cout << clusters.size() << endl;
//	std::cout << "RegionGrowingSegment Done！"  << std::endl;
//}

int main()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Incloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	int errors = pcl::io::loadPCDFile<pcl::PointXYZRGB>("C:\\Users\\Zhihong MA\\Desktop\\KinectPointdata\\xlr\\3.pcd", *Incloud);
	if (errors == -1)
	{
		cout << "Can not find the file!" << endl;
		return -1;
	}
	cout << "Loaded" << endl;
	shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer("原始点云数据"));
	view(viewer, Incloud);
	//滤波处理
	//直通滤波分别在xxyz方向进行过滤，很粗暴粗糙的过滤大量背景
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PassCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	PassCloud = PassThroughway(PassCloud, "x", -0.8, 1.2);
	PassCloud = PassThroughway(PassCloud, "y", -1.3, 1.3);
	PassCloud = PassThroughway(Incloud, "z", -0.5, 1.5);
	shared_ptr<visualization::PCLVisualizer> viewer2(new visualization::PCLVisualizer("直通滤波处理后结果-缩小范围"));
	view(viewer2, PassCloud);

	//统计滤波处理，过滤离群点
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr StatistCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	StatistCloud = StatistCloudway(PassCloud, 20, 2);
	shared_ptr<visualization::PCLVisualizer> viewer3(new visualization::PCLVisualizer("统计滤波处理后结果-剔除离群点"));
	view(viewer3, StatistCloud);

	////双边滤波处理
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr BilaCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//BilaCloud = BilaCloudway(StatistCloud, 5, 0.03);
	//shared_ptr<visualization::PCLVisualizer> viewer4(new visualization::PCLVisualizer("双边滤波-降噪，边缘保存"));
	//view(viewer4, BilaCloud);
	////平面分割
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr SegCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//SegCloud = SegCloudway(SegCloud, 0.01, 25, 0.9);
	//shared_ptr<visualization::PCLVisualizer> viewer5(new visualization::PCLVisualizer("分割初看"));
	//view(viewer5, SegCloud);

	////局部增长分割
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReGrowSegCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//RegionGrowingSeg(StatistCloud, ReGrowSegCloud);
	//shared_ptr<visualization::PCLVisualizer> viewer5(new visualization::PCLVisualizer("局部增长分割处理后结果-剔除离群点"));
	//view(viewer5, ReGrowSegCloud);

	return 0;
}