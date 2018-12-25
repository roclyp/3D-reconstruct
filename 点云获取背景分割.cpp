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
void view(shared_ptr<visualization::PCLVisualizer> viewer,PointCloud<PointXYZRGBA>::Ptr &cloud_a)
{
	//建立可视化窗口，并命名，可定义为全局指针，保证可全局使用
	viewer->initCameraParameters();//设置相机参数，用户从默认的角度和方向观察点云
	//创建不同viewport可以进行对比,参数为：x轴的最小值、最大值，y轴最小值、最大值，标识符
	viewer->setBackgroundColor(0, 0, 0);
	//设置背景颜色
	viewer->addCoordinateSystem(1.0);
	//添加坐标系,坐标轴——Z：蓝，Y：绿，X：红
	//addPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud,
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
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  PassThroughway(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr incloud, string A, float limitDown, float LimitUp)
//输入参数分别，点云，滤波字段，滤波范围下限，滤波范围上限
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud(incloud);
	pass.setFilterFieldName(A);//滤波字段，选择xyz
	pass.setFilterLimits(limitDown, LimitUp);
	pass.filter(*outcloud);
	return outcloud;
}

//统计滤波
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr StatistCloudway(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr incloud, int searchnum, double threshold)
//输入参数分为为：点云，kmeans聚类点数量，设置判断阈值
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	// 创建滤波器
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
	sor.setInputCloud(incloud);
	sor.setMeanK(searchnum);//搜索范围内点个数
	sor.setStddevMulThresh(threshold);//设置判断是否为离群点的阈值
	sor.filter(*outcloud);
	return outcloud;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr BilaCloudway(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr incloud, float sigma_s, float sigma_r)
//输入参数分为为：点云，双边滤波窗口大小，
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	// 创建滤波器
	pcl::FastBilateralFilter<pcl::PointXYZRGBA> fbf;
	fbf.setInputCloud(incloud);
	fbf.setSigmaS(sigma_s);
	fbf.setSigmaR(sigma_r);//高斯方差
	fbf.filter(*outcloud);
	return outcloud;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr SegCloudway(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr incloud, double threshold, int max_iterations, double probability, 
	int modeType = SACMODEL_PLANE, int methodType = SAC_RANSAC)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// 创建平面分割对象
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);//提取目标模型属性
	seg.setMethodType(pcl::SAC_RANSAC);//采样方法，有Ransac，Lmeds
	seg.setDistanceThreshold(threshold);//差选的到目标模型的距离，若大于该阈值，则不再目标模型上，默认值为0
	seg.setMaxIterations(threshold);//最大迭代次数，默认值为50
	seg.setProbability(0.9);//至少一个样本不包含离群点的概率，默认值为0.99
	seg.setInputCloud(incloud);
	seg.segment(*inliers, *coefficients);//输出提取点的索引和目标模型的参数
	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
	}
	//提取特定对象
	
	pcl::ExtractIndices<pcl::PointXYZRGBA> Ei;
	Ei.setIndices(inliers);
	Ei.setInputCloud(incloud);
	Ei.filter(*outcloud);
	return outcloud;
}

int main()
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Incloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//int errors = pcl::io::loadPCDFile<pcl::PointXYZRGBA>("F:/DesktopFile_Pointget/1-DATA/2017-10-18/2017-1/2017-1.pcd", *Incloud);
	int errors = pcl::io::loadPCDFile<pcl::PointXYZRGBA>("H:/数据/2017.11.11/0_5/2017_11_113.pcd", *Incloud);
	if (errors == -1)
	{
		cout << "Can not find the file!" << endl;
		return -1;
	}
	cout << "Loaded" << endl;
	shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer("原始点云数据"));
	view(viewer,Incloud);
	//滤波处理
	//直通滤波分别在xxyz方向进行过滤，很粗暴粗糙的过滤大量背景
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PassCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//添加坐标系,坐标轴——Z：蓝，Y：绿，X：红
	PassCloud = PassThroughway(Incloud, "x", -0.5, 0.5);
	PassCloud = PassThroughway(PassCloud, "y", -0.27, 0.5);
	PassCloud = PassThroughway(PassCloud, "z", -0.5, 1);
	shared_ptr<visualization::PCLVisualizer> viewer2(new visualization::PCLVisualizer("直通滤波处理后结果-缩小范围"));
	view(viewer2, PassCloud);

	//统计滤波处理，过滤离群点
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr StatistCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	StatistCloud = StatistCloudway(PassCloud,20,15);
	shared_ptr<visualization::PCLVisualizer> viewer3(new visualization::PCLVisualizer("统计滤波处理后结果-剔除离群点"));
	view(viewer3, StatistCloud);


	////双边滤波处理
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr BilaCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//BilaCloud = StatistCloudway(PassCloud,20,0.2);
	//shared_ptr<visualization::PCLVisualizer> viewer4(new visualization::PCLVisualizer("双边滤波-降噪，边缘保存"));
	//view(viewer4, BilaCloud);
	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr SegCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//SegCloud = SegCloudway(SegCloud,0.01,50,0.9);
	//shared_ptr<visualization::PCLVisualizer> viewer5(new visualization::PCLVisualizer("分割初看"));
	//view(viewer5,SegCloud);

	return 0;
}