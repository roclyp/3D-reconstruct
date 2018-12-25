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

