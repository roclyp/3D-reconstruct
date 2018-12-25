#include "PCLlibrary.h"
#include <iostream>
#include <string>
#include <pcl/console/time.h>   // TicToc
#include <time.h>

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBPtr;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointXYZPtr;


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

int main()
{
	viewer->setBackgroundColor(0, 0, 0);
	pcl::PolygonMesh meshin;
	string filename = "C:\\Users\\zhihong\\Desktop\\2\\1.obj";
	int error = pcl::io::loadOBJFile(filename, meshin);
	
	/*int error = pcl::io::loadPCDFile(filename, *cloud_IN);*/
	if (error == -1)
	{
		PCL_WARN("Haven't load the Cloud First(The source one)!");
		return -1;
	}
	PCL_INFO("Loaded");

	viewer->addPolygonMesh(meshin, "my2");
	viewer->setRepresentationToWireframeForAllActors(); //网格模型以线框图模式显示
	DWORD Start = ::GetTickCount();
	

	DWORD End = ::GetTickCount();

	cout << "The time taken for test is: " << End - Start << endl;
	//ostringstream saveName;
	////saveName << "C:/Users/Zhihong MA/Desktop/data/" << savenum << ".pcd";
	//////saveName << "C:/Users/Zhihong MA/Desktop/The_whole" << savenum << ".ply";
	////pcl::io::savePCDFile(saveName.str(), *Cloud_Temp_Last);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}