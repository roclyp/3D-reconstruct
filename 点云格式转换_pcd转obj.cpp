#include "PCLlibrary.h"
#include <iostream>
#include <string>
#include <pcl/console/time.h>   // TicToc
using namespace std;

int main()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_first(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_second(new pcl::PointCloud<pcl::PointXYZRGB>);
	/*string filepath1 = "F:/DesktopFile_Pointget/1-DATA/2017-10-18/2017-1-after/A2017-1After_2017-7.pcd";
	string filepath2 = "F:/DesktopFile_Pointget/1-DATA/2017-10-18/2017-1-after/A2017-1After_2017-8.pcd";*/
	string filepath1 = "F:/1-DesktopFile_Pointget/1-DATA/2017.11.11/50_2_2After/After_2017.11.11_1.pcd";
	string filepath2 = "F:/1-DesktopFile_Pointget/1-DATA/2017.11.11/50_2_2After/After_2017.11.11_2.pcd";

	int error1 = pcl::io::loadPCDFile(filepath1, *cloud_first);
	int error2 = pcl::io::loadPCDFile(filepath2, *cloud_second);
	if (error1 == -1)
	{
		PCL_WARN("Haven't load the Cloud First(The source one)!");
		return -1;
	}
	if (error2 == -1)
	{
		PCL_WARN("Haven't load the Cloud Second(The Target one)!");
		return -1;
	}
	PCL_INFO("Loaded");
	
	;
	string saveName1 = "C:/Users/Zhihong MA/Desktop/After_2017.11.11_1.obj";
	string saveName2 = "C:/Users/Zhihong MA/Desktop/After_2017.11.11_2.obj";
	pcl::io::savePLYFileBinary(saveName1, *cloud_first);
	pcl::io::savePLYFileBinary(saveName2, *cloud_second);
}