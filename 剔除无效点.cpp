#include "PCLlibrary.h"
#include <iostream>
#include <string>
#include <pcl/console/time.h>   // TicToc
using namespace std;

int main()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
	/*string filepath1 = "F:/DesktopFile_Pointget/1-DATA/2017-10-18/2017-1-after/A2017-1After_2017-7.pcd";
	string filepath2 = "F:/DesktopFile_Pointget/1-DATA/2017-10-18/2017-1-after/A2017-1After_2017-8.pcd";*/
	/*string filepath1 = "C:/Users/zhihong/Desktop/hpl2/hpl.pcd";
	string filepath2 = "C:/Users/zhihong/Desktop/hpl2/hp2.pcd";*/
	string filepath = "C:/Users/zhihong/Desktop/cup/";

	string filepathout = "C:/Users/zhihong/Desktop/cup/1/";
	for (int i = 1; i <= 64; i++) {
		stringstream pathin, pathout;
		pathin<<filepath<<i<<"_50Frames.pcd";
		pathout << filepathout << i << "_50Frames.pcd";
		int error1 = pcl::io::loadPCDFile(pathin.str(), *cloud_in);
		if (error1 == -1)
		{
			PCL_WARN("Haven't load the Cloud First(The source one)!");
			return -1;
		}
		PCL_INFO("Loaded");
		for (int i = 0; i < cloud_in->size(); i++) {
			auto tempcloud = cloud_in->at(i);
			if (tempcloud.x != 0 || tempcloud.y != 0 || tempcloud.z != 0) {
				cloud_out->push_back(tempcloud);
			}
		}
		pcl::io::savePCDFileBinary(pathout.str(), *cloud_out);
	}
}