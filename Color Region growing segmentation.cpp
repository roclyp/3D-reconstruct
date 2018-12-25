#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/impl/io.hpp>
using namespace std;
using namespace pcl;

struct cloud_num_xyz
{
	double x;
	double y;
	double z;
	int number;
	int size;
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud;
};


int main(int argc, char** argv)
{
	pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> >(new pcl::search::KdTree<pcl::PointXYZRGB>);

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile <pcl::PointXYZRGB>("C:\\Users\\Zhihong MA\\Desktop\\data\\2\\side_20.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}
	pcl::PointCloud <pcl::PointXYZRGBA>::Ptr orgin_cloud(new pcl::PointCloud <pcl::PointXYZRGBA>);
	pcl::copyPointCloud(*cloud, *orgin_cloud);

	//直通滤波选择分区部分，即提供索引，此处对我的数据无用，因为已经过直通滤波处理
	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-0.5, 1.0);
	pass.filter(*indices);

	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	reg.setInputCloud(cloud);
	reg.setIndices(indices);
	reg.setSearchMethod(tree);
	reg.setDistanceThreshold(10);//origin 10
	reg.setPointColorThreshold(20);//origin 6
	reg.setRegionColorThreshold(5);//origin 5
	reg.setMinClusterSize(300);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	pcl::visualization::CloudViewer viewer("Cluster viewer");
	vector<pcl::PointCloud <pcl::PointXYZRGB>::Ptr> Cloud_Seg;
	vector<cloud_num_xyz>PCL_for_range;

	for (int i = 0; i < clusters.size()/*colored_cloud->size()*/; i++)
	{
		pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_inline(new pcl::PointCloud <pcl::PointXYZRGB>);
		pcl::copyPointCloud(*orgin_cloud, clusters[i].indices, *cloud_inline);
		Cloud_Seg.push_back(cloud_inline);
	}
	//计算各个分块点云的坐标均值，进行比较和排序
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr outcloud(new pcl::PointCloud <pcl::PointXYZRGB>);
	for (int i = 0; i < Cloud_Seg.size()/*colored_cloud->size()*/; i++)
	{
		double sumx = 0.0, sumy = 0.0, sumz = 0.0;
		cloud_num_xyz currentPCL;
		double centerx = 0.0, centery = 0.0, centerz = 0.0;
		for (int j=0;j<Cloud_Seg[i]->size();j++)
		{
			sumx += Cloud_Seg[i]->points[j].x;
			sumy += Cloud_Seg[i]->points[j].y;
			sumz += Cloud_Seg[i]->points[j].z;
		}
		currentPCL.x = sumx / Cloud_Seg[i]->size()*1.0f;
		currentPCL.y = sumy / Cloud_Seg[i]->size()*1.0f;
		currentPCL.z = sumz / Cloud_Seg[i]->size()*1.0f;
		currentPCL.number = i;
		currentPCL.size = Cloud_Seg[i]->size();
		currentPCL.cloud = Cloud_Seg[i];
		PCL_for_range.push_back(currentPCL);
	}
	//对PCL_for_range进行排序
	for (int i = 0; i < PCL_for_range.size(); i++)
	{
		for (int j = i; j < PCL_for_range.size(); j++)
		{
			if (PCL_for_range[i].y<=PCL_for_range[j].y)
			{
				//Cloud_Seg[PCL_for_range[i].number].swap(Cloud_Seg[PCL_for_range[j].number]);
				swap(PCL_for_range[i], PCL_for_range[j]);	
			}
		}
	}
	for (int i = 0; i < PCL_for_range.size(); i++)
	{
		cout << "Current block: " << PCL_for_range[i].number << "  " 
			<< "Current block: " << PCL_for_range[i].size << "  "
			<< "Current Y: " << PCL_for_range[i].y << endl;
	}
	cout << "------------------" << endl;
	/*for (int i = 0; i < PCL_for_range.size()-4; i++)
	{
	Cloud_Seg.push_back(PCL_for_range[i].cloud);
	}*/
	for (int i = 0; i < Cloud_Seg.size(); i++)
	{
		cout << "Current Size: " << Cloud_Seg[i]->size() << endl;
	}


	//对排序结果进行剔除并计算结果
	for (int i = 0; i < PCL_for_range.size() - 2/*colored_cloud->size()*/; i++)
	{
		outcloud->operator+=(*PCL_for_range[i].cloud);
		//outcloud->operator+=(*Cloud_Seg[i]);
	}
	viewer.showCloud(outcloud);
	//pcl::io::savePCDFile("C:\\Users\\Zhihong MA\\Desktop\\bin.pcd", *outcloud,true);
	while (!viewer.wasStopped())
	{
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}
	//while (!viewer_Se->wasStopped())
	//{
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100));
	//}
	return (0);
}