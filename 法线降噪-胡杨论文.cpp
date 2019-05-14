#include "PCLlibrary.h"
#include <iostream>
#include <string>
#include <pcl/console/time.h>   // TicToc
#include <time.h>
#include <pcl/search/kdtree.h>
#include  <direct.h> 
using namespace std;

#define Pi 3.141592657;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBPtr;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointXYZPtr;


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

//int main()
//{
//	pcl::PointXYZ pointViewer;
//	pointViewer.x = 0;
//	pointViewer.y = 0;
//	pointViewer.z = 0;
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::io::loadPCDFile("C:\\Users\\zhihong\\Desktop\\cup\\2018.11.21\\1_50FramesNoNan.pcd",*cloud);
//
//	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
//	ne.setInputCloud(cloud);
//	// Create an empty kdtree representation, and pass it to the normal estimation object.
//	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
//	ne.setSearchMethod(tree);
//
//	// Output datasets
//	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
//
//	// Use all neighbors in a sphere of radius 3cm
//	ne.setRadiusSearch(0.006);
//
//	// Compute the features
//	ne.compute(*cloud_normals);
//
//	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
//
//	for (int i = 0; i < cloud->size(); i++) {
//		auto Norsrc = cloud->at(i);
//		auto Norfit = cloud_normals->at(i);
//		auto abvector = Norsrc.x*Norfit.normal_x + Norsrc.y*Norfit.normal_y + Norsrc.z*Norfit.normal_z;
//		auto mod = sqrt((Norsrc.x*Norsrc.x + Norsrc.y*Norsrc.y + Norsrc.z*Norsrc.z)) +
//			sqrt((Norfit.normal_x*Norfit.normal_x + Norfit.normal_y*Norfit.normal_y + Norfit.normal_z*Norfit.normal_z));
//		double tetha = acos(abvector*1.0 / mod);
//	}
//
//	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, cloud_normals, 2, 0.01, "normals");
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce(100);
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//
//	/*getchar();*/
//	//// Create a KD-Tree
//	//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
//
//	//// Output has the PointNormal type in order to store the normals calculated by MLS
//	//pcl::PointCloud<pcl::PointNormal> mls_points;
//
//	//// Init object (second point type is for the normals, even if unused)
//	//pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;
//
//	//mls.setComputeNormals(true);
//
//	//// Set parameters
//	//mls.setInputCloud(cloud);
//	//mls.setPolynomialFit(true);
//	//mls.setSearchMethod(tree);
//	//mls.setSearchRadius(0.03);
//
//	//// Reconstruct
//	//mls.process(mls_points);
//
//	//// Save output
//	//pcl::io::savePCDFile("C:\\Users\\zhihong\\Desktop\\cup\\2018.11.21\\out\\mls.pcd", mls_points);
//
//}

void ConfigFileRead(map<string, string>& m_mapConfigInfo)
{
	ifstream configFile;
	string path = "./shuangbian.ini";
	configFile.open(path.c_str());
	string str_line;
	if (configFile.is_open())
	{
		while (!configFile.eof())
		{
			getline(configFile, str_line);
			if (str_line.find('#') == 0) //过滤掉注释信息，即如果首个字符为#就过滤掉这一行
			{
				continue;
			}
			size_t pos = str_line.find('=');
			string str_key = str_line.substr(0, pos);
			string str_value = str_line.substr(pos + 1);
			m_mapConfigInfo.insert(pair<string, string>(str_key, str_value));
		}
	}
	else
	{
		cout << "Cannot open config file setting.ini, path: ";
		exit(-1);
	}
}

bool NormalIsOk(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempcloud, float threshold_angle, pcl::PointCloud<pcl::Normal>::Ptr &temp_normals, string comFlag)
{
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(tempcloud);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);
	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch(10);
	// Compute the features
	ne.compute(*cloud_normals);

	auto Norsrc = tempcloud->at(4);
	auto Norfit = cloud_normals->at(4);
	auto abvector = Norsrc.x*Norfit.normal_x + Norsrc.y*Norfit.normal_y + Norsrc.z*Norfit.normal_z;
	auto mod = sqrt((Norsrc.x*Norsrc.x + Norsrc.y*Norsrc.y + Norsrc.z*Norsrc.z)) +
		sqrt((Norfit.normal_x*Norfit.normal_x + Norfit.normal_y*Norfit.normal_y + Norfit.normal_z*Norfit.normal_z));
	double tetha = acos(abvector*1.0 / mod);
	double angle = tetha * 180.0 / Pi ;
	temp_normals = cloud_normals;
	if (angle > 90)
		angle = 180 - angle;
	if (comFlag == "小")
	{
		if (angle <= threshold_angle)
			return false;
		else
			return true;
	}
	else
		if (angle > threshold_angle)
			return false;
		else
			return true;
	//for (int i = 0; i < tempcloud->size(); i++) {
	//	auto Norsrc = tempcloud->at(i);
	//	auto Norfit = cloud_normals->at(i);
	//	auto abvector = Norsrc.x*Norfit.normal_x + Norsrc.y*Norfit.normal_y + Norsrc.z*Norfit.normal_z;
	//	auto mod = sqrt((Norsrc.x*Norsrc.x + Norsrc.y*Norsrc.y + Norsrc.z*Norsrc.z)) +
	//		sqrt((Norfit.normal_x*Norfit.normal_x + Norfit.normal_y*Norfit.normal_y + Norfit.normal_z*Norfit.normal_z));
	//	double tetha = acos(abvector*1.0 / mod);
	//}
}

int main()
{
	cout << "Ready.....";
	getchar();
	cout << "Start working" << endl;
	time_t t1 = GetTickCount();
	//viewer->setBackgroundColor(255, 255, 255);
	map<string, string> param;
	ConfigFileRead(param);
	string filepath=param["filepath"];
	istringstream tempangle(param["threshold_angle"]);
	float threshold_angle;
	string comFlag = param["大于小于"];
	tempangle >> threshold_angle;
	pcl::PointXYZ pointViewer;
	pointViewer.x = 0;
	pointViewer.y = 0;
	pointViewer.z = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile(filepath, *cloud);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	auto c = cloud->at(31850 - 513);
	for (int col = 181; col < 331; col++)
	{
		for (int row = 200; row < 324; row++)
		{
			int i = row * 512 + col;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			tempcloud->push_back(cloud->at(i - 513));
			tempcloud->push_back(cloud->at(i - 512));
			tempcloud->push_back(cloud->at(i - 511));
			tempcloud->push_back(cloud->at(i - 1));
			tempcloud->push_back(cloud->at(i));
			tempcloud->push_back(cloud->at(i+1));
			tempcloud->push_back(cloud->at(i + 513));
			tempcloud->push_back(cloud->at(i + 512));
			tempcloud->push_back(cloud->at(i + 511));
			pcl::PointCloud<pcl::Normal>::Ptr temp_normals(new pcl::PointCloud<pcl::Normal>);
			if (NormalIsOk(tempcloud, threshold_angle, temp_normals, comFlag))
			{
				outcloud->push_back(cloud->at(i));
				cloud_normals->push_back(temp_normals->at(4));
			}
				
		}

	}
	
	viewer->addPointCloud<pcl::PointXYZRGB>(outcloud, "cloud");
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(outcloud, cloud_normals, 3, 0.008, "normals");
	time_t t2 = GetTickCount();
	cout << "Use time: " << ((t2 - t1)*1.0 / 1000) << " s" << endl;
	cout << "Finished!..." << endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	getchar();

	/*getchar();*/
	//// Create a KD-Tree
	//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

	//// Output has the PointNormal type in order to store the normals calculated by MLS
	//pcl::PointCloud<pcl::PointNormal> mls_points;

	//// Init object (second point type is for the normals, even if unused)
	//pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;

	//mls.setComputeNormals(true);

	//// Set parameters
	//mls.setInputCloud(cloud);
	//mls.setPolynomialFit(true);
	//mls.setSearchMethod(tree);
	//mls.setSearchRadius(0.03);

	//// Reconstruct
	//mls.process(mls_points);

	//// Save output
	//pcl::io::savePCDFile("C:\\Users\\zhihong\\Desktop\\cup\\2018.11.21\\out\\mls.pcd", mls_points);

}


