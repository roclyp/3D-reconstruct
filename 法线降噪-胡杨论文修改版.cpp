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
	double angle = tetha * 180.0 / Pi;
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
	string filepath = param["filepath"];
	istringstream tempangle(param["threshold_angle"]), knumstr(param["knum"]);
	float threshold_angle; int knum;
	string comFlag = param["大于小于"];
	knumstr >> knum;
	tempangle >> threshold_angle;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile(filepath, *cloud);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud);
	ne.setKSearch(knum);
	ne.compute(*normals);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	vector<int>kdpointID(knum + 1);
	vector<float>kdpointDis(knum + 1);
	// 建立kdtree  
	pcl::search::KdTree<PointXYZRGB>::Ptr kdtree1(new pcl::search::KdTree<PointXYZRGB>);	
	//pcl::KdTreeFLANN<PointXYZ>::Ptr tree1(new pcl::KdTreeFLANN<PointXYZ>);
	kdtree1->setInputCloud(cloud);
	////计算每个点法向量
	//n.setInputCloud(cloud);
	////点云法向计算时，需要所搜的近邻点大小
	//n.setKSearch(knum);
	////开始进行法向计算
	//n.compute(*normals);
	
	int cloudinsize = cloud->size();
	for (int i = 0; i < cloudinsize; i++) {//cloudin->size()
		pcl::PointCloud<PointXYZRGB>::Ptr tempkdcloud(new pcl::PointCloud<PointXYZRGB>);
		pcl::PointCloud<pcl::Normal>::Ptr tempnormals(new pcl::PointCloud<pcl::Normal>);
		tempkdcloud->push_back(cloud->at(i));
		if (kdtree1->nearestKSearch(cloud->at(i), knum + 1, kdpointID, kdpointDis))
		{
			for (int j = 1; j <= knum; j++)
			{
				tempkdcloud->push_back(cloud->at(kdpointID[j]));
			}
			tempnormals->push_back(normals->at(i));
			if (NormalIsOk(tempkdcloud, threshold_angle, tempnormals, comFlag))
			{
				outcloud->push_back(cloud->at(i));
				cloud_normals->push_back(tempnormals->at(4));
			}
		}
	}

	viewer->addPointCloud<pcl::PointXYZRGB>(outcloud, "cloud");
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(outcloud, cloud_normals, 2, 0.008, "normals");
	time_t t2 = GetTickCount();
	cout << "Use time: " << ((t2 - t1)*1.0 / 1000) << " s" << endl;
	cout << "Finished!..." << endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	getchar();

}


