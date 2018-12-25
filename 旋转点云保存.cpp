#include "PCLlibrary.h"
#include <iostream>
#include <string>
#include <pcl/console/time.h>   // TicToc
#include <time.h>
#include <pcl/search/kdtree.h>
#include <direct.h> 
#include <omp.h>
#include <time.h>
#include"getfile.h"
#include "obbBox.h"

using namespace std;

#define Pi 3.141592657;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBPtr;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointXYZPtr;


//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

//读取配置文件
void ConfigFileRead(map<string, string>& m_mapConfigInfo)
{
	ifstream configFile;
	string path = "./旋转.ini";
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

double getangle(const PointXYZ &p1, const PointXYZ &p2)
{
	auto abvector = p1.x*p2.x + p1.y*p2.y + p1.z*p2.z;
	auto mod = sqrt((p1.x*p1.x + p1.y*p1.y + p1.z*p1.z)) +
		sqrt((p2.x*p2.x + p2.y*p2.y + p2.z*p2.z));
	double tetha = acos(abvector*1.0 / mod);
	double angle = tetha * 180.0 / Pi;
	if (angle > 90)
		angle = 180 - angle;
	angle = 90 - angle;
	return angle;
}

void movePointCloud(const pcl::PointCloud<PointXYZRGB>::Ptr &cloudin, pcl::PointCloud<PointXYZRGB>::Ptr &cloudout, PointXYZ &center)
{
	for (int i = 0; i < cloudin->size(); i++)
	{
		PointXYZRGB temp;
		temp = cloudin->at(i);
		temp.x = temp.x - center.x;
		temp.y = temp.y - center.y;
		temp.z = temp.z - center.z;
		cloudout->push_back(temp);
	}
}

void moveBackPointCloud(const pcl::PointCloud<PointXYZRGB>::Ptr &cloudin, pcl::PointCloud<PointXYZRGB>::Ptr &cloudout, PointXYZ &center)
{
	for (int i = 0; i < cloudin->size(); i++)
	{
		PointXYZRGB temp;
		temp = cloudin->at(i);
		temp.z = temp.z + center.z;
		temp.x = temp.x + center.x;
		temp.y = temp.y + center.y;
		cloudout->push_back(temp);
	}
}

int main()
{
	cout << "Ready.....";
	getchar();
	cout << "Start working" << endl;
	time_t t1 = GetTickCount();
	map<string, string> param;
	ConfigFileRead(param);
	string format = param["format"];
	string filepath = param["filepath"];
	string filepathCircle = param["filepath_Circle"];
	string outfilepath = param["outfilepath"];
	string outfilepathCircle = param["outfilepath_Circle"];
	string kdtreeRadiusOrNum = param["kdtreeRadiusOrNum"];
	string xrotate = param["xrotate"];
	string yrotate = param["yrotate"];
	string zrotate = param["zrotate"];
	istringstream sigmacstr(param["sigmac"]), sigmasstr(param["sigmas"]), neipointnum(param["knum"]), kdtreeradiusstr(param["kdtreeradius"]);
	float sigmac, sigmas, kdtreeradius;
	sigmacstr >> sigmac;
	sigmasstr >> sigmas;
	kdtreeradiusstr >> kdtreeradius;
	int k;
	neipointnum >> k;
	cout << "Input Filepath: " << filepathCircle << endl;
	cout << "Output FIlepath: " << outfilepathCircle << endl;

	//循环版本
	vector<string> Allname;
	vector< pcl::PointCloud<PointXYZRGB>::Ptr> AllCloud;
	GetAllFiles_CertainFormat(filepathCircle, Allname, format);
	cout << "File numbers: " << Allname.size() << endl;
	for (int i = 0; i< Allname.size(); i++)
	{
		pcl::PointCloud<PointXYZRGB>::Ptr cloudtemp(new pcl::PointCloud<PointXYZRGB>);
		pcl::io::loadPCDFile(Allname.at(i), *cloudtemp);
		AllCloud.push_back(cloudtemp);
	}
	omp_set_num_threads(8);
#pragma omp parallel for
	for (int id = 0; id < AllCloud.size(); id++)//
	{
		pcl::PointCloud<PointXYZRGB>::Ptr cloudin(new pcl::PointCloud<PointXYZRGB>);
		pcl::PointCloud<PointXYZ>::Ptr cloudCalbox(new pcl::PointCloud<PointXYZ>);
		pcl::PointCloud<PointXYZRGB>::Ptr cloudout(new pcl::PointCloud<PointXYZRGB>);

		cloudin = AllCloud.at(id);
		for (int i = 0; i < AllCloud.at(0)->size(); i++)
		{
			PointXYZ temppoint;
			if (AllCloud.at(0)->at(i).x == 0 || AllCloud.at(0)->at(i).y == 0 || AllCloud.at(0)->at(i).z == 0)
				continue;
			else
			{
				temppoint.x = AllCloud.at(0)->at(i).x;
				temppoint.y = AllCloud.at(0)->at(i).y;
				temppoint.z = AllCloud.at(0)->at(i).z;
				cloudCalbox->push_back(temppoint);
			}
		}

		OBB_matrix myobb;
		vector<pcl::PointXYZ> printPointsetobb;
		//obb
		pclOBBbox(cloudCalbox, myobb, printPointsetobb);

		//旋转点云
		//获取旋转角度
		PointXYZ center = myobb.center;
		PointXYZ x_axis(1, 0, 0);
		PointXYZ y_axis(0, 1, 0);
		PointXYZ z_axis(0, 0, 1);
		PointXYZ xoz(center.x, 0, center.z);
		PointXYZ xoy(center.x, center.y, 0);
		PointXYZ yoz(0, center.y, center.z);
		double anglexr = getangle(yoz, y_axis);
		double angleyr = getangle(xoz, x_axis);
		double anglezr = getangle(xoy, x_axis);
		//cout << "anglex: " << anglexr << endl
		//	<< "angley: " << angleyr << endl
		//	<< "anglez: " << anglezr << endl;
		//先平移值原点
		pcl::PointCloud<PointXYZRGB>::Ptr cloudmove(new pcl::PointCloud<PointXYZRGB>);
		movePointCloud(cloudin, cloudmove, center);
		//旋转顺序z，x，y
		Eigen::Matrix4f transform1 = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f transform2 = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f transform3 = Eigen::Matrix4f::Identity();
		double theta = anglezr * M_PI / 180;
		double theta2 = anglexr * M_PI / 180;
		double theta3 = angleyr * M_PI / 180;
		//旋转z
		transform1(0, 0) = cos(theta);
		if (zrotate == "+") 
		{
			transform1(0, 1) = -sin(theta);
			transform1(1, 0) = sin(theta);
		}
		else
		{
			transform1(0, 1) = sin(theta);
			transform1(1, 0) = -sin(theta);
		}
		transform1(1, 1) = cos(theta);
		//旋转x
		transform2(1, 1) = cos(theta2);
		transform2(2, 2) = cos(theta2);
		if (xrotate == "+")
		{
			transform2(1, 2) = sin(theta2);
			transform2(2, 1) = -sin(theta2);
		}
		else
		{
			transform2(1, 2) = -sin(theta2);
			transform2(2, 1) = +sin(theta2);
		}
		//旋转y
		transform3(0, 0) = cos(theta3);
		transform3(2, 2) = cos(theta3);
		if (yrotate == "+")
		{
			transform3(0, 2) = -sin(theta3);
			transform3(2, 0) = sin(theta3);
		}
		else
		{
			transform3(0, 2) = sin(theta3);
			transform3(2, 0) = -sin(theta3);
		}
		

		pcl::transformPointCloud<PointXYZRGB>(*cloudmove, *cloudout, transform1);
		pcl::transformPointCloud<PointXYZRGB>(*cloudout, *cloudout, transform2);
		pcl::transformPointCloud<PointXYZRGB>(*cloudout, *cloudout, transform3);

		pcl::PointCloud<PointXYZRGB>::Ptr cloudout1(new pcl::PointCloud<PointXYZRGB>);
		moveBackPointCloud(cloudout, cloudout1, center);

		string outpath = outfilepathCircle + "\\" + "Rotate_" + getpostfixname(Allname.at(id));
		pcl::io::savePCDFileBinary(outpath, *cloudout1);
	}

	time_t t2 = GetTickCount();
	cout << "Use time: " << ((t2 - t1)*1.0 / 1000) << " s" << endl;
	cout << "Finished!..." << endl;

	AllCloud.clear();
	Allname.clear();
	getchar();
}
