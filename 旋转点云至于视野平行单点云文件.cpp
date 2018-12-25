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

//The Bilateral Filter for Point Clouds中双边滤波
PointXYZRGB Bilateral_Filters(pcl::PointCloud<PointXYZRGB>::Ptr cloudin, pcl::PointCloud<pcl::Normal>::Ptr normalsin, vector<float> kdpointDis,
	float sigmac, float sigmas)
{
	PointXYZRGB oricloud = cloudin->at(0);//计算点
	Normal orinormal = normalsin->at(0);//计算点法向量
	double norx = orinormal.normal_x;//法线的x
	double nory = orinormal.normal_y;//法线的y
	double norz = orinormal.normal_z;//法线的z

	float sigmac2 = pow(sigmac, 2);//sigamc平方
	float sigmas2 = pow(sigmas, 2);//sigams平方
	double up = 0, down = 0;
	double dis2 = (oricloud.x - cloudin->at(1).x)*(oricloud.x - cloudin->at(1).x) +
		(oricloud.y - cloudin->at(1).y)*(oricloud.y - cloudin->at(1).y) +
		(oricloud.z - cloudin->at(1).z)*(oricloud.z - cloudin->at(1).z);
	double dis = sqrt(dis2);
	for (int i = 1; i < cloudin->size(); i++)
	{
		Normal Jnormal = normalsin->at(i);
		double pi_jx = (oricloud.x - cloudin->at(i).x);//pi-pj的x
		double pi_jy = (oricloud.y - cloudin->at(i).y);//pi-pj的y
		double pi_jz = (oricloud.z - cloudin->at(i).z);//pi-pj的z

		double dd = kdpointDis.at(i);
		double dd2 = pow(kdpointDis.at(i), 2);//欧氏距离平方
		double dn = (pi_jx * norx + pi_jy * nory + pi_jz * norz);// (pi_jz * norz);
		double dn2 = pow((pi_jx * norx + pi_jy * nory + pi_jz * norz), 2);//向量内积的平方
		double Wc = exp(-dd2 / (2 * sigmac2));//Wc,光顺滤波权重函数；
		double Ws = exp(-dn2 / (2 * sigmas2));//Ws,特征保持权重函数；
		up += Wc * Ws * dn;
		down += Wc * Ws;
	}
	double a = up / down;
	a = -a;
	PointXYZRGB newpoint;
	newpoint = oricloud;
	newpoint.x = oricloud.x + a * orinormal.normal_x;
	newpoint.y = oricloud.y + a * orinormal.normal_y;
	newpoint.z = oricloud.z + a * orinormal.normal_z;
	return newpoint;
}

//int main()
//{
//	cout << "Ready.....";
//	getchar();
//	cout << "Start working" << endl;
//	time_t t1 = GetTickCount();
//	map<string, string> param;
//	ConfigFileRead(param);
//	string format = param["format"];
//	string filepath = param["filepath"];
//	string filepathCircle = param["filepath_Circle"];
//	string outfilepath = param["outfilepath"];
//	string outfilepathCircle = param["outfilepath_Circle"];
//	string kdtreeRadiusOrNum = param["kdtreeRadiusOrNum"];
//	istringstream sigmacstr(param["sigmac"]), sigmasstr(param["sigmas"]), neipointnum(param["knum"]), kdtreeradiusstr(param["kdtreeradius"]);
//	float sigmac, sigmas, kdtreeradius;
//	sigmacstr >> sigmac;
//	sigmasstr >> sigmas;
//	kdtreeradiusstr >> kdtreeradius;
//	int k;
//	neipointnum >> k;
//	cout << "Input Filepath: " << filepathCircle << endl;
//	cout << "Output FIlepath: " << outfilepathCircle << endl;
//
//	//循环版本
//	vector<string> Allname;
//	vector< pcl::PointCloud<PointXYZRGB>::Ptr> AllCloud;
//	GetAllFiles_CertainFormat(filepathCircle, Allname, format);
//	cout << "File numbers: " << Allname.size() << endl;
//	for (int i = 0; i < 1; i++)
//	{
//		pcl::PointCloud<PointXYZRGB>::Ptr cloudtemp(new pcl::PointCloud<PointXYZRGB>);
//		pcl::io::loadPCDFile(Allname.at(i), *cloudtemp);
//		AllCloud.push_back(cloudtemp);
//	}
//
//
//	//单个文件版本
//	//pcl::PointCloud<PointXYZRGB>::Ptr cloudin(new pcl::PointCloud<PointXYZRGB>);
//	////pcl::io::loadPCDFile("C:\\Users\\zhihong\\Desktop\\2\\2\\A619-T-3_side29pcd.pcd", *cloud);
//
//	pcl::PointCloud<PointXYZRGB>::Ptr cloudin(new pcl::PointCloud<PointXYZRGB>);
//	pcl::PointCloud<PointXYZRGB>::Ptr cloudout(new pcl::PointCloud<PointXYZRGB>);
//	pcl::PointCloud<PointXYZ>::Ptr cloudbox(new pcl::PointCloud<PointXYZ>);
//	pcl::PointCloud<PointXYZRGB>::Ptr cloudplane(new pcl::PointCloud<PointXYZRGB>);
//	cloudin = AllCloud.at(0);
//
//	double xmax, xmin, ymax, ymin, zmax, zmin;
//	getbox(cloudin, cloudbox,xmax, xmin, ymax, ymin, zmax, zmin, cloudplane);
//	for (int i = 0; i < cloudbox->size(); i++)
//	{
//		auto tempoint = cloudbox->at(i);
//		cout << "Point: " << i <<  " Coordinate:( " << tempoint.x << "," <<
//			tempoint.y << "," << tempoint.z << " )" << endl;
//	}
//
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//	viewer->setBackgroundColor(255, 255, 255);
//	viewer->addPointCloud(cloudin, "pointin");
//	//show box
//	viewer->addLine(cloudbox->at(0), cloudbox->at(1), 255, 0, 0, "Line1");
//	viewer->addLine(cloudbox->at(0), cloudbox->at(2), 255, 0, 0, "Line2");
//	viewer->addLine(cloudbox->at(1), cloudbox->at(3), 255, 0, 0, "Line3");
//	viewer->addLine(cloudbox->at(2), cloudbox->at(3), 255, 0, 0, "Line4");
//
//	viewer->addLine(cloudbox->at(4), cloudbox->at(5), 0, 255, 0, "Line5");
//	viewer->addLine(cloudbox->at(4), cloudbox->at(6), 0, 255, 0, "Line6");
//	viewer->addLine(cloudbox->at(5), cloudbox->at(7), 0, 255, 0, "Line7");
//	viewer->addLine(cloudbox->at(6), cloudbox->at(7), 0, 255, 0, "Line8");
//
//	viewer->addLine(cloudbox->at(0), cloudbox->at(4), 0, 0, 255, "Line9");
//	viewer->addLine(cloudbox->at(1), cloudbox->at(5), 0, 0, 255, "Line10");
//	viewer->addLine(cloudbox->at(2), cloudbox->at(6), 0, 0, 255, "Line11");
//	viewer->addLine(cloudbox->at(3), cloudbox->at(7), 0, 0, 255, "Line12");
//
//	//show plane
//	viewer->addLine(cloudplane->at(0), cloudplane->at(2), 0, 0, 255, "plane1");
//	viewer->addLine(cloudplane->at(0), cloudplane->at(1), 0, 255, 0, "plane2");
//	viewer->addLine(cloudplane->at(1), cloudplane->at(2), 255, 0, 0, "plane3");
//
//	time_t t2 = GetTickCount();	
//	cout << "Use time: " << ((t2 - t1)*1.0 / 1000) << " s" << endl;
//	cout << "Finished!..." << endl;
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce(100);
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//	AllCloud.clear();
//	Allname.clear();
//	getchar();
//}

double getangle(const PointXYZ &p1,const PointXYZ &p2)
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

void movePointCloud(const pcl::PointCloud<PointXYZRGB>::Ptr &cloudin, pcl::PointCloud<PointXYZRGB>::Ptr &cloudout,PointXYZ &center)
{
	for (int i=0;i<cloudin->size();i++)
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
		temp.x = temp.x + center.x;
		temp.y = temp.y + center.y;
		temp.z = temp.z + center.z;
		cloudout->push_back(temp);
	}
}


int main()
{
	cout << "Ready.....";
	//getchar();
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
	for (int i = 0; i< 1; i++)
	{
		pcl::PointCloud<PointXYZRGB>::Ptr cloudtemp(new pcl::PointCloud<PointXYZRGB>);
		pcl::io::loadPCDFile(Allname.at(i), *cloudtemp);
		AllCloud.push_back(cloudtemp);
	}

	pcl::PointCloud<PointXYZRGB>::Ptr cloudin(new pcl::PointCloud<PointXYZRGB>);
	pcl::PointCloud<PointXYZ>::Ptr cloudCalbox(new pcl::PointCloud<PointXYZ>);
	pcl::PointCloud<PointXYZRGB>::Ptr cloudout1(new pcl::PointCloud<PointXYZRGB>);
	
	cloudin = AllCloud.at(0);
	for (int i=0;i< AllCloud.at(0)->size();i++)
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


	//计算点云重心
	/*Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloudin, centroid);*/
	//获取AABB,OBB包围盒
	//getOBBbox(cloudin, centroid, cloudbox);
	OBB_matrix myobb;
	AABB_matrix myaabb;
	vector<pcl::PointXYZ> printPointsetobb, printPointsetaabb;
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
	cout << "anglex: " << anglexr << endl
		<< "angley: " << angleyr << endl
		<< "anglez: " << anglezr << endl;
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

	pcl::transformPointCloud<PointXYZRGB>(*cloudmove, *cloudout1, transform1);
	pcl::transformPointCloud<PointXYZRGB>(*cloudout1, *cloudout1, transform2);
	pcl::transformPointCloud<PointXYZRGB>(*cloudout1, *cloudout1, transform3);

	pcl::PointCloud<PointXYZRGB>::Ptr cloudout(new pcl::PointCloud<PointXYZRGB>);
	moveBackPointCloud(cloudout1, cloudout, center);
	
	//旋转后用于显示框线点云
	pcl::PointCloud<PointXYZ>::Ptr cloudbox(new pcl::PointCloud<PointXYZ>);
	for (int i = 0; i < cloudout->size(); i++)
	{
		PointXYZ temppoint;
		temppoint.x = cloudout->at(i).x;
		temppoint.y = cloudout->at(i).y;
		temppoint.z = cloudout->at(i).z;
		cloudbox->push_back(temppoint);
	}

	//可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->addCoordinateSystem(1.0);
	viewer->setBackgroundColor(1, 1, 1);
	for (int i = 0; i < cloudin->size(); i++)
	{
		cloudin->at(i).r = 255;
		cloudin->at(i).g = 0;
		cloudin->at(i).b = 0;
	}
	viewer->addPointCloud(cloudin, "cloud1");
	
	//显示旋转前点云框
	PointXYZ zeropoint(0, 0, 0);
	viewer->addLine(myobb.center, zeropoint, 0.5f, 0.0f, 0.5f, "Z-C");
	viewbox(viewer, printPointsetobb, "cloudin");
	
	//旋转后显示
	vector<pcl::PointXYZ> printPointsetoutobb;
	OBB_matrix myoutobb;
	pclOBBbox(cloudbox, myoutobb, printPointsetoutobb);
	viewer->addPointCloud(cloudout, "cloudout");;
	viewbox(viewer, printPointsetoutobb, "cloudout");



	time_t t2 = GetTickCount();
	cout << "Use time: " << ((t2 - t1)*1.0 / 1000) << " s" << endl;
	cout << "Finished!..." << endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	AllCloud.clear();
	Allname.clear();
	getchar();
}

/*viewer->addLine(myobb.center, myobb.x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
	viewer->addLine(myobb.center, myobb.y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
	viewer->addLine(myobb.center, myobb.z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");*/

	////aabb
	//pclAABBbox(cloudCalbox, myaabb, printPointsetaabb);
	//PointXYZ min_point_AABB = printPointsetaabb.at(0);
	//PointXYZ max_point_AABB = printPointsetaabb.at(1);
	////viewer->addLine(printPointsetaabb.at(0), printPointsetaabb.at(1), 1.0, 0, 0, "AABB");
	//viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 0.0, 0.0, 0.5, "AABBcube");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABBcube");

	//PointXYZ zeroplane1(min_point_AABB.x - 0.5, 0, 0);
	//PointXYZ zeroplane2(max_point_AABB.x + 0.5, 0, 0);
	//PointXYZ zeroplane3(max_point_AABB.x + 0.5, 0, min_point_AABB.z);
	//PointXYZ zeroplane4(min_point_AABB.x - 0.5, 0, min_point_AABB.z);

	//viewer->addLine(zeroplane1, zeroplane2, 1.0, 0.0, 1.0, "1 plane");
	//viewer->addLine(zeroplane2, zeroplane3, 1.0, 0.0, 1.0, "2 plane");
	//viewer->addLine(zeroplane3, zeroplane4, 1.0, 0.0, 1.0, "3 plane");
	//viewer->addLine(zeroplane1, zeroplane4, 1.0, 0.0, 1.0, "4 plane");

	////旋转后结果
	//
	////
	////transform(0, 0) = 
	////transform(0, 1) = -sin(theta);
	////transform(1, 0) = sin(theta);
	////transform(1, 1) = cos(theta);
	////Eigen::Matrix3f rotational_matrix_OBB = myobb.rotational_matrix_OBB;
	////Eigen::Matrix4f rotatematrix;
	/*Eigen::Vector3f position(myobb.position_OBB.x, myobb.position_OBB.y, myobb.position_OBB.z);
	Eigen::Quaternionf quat(myobb.rotational_matrix_OBB);*/
	/*viewer->addCube(position, quat, myobb.max_point_OBB.x - myobb.min_point_OBB.x,
		myobb.max_point_OBB.y - myobb.min_point_OBB.y, myobb.max_point_OBB.z - myobb.min_point_OBB.z, "OBB");*/
		//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
		//viewer->addPointCloud(cloudin, "pointin");