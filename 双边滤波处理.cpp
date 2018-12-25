#include "PCLlibrary.h"
#include <iostream>
#include <string>
#include <pcl/console/time.h>   // TicToc
#include <time.h>
#include <pcl/search/kdtree.h>
#include <direct.h> 
#include <omp.h>
#include"getfile.h"
#include <time.h>

using namespace std;

#define Pi 3.141592657;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBPtr;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointXYZPtr;


//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

//int main()
//{
//	//viewer->setBackgroundColor(0, 0, 0);
//	//pcl::PolygonMesh meshin;
//	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_IN(new pcl::PointCloud<pcl::PointXYZRGB>);
//	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
//	
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_IN(new pcl::PointCloud<pcl::PointXYZRGB>);
//	//string filename = "C:\\Users\\zhihong\\Desktop\\2\\1.obj";
//	string filename = "C:\\Users\\zhihong\\Desktop\\2\\HY_pcd\\0.001.pcd";
//	filename = "C:\\Users\\zhihong\\Desktop\\2\\2\\A619-T-3_Deal4.pcd";
//	filename = "C:\\Users\\zhihong\\Desktop\\2\\2\\A619-T-3_NoCompress.pcd";
//	//int error = pcl::io::loadOBJFile(filename, meshin);
//	int error = pcl::io::loadPCDFile(filename, *cloud_IN);
//	if (error == -1)
//	{
//		PCL_WARN("Haven't load the Cloud First(The source one)!");
//		return -1;
//	}
//	PCL_INFO("Loaded");
//
//	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	//获得K领域点
//	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
//	//pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree(pcl::KdTreeFLANN<pcl::PointXYZRGB>);
//	tree->setInputCloud(cloud_IN);
//	n.setInputCloud(cloud_IN);
//	n.setSearchMethod(tree);
//	n.setKSearch(10);
//	//开始进行法向计算
//	n.compute(*normals);
//
//	
//	//显示
//	int v1(0), v2(1);
//	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//	viewer->addPointCloud(cloud_IN, "in",v1);
//	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_IN, "sample cloud0", v2);
//	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud_IN, normals, 50, 0.01, "normals", v2);
//
//
//
//	////viewer->addPolygonMesh(meshin, "my2");
//	////viewer->setRepresentationToWireframeForAllActors(); //网格模型以线框图模式显示
//	//DWORD Start = ::GetTickCount();
//
//	//for (int i = 0; i < cloud_IN->size(); i++) {
//	//	PointXYZI temp;
//	//	temp.x = cloud_IN->at(i).x;
//	//	temp.y = cloud_IN->at(i).y;
//	//	temp.z = cloud_IN->at(i).z;
//	//	temp.intensity = 0.5;
//	//	cloud_out->push_back(temp);
//	//}
//	//cout << "step1" << endl;
//	//pcl::search::KdTree<pcl::PointXYZI>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZI>);
//	//pcl::BilateralFilter<pcl::PointXYZI> bf;
//	//bf.setInputCloud(cloud_out);
//	//bf.setSearchMethod(tree1);
//	//bf.setHalfSize(5.0);
//	//bf.setStdDev(0.003);
//	//bf.filter(*cloud_out2);
//	//cout << "step2" << endl;
//
//	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out3(new pcl::PointCloud<pcl::PointXYZ>);
//	//for (int i = 0; i < cloud_IN->size(); i++) {
//	//	PointXYZ temp;
//	//	temp.x = cloud_out2->at(i).x;
//	//	temp.y = cloud_out2->at(i).y;
//	//	temp.z = cloud_out2->at(i).z;
//	//	cloud_out3->push_back(temp);
//	//}
//
//	//DWORD End = ::GetTickCount();
//
//	//cout << "The time taken for test is: " << End - Start << endl;
//	////ostringstream saveName;
//	//////saveName << "C:/Users/Zhihong MA/Desktop/data/" << savenum << ".pcd";
//	////////saveName << "C:/Users/Zhihong MA/Desktop/The_whole" << savenum << ".ply";
//	
//
//	while (!viewer->wasStopped())
//	{
//		viewer->spinOnce(100);
//		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//	}
//	return 0;
//}

//int
//main(int argc, char*argv[])
//{
//
//	// 读入点云文件  
//	
//	pcl::PointCloud<PointXYZRGB>::Ptr cloudin(new pcl::PointCloud<PointXYZRGB>);
//	pcl::PointCloud<PointXYZI>::Ptr cloud(new pcl::PointCloud<PointXYZI>);
//	//pcl::io::loadPCDFile("C:\\Users\\zhihong\\Desktop\\2\\2\\A619-T-3_side29pcd.pcd", *cloud);
//	pcl::io::loadPCDFile("C:\\Users\\zhihong\\Desktop\\cup\\2018.11.21\\1_50FramesNoNan.pcd", *cloudin);
//	pcl::PointCloud<PointXYZI>outcloud;
//	//cloud->height = 424;
//	//cloud->width = 512;
//	for (int i = 0; i < cloudin->size(); i++) {
//		PointXYZI temppoint;
//		temppoint.x = cloudin->at(i).x;
//		temppoint.y = cloudin->at(i).y;
//		temppoint.z = cloudin->at(i).z;
//		temppoint.intensity = (cloudin->at(i).r + cloudin->at(i).g + cloudin->at(i).b) / 3;
//		cloud->push_back(temppoint);
//	}
//
//	// 建立kdtree  
//	//pcl::KdTreeFLANN<PointXYZ>::Ptr tree1(new pcl::KdTreeFLANN<PointXYZ>);
//	pcl::search::KdTree<PointXYZI>::Ptr tree1(new pcl::search::KdTree<PointXYZI>);
//	pcl::BilateralFilter<PointXYZI>::Ptr bf;
//	bf->setInputCloud(cloud);
//	bf->setSearchMethod(tree1);
//	bf->setHalfSize(5.0);
//	bf->setStdDev(0.003);
//	bf->filter(outcloud);
//
//
//	// 保存滤波输出点云文件  
//	pcl::io::savePCDFile("C:\\Users\\zhihong\\Desktop\\cup\\2018.11.21\\out\\shuangbian_1.pcd", outcloud);
//	return (0);
//}


//string getpointpath()
//{
//	ifstream PointFile;
//	/*infile.open()*/
//	char buffer[MAX_PATH];
//	getcwd(buffer, MAX_PATH);
//	string filepath(buffer);
//	filepath += "\\point.txt";
//	PointFile.open(filepath, ios::in);
//	if (!PointFile)
//	{
//		cout << "There is no PointFile.txt in current directory! \nNow is creating..." << endl;
//		ofstream Pointout(filepath);
//		return 0;
//	}
//	else
//	{
//		cout << "point.txt right!" << endl;
//		assert(PointFile.is_open());
//	}
//	string instr, filename;
//	while (getline(PointFile, instr))
//	{
//		if (!instr.find("#"))
//			filename = instr;
//	}
//	PointFile.close();
//	return filename;
//}
//
//
//int main()
//{
//	string filename = getpointpath();
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::io::loadPCDFile(filename,*cloud_in);
//	pcl::search::KdTree<pcl::PointXYZRGB>tree(new pcl::search::KdTree<PointXYZRGB>);
//	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
//	kdtree.setInputCloud(cloud_in);
//
//
//	return 0;
//}

//读取配置文件
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

//基于 TOF 深度传感的植物三维点云数据获取与去噪方法中的双边滤波

//PointXYZRGB Bilateral_Filters(pcl::PointCloud<PointXYZRGB>::Ptr cloudin, pcl::PointCloud<pcl::Normal>::Ptr normalsin, vector<float> kdpointDis,
//	float sigmac, float sigmas)
//{
//	PointXYZRGB oricloud = cloudin->at(0);//计算点
//	Normal orinormal = normalsin->at(0);//计算点法向量
//	float sigmac2 = pow(sigmac, 2);//sigamc平方
//	float sigmas2 = pow(sigmas, 2);//sigams平方
//	double up=0,down=0;
//	double dis2 = (oricloud.x - cloudin->at(1).x)*(oricloud.x - cloudin->at(1).x) +
//		(oricloud.y - cloudin->at(1).y)*(oricloud.y - cloudin->at(1).y) +
//		(oricloud.z - cloudin->at(1).z)*(oricloud.z - cloudin->at(1).z);
//	double dis = sqrt(dis2);
//	for (int i = 1; i < cloudin->size(); i++)
//	{
//		Normal Jnormal = normalsin->at(i);
//		double pi_jx = (oricloud.x - cloudin->at(i).x);//pi-pj的x
//		double pi_jy = (oricloud.y - cloudin->at(i).y);//pi-pj的y
//		double pi_jz = (oricloud.z - cloudin->at(i).z);//pi-pj的z
//		double norx = orinormal.normal_x ;//法线的x
//		double nory = orinormal.normal_y ;//法线的y
//		double norz = orinormal.normal_z ;//法线的z
//
//		double x2 = pow(kdpointDis.at(i), 2);//欧氏距离平方
//		double y2 = pow((pi_jx * norx + pi_jy * nory + pi_jz * norz)/(pi_jz * norz), 2);//向量内积的平方
//		double Wc = exp(-x2 / (2 * sigmac2));//Wc,光顺滤波权重函数；
//		double Ws = exp(-y2 / (2 * sigmas2));//Ws,特征保持权重函数；
//		up += Wc * Ws*(pi_jx * Jnormal.normal_x + pi_jy * Jnormal.normal_y + pi_jz * Jnormal.normal_z);
//		down += Wc * Ws;
//	}
//	double a = up / down;
//	PointXYZRGB newpoint;
//	newpoint = oricloud;
//	newpoint.x = oricloud.x + a * orinormal.normal_x;
//	newpoint.y = oricloud.y + a * orinormal.normal_y;
//	newpoint.z = oricloud.z + a * orinormal.normal_z;
//	return newpoint;
//}

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

int main()
{
	cout << "Ready.....";
	getchar();
	cout << "Start working" << endl;
	time_t t1 = GetTickCount();
	map<string, string> param;
	ConfigFileRead(param);
	string format= param["format"];
	string filepath = param["filepath"];
	string filepathCircle = param["filepath_Circle"];
	string outfilepath = param["outfilepath"];
	string outfilepathCircle = param["outfilepath_Circle"];
	string kdtreeRadiusOrNum = param["kdtreeRadiusOrNum"];
	istringstream sigmacstr(param["sigmac"]), sigmasstr(param["sigmas"]),neipointnum(param["knum"]), kdtreeradiusstr(param["kdtreeradius"]);
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
	GetAllFiles_CertainFormat(filepathCircle,Allname,format);
	cout << "File numbers: " << Allname.size() << endl;
	for (int i = 0; i < Allname.size(); i++)
	{
		pcl::PointCloud<PointXYZRGB>::Ptr cloudtemp(new pcl::PointCloud<PointXYZRGB>);
		pcl::io::loadPCDFile(Allname.at(i), *cloudtemp);
		AllCloud.push_back(cloudtemp);
	}

	
	//单个文件版本
	//pcl::PointCloud<PointXYZRGB>::Ptr cloudin(new pcl::PointCloud<PointXYZRGB>);
	////pcl::io::loadPCDFile("C:\\Users\\zhihong\\Desktop\\2\\2\\A619-T-3_side29pcd.pcd", *cloud);
	//pcl::io::loadPCDFile(filepath, *cloudin);
	//pcl::PointCloud<PointXYZRGB>::Ptr cloudout(new pcl::PointCloud<PointXYZRGB>);
	omp_set_num_threads(8);
#pragma omp parallel for
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	for (int id = 0; id < AllCloud.size() ; id++)//
	{
		pcl::PointCloud<PointXYZRGB>::Ptr cloudin(new pcl::PointCloud<PointXYZRGB>);
		pcl::PointCloud<PointXYZRGB>::Ptr cloudout(new pcl::PointCloud<PointXYZRGB>);
		cloudin = AllCloud.at(id);
		vector<int>kdpointID(k+1);
		vector<float>kdpointDis(k+1);
		// 建立kdtree  
		pcl::search::KdTree<PointXYZRGB>::Ptr kdtree1(new pcl::search::KdTree<PointXYZRGB>);
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		//pcl::KdTreeFLANN<PointXYZ>::Ptr tree1(new pcl::KdTreeFLANN<PointXYZ>);
		if (kdtreeRadiusOrNum == "Num")
		{
			kdtree1->setInputCloud(cloudin);
			//计算每个点法向量
			n.setInputCloud(cloudin);
			//点云法向计算时，需要所搜的近邻点大小
			n.setKSearch(k);
			//开始进行法向计算
			n.compute(*normals);
		}
		else if(kdtreeRadiusOrNum == "Radius")
		{
			kdtree1->setInputCloud(cloudin);
			n.setInputCloud(cloudin);
			n.setRadiusSearch(kdtreeradius);
			n.compute(*normals);
		}
		else
		{
			cout << "The Kdtree method of " << id << " is not 'Num' or 'Radius' " << endl;
			continue;
		}
		int cloudinsize = cloudin->size();
		for (int i = 0; i < cloudinsize; i++) {//cloudin->size()
			pcl::PointCloud<PointXYZRGB>::Ptr tempkdcloud(new pcl::PointCloud<PointXYZRGB>);
			pcl::PointCloud<pcl::Normal>::Ptr tempnormals(new pcl::PointCloud<pcl::Normal>);
			
			if (kdtree1->nearestKSearch(cloudin->at(i), k+1, kdpointID, kdpointDis)) {
				//tempcloud中第一个点为计算点,tempnormals第一个为计算点法向量
				int cc = i;
				tempkdcloud->push_back(cloudin->at(i));
				tempnormals->push_back(normals->at(i));
			/*	cloudin->at(i).r = 0;
				cloudin->at(i).g = 0;
				cloudin->at(i).b = 255;*/
				for (int j = 1; j <= k; j++)
				{
					//检查领域点
					tempkdcloud->push_back(cloudin->at(kdpointID[j]));
					tempnormals->push_back(normals->at(kdpointID[j]));
				}
				PointXYZRGB tempnew_point;
				tempnew_point = Bilateral_Filters(tempkdcloud, tempnormals, kdpointDis, sigmac, sigmas);
				cloudout->push_back(tempnew_point);
			}
		}
		string outpath = outfilepathCircle + "\\" + "Filter_" + getpostfixname(Allname.at(id));
		pcl::io::savePCDFileBinary(outpath, *cloudout);
	}
	time_t t2 = GetTickCount();
	AllCloud.clear();
	Allname.clear();
	cout << "Use time: " << ((t2 - t1)*1.0 / 1000) << " s" << endl;
	cout << "Finished!..." << endl;
	getchar();
}