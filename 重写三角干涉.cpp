#include "PCLlibrary.h"
#include "getfile.h"
//#include "obbBox.h"
#include "MyClassType.h"

using namespace std;

#define Pi 3.141592657;

//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//读取配置文件

//获取配置文件
void ConfigFileRead(map<string, string>& m_mapConfigInfo)
{
	ifstream configFile;
	string path = "./ICP.ini";
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
//获取参数
void getPara(mypara &mpara)
{
	map<string, string> param;
	ConfigFileRead(param);
	//文件获取
	mpara.format = param["format"];
	mpara.filepathcloud1 = param["filepath1"];
	mpara.filepathcloud2 = param["filepath2"];
	mpara.outfilepath = param["outfilepath"];
	mpara.filepath_Circle = param["filepath_Circle"];
	mpara.outfilepath_Circle = param["outfilepath_Circle"];
	mpara.rotateMatrixPath = param["rotateMatrixPath"];

	//downsample体素格叶片大小
	istringstream leafsizestr(param["leafsize"]);
	leafsizestr >> mpara.leafsize;

	//fpfh参数
	istringstream kfpfhstr(param["Kfpfh"]), sacSamNumstr(param["sacSamNum"]),
		CorspRnstr(param["CorrespondenceRandomness"]);
	kfpfhstr >> mpara.Kfpfh;
	sacSamNumstr >> mpara.sacSamNum;
	CorspRnstr >> mpara.CorspRn;

	//ICP参数
	istringstream kicpstr(param["Kicp"]), opencorestr(param["opencores"]), maxiIterstr(param["MaximumIterations"]),
		manuIters(param["manulIterations"]),transEpstr(param["TransformationEpsilon"]),maxCorspsDis(param["MaxCorrespondenceDistance"]),
		euclideanEpstr(param["EuclideanFitnessEpsilon"]);
	kicpstr >> mpara.Kicp; opencorestr >> mpara.openmpcores; maxiIterstr >> mpara.MaximumIterations;
	manuIters >> mpara.manulIterations; transEpstr >> mpara.TransformationEpsilon; maxCorspsDis >> mpara.MaxCorrespondenceDistance;
	euclideanEpstr >> mpara.EuclideanFitnessEpsilon;

	//三角化参数-greedy-triangle
	istringstream Ktristr(param["Ktri"]), SearchRadiusstr(param["SearchRadius"]), mustr(param["mu"]), maxneiborsstr(param["maxneibors"]),
		maxsuranglestr(param["maxsurangle"]), minanglestr(param["minangle"]), maxanglestr(param["maxangle"]);
	Ktristr >> mpara.Ktri; SearchRadiusstr >> mpara.SearchRadius; mustr >> mpara.mu; maxneiborsstr >> mpara.maxneibors;
	maxsuranglestr >> mpara.maxsurangle; minanglestr >> mpara.minangle; maxanglestr >> mpara.maxangle;

	//三角面片彩色化
	istringstream Kcolorstr(param["Kcolor"]);
	Kcolorstr >> mpara.Kcolor;

	//干涉参数
	istringstream KneiTri(param["KneiTri"]), neiTriRadiusstr(param["NeiTriRadius"]);
	KneiTri >> mpara.KneiTri;
	neiTriRadiusstr >> mpara.neiTriRadius;
	mpara.norConsis = param["norConsis"];
	mpara.neibormethod = param["neibormethod"];

	//处理三角面片相交情况的t的范围
	istringstream Tthreholdstr(param["Tthrehold"]);
	double Tthrehold;
	Tthreholdstr >> Tthrehold;
}

int main()
{
	cout << "----------Ready----------";
	getchar();
	//计时
	time_t t1 = GetTickCount();

	//获取参数
	mypara para;
	getPara(para);

	//获取点云文件1,2
	PointCloud<PointXYZRGB>::Ptr cloud1(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud2(new PointCloud<PointXYZRGB>);
	if (para.format == ".pcd")
	{
		pcl::io::loadPCDFile(para.filepathcloud1, *cloud1);
		pcl::io::loadPCDFile(para.filepathcloud2, *cloud2);
	}
	else if (para.format == ".ply")
	{
		pcl::io::loadPLYFile(para.filepathcloud1, *cloud1);
		pcl::io::loadPLYFile(para.filepathcloud2, *cloud2);
	}
	else
	{
		cout << "Wrong Format!" << endl;
		return -1;
	}

	//降采样
	time_t t_fpfhstart = GetTickCount();
	PointCloud<PointXYZRGB>::Ptr cloud1_down(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud2_down(new PointCloud<PointXYZRGB>);
	downsampling(cloud1, cloud1_down, para.leafsize);
	downsampling(cloud2, cloud2_down, para.leafsize);

	//计算FPFH算子
	fpfhFeature::Ptr fpfh1 = compute_fpfh_feature(cloud1_down, para.Kfpfh, para.openmpcores);
	fpfhFeature::Ptr fpfh2 = compute_fpfh_feature(cloud2_down, para.Kfpfh, para.openmpcores);

	//粗配准
	cout << "Start FPFH Registeration" << endl;
	Eigen::Matrix4f fpfh_trans;
	sac_Align(cloud1_down, fpfh1, cloud2_down, fpfh2, para, fpfh_trans);
	//粗配准计时
	time_t t_fpfh = GetTickCount();
	cout << "FPFH Registeration use: " << ((t_fpfh - t_fpfhstart)*1.0 / 1000) << "s" << endl;

	//旋转降采样后的点云2
	pcl::transformPointCloud(*cloud2_down, *cloud2_down, fpfh_trans);

	//精配准
	time_t t_icpstart = GetTickCount();
	Eigen::Matrix4f ICP_trans;
	ICPregister(cloud1_down, cloud2_down, para, ICP_trans);
	Eigen::Matrix4f final_trans = ICP_trans * fpfh_trans;

	//匹配时间计时
	time_t t_icp = GetTickCount();
	cout << "ICP Registeration use: " << ((t_icp - t_icpstart)*1.0 / 1000) << "s" << endl;
	cout << "Registration use: " << ((t_icp - t_fpfhstart)*1.0 / 1000) << "s" << endl;
	cout << "--------------------" << endl;

	//三角化,处理的点云数据是点云1以及经过粗配精配后的点云2
	PointCloud<PointXYZRGB>::Ptr cloud2trans(new PointCloud<PointXYZRGB>);
	pcl::transformPointCloud(*cloud2, *cloud2trans, fpfh_trans);
	pcl::transformPointCloud(*cloud2trans, *cloud2trans, ICP_trans);
	//三角化准以前都是对的


	//测试阶段为了让区分明显， 因此点云1网格显示为彩色
	PolygonMesh meshcloud1,meshcloud2;
	trianglulation(cloud1, meshcloud1, para, true);
	trianglulation(cloud2trans, meshcloud2, para);



	//先获取点云1，点云2三角面片对应的中心点云1，2；
	CloudCenVers cloudCenVer1(cloud1, meshcloud1);
	CloudCenVers cloudCenVer2(cloud2trans, meshcloud2);

	//由于将点云2转换到点云1的坐标系下,处理的都是点云2的数据，
	//因此需要计算点云2中的每个三角面片是否和点云1中的三角面片相交
	//因此程序要讲点云2中的每个点添加到点云1中去计算，最后得到点云2中各个三角面片的邻近三角面片
	getNeiborTriangle(cloudCenVer2, cloudCenVer1, para);

	//对点云2处理三角面片相交的情况
	TriCross(cloudCenVer2, cloud2trans, cloudCenVer1, cloud1, para);
	
	//处理平行、相交面片
	PointCloud<PointXYZRGB>::Ptr outcloud(new PointCloud<PointXYZRGB>);
	DealWithTri(cloudCenVer2, cloud2trans, cloud1, para);


	//该店云目的是为了显示
	PolygonMesh meshcloud_down, meshbefore, meshafter;
	PointCloud<PointXYZRGB>::Ptr cloud2_downtrans(new PointCloud<PointXYZRGB>);
	pcl::transformPointCloud(*cloud2_down, *cloud2_downtrans, ICP_trans);
	trianglulation(cloud2, meshbefore, para);
	trianglulation(cloud2_down, meshcloud_down, para);
	trianglulation(cloud2_downtrans, meshafter, para);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer1"));
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("viewer2"));
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("viewer3"));

	viewer->addPointCloud(cloud1, "cloud1");
	viewer->addPointCloud(cloud2, "cloud2");
	viewer->addPolygonMesh(meshcloud1, "mesh1");
	viewer->addPolygonMesh(meshbefore, "mesh2");

	viewer2->addPointCloud(cloud1, "cloud1");
	viewer2->addPointCloud(cloud2trans, "cloud2");
	viewer2->addPolygonMesh(meshcloud1, "mesh1");
	viewer2->addPolygonMesh(meshcloud2, "mesh2");


	while (!viewer->wasStopped() && !viewer2->wasStopped())
	{
		viewer->spinOnce(100);
		viewer2->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	//因此输入为点云1，点云2，点云2的三角面片，
	//输出为与点云1存在相交或者平行情况的三角面片(包括了该三角面片与相交或平行的点云1的三角面片)的vector；
	

}