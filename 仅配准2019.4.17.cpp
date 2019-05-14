#include "PCLlibrary.h"
#include "getfile.h"
//#include "obbBox.h"
#include "MyClassType.h"
#include "DealwithPointCloud.h"
#include "Ray_Triangulation.h"
using namespace std;

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
	mpara.outname_Circle = param["outname_Circle"];
	mpara.rotateMatrixPath = param["rotateMatrixPath"];

	//迭代参数
	istringstream  circleBeginstr(param["circleBegin"]),
		circleEndstr(param["circleEnd"]),
		circleStepstr(param["circleStep"]);
	circleBeginstr >> mpara.circleBegin;
	circleEndstr >> mpara.circleEnd;
	circleStepstr >> mpara.circleStep;

	//open3d粗配准参数
	istringstream open3dleafsizestr(param["open3dleafsize"]),
		open3dKdradiusstr(param["open3dKdradius"]),
		open3dKdnumstr(param["open3dKdnum"]),
		FastGlobalRegistroptstr(param["FastGlobalRegistrationOption"]);
	open3dleafsizestr >> mpara.open3dleafsize;
	open3dKdradiusstr >> mpara.open3dKdradius;
	open3dKdnumstr >> mpara.open3dKdnum;
	FastGlobalRegistroptstr >> mpara.FastGlobalRegistropt;

	//downsample体素格叶片大小
	istringstream leafsizestr(param["leafsize"]);
	leafsizestr >> mpara.leafsize;

	//fpfh参数
	istringstream kfpfhstr(param["Kfpfh"]), sacSamNumstr(param["sacSamNum"]),
		CorspRnstr(param["CorrespondenceRandomness"]);
	kfpfhstr >> mpara.Kfpfh;
	sacSamNumstr >> mpara.sacSamNum;
	CorspRnstr >> mpara.CorspRn;

	// ICP参数
	istringstream kicpstr(param["Kicp"]), opencorestr(param["opencores"]), maxiIterstr(param["MaximumIterations"]),
		manuIters(param["manulIterations"]), transEpstr(param["TransformationEpsilon"]), maxCorspsDis(param["MaxCorrespondenceDistance"]),
		euclideanEpstr(param["EuclideanFitnessEpsilon"]);
	kicpstr >> mpara.Kicp; opencorestr >> mpara.openmpcores; maxiIterstr >> mpara.MaximumIterations;
	manuIters >> mpara.manulIterations; transEpstr >> mpara.TransformationEpsilon; maxCorspsDis >> mpara.MaxCorrespondenceDistance;
	euclideanEpstr >> mpara.EuclideanFitnessEpsilon;

	// 三角化参数-greedy-triangle
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
	istringstream Tthreholdstr(param["Tthrehold"]), dealnumstr(param["dealnum"]);
	Tthreholdstr >> mpara.Tthrehold;
	dealnumstr >> mpara.dealnum;
}


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

void downinone(pcl::PointCloud<PointXYZRGB>::Ptr &in, pcl::PointCloud<PointXYZRGB>::Ptr &out,
	pcl::search::KdTree<PointXYZRGB>::Ptr &kdtree1)
{
	int cloudinsize = in->size();
	vector<int>kdpointID;
	vector<float>kdpointDis;
	for (int i = 0; i < cloudinsize; i++) {//cloudin->size()
		pcl::PointCloud<pcl::Normal>::Ptr tempnormals(new pcl::PointCloud<pcl::Normal>);

		if (kdtree1->radiusSearch(in->at(i), 0.001, kdpointID, kdpointDis)) {
			//tempcloud中第一个点为计算点,tempnormals第一个为计算点法向量
			double tx = 0, ty = 0, tz = 0, tr = 0, tg = 0, tb = 0;
			for (int j = 0; j < kdpointID.size(); j++)
			{
				tx += in->at(kdpointID.at(j)).x;
				ty += in->at(kdpointID.at(j)).y;
				tz += in->at(kdpointID.at(j)).z;
				tr += in->at(kdpointID.at(j)).r;
				tg += in->at(kdpointID.at(j)).g;
				tb += in->at(kdpointID.at(j)).b;
			}
			PointXYZRGB tp;
			tp.x = tx * 1.0 / kdpointID.size();
			tp.y = ty * 1.0 / kdpointID.size();
			tp.z = tz * 1.0 / kdpointID.size();
			tp.r = tr * 1.0 / kdpointID.size();
			tp.g = tg * 1.0 / kdpointID.size();
			tp.b = tb * 1.0 / kdpointID.size();
			out->push_back(tp);
		}
	}
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
	cout << para.filepathcloud1 << endl;
	cout << para.filepathcloud2 << endl;
	cout << "-----------------------------------" << endl;
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

	//open3d点云赋值
	std::shared_ptr<open3d::PointCloud> cloud3d1(new open3d::PointCloud);
	std::shared_ptr<open3d::PointCloud> cloud3d2(new open3d::PointCloud);
	open3d::ReadPointCloud(para.filepathcloud1, *cloud3d1, "pcd");
	open3d::ReadPointCloud(para.filepathcloud2, *cloud3d2, "pcd");

	//降采样
	cout << "Start FPFH Registeration" << endl;
	time_t t_fpfhstart = GetTickCount();
	PointCloud<PointXYZRGB>::Ptr cloud1_down(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud2_down(new PointCloud<PointXYZRGB>);
	downsampling(cloud1, cloud1_down, para.leafsize);
	downsampling(cloud2, cloud2_down, para.leafsize);

	/*Eigen::Matrix4f fpfh_trans;
	FPFHRegister_open3d(cloud3d1, cloud3d2, fpfh_trans,
		para.open3dleafsize, para.open3dKdradius, para.open3dKdnum);*/

	open3d::RegistrationResult fpfhresult;
	Eigen::Matrix4f fpfh_trans;
	/*FPFHRegister_open3d(cloud3d1, cloud3d2, fpfh_trans, fpfhresult, para.open3dleafsize,
		para.open3dKdradius, para.open3dKdnum, para.FastGlobalRegistropt);*/
	std::shared_ptr<open3d::PointCloud> cloud1_down_nor(new open3d::PointCloud);
	std::shared_ptr<open3d::PointCloud> cloud2_down_nor(new open3d::PointCloud);
	FPFHRegister_open3d(cloud3d1, cloud3d2, fpfh_trans,
		cloud1_down_nor, cloud2_down_nor, fpfhresult, para.open3dleafsize,
		para.open3dKdradius, para.open3dKdnum, para.FastGlobalRegistropt);

	//粗配准计时
	time_t t_fpfh = GetTickCount();
	cout << "FPFH Registeration use: " << ((t_fpfh - t_fpfhstart)*1.0 / 1000) << "s" << endl;

	//旋转降采样后的点云2
	pcl::transformPointCloud(*cloud2_down, *cloud2_down, fpfh_trans);

	//精配准
	time_t t_icpstart = GetTickCount();
	Eigen::Matrix4f ICP_trans;
	ICPregister(cloud1_down, cloud2_down, para, ICP_trans);
	Eigen::Matrix4f final_trans = final_trans.Identity();
	final_trans = final_trans * fpfh_trans * ICP_trans;

	//匹配时间计时
	time_t t_icp = GetTickCount();
	cout << "ICP Registeration use: " << ((t_icp - t_icpstart)*1.0 / 1000) << "s" << endl;
	cout << "Registration use: " << ((t_icp - t_fpfhstart)*1.0 / 1000) << "s" << endl;
	cout << "--------------------" << endl;

	//三角化,处理的点云数据是点云1以及经过粗配精配后的点云2
	PointCloud<PointXYZRGB>::Ptr cloud2trans(new PointCloud<PointXYZRGB>);
	pcl::transformPointCloud(*cloud2, *cloud2trans, fpfh_trans);
	pcl::transformPointCloud(*cloud2trans, *cloud2trans, ICP_trans);
	//三角化以前都是对的


	//测试阶段为了让区分明显， 因此点云1网格显示为彩色
	PolygonMesh meshcloud1, meshcloud2, meshcloud3;
	trianglulation(cloud1, meshcloud1, para, true);
	trianglulation(cloud2, meshcloud2, para);
	trianglulation(cloud2trans, meshcloud3, para);


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer1"));
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("viewer2"));
	viewer->setBackgroundColor(0.5, 0.5, 0.5);
	viewer2->setBackgroundColor(0.9, 0.9, 0.9);

	viewer->addPointCloud(cloud1, "cloud1");
	viewer->addPolygonMesh(meshcloud1, "mesh1");
	viewer->addPointCloud(cloud2, "cloud2");
	viewer->addPolygonMesh(meshcloud2, "mesh2");

	viewer2->addPointCloud(cloud1, "cloud1");
	viewer2->addPolygonMesh(meshcloud1, "mesh1");
	viewer2->addPointCloud(cloud2trans, "cloud2");
	viewer2->addPolygonMesh(meshcloud3, "mesh2");

	cout << cloud2trans->size() << endl;
	/*cout << cross2cloud->size() << endl;
	cout << cross1cloud->size() << endl;
	cout << parallelcloud->size() << endl;*/
	/*viewer3->addPolygonMesh(outmesh, "mesh");*/

	while (!viewer->wasStopped() && !viewer2->wasStopped())
	{
		viewer->spinOnce(100);
		viewer2->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	// pcl::io::savePCDFile("C:\\Users\\zhihong\\Desktop\\cup\\2019.2.25\\ceshi2.pcd",*outcloud);
	pcl::io::savePCDFile("G:\\oilrape\\2017_OilRape\\2018.1.3\\out2019.4.16\\ceju\\old4.pcd", *cloud2trans);
	//因此输入为点云1，点云2，点云2的三角面片，
	//输出为与点云1存在相交或者平行情况的三角面片(包括了该三角面片与相交或平行的点云1的三角面片)的vector；

}