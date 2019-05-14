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
	PolygonMesh meshcloud1, meshcloud2;
	trianglulation(cloud1, meshcloud1, para, true);
	trianglulation(cloud2trans, meshcloud2, para);


	//先获取点云1，点云2三角面片对应的中心点云1，2；
	CloudCenVers cloudCenVer1(cloud1, meshcloud1);
	CloudCenVers cloudCenVer2(cloud2trans, meshcloud2);

	//由于将点云2转换到点云1的坐标系下,处理的都是点云2的数据，
	//因此需要计算点云2中的每个三角面片是否和点云1中的三角面片相交
	//因此程序要讲点云2中的每个点添加到点云1中去计算，最后得到点云2中各个三角面片的邻近三角面片
	getNeiborTriangle(cloudCenVer2, cloudCenVer1, para);

	//for (int i = 0; i < cloudCenVer2.interAll.at(1000).vertices.vertices.size(); i++)
	//{
	//	cloud2trans->at(cloudCenVer2.interAll.at(1000).vertices.vertices[i]).r = 0;
	//	cloud2trans->at(cloudCenVer2.interAll.at(1000).vertices.vertices[i]).g = 0;
	//	cloud2trans->at(cloudCenVer2.interAll.at(1000).vertices.vertices[i]).b = 255;
	//}
	//for (int i = 0; i < cloudCenVer2.interAll.at(1000).neiborvers.size(); i++)
	//{
	//	for (int j = 0; j < cloudCenVer2.interAll.at(1000).neiborvers.at(i).vertices.size(); j++)
	//	{
	//		cloud1->at(cloudCenVer2.interAll.at(1000).neiborvers.at(i).vertices[j]).r = 255;
	//		cloud1->at(cloudCenVer2.interAll.at(1000).neiborvers.at(i).vertices[j]).g = 0;
	//		cloud1->at(cloudCenVer2.interAll.at(1000).neiborvers.at(i).vertices[j]).b = 0;
	//	}
	//}

	PolygonMesh meshcloud3, meshcloud4;
	trianglulation(cloud1, meshcloud3, para, true);
	trianglulation(cloud2trans, meshcloud4, para);

	//对点云2处理三角面片相交的情况
	TriCross(cloudCenVer2, cloud2trans, cloudCenVer1, cloud1, para);

	//处理平行、相交面片
	PointCloud<PointXYZRGB>::Ptr outcloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cross2cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cross1cloud(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr parallelcloud(new PointCloud<PointXYZRGB>);
	//DealWithTri(cloudCenVer2, cloud2trans, cloud1, outcloud,
	//	cross2cloud, cross1cloud, parallelcloud, para);

	DealCross2cLoud_new(cloudCenVer2, cloud2trans, cloud1, outcloud, cross2cloud, para);
	DealCross1cLoud_new(cloudCenVer2, cloud2trans, cloud1, outcloud, cross1cloud, para);
	DealParalledcLoud_new(cloudCenVer2, cloud2trans, cloud1, outcloud, parallelcloud, para);
	//DealCross2cLoud(cloudCenVer2, cloud2trans, cloud1, outcloud, cross2cloud, para);
	//DealCross1cLoud(cloudCenVer2, cloud2trans, cloud1, outcloud, cross1cloud, para);
	//DealParalledcLoud(cloudCenVer2, cloud2trans, cloud1, outcloud, parallelcloud, para);

	//pcl::io::savePCDFile("C:\\Users\\zhihong\\Desktop\\cup\\2019.1.19\\三角变化后2.pcd",*outcloud);
	


	//双边滤波处理
	PointCloud<PointXYZRGB>::Ptr outbl(new PointCloud<PointXYZRGB>);
	pcl::search::KdTree<PointXYZRGB>::Ptr kdtree1(new pcl::search::KdTree<PointXYZRGB>);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	int k = 30;
	kdtree1->setInputCloud(outcloud);
	//计算每个点法向量
	n.setInputCloud(outcloud);
	//点云法向计算时，需要所搜的近邻点大小
	n.setKSearch(k);
	//开始进行法向计算
	n.compute(*normals);
	int cloudinsize = outcloud->size();
	vector<int>kdpointID(k + 1);
	vector<float>kdpointDis(k + 1);
	for (int i = 0; i < cloudinsize; i++) {//cloudin->size()
		pcl::PointCloud<PointXYZRGB>::Ptr tempkdcloud(new pcl::PointCloud<PointXYZRGB>);
		pcl::PointCloud<pcl::Normal>::Ptr tempnormals(new pcl::PointCloud<pcl::Normal>);

		if (kdtree1->nearestKSearch(outcloud->at(i), k + 1, kdpointID, kdpointDis)) {
			//tempcloud中第一个点为计算点,tempnormals第一个为计算点法向量
			int cc = i;
			tempkdcloud->push_back(outcloud->at(i));
			tempnormals->push_back(normals->at(i));
			/*	cloudin->at(i).r = 0;
				cloudin->at(i).g = 0;
				cloudin->at(i).b = 255;*/
			for (int j = 1; j <= k; j++)
			{
				//检查领域点
				tempkdcloud->push_back(outcloud->at(kdpointID[j]));
				tempnormals->push_back(normals->at(kdpointID[j]));
			}
			PointXYZRGB tempnew_point;
			tempnew_point = Bilateral_Filters(tempkdcloud, tempnormals, kdpointDis, 100, 0.1);
			outbl->push_back(tempnew_point);
		}
	}



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
	viewer->setBackgroundColor(0.5, 0.5, 0.5);
	viewer2->setBackgroundColor(0.5, 0.5, 0.5);
	viewer3->setBackgroundColor(0.7, 0.7, 0.7);

	viewer->addPointCloud(cloud1, "cloud1");
	viewer->addPolygonMesh(meshcloud1, "mesh1");
	viewer->addPointCloud(cloud2, "cloud2");
	viewer->addPolygonMesh(meshbefore, "mesh2");

	viewer2->addPointCloud(cloud1, "cloud1");
	viewer2->addPolygonMesh(meshcloud3, "mesh1");
	viewer2->addPointCloud(cloud2trans, "cloud2");
	viewer2->addPolygonMesh(meshcloud4, "mesh2");


	//PolygonMesh outmesh;
	/*PointCloud<PointXYZRGB>::Ptr cloudn(new PointCloud<PointXYZRGB>);
	pcl::copyPointCloud(*cloud1, *cloudn);*/

	PolygonMesh outmesh;
	//viewer3->addPointCloud(cloud1, "cloud1");
	viewer3->addPolygonMesh(meshcloud1, "mesh1");

	//trianglulation(outcloud, outmesh, para);
	//viewer3->addPointCloud(cloud2trans, "cloud2");
	/*trianglulation(outcloud, outmesh, para);
	viewer3->addPolygonMesh(outmesh, "mesh2");*/
	//viewer3->addPolygonMesh(outmesh, "mesh2");
	/*for (int i = 0; i < outcloud->size(); i++)
	{
		outcloud->at(i).r = 255;
		outcloud->at(i).g = 0;
		outcloud->at(i).b = 0;
	}*/
	//看双边滤波后的结果
	PointCloud<PointXYZRGB>::Ptr outbl1_2(new PointCloud<PointXYZRGB>);
	downinone(outbl, outbl1_2, kdtree1);
	outcloud->clear();
	pcl::copyPointCloud(*outbl1_2, *outcloud);


	PointCloud<PointXYZRGB>::Ptr cloudout_down(new PointCloud<PointXYZRGB>);
	double outleafsize = para.leafsize;
	//viewer3->addPointCloud(outcloud, "cloud2trans");
	downsampling(outcloud, cloudout_down, outleafsize);

	//双边滤波处理处理降采样后的点云
	PointCloud<PointXYZRGB>::Ptr outbl2(new PointCloud<PointXYZRGB>);
	pcl::search::KdTree<PointXYZRGB>::Ptr kdtree2(new pcl::search::KdTree<PointXYZRGB>);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n2;
	pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
	int k2 = 10;
	kdtree2->setInputCloud(cloudout_down);
	//计算每个点法向量
	n2.setInputCloud(cloudout_down);
	//点云法向计算时，需要所搜的近邻点大小
	n2.setKSearch(k2);
	//开始进行法向计算
	n2.compute(*normals2);
	int cloudinsize2 = cloudout_down->size();
	vector<int>kdpointID2(k2 + 1);
	vector<float>kdpointDis2(k2 + 1);
	for (int i = 0; i < cloudinsize2; i++) {//cloudin->size()
		pcl::PointCloud<PointXYZRGB>::Ptr tempkdcloud(new pcl::PointCloud<PointXYZRGB>);
		pcl::PointCloud<pcl::Normal>::Ptr tempnormals(new pcl::PointCloud<pcl::Normal>);

		if (kdtree2->nearestKSearch(cloudout_down->at(i), k2 + 1, kdpointID2, kdpointDis2)) {
			//tempcloud中第一个点为计算点,tempnormals第一个为计算点法向量
			int cc = i;
			tempkdcloud->push_back(cloudout_down->at(i));
			tempnormals->push_back(normals2->at(i));
			/*	cloudin->at(i).r = 0;
				cloudin->at(i).g = 0;
				cloudin->at(i).b = 255;*/
			for (int j = 1; j <= k2; j++)
			{
				//检查领域点
				tempkdcloud->push_back(cloudout_down->at(kdpointID2[j]));
				tempnormals->push_back(normals2->at(kdpointID2[j]));
			}
			PointXYZRGB tempnew_point;
			tempnew_point = Bilateral_Filters(tempkdcloud, tempnormals, kdpointDis2, 100, 0.1);
			outbl2->push_back(tempnew_point);
		}
	}
	cloudout_down->clear();
	pcl::copyPointCloud(*outbl2, *cloudout_down);

	viewer3->addPointCloud(cloudout_down, "cloud");
	trianglulation(cloudout_down, outmesh, para);
	viewer3->addPolygonMesh(outmesh, "mesh2");

	cout << outcloud->size() << endl;
	cout << cloudout_down->size() << endl;
	/*cout << cross2cloud->size() << endl;
	cout << cross1cloud->size() << endl;
	cout << parallelcloud->size() << endl;*/
	/*viewer3->addPolygonMesh(outmesh, "mesh");*/

	while (!viewer->wasStopped() && !viewer2->wasStopped() /*&& !viewer3->wasStopped()*/)
	{
		viewer->spinOnce(100);
		viewer2->spinOnce(100);
		viewer3->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	// pcl::io::savePCDFile("C:\\Users\\zhihong\\Desktop\\cup\\2019.2.25\\ceshi2.pcd",*outcloud);
	pcl::io::savePCDFile("G:\\oilrape\\2017_OilRape\\2018.1.3\\out2019.4.16\\ceju\\33new.pcd", *cloudout_down);
	//因此输入为点云1，点云2，点云2的三角面片，
	//输出为与点云1存在相交或者平行情况的三角面片(包括了该三角面片与相交或平行的点云1的三角面片)的vector；

}