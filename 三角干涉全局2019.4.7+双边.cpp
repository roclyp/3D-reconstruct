#include "PCLlibrary.h"
#include "getfile.h"
//#include "obbBox.h"
#include "MyClassType.h"
#include "DealwithPointCloud.h"
#include "Ray_Triangulation.h"
using namespace std;

#define Pi 3.141592657;
//PointCloud<PointXYZRGB>::Ptr outcloud_whole(new PointCloud<PointXYZRGB>);
//PointCloud<PointXYZRGB>::Ptr outcloud_wholedown(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr outcloud_whole(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr outcloud_wholedown(new PointCloud<PointXYZRGB>);

//PointCloud<PointXYZRGB>::Ptr outcloud(new PointCloud<PointXYZRGB>);
//PointCloud<PointXYZRGB>::Ptr cross2cloud(new PointCloud<PointXYZRGB>);
//PointCloud<PointXYZRGB>::Ptr cross1cloud(new PointCloud<PointXYZRGB>);
//PointCloud<PointXYZRGB>::Ptr parallelcloud(new PointCloud<PointXYZRGB>);
//PointCloud<PointXYZRGB>::Ptr cloudout_down(new PointCloud<PointXYZRGB>);

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

void deletePoint(const CloudCenVers &cloudCenVer, const PointCloud<PointXYZRGB>::Ptr &cloudin, PointCloud<PointXYZRGB>::Ptr &cloudout)
{
	vector<int> deletenum;
	for (int i = 0; i < cloudCenVer.interAll.size(); i++)
	{
		auto tempver1 = cloudCenVer.interAll.at(i).Cross2itervers;
		auto tempver2 = cloudCenVer.interAll.at(i).Cross1itervers;
		auto tempver3 = cloudCenVer.interAll.at(i).Parallitervers;
		if (tempver1.size() != 0)
		{
			int id1 = tempver1.at(0).vertices[0];
			int id2 = tempver1.at(0).vertices[1];
			int id3 = tempver1.at(0).vertices[2];
			deletenum.push_back(id1); deletenum.push_back(id2); deletenum.push_back(id3);
		}
		if (tempver2.size() != 0)
		{
			int id4 = tempver2.at(0).vertices[0];
			int id5 = tempver2.at(0).vertices[1];
			int id6 = tempver2.at(0).vertices[2];
			deletenum.push_back(id4); deletenum.push_back(id5); deletenum.push_back(id6);
		}
		if (tempver3.size() != 0)
		{
			int id7 = tempver3.at(0).vertices[0];
			int id8 = tempver3.at(0).vertices[1];
			int id9 = tempver3.at(0).vertices[2];
			deletenum.push_back(id7); deletenum.push_back(id8); deletenum.push_back(id9);
		}
		sort(deletenum.begin(), deletenum.end(), greater<int>());
		deletenum.erase(unique(deletenum.begin(), deletenum.end()), deletenum.end());
	}
	for (int i = 0; i < cloudin->size(); i++)
	{
		vector<int>::iterator hasnum = find(deletenum.begin(), deletenum.end(), i);
		if (hasnum == deletenum.end())
		{
			cloudout->push_back(cloudin->at(i));
		}
	}
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

int main()
{
	cout << "----------Ready----------";
	getchar();
	//计时
	time_t t1 = GetTickCount();

	//获取参数
	mypara para;
	getPara(para);

	//循环版本
	vector<string> Allname;
	vector<pcl::PointCloud<PointXYZRGB>::Ptr> AllCloud;
	GetAllFiles_CertainFormat(para.filepath_Circle, Allname, para.format);

	vector<std::shared_ptr<open3d::PointCloud>> cloud3dall;

	cout << "File numbers: " << Allname.size() << endl;

	for (int i = 0; i < Allname.size(); i++)
	{
		pcl::PointCloud<PointXYZRGB>::Ptr cloudtemp(new pcl::PointCloud<PointXYZRGB>);
		pcl::io::loadPCDFile(Allname.at(i), *cloudtemp);
		AllCloud.push_back(cloudtemp);
		//open3d点云
		std::shared_ptr<open3d::PointCloud> cloudopen3d(new open3d::PointCloud);
		open3d::ReadPointCloud(Allname.at(i), *cloudopen3d, "pcd");
		cloud3dall.push_back(cloudopen3d);
	}

	//PointCloud<PointXYZRGB>::Ptr outcloud_whole(new PointCloud<PointXYZRGB>);
	//PointCloud<PointXYZRGB>::Ptr outcloud_wholedown(new PointCloud<PointXYZRGB>);
	int inicloudid = para.circleBegin;
	int endcloudid = para.circleEnd;
	int iterationnum = para.circleStep;
	*outcloud_whole += *AllCloud.at(inicloudid);
	*outcloud_wholedown += *AllCloud.at(inicloudid);
	//初始化全局旋转矩阵
	Eigen::Matrix4f global_transform = global_transform.Identity();

	for (int i = inicloudid; i < endcloudid - 1; i = i + iterationnum)
	{
		cout << "-------------------------------------------------------------------" << endl;
		//获取点云文件1,2
		cout << "Frame: " << i << " and " << "Frame: " << i + iterationnum << " is Dealing" << endl;
		//获取点云文件1,2
		PointCloud<PointXYZRGB>::Ptr cloud1(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr cloud2(new PointCloud<PointXYZRGB>);
		pcl::copyPointCloud(*AllCloud.at(i), *cloud1);
		pcl::copyPointCloud(*AllCloud.at(i + iterationnum), *cloud2);
		/*	pcl::transformPointCloud(*cloud1, *cloud1, global_transform);
			pcl::transformPointCloud(*cloud2, *cloud2, global_transform);*/

			//open3d点云赋值
		std::shared_ptr<open3d::PointCloud> cloud3d1(new open3d::PointCloud);
		std::shared_ptr<open3d::PointCloud> cloud3d2(new open3d::PointCloud);
		cloud3d1 = cloud3dall.at(i);
		cloud3d2 = cloud3dall.at(i + iterationnum);

		//粗配准计时
		cout << "Start FPFH Registeration" << endl;

		//降采样
		time_t t_fpfhstart = GetTickCount();
		PointCloud<PointXYZRGB>::Ptr cloud1_down(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr cloud2_downbefore(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr cloud2_down(new PointCloud<PointXYZRGB>);
		downsampling(cloud1, cloud1_down, para.leafsize);
		downsampling(cloud2, cloud2_downbefore, para.leafsize);

		////计算FPFH算子
		//fpfhFeature::Ptr fpfh1 = compute_fpfh_feature(cloud1_down, para.Kfpfh, para.openmpcores);
		//fpfhFeature::Ptr fpfh2 = compute_fpfh_feature(cloud2_down, para.Kfpfh, para.openmpcores);

		////粗配准
		//cout << "Start FPFH Registeration" << endl;
		//Eigen::Matrix4f fpfh_trans;
		//sac_Align(cloud1_down, fpfh1, cloud2_down, fpfh2, para, fpfh_trans);

		open3d::RegistrationResult fpfhresult;
		Eigen::Matrix4f fpfh_trans;
		/*FPFHRegister_open3d(cloud3d1, cloud3d2, fpfh_trans, fpfhresult, para.open3dleafsize,
			para.open3dKdradius, para.open3dKdnum, para.FastGlobalRegistropt);*/
		std::shared_ptr<open3d::PointCloud> cloud1_down_nor(new open3d::PointCloud);
		std::shared_ptr<open3d::PointCloud> cloud2_down_nor(new open3d::PointCloud);
		FPFHRegister_open3d(cloud3d1, cloud3d2, fpfh_trans,
			cloud1_down_nor, cloud2_down_nor, fpfhresult, para.open3dleafsize,
			para.open3dKdradius, para.open3dKdnum, para.FastGlobalRegistropt);

		// fpfh结束
		time_t t_fpfh = GetTickCount();
		cout << "FPFH Registeration use: " << ((t_fpfh - t_fpfhstart)*1.0 / 1000) << "s" << endl;

		////旋转降采样后的点云2
		//pcl::transformPointCloud(*cloud2_downbefore, *cloud2_down, fpfh_trans);
		////精配准
		//time_t t_icpstart = GetTickCount();
		//Eigen::Matrix4f ICP_trans;
		//ICPregister(cloud1_down, cloud2_down, para, ICP_trans);
		//Eigen::Matrix4f final_trans = final_trans.Identity();
		//final_trans = final_trans * ICP_trans * fpfh_trans;

		//open3d精配准
		time_t t_icpstart = GetTickCount();
		open3d::RegistrationResult ICPresult;
		ICPresult = open3d::RegistrationICP(*cloud2_down_nor, *cloud1_down_nor, 0.002, fpfhresult.transformation_,
			open3d::TransformationEstimationPointToPlane(),
			open3d::ICPConvergenceCriteria(1e-6, 1e-6, 1000));
		Eigen::Matrix4f ICP_trans1 = ICPresult.transformation_.cast<float>();
		Eigen::Matrix4f final_trans = final_trans.Identity();
		final_trans = final_trans * ICP_trans1;

		//匹配时间计时
		time_t t_icp = GetTickCount();
		cout << "ICP Registeration use: " << ((t_icp - t_icpstart)*1.0 / 1000) << "s" << endl;
		cout << "Registration use: " << ((t_icp - t_fpfhstart)*1.0 / 1000) << "s" << endl;
		//cout << "--------------------" << endl;

		//三角化,处理的点云数据是点云1以及经过粗配精配后的点云2
		PointCloud<PointXYZRGB>::Ptr cloud2trans(new PointCloud<PointXYZRGB>);
		/*pcl::transformPointCloud(*cloud2, *cloud2trans, fpfh_trans);
		pcl::transformPointCloud(*cloud2trans, *cloud2trans, ICP_trans);*/
		pcl::transformPointCloud(*cloud2, *cloud2trans, final_trans);
		pcl::transformPointCloud(*cloud2trans, *cloud2trans, global_transform);
		global_transform = global_transform * final_trans;
		//三角化以前都是对的



		//测试阶段为了让区分明显， 因此点云1网格显示为彩色
		PolygonMesh meshcloud1, meshcloud2;
		trianglulation(cloud1, meshcloud1, para);
		trianglulation(cloud2trans, meshcloud2, para);


		//先获取点云1，点云2三角面片对应的中心点云1，2；
		CloudCenVers cloudCenVer1(cloud1, meshcloud1);
		CloudCenVers cloudCenVer2(cloud2trans, meshcloud2);

		//由于将点云2转换到点云1的坐标系下,处理的都是点云2的数据，
		//因此需要计算点云2中的每个三角面片是否和点云1中的三角面片相交
		//因此程序要讲点云2中的每个点添加到点云1中去计算，最后得到点云2中各个三角面片的邻近三角面片
		getNeiborTriangle(cloudCenVer2, cloudCenVer1, para);

		time_t t_tri = GetTickCount();
		cout << "Trianglulation use: " << ((t_tri - t_icp)*1.0 / 1000) << "s" << endl;

		//对点云2处理三角面片相交的情况
		TriCross(cloudCenVer2, cloud2trans, cloudCenVer1, cloud1, para);

		//处理平行、相交面片
		PointCloud<PointXYZRGB>::Ptr outcloud(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr cross2cloud(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr cross1cloud(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr parallelcloud(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr cloudout_down(new PointCloud<PointXYZRGB>);

		DealCross2cLoud_new(cloudCenVer2, cloud2trans, cloud1, outcloud, cross2cloud, para);
		DealCross1cLoud_new(cloudCenVer2, cloud2trans, cloud1, outcloud, cross1cloud, para);
		//DealParalledcLoud_new(cloudCenVer2, cloud2trans, cloud1, outcloud, parallelcloud, para);

		////删除旧点添加新点
		PointCloud<PointXYZRGB>::Ptr cloud2transDelete(new PointCloud<PointXYZRGB>);
		deletePoint(cloudCenVer2, cloud2trans, cloud2transDelete);
		*outcloud += *cloud2transDelete;

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
		outcloud->clear();
		pcl::copyPointCloud(*outbl, *outcloud);

		double outleafsize = para.leafsize*1.5;
		downsampling(outcloud, cloudout_down, outleafsize);
		while (cloudout_down->size() > 12000)
		{
			outleafsize = outleafsize * 1.05;
			downsampling(cloudout_down, cloudout_down, outleafsize);
		}
		*outcloud_whole += *outcloud;
		*outcloud_wholedown += *cloudout_down;

		/*	PointCloud<PointXYZRGB>::Ptr downinverse(new PointCloud<PointXYZRGB>);
			Eigen::Matrix4f globalinvers = global_transform.inverse();
			pcl::transformPointCloud(*cloudout_down, *downinverse, globalinvers);*/

			/*AllCloud.at(i + iterationnum)->clear();
			*AllCloud.at(i + iterationnum) += *downinverse;*/

		cout << "All frame size:" << outcloud_whole->size() << endl;
		cout << "All frame down size:" << outcloud_wholedown->size() << endl;
		time_t crosstime = GetTickCount();
		cout << "Triangulation dealing use: " << ((crosstime - t_tri)*1.0 / 1000) << "s" << endl;

	}

	cout << "-------------------------------------------------------------------" << endl;
	time_t finaltime = GetTickCount();
	cout << "Final use time: " << ((finaltime - t1)*1.0 / 1000) << "s" << endl;

	stringstream outname_whole, outname_wholedown;
	outname_whole << para.outfilepath_Circle << "\\" << para.outname_Circle << ".pcd";
	outname_wholedown << para.outfilepath_Circle << "\\" << para.outname_Circle << "_down.pcd";

	cout << outname_whole.str() << endl;
	cout << outname_wholedown.str() << endl;
	pcl::io::savePCDFileBinary(outname_whole.str(), *outcloud_whole);
	pcl::io::savePCDFileBinary(outname_wholedown.str(), *outcloud_wholedown);
	//因此输入为点云1，点云2，点云2的三角面片，
	//输出为与点云1存在相交或者平行情况的三角面片(包括了该三角面片与相交或平行的点云1的三角面片)的vector；

}