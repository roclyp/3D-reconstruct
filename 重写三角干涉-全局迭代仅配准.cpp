#include "PCLlibrary.h"
#include "getfile.h"
//#include "obbBox.h"
#include "MyClassType.h"

using namespace std;

#define Pi 3.141592657;
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

	
	for (int i = inicloudid; i < endcloudid; i = i + iterationnum)
	{
		cout << "------------------------------------------------------" << endl;
		//获取点云文件1,2
		cout << "Frame: " << i << " and " << "Frame: " << i + iterationnum << " is registering" << endl;
		time_t sin_start = GetTickCount();
		PointCloud<PointXYZRGB>::Ptr cloud1(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr cloud2(new PointCloud<PointXYZRGB>);
		pcl::copyPointCloud(*AllCloud.at(i), *cloud1);
		pcl::copyPointCloud(*AllCloud.at(i + iterationnum), *cloud2);
		/*	pcl::transformPointCloud(*AllCloud.at(i), *cloud1, global_transform);
			pcl::transformPointCloud(*AllCloud.at(i+1), *cloud2, global_transform);	*/

			//open3d点云赋值
		std::shared_ptr<open3d::PointCloud> cloud3d1(new open3d::PointCloud);
		std::shared_ptr<open3d::PointCloud> cloud3d2(new open3d::PointCloud);
		cloud3d1 = cloud3dall.at(i);
		cloud3d2 = cloud3dall.at(i + iterationnum);

		//粗配准计时
		//cout << "Start FPFH Registeration" << endl;
		//降采样
		//time_t t_fpfhstart = GetTickCount();
		PointCloud<PointXYZRGB>::Ptr cloud1_down(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr cloud2_downbefore(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr cloud2_down(new PointCloud<PointXYZRGB>);
		downsampling(cloud1, cloud1_down, para.leafsize);
		downsampling(cloud2, cloud2_downbefore, para.leafsize);

		//SAC方法
		////计算FPFH算子
		//fpfhFeature::Ptr fpfh1 = compute_fpfh_feature(cloud1_down, para.Kfpfh, para.openmpcores);
		//fpfhFeature::Ptr fpfh2 = compute_fpfh_feature(cloud2_downbefore, para.Kfpfh, para.openmpcores);

		//////粗配准
		////cout << "Start FPFH Registeration" << endl;
		//Eigen::Matrix4f fpfh_trans;
		//sac_Align(cloud1_down, fpfh1, cloud2_downbefore, fpfh2, para, fpfh_trans);

		open3d::RegistrationResult fpfhresult;
		Eigen::Matrix4f fpfh_trans;
		FPFHRegister_open3d(cloud3d1, cloud3d2, fpfh_trans, fpfhresult, para.open3dleafsize,
			para.open3dKdradius, para.open3dKdnum, para.FastGlobalRegistropt);
		
		/*open3d::RegistrationResult ICPresult;
		ICPresult = open3d::RegistrationICP(*cloud3d2, *cloud3d1, 0.002, fpfhresult.transformation_,
			open3d::TransformationEstimationPointToPlane(),
			open3d::ICPConvergenceCriteria(1e-6, 1e-6, 1000));

		Eigen::Matrix4f ICP_trans1 = ICPresult.transformation_.cast<float>();*/
		// fpfh结束
		//time_t t_fpfh = GetTickCount();
		//cout << "FPFH Registeration use: " << ((t_fpfh - t_fpfhstart)*1.0 / 1000) << "s" << endl;

		//旋转降采样后的点云2
		pcl::transformPointCloud(*cloud2_downbefore, *cloud2_down, fpfh_trans);

		//精配准
		//time_t t_icpstart = GetTickCount();
		Eigen::Matrix4f ICP_trans;
		ICPregister(cloud1_down, cloud2_down, para, ICP_trans);
		Eigen::Matrix4f final_trans = final_trans.Identity();
		final_trans = final_trans * ICP_trans * fpfh_trans;

		////匹配时间计时
		//time_t t_icp = GetTickCount();
		//cout << "ICP Registeration use: " << ((t_icp - t_icpstart)*1.0 / 1000) << "s" << endl;
		//cout << "Registration use: " << ((t_icp - t_fpfhstart)*1.0 / 1000) << "s" << endl;
		//cout << "--------------------" << endl;

		//三角化,处理的点云数据是点云1以及经过粗配精配后的点云2
		PointCloud<PointXYZRGB>::Ptr cloud2trans(new PointCloud<PointXYZRGB>);
		pcl::transformPointCloud(*cloud2, *cloud2trans, fpfh_trans);
		pcl::transformPointCloud(*cloud2trans, *cloud2trans, ICP_trans);
		pcl::transformPointCloud(*cloud2trans, *cloud2trans, global_transform);
		global_transform = final_trans * global_transform;
		//三角化以前都是对的
		*outcloud_whole += *cloud2trans;

		time_t t_single = GetTickCount();
		cout << "Single frame usetime:" << ((t_single - sin_start)*1.0 / 1000) << "s" << endl;
		//cout << "All frame size:" << outcloud_whole->size() << endl;
		//cout << "------------------------------------------------------" << endl;
	}
	cout << "------------------------------------------------------" << endl;
	cout << "All frame size:" << outcloud_whole->size() << endl;
	time_t finaltime = GetTickCount();
	cout << "Final use time: " << ((finaltime - t1)*1.0 / 1000) << "s" << endl;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->addPointCloud(outcloud_whole, "p1");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	pcl::io::savePCDFileBinary("C:\\Users\\zhihong\\Desktop\\cup\\2019.2.25\\peizhun\\peizhun5.pcd", *outcloud_whole);
	//因此输入为点云1，点云2，点云2的三角面片，
	//输出为与点云1存在相交或者平行情况的三角面片(包括了该三角面片与相交或平行的点云1的三角面片)的vector；

}