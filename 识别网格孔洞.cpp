#include "PCLlibrary.h"
#include "getfile.h"
//#include "obbBox.h"
#include "MyClassType.h"
#include "DealwithPointCloud.h"
#include "Ray_Triangulation.h"
#include "fillholes.h"
using namespace std;

using namespace std;

#define Pi 3.141592657;

//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//读取配置文件

//获取配置文件
void ConfigFileRead(map<string, string>& m_mapConfigInfo)
{
	ifstream configFile;
	string path = "./识别网格孔洞.ini";
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

void deleteUnnormal(vector<pcl::Vertices>& vers, PointCloud<PointXYZRGB>::Ptr &cloud)
{
	vector<pcl::Vertices> temp;
	for (int i = 0; i < vers.size(); i++)
	{
		int id1 = vers.at(i).vertices[0];
		int id2 = vers.at(i).vertices[1];
		int id3 = vers.at(i).vertices[2];
		if (id1 == id2 || id1 == id3 || id3 == id2)
			continue;
		PointXYZRGB p1 = cloud->at(id1);
		PointXYZRGB p2 = cloud->at(id2);
		PointXYZRGB p3 = cloud->at(id3);
		double d1 = sqrt((p1.x - p2.x)*(p1.x - p2.x) +
			(p1.y - p2.y)*(p1.y - p2.y) +
			(p1.z - p2.z)*(p1.z - p2.z));
		double d2 = sqrt((p1.x - p3.x)*(p1.x - p3.x) +
			(p1.y - p3.y)*(p1.y - p3.y) +
			(p1.z - p3.z)*(p1.z - p3.z));
		double d3 = sqrt((p3.x - p2.x)*(p3.x - p2.x) +
			(p3.y - p2.y)*(p3.y - p2.y) +
			(p3.z - p2.z)*(p3.z - p2.z));
		if (d1 == 0 || d2 == 0 || d3 == 0)
			continue;
		else
		{
			temp.push_back(vers.at(i));
		}
	}
	vers.clear();
	vers.resize(temp.size());
	vers = temp;
}

int main()
{
	cout << "----------Ready----------";
	getchar();
	//计时
	time_t t1 = GetTickCount();

	map<string, string> path3para;
	ConfigFileRead(path3para);
	string filepath3 = path3para["filepath3"];

	//获取参数
	mypara para;
	getPara(para);

	//获取点云文件1,2
	PointCloud<PointXYZRGB>::Ptr cloud1(new PointCloud<PointXYZRGB>);
	cout << para.filepathcloud1 << endl;
	cout << "---------------------------------------------------" << endl;
	if (para.format == ".pcd")
	{
		pcl::io::loadPCDFile(para.filepathcloud1, *cloud1);
	}
	else if (para.format == ".ply")
	{
		pcl::io::loadPLYFile(para.filepathcloud1, *cloud1);
	}
	else
	{
		cout << "Wrong Format!" << endl;
		return -1;
	}

	//计算网格
	PolygonMesh meshcloud1, meshcloud2;
	trianglulation(cloud1, meshcloud1, para, true);
	deleteUnnormal(meshcloud1.polygons, cloud1);

	vector<PointCloud<PointXYZRGB>::Ptr> holes_all;
	fillholes(meshcloud1, cloud1, holes_all, meshcloud2);

	cout << "Cloud1 has meshes: " << meshcloud1.polygons.size() << endl;
	cout << "Cloud2 has meshes: " << meshcloud2.polygons.size() << endl;


	PointCloud<PointXYZRGB>::Ptr hole1(new PointCloud<PointXYZRGB>);
	hole1 = holes_all.at(34);

	PolygonMesh holemesh;
	trianglulation(hole1, holemesh, para);

	// 计算法线
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	n.setInputCloud(cloud1);
	n.setKSearch(para.Ktri);
	n.compute(*normals);


	// 可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer1"));
	viewer->setBackgroundColor(0.9, 0.9, 0.9);
	viewer->addPointCloud(cloud1, "cloud1");
	viewer->addPolygonMesh(meshcloud1, "mesh1");
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud1, normals, 1, 0.01, "normals");
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("viewer2"));
	viewer2->setBackgroundColor(0.9, 0.9, 0.9);
	viewer2->addPointCloud(hole1, "hole_1");
	viewer2->addPolygonMesh(holemesh, "hole1");
	cout << "holemesh has meshes: " << holemesh.polygons.size() << endl;
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("viewer2"));
	//viewer3->setBackgroundColor(0.9, 0.9, 0.9);
	stringstream lineend;
	lineend << "line0end";
	for (int i=0;i<holes_all.size();i++)
	{
		stringstream cloudid;
		cloudid << "cloud" << i;
		viewer2->addPointCloud(holes_all.at(i), cloudid.str());
		//viewer3->addPointCloud(holes_all.at(i), cloudid.str());
		int cr = (rand() % 255);
		int cg = (rand() % 255);
		int cb = (rand() % 255);
		stringstream lineid;
		lineid << "line" << i;
		for (int j=0;j<holes_all.at(i)->size()-1;j++)
		{
			lineid << "aa";
			viewer2->addLine(holes_all.at(i)->at(j),
				holes_all.at(i)->at(j + 1), lineid.str());
			/*viewer2->addLine(holes_all.at(i)->at(j),
				holes_all.at(i)->at(j + 1), cr, cg, cb, lineid);*/
		}
		lineend << "bb";
		viewer2->addLine(holes_all.at(i)->at(0),
			holes_all.at(i)->at(holes_all.at(i)->size() - 1), lineend.str());
		/*viewer2->addLine(holes_all.at(i)->at(0),
			holes_all.at(i)->at(holes_all.at(i)->size() - 1), cr, cg, cb, "line0end");*/
	}
	


	while (!viewer->wasStopped()&& !viewer2->wasStopped())
	{
		viewer->spinOnce(100);
		viewer2->spinOnce(100);
		//viewer3->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	//// pcl::io::savePCDFile("C:\\Users\\zhihong\\Desktop\\cup\\2019.2.25\\ceshi2.pcd",*outcloud);
	//pcl::io::savePCDFileBinary("G:\\oilrape\\2017_OilRape\\2018.1.3\\out2019.4.16\\ceju\\33new.pcd", *cloudout_down);
	//因此输入为点云1，点云2，点云2的三角面片，
	//输出为与点云1存在相交或者平行情况的三角面片(包括了该三角面片与相交或平行的点云1的三角面片)的vector；

}

