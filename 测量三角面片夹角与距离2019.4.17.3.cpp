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
	string path = "./测量夹角与距离.ini";
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

template<typename T>
double computedis2(const T &cloud1, const T &cloud2)
{
	double x2 = (cloud1.x - cloud2.x)*(cloud1.x - cloud2.x);
	double y2 = (cloud1.y - cloud2.y)*(cloud1.y - cloud2.y);
	double z2 = (cloud1.z - cloud2.z)*(cloud1.z - cloud2.z);
	double dis = sqrt(x2 + y2 + z2);
	return dis;
}

//给邻近三角面片排序
void sortDueDis2(vector<pcl::Vertices> &vers, vector<double> &dis)
{
	for (int i = 0; i < vers.size(); i++)
	{
		for (int j = i + 1; j < vers.size(); j++)
		{
			if (dis.at(i) >= dis.at(j))
			{
				swap(vers.at(i), vers.at(j));
				swap(dis.at(i), dis.at(j));
			}
		}
	}
}

void getCross1Angle(CloudCenVers &cloudCenVer2, PointCloud<PointXYZRGB>::Ptr &cloud2trans,
	PointCloud<PointXYZRGB>::Ptr &cloud1,
	vector<double> &dangle, mypara &para)
{
	int meshes = 0;
	for (int i = 0; i < cloudCenVer2.interAll.size(); i++)
	{
		auto temp = cloudCenVer2.interAll.at(i);
		int crosssize = temp.Cross1itervers.size();
		if (crosssize == 0)
		{
			continue;
			//cout << "Triangle " << i << "has no 1 sides cross triangles" << endl;
		}
		else
		{
			sortDueDis2(temp.Cross1itervers, temp.Cross1dis);
			int dealnum;
			if (temp.Cross1itervers.size() >= para.dealnum)
				dealnum = para.dealnum;
			else
				dealnum = temp.Cross1itervers.size();
			meshes += dealnum;//处理网格数
			PointXYZRGB cenpoint;
			pcl::copyPoint(temp.cenPoint, cenpoint);
			for (int j = 0; j < dealnum; j++)
			{
				auto temp2 = temp.Cross1itervers.at(j);

				spaceLine crossLine;
				PointXYZRGB cloud2p1 = cloud2trans->at(temp.vertices.vertices[0]);
				PointXYZRGB cloud2p2 = cloud2trans->at(temp.vertices.vertices[1]);
				PointXYZRGB cloud2p3 = cloud2trans->at(temp.vertices.vertices[2]);
				PointXYZRGB cloud1p1 = cloud1->at(temp2.vertices[0]);
				PointXYZRGB cloud1p2 = cloud1->at(temp2.vertices[1]);
				PointXYZRGB cloud1p3 = cloud1->at(temp2.vertices[2]);
				/*spacePlane plane2(cloud2p1, cloud2p2, cloud2p3);
				spacePlane plane1(cloud1p1, cloud1p2, cloud1p3);
				spacePlane outplane;*/
				//获得旋转角度的两倍，相交线，以及中值平面
				PointXYZRGB adjustC2p1 = cloud2p1, adjustC2p2 = cloud2p2, adjustC2p3 = cloud2p3;
				spacePlane plane1(cloud1p1, cloud1p2, cloud1p3);
				
				double doubleangle = 100.0;
				int samevalue = 0;
				int maxitration = 0;
				do
				{
					double anglebefore = doubleangle;
					spacePlane plane2(adjustC2p1, adjustC2p2, adjustC2p3);
					
					spacePlane outplane;
					doubleangle = fabs(getAngle(plane2, plane1, outplane, crossLine));
					//将点云2投影至中值平面
					PointXYZRGB out1, out2, out3;
					if (getAdjustPoint(adjustC2p1, outplane, out1, para.Tthrehold) == true
						&& computedis2(cenpoint, out1) <= para.SearchRadius)
						adjustC2p1 = out1;
					if (getAdjustPoint(adjustC2p2, outplane, out2, para.Tthrehold) == true
						&& computedis2(cenpoint, out2) <= para.SearchRadius)
						adjustC2p2 = out2;
					if (getAdjustPoint(adjustC2p3, outplane, out3, para.Tthrehold) == true
						&& computedis2(cenpoint, out3) <= para.SearchRadius)
						adjustC2p3 = out3;
					if (fabs(doubleangle - anglebefore) <= 0.001)
						samevalue++;
					if (samevalue == 10 || maxitration == 100)
						break;
					maxitration++;
				} while (doubleangle >= (10.0 / 180 * 3.141592657));


				spacePlane plane3(adjustC2p1, adjustC2p3, adjustC2p3);
				spacePlane outplane2;
				spaceLine crossLine2;
				double doubleangle2 = fabs(getAngle(plane3, plane1, outplane2, crossLine2));
				//将点云2投影至中值平面
				//将点云2投影至中值平面
				dangle.push_back(doubleangle2);
			}
		}
	}
	cout << "Cross1 meshes: " << meshes << endl;
}

void getCross2Angle(CloudCenVers &cloudCenVer2, PointCloud<PointXYZRGB>::Ptr &cloud2trans,
	PointCloud<PointXYZRGB>::Ptr &cloud1,
	vector<double> &dangle, mypara &para)
{
	int meshes = 0;
	for (int i = 0; i < cloudCenVer2.interAll.size(); i++)
	{
		auto temp = cloudCenVer2.interAll.at(i);
		int crosssize = temp.Cross2itervers.size();
		if (crosssize == 0)//temp.Cross2itervers.size() == 0)
		{
			continue;
			//cout << "Triangle " << i << "has no 1 sides cross triangles" << endl;
		}
		else
		{
			sortDueDis2(temp.Cross2itervers, temp.Cross2dis);
			int dealnum;
			if (temp.Cross2itervers.size() >= para.dealnum)
				dealnum = para.dealnum;
			else
				dealnum = temp.Cross2itervers.size();
			meshes += dealnum;//处理网格数
			PointXYZRGB cenpoint;
			pcl::copyPoint(temp.cenPoint, cenpoint);
			for (int j = 0; j < dealnum; j++)
			{
				auto temp2 = temp.Cross2itervers.at(j);
				spaceLine crossLine;
				PointXYZRGB cloud2p1 = cloud2trans->at(temp.vertices.vertices[0]);
				PointXYZRGB cloud2p2 = cloud2trans->at(temp.vertices.vertices[1]);
				PointXYZRGB cloud2p3 = cloud2trans->at(temp.vertices.vertices[2]);
				PointXYZRGB cloud1p1 = cloud1->at(temp2.vertices[0]);
				PointXYZRGB cloud1p2 = cloud1->at(temp2.vertices[1]);
				PointXYZRGB cloud1p3 = cloud1->at(temp2.vertices[2]);
				PointXYZRGB adjustC2p1 = cloud2p1, adjustC2p2 = cloud2p2, adjustC2p3 = cloud2p3;
				spacePlane plane1(cloud1p1, cloud1p2, cloud1p3);

				double doubleangle = 100.0;
				int samevalue = 0;
				int maxitration = 0;
				do
				{
					double anglebefore = doubleangle;
					spacePlane plane2(adjustC2p1, adjustC2p2, adjustC2p3);

					spacePlane outplane;
					doubleangle = fabs(getAngle(plane2, plane1, outplane, crossLine));
					//将点云2投影至中值平面
					PointXYZRGB out1, out2, out3;
					if (getAdjustPoint(adjustC2p1, outplane, out1, para.Tthrehold) == true
						&& computedis2(cenpoint, out1) <= para.SearchRadius)
						adjustC2p1 = out1;
					if (getAdjustPoint(adjustC2p2, outplane, out2, para.Tthrehold) == true
						&& computedis2(cenpoint, out2) <= para.SearchRadius)
						adjustC2p2 = out2;
					if (getAdjustPoint(adjustC2p3, outplane, out3, para.Tthrehold) == true
						&& computedis2(cenpoint, out3) <= para.SearchRadius)
						adjustC2p3 = out3;
					if (fabs(doubleangle - anglebefore) <= 0.001)
						samevalue++;
					if (samevalue == 10 || maxitration == 100)
						break;
					maxitration++;
				} while (doubleangle >= (10.0 / 180 * 3.141592657));


				spacePlane plane3(adjustC2p1, adjustC2p2, adjustC2p3);
				spacePlane outplane2;
				spaceLine crossLine2;
				double doubleangle2 = fabs(getAngle(plane3, plane1, outplane2, crossLine2));
				//将点云2投影至中值平面
				//将点云2投影至中值平面
				dangle.push_back(doubleangle2);
			}
		}
	}
	cout << "Cross2 meshes: " << meshes << endl;
}

void getParaDis(CloudCenVers &cloudCenVer2, PointCloud<PointXYZRGB>::Ptr &cloud2trans,
	PointCloud<PointXYZRGB>::Ptr &cloud1,
	vector<double> &disall, mypara &para)
{
	int meshes = 0;
	for (int i = 0; i < cloudCenVer2.interAll.size(); i++)
	{
		auto temp = cloudCenVer2.interAll.at(i);
		int parallsize = temp.Parallitervers.size();
		if (parallsize == 0)
		{
			continue;
			//cout << "Triangle " << i << "has no parallel triangles" << endl;
		}
		else
		{
			sortDueDis2(temp.Parallitervers, temp.Paralledldis);
			int dealnum;
			if (temp.Parallitervers.size() >= para.dealnum)
				dealnum = para.dealnum;
			else
				dealnum = temp.Parallitervers.size();
			meshes += dealnum;//处理网格数
			PointXYZRGB cenpoint;
			pcl::copyPoint(temp.cenPoint, cenpoint);
			for (int j = 0; j < dealnum; j++)
			{
				auto temp2 = temp.Parallitervers.at(j);
				spaceLine crossLine;
				PointXYZRGB cloud2p1 = cloud2trans->at(temp.vertices.vertices[0]);
				PointXYZRGB cloud2p2 = cloud2trans->at(temp.vertices.vertices[1]);
				PointXYZRGB cloud2p3 = cloud2trans->at(temp.vertices.vertices[2]);
				PointXYZRGB cloud1p1 = cloud1->at(temp2.vertices[0]);
				PointXYZRGB cloud1p2 = cloud1->at(temp2.vertices[1]);
				PointXYZRGB cloud1p3 = cloud1->at(temp2.vertices[2]);
				
				spacePlane plane1(cloud1p1, cloud1p2, cloud1p3);
				double dis = 10;
				double doubleangle = 100.0;
				int samevalue = 0;
				int maxitration = 0;
				PointXYZRGB adjustC2p1 = cloud2p1, adjustC2p2 = cloud2p2, adjustC2p3 = cloud2p3;
				do
				{
					double disbefore = dis;
					spacePlane plane2(adjustC2p1, adjustC2p2, adjustC2p3);

					spacePlane outplane;
					outplane.A = (plane1.A + plane2.A)*1.0 / 2;
					outplane.B = (plane1.B + plane2.B)*1.0 / 2;
					outplane.C = (plane1.C + plane2.C)*1.0 / 2;
					outplane.D = (plane1.D + plane2.D)*1.0 / 2;
					double mu = sqrt(plane1.A*plane1.A + plane1.B*plane1.B + plane1.C*plane1.C);
					double son = plane1.D - plane2.D;
					dis = fabs((son)/ mu);
					//将点云2投影至中值平面
					PointXYZRGB out1, out2, out3;
					if (getAdjustPoint(adjustC2p1, outplane, out1, para.Tthrehold) == true)
						adjustC2p1 = out1;
					if (getAdjustPoint(adjustC2p2, outplane, out2, para.Tthrehold) == true)
						adjustC2p2 = out2;
					if (getAdjustPoint(adjustC2p3, outplane, out3, para.Tthrehold) == true)
						adjustC2p3 = out3;
					if (fabs(dis - disbefore) <= 10e-10)
						samevalue++;
					if (samevalue == 10 || maxitration == 100)
						break;
					maxitration++;
				} while (dis > 5 * 10e-10);

				spacePlane plane3(adjustC2p1, adjustC2p2, adjustC2p3);
				double dis2 = fabs(plane1.D - plane3.D);
				disall.push_back(dis2);
			}
		}
	}
	cout << "Parell meshes: " << meshes << endl;
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
		if (d1 == 0 || d2 == 0 || d3 == 0 )
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
	PointCloud<PointXYZRGB>::Ptr cloud2(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud3(new PointCloud<PointXYZRGB>);
	cout << para.filepathcloud1 << endl;
	cout << para.filepathcloud2 << endl;
	cout << filepath3 << endl;
	cout << "---------------------------------------------------" << endl;
	if (para.format == ".pcd")
	{
		pcl::io::loadPCDFile(para.filepathcloud1, *cloud1);
		pcl::io::loadPCDFile(para.filepathcloud2, *cloud2);
		pcl::io::loadPCDFile(filepath3, *cloud3);

	}
	else if (para.format == ".ply")
	{
		pcl::io::loadPLYFile(para.filepathcloud1, *cloud1);
		pcl::io::loadPLYFile(para.filepathcloud2, *cloud2);
		pcl::io::loadPCDFile(filepath3, *cloud3);
	}
	else
	{
		cout << "Wrong Format!" << endl;
		return -1;
	}


	//测试阶段为了让区分明显， 因此点云1网格显示为彩色
	PolygonMesh meshcloud1, meshcloud2;
	trianglulation(cloud1, meshcloud1, para, true);
	trianglulation(cloud2, meshcloud2, para);

	deleteUnnormal(meshcloud1.polygons, cloud1);
	deleteUnnormal(meshcloud2.polygons, cloud2);


	//先获取点云1，点云2三角面片对应的中心点云1，2；
	CloudCenVers cloudCenVer1(cloud1, meshcloud1);
	CloudCenVers cloudCenVer2(cloud2, meshcloud2);

	for (int i = 0; i < cloudCenVer2.interAll.size(); i++)
	{
		deleteUnnormal(cloudCenVer2.interAll.at(i).Parallitervers, cloud1);
		deleteUnnormal(cloudCenVer2.interAll.at(i).Cross1itervers, cloud1);
		deleteUnnormal(cloudCenVer2.interAll.at(i).Cross2itervers, cloud1);
	}

	//由于将点云2转换到点云1的坐标系下,处理的都是点云2的数据，
	//因此需要计算点云2中的每个三角面片是否和点云1中的三角面片相交
	//因此程序要讲点云2中的每个点添加到点云1中去计算，最后得到点云2中各个三角面片的邻近三角面片
	getNeiborTriangle(cloudCenVer2, cloudCenVer1, para);

	//对点云2处理三角面片相交的情况
	TriCross(cloudCenVer2, cloud2, cloudCenVer1, cloud1, para);
	vector<double>angleall;
	vector<double>disall;

	cout << "Cloud1 has meshes: " << meshcloud1.polygons.size() << endl;
	cout << "Cloud2 has meshes: " << meshcloud2.polygons.size() << endl;
	


	getCross1Angle(cloudCenVer2, cloud2, cloud1, angleall, para);
	getCross2Angle(cloudCenVer2, cloud2, cloud1, angleall, para);
	getParaDis(cloudCenVer2, cloud2, cloud1, disall, para);

	double avedis = 0;
	double aveangle = 0;
	int anglesize = angleall.size();
	for (int i = 0; i < angleall.size(); i++)//angleall.size()
	{
		if (isnan(angleall.at(i)))
			anglesize--;
		else
			aveangle = aveangle + angleall.at(i);
	}

	for (int i = 0; i < disall.size(); i++)
	{
		avedis += disall.at(i);
	}

	cout << "Average angle = " << aveangle * 1.0 / anglesize << endl;
	cout << "Average distance = " << avedis * 1000.0 / disall.size() << endl;



	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer1"));
	viewer->setBackgroundColor(0.9, 0.9, 0.9);
	viewer->addPointCloud(cloud1, "cloud1");
	viewer->addPolygonMesh(meshcloud1, "mesh1");
	viewer->addPointCloud(cloud2, "cloud2");
	viewer->addPolygonMesh(meshcloud2, "mesh2");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	//// pcl::io::savePCDFile("C:\\Users\\zhihong\\Desktop\\cup\\2019.2.25\\ceshi2.pcd",*outcloud);
	//pcl::io::savePCDFileBinary("G:\\oilrape\\2017_OilRape\\2018.1.3\\out2019.4.16\\ceju\\33new.pcd", *cloudout_down);
	//因此输入为点云1，点云2，点云2的三角面片，
	//输出为与点云1存在相交或者平行情况的三角面片(包括了该三角面片与相交或平行的点云1的三角面片)的vector；

}

