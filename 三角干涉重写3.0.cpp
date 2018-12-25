#include "PCLlibrary.h"
#include <iostream>
#include <string>
#include <pcl/console/time.h>   // TicToc
#include <time.h>
#include <pcl/search/kdtree.h>
#include  <direct.h> 
#include "TriAngle.h"
using namespace std;

#define Pi 3.141592657;
int neiborTrinum = 10;
double neiborTriRadius = 0.025;
string neibormethod = "numbers";
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBPtr;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointXYZPtr;

void ConfigFileRead(map<string, string>& m_mapConfigInfo)
{
	ifstream configFile;
	string path = "./三角干涉.ini";
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

void getPara(mypara &mpara)
{
	map<string, string> param;
	ConfigFileRead(param);
	mpara.format = param["format"];
	mpara.filepathcloud1 = param["filepath1"];
	mpara.filepathcloud2 = param["filepath2"];
	mpara.outfilepath = param["outfilepath"];
	mpara.filepath_Circle = param["filepath_Circle"];
	mpara.outfilepath_Circle = param["outfilepath_Circle"];
	mpara.rotateMatrixPath = param["rotateMatrixPath"];
	istringstream KSearchstr(param["KSearchnum"]), SearchRadiusstr(param["SearchRadius"]), mustr(param["mu"]),
		maxneiborsstr(param["maxneibors"]), maxsuranglestr(param["maxsurangle"]),
		minanglestr(param["minangle"]), maxanglestr(param["maxangle"]), neiborTrinumstr(param["neiborTrinum"]), 
		neiborTriRadiusstr(param["neiborTriRadius"]);
	neiborTrinumstr >> mpara.neiborTrinum;
	neiborTriRadiusstr >> mpara.neiborTriRadius;
	KSearchstr >> mpara.KSearchnum;
	SearchRadiusstr >> mpara.SearchRadius; mustr >> mpara.mu; maxneiborsstr >> mpara.maxneibors;
	maxsuranglestr >> mpara.maxsurangle; minanglestr >> mpara.minangle; maxanglestr >> mpara.maxangle;
	mpara.norConsis = param["norConsis"];
	mpara.neibormethod = param["neibormethod"];
}

void greedy_triangle(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudin, pcl::PolygonMesh &triangles, mypara &paraall)
{
	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloudin);
	n.setInputCloud(cloudin);
	n.setSearchMethod(tree);
	n.setKSearch(paraall.KSearchnum);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloudin, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects

	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

	//三角化网格

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(paraall.SearchRadius);//设置连接点之间的最大距离（即为三角形最大边长）为0.025
	// Set typical values for the parameters
	gp3.setMu(paraall.mu); // 设置被样本点搜索其邻近点的最远距离为2.5，为了适应点云密度的变化
	gp3.setMaximumNearestNeighbors(paraall.maxneibors);//设置样本点可搜索的邻域个数为100
	gp3.setMaximumSurfaceAngle(M_PI * paraall.maxsurangle*1.0 / 180.0);   //设置某点法线方向偏离样本点法线方向的最大角度为
	gp3.setMinimumAngle(M_PI * paraall.minangle*1.0 / 180); // //设置三角化后得到三角形内角最小角度为10度
	gp3.setMaximumAngle(M_PI * paraall.maxangle*1.0 / 180);  //设置三角化后得到三角形内角最大角度为120度
	if (paraall.norConsis != "fales")
		gp3.setNormalConsistency(true);
	else
		gp3.setNormalConsistency(false);
	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);
	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
}

void getcolormesh(const pcl::PolygonMesh &triangles1, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud1_RGB,
	pcl::PolygonMesh &meshout)
{
	meshout = triangles1;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_mesh(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromPCLPointCloud2(triangles1.cloud, *cloud_color_mesh);
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloud1_RGB);
	// K nearest neighbor search
	int K = 5;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	for (int i = 0; i < cloud_color_mesh->points.size(); ++i)
	{
		uint8_t r = 0;
		uint8_t g = 0;
		uint8_t b = 0;
		float dist = 0.0;
		int red = 0;
		int green = 0;
		int blue = 0;
		if (kdtree.nearestKSearch(cloud_color_mesh->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			for (int j = 0; j < pointIdxNKNSearch.size(); ++j)
			{

				r = cloud1_RGB->points[pointIdxNKNSearch[j]].r;
				g = cloud1_RGB->points[pointIdxNKNSearch[j]].g;
				b = cloud1_RGB->points[pointIdxNKNSearch[j]].b;
				red += int(r);
				green += int(g);
				blue += int(b);
				dist += 1.0 / pointNKNSquaredDistance[j];
				//std::cout << "red: " << int(r) << std::endl;
				//std::cout << "green: " << int(g) << std::endl;
				//std::cout << "blue: " << int(b) << std::endl;
				//cout << "dis:" << dist << endl;
			}
		}
		cloud_color_mesh->points[i].r = int(red / pointIdxNKNSearch.size() + 0.5);
		cloud_color_mesh->points[i].g = int(green / pointIdxNKNSearch.size() + 0.5);
		cloud_color_mesh->points[i].b = int(blue / pointIdxNKNSearch.size() + 0.5);
	}
	pcl::toPCLPointCloud2(*cloud_color_mesh, meshout.cloud);
}

//获得三角面片的平面-得到三角网格的平面容器，3个顶点容器（3个顶点为一个对象），中心点容器，与中心点点云
void getspaceplane(TrianglePlane &mytriangles)
{
	for (int i = 0; i < mytriangles.triangle.polygons.size(); i++)
	{
		spacePlane myplane;
		Eigen::Vector4f centroid;
		pcl::PointCloud<PointXYZ>::Ptr pointgetcenter(new pcl::PointCloud<PointXYZ>);
		Vertices myvertices = mytriangles.triangle.polygons.at(i);
		PointXYZ p1 = mytriangles.cloudxyz->at(myvertices.vertices.at(0));
		PointXYZ p2 = mytriangles.cloudxyz->at(myvertices.vertices.at(1));
		PointXYZ p3 = mytriangles.cloudxyz->at(myvertices.vertices.at(2));
		//verticesPoint  *mypoints;
		//mypoints->Point3vertices.push_back(p1);
		//mypoints->Point3vertices.push_back(p2);
		//mypoints->Point3vertices.push_back(p3);
		verticesPoint mypoints(p1, p2, p3);
		pointgetcenter->push_back(p1);
		pointgetcenter->push_back(p2);
		pointgetcenter->push_back(p3);//质心 
		pcl::compute3DCentroid(*pointgetcenter, centroid);
		PointXYZ centerpoint;
		centerpoint.x = centroid(0);
		centerpoint.x = centroid(1);
		centerpoint.x = centroid(2);
		myplane.A = p1.y*(p2.z - p3.z) + p2.y*(p3.z - p1.z) + p3.y*(p1.z - p2.z);
		myplane.B = p1.z*(p2.x - p3.x) + p2.z*(p3.x - p1.x) + p3.z*(p1.x - p2.x);
		myplane.C = p1.x*(p2.y - p3.y) + p2.x*(p3.y - p1.y) + p3.x*(p1.y - p2.y);
		myplane.D = -(myplane.A*p1.x + myplane.B*p1.y + myplane.C*p1.z);
		mytriangles.myplaneall.push_back(myplane);
		mytriangles.Ver3Point.push_back(mypoints);
		//mytriangles.centroidall.push_back(centroid);
		mytriangles.centerPoinCloud->push_back(centerpoint);
	}
}

//计算三角形面积
double caltriAngleArea(const PointXYZ &proPoint, const PointXYZ &p1, const PointXYZ &p2)
{
	double a2 = (proPoint.x - p1.x)*(proPoint.x - p1.x)
		+ (proPoint.y - p1.y)*(proPoint.y - p1.y)
		+ (proPoint.z - p1.z)*(proPoint.z - p1.z);
	double b2 = (proPoint.x - p2.x)*(proPoint.x - p2.x)
		+ (proPoint.y - p2.y)*(proPoint.y - p2.y)
		+ (proPoint.z - p2.z)*(proPoint.z - p2.z);
	double ab = (proPoint.x - p1.x)*(proPoint.x - p2.x)
		+ (proPoint.y - p1.y)*(proPoint.y - p2.y)
		+ (proPoint.z - p1.z)*(proPoint.z - p2.z);
	return 0.5*sqrt(a2*b2 - ab * ab);
}

//获得领域三角面片
void getNeiborTriangle(const TrianglePlane::Ptr &mytriangles2, const pcl::PointXYZ &point1, TrianglePlane &temptriangles)
{
	pcl::PointCloud<PointXYZ>::Ptr tempcenterPointcloud(new pcl::PointCloud<PointXYZ>);
	pcl::copyPointCloud<PointXYZ>(*mytriangles2->centerPoinCloud, *tempcenterPointcloud);
	/*= mytriangles2.centerPoinCloud;*/
	tempcenterPointcloud->push_back(point1);
	if (neibormethod == "numbers")
	{
		vector<int>kdpointID(neiborTrinum + 1);
		vector<float>kdpointDis(neiborTrinum + 1);
		// 建立kdtree  
		pcl::search::KdTree<PointXYZ>::Ptr kdtree1(new pcl::search::KdTree<PointXYZ>);
		//pcl::KdTreeFLANN<PointXYZ>::Ptr tree1(new pcl::KdTreeFLANN<PointXYZ>);
		kdtree1->setInputCloud(tempcenterPointcloud);
		if (kdtree1->nearestKSearch(point1, neiborTrinum + 1, kdpointID, kdpointDis) > 0)
		{
			//保存邻域三角面片
			for (int j = 1; j < kdpointID.size(); j++)
			{
				temptriangles.myplaneall.push_back(mytriangles2->myplaneall.at(kdpointID[j]));
				temptriangles.Ver3Point.push_back(mytriangles2->Ver3Point.at(kdpointID[j]));
				//temptriangles.centroidall.push_back(mytriangles2.centroidall.at(kdpointID[j]));
				temptriangles.centerPoinCloud->push_back(mytriangles2->centerPoinCloud->at(kdpointID[j]));
			}
		}
	}
	else
	{
		vector<int>kdpointID;
		vector<float>kdpointDis;
		// 建立kdtree  
		pcl::search::KdTree<PointXYZ>::Ptr kdtree1(new pcl::search::KdTree<PointXYZ>);
		//pcl::KdTreeFLANN<PointXYZ>::Ptr tree1(new pcl::KdTreeFLANN<PointXYZ>);
		kdtree1->setInputCloud(tempcenterPointcloud);
		if (kdtree1->radiusSearch(point1, neiborTriRadius, kdpointID, kdpointDis) > 0)
		{
			//保存邻域三角面片
			for (int j = 1; j <= kdpointID.size(); j++)
			{
				temptriangles.myplaneall.push_back(mytriangles2->myplaneall.at(kdpointID[j]));
				temptriangles.Ver3Point.push_back(mytriangles2->Ver3Point.at(kdpointID[j]));
				//temptriangles.centroidall.push_back(mytriangles2.centroidall.at(kdpointID[j]));
				temptriangles.centerPoinCloud->push_back(mytriangles2->centerPoinCloud->at(kdpointID[j]));
			}
		}
	}
}

//判断是否干涉
bool isInterference(const spaceLine &Zer2P, const spacePlane &myPlane, const verticesPoint &verticesPoint,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2, double &tempdis, PointXYZ &cloudProjected)
{
	double Erate = -(myPlane.D + myPlane.A*Zer2P.x0 + myPlane.B*Zer2P.y0 + myPlane.C*Zer2P.z0)*1.0 /
		(myPlane.A*Zer2P.dx + myPlane.B*Zer2P.dy + myPlane.C*Zer2P.dz);
	PointXYZ proPoint;//投影点坐标
	proPoint.x = Erate * Zer2P.dx + Zer2P.x0;
	proPoint.y = Erate * Zer2P.dy + Zer2P.y0;
	proPoint.z = Erate * Zer2P.dz + Zer2P.z0;
	//计算投影点到视角的距离
	double dis = sqrt((Zer2P.viewPoint.x - proPoint.x)*(Zer2P.viewPoint.x - proPoint.x)
		+ (Zer2P.viewPoint.y - proPoint.y)*(Zer2P.viewPoint.y - proPoint.y)
		+ (Zer2P.viewPoint.z - proPoint.z)*(Zer2P.viewPoint.z - proPoint.z));
	double area = caltriAngleArea(proPoint, verticesPoint.Point3vertices.at(0), verticesPoint.Point3vertices.at(1));
	area += caltriAngleArea(proPoint, verticesPoint.Point3vertices.at(0), verticesPoint.Point3vertices.at(2));
	area += caltriAngleArea(proPoint, verticesPoint.Point3vertices.at(1), verticesPoint.Point3vertices.at(2));
	//如果点在三角内，说明于三角面片干涉,那么记录该视角点到投影点的距离
	if (area <= verticesPoint.triarea*1.05)
	{
		cloudProjected = proPoint;
		tempdis = dis;
		return true;
	}
	else
	{
		tempdis = INT_MAX;
		return false;
	}
}

//获取干涉情况,只对阈值距离内的三角面去计算是否干涉
void getInterference(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1, const TrianglePlane::Ptr &mytriangles2,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud2, pcl::PointXYZ &zerop, vector<interSituation> &interferenceCloud1)
{
	//建立kdtree去检索点云2中点云1三角面片领域点的三角面片点，对距离阈值范围的三角面片进行干涉检查
	for (int i = 0; i < cloud1->size(); i++)
	{
		interSituation temp_inter;
		TrianglePlane temptriangle;
		//获得领域三角面片
		getNeiborTriangle(mytriangles2, cloud1->at(i), temptriangle);
		spaceLine Zer2P;
		//得到直线方程
		Zer2P.dx = cloud1->at(i).x - zerop.x;
		Zer2P.dy = cloud1->at(i).y - zerop.y;
		Zer2P.dz = cloud1->at(i).z - zerop.z;
		Zer2P.x0 = cloud1->at(i).x;
		Zer2P.y0 = cloud1->at(i).y;
		Zer2P.z0 = cloud1->at(i).z;
		Zer2P.viewPoint = zerop;
		Zer2P.endpoint = cloud1->at(i);
		double mindis = INT_MAX;
		temp_inter.Pinter = cloud1->at(i);
		for (int j = 0; j < temptriangle.myplaneall.size(); j++)
		{
			//检测是否干涉，输入：点云1该点，点云2的j号三角面片平面，点云2的3个顶点，点云2，临时距离
			double tempdis;
			PointXYZ cloudProjected;
			//如果是干涉情况，则保留最近的干涉
			if (isInterference(Zer2P, mytriangles2->myplaneall.at(j), mytriangles2->Ver3Point.at(j), cloud2, tempdis, cloudProjected) == true)
			{
				if (tempdis <= mindis)
				{
					mindis = tempdis;
					temp_inter.distance = mindis;
					temp_inter.ICross = cloudProjected;
					temp_inter.Verticespolygons = mytriangles2->Ver3Point.at(j);
					temp_inter.verticesID = mytriangles2->triangle.polygons.at(j);
				}
				temp_inter.has_inter = true;
				cout << "point: " << i << " has interation" << endl;
			}
		}
		interferenceCloud1.push_back(temp_inter);
	}
}

int main()
{
	cout << "Ready.....";
	getchar();
	//获取参数
	cout << "Start working" << endl;
	time_t t1 = GetTickCount();
	mypara paraall;
	getPara(paraall);
	neiborTrinum = paraall.neiborTrinum;
	neiborTriRadius = paraall.neiborTriRadius;
	neibormethod = paraall.neibormethod;
	//读取点云
	TrianglePlane::Ptr mytriangles1 (new TrianglePlane);
	TrianglePlane::Ptr mytriangles2 (new TrianglePlane);
	//pcl::PointCloud<PointXYZ>::Ptr Cloud1xyz(new pcl::PointCloud<PointXYZ>);
	//pcl::PointCloud<PointXYZRGB>::Ptr Cloud1xyzrgb(new pcl::PointCloud<PointXYZRGB>);
	pcl::PointCloud<PointXYZ>::Ptr Cloud2xyz(new pcl::PointCloud<PointXYZ>);
	pcl::PointCloud<PointXYZRGB>::Ptr Cloud2xyzrgb(new pcl::PointCloud<PointXYZRGB>);
	pcl::io::loadPCDFile(paraall.filepathcloud1, *mytriangles1->cloudxyz);
	pcl::io::loadPCDFile(paraall.filepathcloud1, *mytriangles1->cloudxyzrgb);
	pcl::io::loadPCDFile(paraall.filepathcloud2, *Cloud2xyz);
	pcl::io::loadPCDFile(paraall.filepathcloud2, *Cloud2xyzrgb);
	//*mytriangles1->cloudxyz = *Cloud1xyz;
	//*mytriangles1->cloudxyzrgb = *Cloud1xyzrgb;
	//读取旋转矩阵
	Eigen::Matrix4d rotateMatrix;
	ifstream rotatetxt;
	rotatetxt.open(paraall.rotateMatrixPath);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			rotatetxt >> rotateMatrix(i, j);
		}
	}
	rotatetxt.close();

	//视点旋转
	pcl::PointCloud<pcl::PointXYZ>::Ptr ZeroPoint(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ zerop;
	zerop.x = 0; zerop.y = 0; zerop.z = 0;
	ZeroPoint->push_back(zerop);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ZeroPoint_Trans(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Matrix4d rotateMatrixinverse = rotateMatrix;
	pcl::transformPointCloud<PointXYZ>(*ZeroPoint, *ZeroPoint_Trans, rotateMatrixinverse);
	//添加视点到对应点云的triplane类型中
	mytriangles1->viewpoints = ZeroPoint;
	mytriangles2->viewpoints = ZeroPoint_Trans;
	//点云旋转
	pcl::transformPointCloud<PointXYZ>(*Cloud2xyz, *mytriangles2->cloudxyz, rotateMatrix);
	pcl::transformPointCloud<PointXYZRGB>(*Cloud2xyzrgb, *mytriangles2->cloudxyzrgb, rotateMatrix);

	//pcl::PolygonMesh triangles1;
	//pcl::PolygonMesh triangles2;
	greedy_triangle(mytriangles1->cloudxyz, mytriangles1->triangle, paraall);
	greedy_triangle(mytriangles2->cloudxyz, mytriangles2->triangle, paraall);
	//mytriangles1->triangle = triangles1;
	//mytriangles2->triangle = triangles2;
	getspaceplane(*mytriangles1);
	getspaceplane(*mytriangles2);



	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(cloud1, 250, 0, 0);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color2(cloud2, 0, 255, 0);
	//viewer->addPointCloud<pcl::PointXYZ>(cloud1, sources_cloud_color,"cloud1");
	//viewer->addPolygonMesh(triangles1,"mesh1");
	//viewer->addPointCloud<pcl::PointXYZ>(cloud2_tran, sources_cloud_color2, "cloud2");
	//viewer->addPolygonMesh(triangles2, "mesh2");
	//viewer->addPointCloud(cloud1_RGB, "cloud1");
	//viewer->addPointCloud(cloud2Tran_RGB, "cloud2");

	vector<interSituation> interferenceCloud1;
	vector<interSituation> interferenceCloud2;
	getInterference(mytriangles1->cloudxyz, mytriangles2, mytriangles2->cloudxyz, ZeroPoint->at(0), interferenceCloud1);
	cout << "Cloud2 —————————————————————————————— Cloud2" << endl;
	getInterference(mytriangles2->cloudxyz, mytriangles1, mytriangles1->cloudxyz, ZeroPoint_Trans->at(0), interferenceCloud2);


	//for(int j=0;j<mytriangles1->triangle.polygons.size();j++)
	std::vector<std::vector<verticesPoint>> crossintermap;
	std::vector<std::vector<verticesPoint>> Notcrossintermap;
	for (int i = 0; i < interferenceCloud1.size(); i++)
	{
		if (interferenceCloud1.at(i).has_inter == true)
		{
			for (int id = 0; id < 3; id++)
			{
				auto pid=interferenceCloud1.at(i).verticesID.vertices.at(id);
				if (interferenceCloud2.at(pid).has_inter == false)
				{
					continue;
				}
				else
				{

				}
			}
		}
	}
	//for (int i = 0; i < interferenceCloud1.size(); i++)
	//{
	//	auto temp_inter = interferenceCloud1.at(i);
	//	if (temp_inter.has_inter == true)
	//	{
	//		temp_inter.Verticespolygons.Point3vertices.at(0);
	//	}

	//}



	//彩色网格
	//pcl::PolygonMesh meshout1;
	//pcl::PolygonMesh meshout2;
	//getcolormesh(triangles1, cloud1_RGB, meshout1);
	//getcolormesh(triangles2, cloud2Tran_RGB, meshout2);
	///*viewer->addPointCloud(ZeroPoint, "zero");
	//viewer->addPointCloud(ZeroPoint_Trans, "zero_trans");*/
	////viewer->addPointCloud(cloud1_RGB);
	//viewer->addPolygonMesh(meshout1, "color_mesh1");
	//viewer->addPolygonMesh(triangles2, "mesh2");
	//viewer->addPolygonMesh(meshout2, "color_mesh2");


	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("3D Viewer2"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color3(cloud1, 250, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color4(cloud2, 0, 255, 0);
	viewer2->addPointCloud<pcl::PointXYZ>(cloud1, sources_cloud_color3, "cloud11");
	viewer2->addPolygonMesh(triangles1, "mesh11");

	viewer2->addPointCloud<pcl::PointXYZ>(cloud2_tran, sources_cloud_color4, "cloud22");*/

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0.5, 0.5, 0.5);
	viewer->addPolygonMesh(mytriangles1->triangle, "mesh1");
	pcl::PolygonMesh colormesh2;
	getcolormesh(mytriangles2->triangle, mytriangles2->cloudxyzrgb, colormesh2);
	viewer->addPolygonMesh(colormesh2, "colormesh2");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	//while (!viewer2->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}
	return 0;
}