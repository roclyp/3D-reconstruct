#include "PCLlibrary.h"
#include <iostream>
#include <string>
#include <pcl/console/time.h>   // TicToc
#include <time.h>

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBPtr;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointXYZPtr;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
bool tri = false;


void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
	void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown()) {
		tri = true;
	}
}

int main()//分80组，先1和2配准，3和4配准，4和6配准。。。然后结果12和34配准，56与78配准。。一直到最后位置，存在多个循环
{
	//string pathdir = "F:/1-DesktopFile_Pointget/1-DATA/2017.11.11/50_5_2After";
	//string pathdir = "F:/1-DesktopFile_Pointget/1-DATA/xlr data/New folder";
	viewer->setBackgroundColor(0, 0, 0);
	string filename = "C:\\Users\\Zhihong MA\\Desktop\\2\\2018.1.1A619-T-3\\side_1.pcd";
	//filename = "F:\\1-DesktopFile_Pointget\\wwq\\pointcloudshow\\CloudViewer-1.0.1\\CloudViewer\\PointCloudDate\\rabbit.pcd";
	//string filename = "F:/1-DesktopFile_Pointget/1-DATA/2018.1.3/A619-T-3/side_1.pcd";
	PointXYZRGBPtr cloud_IN(new pcl::PointCloud<pcl::PointXYZRGB>);
	int error = pcl::io::loadPCDFile(filename, *cloud_IN);
	PointXYZPtr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ point;
	for (int i = 0; i < cloud_IN->size();i++){
		point.x = cloud_IN->at(i).x;
		point.y = cloud_IN->at(i).y;
		point.z = cloud_IN->at(i).z;
		cloud_xyz->push_back(point);
	}

	if (error == -1)
	{
		PCL_WARN("Haven't load the Cloud First(The source one)!");
		return -1;
	}
	PCL_INFO("Loaded");

	viewer->addPointCloud(cloud_IN, "my");

	DWORD Start = ::GetTickCount();

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> nor;//法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针
	tree->setInputCloud(cloud_xyz);                        //用cloud构建tree对象
	nor.setInputCloud(cloud_xyz);                            //为法线估计对象设置输入点云
	nor.setSearchMethod(tree);                          //设置搜索方法
	nor.setKSearch(100);                                 //设置k搜索的k值为20
	nor.compute(*normals);                              //估计法线存储结果到normals中

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud_xyz, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(5);

	// Set typical values for the parameters
	gp3.setMu(3);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 6); // 30 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);
	//pcl::io::saveOBJFile("C:\\Users\\Zhihong MA\\Desktop\\2\\1.obj", triangles);

	pcl::PolygonMesh triout;
	triout.header = triangles.header;
	triout.cloud = triangles.cloud;
	//triout.polygons = triangles.polygons;
	DWORD End = ::GetTickCount();
	for (int i = 0; i < triangles.polygons.size(); i++)
	{
		auto point1 = triangles.polygons.at(i).vertices.at(0);
		auto point2 = triangles.polygons.at(i).vertices.at(1);
		auto point3 = triangles.polygons.at(i).vertices.at(2);
		if (point1 == point2 || point3 == point2 || point1 == point3)
		{
			/*triangles.polygons.erase(triangles.polygons.begin()+i);*/
			//triangles.polygons.at(i).vertices.at(0) = 0;
			//triangles.polygons.at(i).vertices.at(1) = 0;
			//triangles.polygons.at(i).vertices.at(2) = 0;
			continue;
		}
		else
		{
			triout.polygons.push_back(triangles.polygons.at(i));
		}
	}

	/*pcl::io::saveVTKFile("C:\\Users\\Zhihong MA\\Desktop\\2\\5.vtk", triangles);
	pcl::io::saveOBJFile("C:\\Users\\Zhihong MA\\Desktop\\2\\5.obj", triangles);*/

	cout << "The time taken for test is: " << End - Start << endl;
	viewer->addPolygonMesh(triout, "my2");
	//viewer->setRepresentationToSurfaceForAllActors();
	viewer->setRepresentationToWireframeForAllActors(); //网格模型以线框图模式显示
	//ostringstream saveName;
	////saveName << "C:/Users/Zhihong MA/Desktop/data/" << savenum << ".pcd";
	//////saveName << "C:/Users/Zhihong MA/Desktop/The_whole" << savenum << ".ply";
	////pcl::io::savePCDFile(saveName.str(), *Cloud_Temp_Last);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
		//if (tri==true) {
		//	viewer->addPolygonMesh(triangles, "my2");
		//	//viewer->setRepresentationToSurfaceForAllActors();
		//	viewer->setRepresentationToWireframeForAllActors(); //网格模型以线框图模式显示
		//}
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}