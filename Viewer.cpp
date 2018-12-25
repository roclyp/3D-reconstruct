#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
//点云三维重建过程：点云数据预处理、分割、三角网格化、网格渲染
//可视化
//坐标轴——X：蓝，Y：绿，Z：红
using namespace pcl;
using namespace pcl::io;
using namespace io;
using namespace std;



int main()
{
	bool simple(false), rgb(false), custom_c(false), normals(false),
		shapes(false), viewports(false), interaction_customization(false);

	//导入点云
	PointCloud<PointXYZ>::Ptr cloud_a(new PointCloud<PointXYZ>);
	//PointCloud<PointXYZRGBA>::Ptr cloud_b(new PointCloud<PointXYZRGBA>);
	//string filepath1 = "F:/1-DesktopFile_Pointget/1-DATA/2017.11.11/50_2After/After_2017_11_1154.pcd";
	//string filepath2 = "F:/1-DesktopFile_Pointget/1-DATA/2017.11.11/50_2After/After_2017_11_1153.pcd";
	//string filepath1 = "F:/1-DesktopFile_Pointget/1-DATA/2017.11.11/50_5_2After_2/After_2017.11.11_33.pcd";
	//string filepath1 = "F:/1-DesktopFile_Pointget/1-DATA/2017.11.11/50_5_2/2017.11.11_1.pcd";
	
	//string filepath1= "C:\\Users\\Zhihong MA\\Desktop\\box.pcd";
	string filepath1 = "F:\\1-DesktopFile_Pointget\\2_2018_0918植物三维重建论文\\多视角图像\\1.obj";
	/*cout << filepath1 << endl;*/
	/*int errors = pcl::io::loadPCDFile(filepath1, *cloud_a);*/
	int errors = pcl::io::loadOBJFile(filepath1, *cloud_a);
	//int errors = pcl::io::loadPLYFile(filepath1, *cloud_a);
	//int errors2 = pcl::io::loadPCDFile(filepath2, *cloud_b);


	//int errors = loadPCDFile("C:/Users/Zhihong MA/Desktop/test2017.9.22/AfterFilter/Test1.pcd_Filter.pcd", *cloud_a);
	if (errors == -1)
	{
		cout << "Can not find the file" << endl;
		return -1;
	}
	cout << "loaded" << endl;
	//getchar();

	cout << "Set Visualization" << endl;
	//建立可视化窗口，并命名，可定义为全局指针，保证可全局使用
	shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer("3D Viewer2"));
	viewer->initCameraParameters();//设置相机参数，用户从默认的角度和方向观察点云
	//创建不同viewport可以进行对比,参数为：x轴的最小值、最大值，y轴最小值、最大值，标识符
	//viewer->setBackgroundColor(0, 0, 0);

	//设置背景颜色
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud(cloud_a, "cloud1");
	//viewer->addPointCloud(cloud_b, "cloud2");
	//pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBA>  rgbx(cloud_a);//显示色彩信息
	//viewer->addPointCloud<PointXYZRGBA>(cloud_a, rgbx, "cloud1");
	//viewer->addPointCloud<PointXYZRGB>(cloud_a, "cloud12");
	//添加点云，并命名（标识符），用以区分和调用，
	//如果想更新一个已经显示的点云，用户必须先调用removePointCloud()，并提供需要更新的点云的ID号。
	//PCL1.1版本以上可以用updatePointCloud()直接实现点云的更新
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");//设置点云特征,此处为size
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");//设置点云特征
	//viewer->addCoordinateSystem(1.0);//添加坐标系

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> colorxx(cloud_a, 0, 255, 0);
	////创建一个自定义颜色处理器PointCloudColorHandlerCustom对象，并设置颜色为某种颜色
	//viewer->updatePointCloud(cloud_a, colorxx, "cloud12");
	//viewer->addPointCloud(cloud_a, "cloud12");
	//两种方式都是使得cloud_a中点云的点颜色改为绿色，但是前者是新建一个id，后者是在原id上做修改,你后面再调用cloud1的时候就还是绿色的
	//后者是添加新id cloud12，再调用cloud1时，还是原来的颜色
	//但是两者都不会对cloud_a做修改


	//调用spinonce函数给视窗处理时间的时间，允许鼠标键盘等交互操作
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}