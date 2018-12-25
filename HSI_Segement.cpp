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
#include "math.h"

using namespace std;
using namespace pcl;

class PointwithlAble
{

};


double minrgb(double r, double g, double b)
{
	double min;
	if (r <= g)
		min = r;
	else
		min = g;
	if (min <= b)
		min = min;
	else
		min = b;
	return min;
}

int main()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Incloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr copycloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//int errors = pcl::io::loadPCDFile("F:\\1-DesktopFile_Pointget\\1-DATA\\2018.1.3\\After619-C-1\\After_top_1.pcd", *Incloud);
	int errors = pcl::io::loadPCDFile("C:\\Users\\zhihong\\Desktop\\out\\2\\1109-619-T-3.pcd", *Incloud);
	pcl::copyPointCloud(*Incloud, *copycloud);
	for (int i = 0; i < copycloud->size(); i++)
	{
		
			
		double r = copycloud->points[i].r/255.0;
		double g = copycloud->points[i].g / 255.0;
		double b = copycloud->points[i].b / 255.0;
		double Cmolecule = 0.5*((r - g) + (r - b));
		double CDenominator2 = (pow(r - g,2) + (r - b)*(g - b));
		double CDenominator = pow(CDenominator2, 0.5);
		double theta = acos(Cmolecule / (1.0*CDenominator));
		double h;
		if ( g>=b )
		{
			h = theta;
		}
		else if (g < b)
		{
			h = 2 * M_PI - theta;
		}

		double s = 1 - 3 * minrgb(r, g, b) / (1.0*(r + g + b));
		double intensityInone = (r + g + b) / 3.0;
		double H = h / (1.0*M_PI);
		double Fhs = 1.114 - 4.9*H - 0.229*s + 6.066*pow(H, 2) - 0.1638*H*s + 0.1852*pow(s, 2);
		double Ghs = 0.6475 - 2.777*H - 0.2797*s + 3.714*pow(H, 2) + 0.1326*H*s + 0.1133*pow(s, 2);
		copycloud->points[i].r = h*255;
		copycloud->points[i].g = s*255;
		copycloud->points[i].b = intensityInone*255;
	}
	shared_ptr<visualization::PCLVisualizer> viewer(new visualization::PCLVisualizer("3D Viewer2"));
	viewer->initCameraParameters();//设置相机参数，用户从默认的角度和方向观察点云
	//创建不同viewport可以进行对比,参数为：x轴的最小值、最大值，y轴最小值、最大值，标识符
	//viewer->setBackgroundColor(0, 0, 0);
	//设置背景颜色
	viewer->setBackgroundColor(0.1, 0.1, 0.1);
	viewer->addPointCloud(copycloud, "cloud1");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}