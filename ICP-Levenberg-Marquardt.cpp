#include "PCLlibrary.h"
#include <iostream>
#include <string>
#include <pcl/console/time.h>   // TicToc

using namespace std;
pcl::visualization::PCLVisualizer *viewer(new pcl::visualization::PCLVisualizer("Registration Viewer"));
int vp1, vp2;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//穿件PCLFile结构
struct PCLFile
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	std::string f_name;
	PCLFile() :cloud(new pcl::PointCloud<pcl::PointXYZRGB>) {};
};

//自定义类，用以表示坐标与曲率信息
class MyPointRepresentation :public pcl::PointRepresentation <PointNormalT>
{
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
	MyPointRepresentation()
	{
		nr_dimensions_ = 4;
	}
	virtual void copyToFloatArray(const PointNormalT &p, float * out) const
	{
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};

void showPreReg(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_first, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_second)
{
	viewer->addPointCloud(cloud_first, "First", vp1);
	viewer->addPointCloud(cloud_second, "Second", vp1);
}

void showAftReg(const PointCloudWithNormals::Ptr cloud_first, const PointCloudWithNormals::Ptr cloud_second)
{
	viewer->removePointCloud("First_Cloud");
	viewer->removePointCloud("Second_Cloud");
	//点云彩色句柄加载颜色，本身具有颜色就不添加，赋予颜色语句如下
	//PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0); //目标点云绿色

	pcl::visualization::PointCloudColorHandlerGenericField<PointNormalT> Cloud_First(cloud_first, "Curvature");
	/*if (!Cloud_First.isCapable())
		PCL_WARN("Cannot creat First curvature color handler");*/
	pcl::visualization::PointCloudColorHandlerGenericField<PointNormalT> Cloud_Second(cloud_second, "Curvature");
	/*if (!Cloud_Second.isCapable())
		PCL_WARN("Cannot creat Second curvature color handler");*/
	viewer->addPointCloud(cloud_first, Cloud_First, "First_Cloud", vp2);
	viewer->addPointCloud(cloud_second, Cloud_Second, "Second_Cloud", vp2);

	//viewer->spinOnce();
}

void AlignCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_first, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_second,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_afterTrans, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_after_add, Eigen::Matrix4f &final_transform, bool downsample = false)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr First_could(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Second_could(new pcl::PointCloud<pcl::PointXYZRGB>);

	//下采样
	pcl::VoxelGrid<PointT> grid;//建立体素格，将点云分配给局部三维网格中，对网格中的点云数据进行下采样
	if (downsample)
	{
		grid.setLeafSize(0.02, 0.02, 0.02);
		grid.setInputCloud(cloud_first);
		grid.filter(*First_could);
		grid.setInputCloud(cloud_second);
		grid.filter(*Second_could);
	}
	else
	{
		First_could = cloud_first;
		Second_could = cloud_second;
	}

	//计算曲面法线和曲率
	PointCloudWithNormals::Ptr PointCloudNormals_1(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr PointCloudNormals_2(new PointCloudWithNormals);
	pcl::NormalEstimation<PointT, PointNormalT> Cal_normal;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	Cal_normal.setSearchMethod(tree);
	Cal_normal.setKSearch(30);
	Cal_normal.setInputCloud(First_could);
	Cal_normal.compute(*PointCloudNormals_1);
	//复制点云坐标到创建的PointCloudNormals_1中
	pcl::copyPointCloud(*First_could, *PointCloudNormals_1);
	Cal_normal.setInputCloud(Second_could);
	Cal_normal.compute(*PointCloudNormals_2);
	pcl::copyPointCloud(*Second_could, *PointCloudNormals_2);

	//创建自定义类实例
	MyPointRepresentation Point_present;
	float weight_value[4] = { 1.0, 1.0, 1.0, 1.0 };
	Point_present.setRescaleValues(weight_value);//用于缩放

												 //创建非线性ICP对象
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> icp;
	icp.setTransformationEpsilon(1e-7);//容许最大误差
	icp.setMaxCorrespondenceDistance(0.1);//设置对应点之间的最大距离，0.1即为0.1m，配准过程中，大于该阈值点忽略
	icp.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(Point_present));
	icp.setInputCloud(PointCloudNormals_1);
	icp.setInputTarget(PointCloudNormals_2);
	icp.setMaximumIterations(10);//内部优化的迭代次数

	Eigen::Matrix4f MatrixTran = Eigen::Matrix4f::Identity(), prev, sourceToTarget;
	PointCloudWithNormals::Ptr icp_result = PointCloudNormals_1;
	int iter_num = 30;
	for (int i = 0; i < iter_num; ++i)
	{
		PCL_INFO("Iteration No. %d. \n", i);
		PointCloudNormals_1 = icp_result;
		icp.setInputCloud(PointCloudNormals_1);
		icp.align(*icp_result);
		MatrixTran = icp.getFinalTransformation()*MatrixTran;//每次迭代的变换矩阵都相乘，累积
															 //如果此次转换和前面累积转换的误差小于阈值，则通过减小最大对应距离来改善
		if (fabs((icp.getLastIncrementalTransformation() - prev).sum()) < icp.getTransformationEpsilon())
			icp.setMaxCorrespondenceDistance(icp.getMaxCorrespondenceDistance() - 0.01);
		prev = icp.getLastIncrementalTransformation();
		cout << prev << endl;
		//showAftReg(PointCloudNormals_2, PointCloudNormals_2);
	}
	//计算从源点云到目标点云
	sourceToTarget = MatrixTran;
	pcl::transformPointCloud(*cloud_first, *cloud_afterTrans, sourceToTarget);
	viewer->removePointCloud("First_Cloud");
	viewer->removePointCloud("Second_Cloud");
	viewer->addPointCloud(cloud_afterTrans, "First_Cloud", vp2);
	viewer->addPointCloud(cloud_first, "Second_Cloud", vp2);
	*cloud_after_add = *cloud_afterTrans + *cloud_second;
	final_transform = sourceToTarget;
}

int main()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_first(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_second(new pcl::PointCloud<pcl::PointXYZRGB>);
	/*string filepath1 = "F:/DesktopFile_Pointget/1-DATA/2017-10-18/2017-1-after/A2017-1After_2017-7.pcd";
	string filepath2 = "F:/DesktopFile_Pointget/1-DATA/2017-10-18/2017-1-after/A2017-1After_2017-8.pcd";*/
	string filepath1 = "F:/1-DesktopFile_Pointget/1-DATA/2017.11.11/50_5_2After/After_2017.11.11_1.pcd";
	string filepath2 = "F:/1-DesktopFile_Pointget/1-DATA/2017.11.11/50_5_2After/After_2017.11.11_7.pcd";
	//string filepath1 = "F:/1-DesktopFile_Pointget/1-DATA/xlr data/New folder/pot_id_2_scan_0.pcd";
	//string filepath2 = "F:/1-DesktopFile_Pointget/1-DATA/xlr data/New folder/pot_id_2_scan_1.pcd";


	int error1 = pcl::io::loadPCDFile(filepath1, *cloud_first);
	int error2 = pcl::io::loadPCDFile(filepath2, *cloud_second);
	if (error1 == -1)
	{
		PCL_WARN("Haven't load the Cloud First(The source one)!");
		return -1;
	}
	if (error2 == -1)
	{
		PCL_WARN("Haven't load the Cloud Second(The Target one)!");
		return -1;
	}
	PCL_INFO("Loaded");

	viewer->createViewPort(0.0, 0, 0.5, 1.0, vp1);
	viewer->createViewPort(0.5, 0, 1.0, 1.0, vp2);
	viewer->initCameraParameters();
	viewer->setBackgroundColor(0, 0, 0);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud_after_Reg(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud_comblined(new pcl::PointCloud<pcl::PointXYZRGB>);
	Eigen::Matrix4f Trans_matrix_global = Eigen::Matrix4f::Identity(), Trans_matrix;
	showPreReg(cloud_first, cloud_second);
	AlignCloud(cloud_first, cloud_second, Cloud_after_Reg, Cloud_comblined, Trans_matrix, false);
	/*pcl::transformPointCloud(*Cloud_after_Reg, *Cloud_after_Reg, Trans_matrix_global);
	Trans_matrix_global = Trans_matrix*Trans_matrix_global;*/
	cout << "Cloud_comblined->size()" << Cloud_comblined->size() << endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}