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
#include <boost/make_shared.hpp>
#include <pcl/point_representation.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/transforms.h>
#include <fstream>
using namespace std;
//#include <pcl/registration/icp.h>
//建立可视化窗口，并命名，可定义为全局指针，保证可全局使用
pcl::visualization::PCLVisualizer *viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//建立左右视窗进行比较
int vp1, vp2;
//简单类型定义
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//点云三维重建过程：点云数据预处理、分割、三角网格化、网格渲染

//什么结构体，对点云对象进行成对处理管理
struct PCLFile
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	std::string f_name;
	PCLFile() :cloud(new pcl::PointCloud<pcl::PointXYZRGB>){};
};

//以< x, y, z, curvature >形式定义一个新的点表示（x，y，z，曲率）
class MyPointRepresentation :public pcl::PointRepresentation <PointNormalT>
{
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
	MyPointRepresentation()
	{
		nr_dimensions_ = 4;//定义点的维度
	}
	//重载copyToFloatArray方法来将点转化为4维数组
	virtual void copyToFloatArray(const PointNormalT &p, float* out) const
	{
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};
//左视图显示,显示源点云和目标点晕
void showCloudsLeft(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source)
{
	//viewer->removePointCloud("vp1_target");
	//viewer->removePointCloud("vp1_source");
	viewer->addPointCloud(cloud_target, "vp1_target", vp1);
	viewer->addPointCloud(cloud_source, "vp1_source", vp1);
}
//右视图显示
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
	//viewer->removePointCloud("source");
	//viewer->removePointCloud("target");
	pcl::visualization::PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(cloud_target, "curvature");
	if (!tgt_color_handler.isCapable())
		PCL_WARN("Cannot create curvature color handler!");
	pcl::visualization::PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(cloud_source, "curvature");
	if (!src_color_handler.isCapable())
		PCL_WARN("Cannot create curvature color handler!");
	viewer->addPointCloud(cloud_target, tgt_color_handler, "target", vp2);
	viewer->addPointCloud(cloud_source, src_color_handler, "source", vp2);
	//viewer->spinOnce();
}
////加载数据
//void loadData(std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
//{
//	std::string extension(".pcd");
//	//假定第一个参数是实际测试模型
//	for (int i = 1; i < argc; i++)
//	{
//		std::string fname = std::string(argv[i]);
//		// 至少需要5个字符长（因为.plot就有 5个字符）
//		if (fname.size() <= extension.size())
//			continue;
//		std::transform(fname.begin(), fname.end(), fname.begin(), (int(*)(int))tolower);
//		//检查参数是一个pcd文件
//		if (fname.compare(fname.size() - extension.size(), extension.size(), extension) == 0)
//		{
//			//加载点云并保存在总体的模型列表中
//			PCD m;
//			m.f_name = argv[i];
//			pcl::io::loadPCDFile(argv[i], *m.cloud);
//			//从点云中移除NAN点
//			std::vector<int> indices;
//			pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);
//			models.push_back(m);
//		}
//	}
//}



//两两匹配
void pairAlign(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
	//为了一致性和高速的下采样
	//注意：为了大数据集需要允许这项
	PointCloud::Ptr src(new PointCloud);
	PointCloud::Ptr tgt(new PointCloud);
	pcl::VoxelGrid<PointT> grid; //把一个给定的点云，聚集在一个局部的3D网格上, 并下采样和滤波点云数据
	//下采样
	if (downsample)
	{
		grid.setLeafSize(0.05, 0.05, 0.05);
		grid.setInputCloud(cloud_src);
		grid.filter(*src);
		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
	}

	//计算曲面法线和曲率
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);//创建源点云指针（注意点的类型包含坐标和法向量）
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);//创建目标点云指针（注意点的类型包含坐标和法向量）
	pcl::NormalEstimation<PointT, PointNormalT> norm_est;//该对象用于计算法向量
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>()); //创建kd树，用于计算法向量的搜索方法
	norm_est.setSearchMethod(tree);//设置搜索方法
	norm_est.setKSearch(30); // 设置最近邻的数量
	norm_est.setInputCloud(src);//设置输入云
	norm_est.compute(*points_with_normals_src); //计算法向量，并存储在points_with_normals_src
	pcl::copyPointCloud(*src, *points_with_normals_src); //复制点云（坐标）到points_with_normals_src（包含坐标和法向量）
	norm_est.setInputCloud(tgt);//这3行计算目标点云的法向量，同上
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);


	//创建一个自定义点的表达方式 实例
	MyPointRepresentation point_representation;
	//加权曲率维度，以和坐标xyz保持平衡
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);//设置缩放值（向量化点时使用）

	//创建非线性ICP对象 并设置参数
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;//创建非线性ICP对象（ICP变体，使用Levenberg-Marquardt最优化）
	reg.setTransformationEpsilon(1e-6);//设置容许的最大误差（迭代最优化）
	//***** 注意：根据自己数据库的大小调节该参数

	reg.setMaxCorrespondenceDistance(0.0001);//设置对应点之间的最大距离（0.01m）,在配准过程中，忽略大于该阈值的点
	//设置点表示
	reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));
	//设置源点云和目标点云
	reg.setInputSource(points_with_normals_src); // 设置输入点云（待变换的点云）
	reg.setInputTarget(points_with_normals_tgt); // 设置目标点云
	reg.setMaximumIterations(30); //设置内部优化的迭代次数

	//在一个循环中运行相同的最优化并且使结果可视化
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src; //用于存储结果（坐标+法向量）
	reg.setMaximumIterations(30);
	for (int i = 0; i < 30; ++i)//迭代
	{
		PCL_INFO("Iteration Nr. %d.\n", i); //命令行显示迭代的次数
		//保存点云，用于可视化
		points_with_normals_src = reg_result;
		//估计
		reg.setInputSource(points_with_normals_src); //重新设置输入点云（待变换的点云），因为经过上一次迭代，已经发生变换了
		reg.align(*reg_result);//对齐（配准）两个点云

		Ti = reg.getFinalTransformation() * Ti; //累积（每次迭代的）变换矩阵
		//如果这次转换和之前转换之间的差异小于阈值
		//则通过减小最大对应距离来改善程序
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.00001);//减小对应点之间的最大距离（上面设置过）
		prev = reg.getLastIncrementalTransformation();//上一次变换的误差
		//显示当前配准状态，在窗口的右视区，简单的显示源点云和目标点云
		//showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	}
	//
	//计算从目标点云到源点云的变换矩阵
	targetToSource = Ti.inverse();
	//
	//将目标点云 变换回到 源点云帧
	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

	viewer->addPointCloud(output, "target", vp2);
	viewer->addPointCloud(cloud_src, "source", vp2);

	PCL_INFO("Press q to continue the registration.\n");

	//添加源点云到转换目标
	*output += *cloud_src; // 拼接点云图（的点）点数数目是两个点云的点数和
	final_transform = targetToSource; //最终的变换。目标点云到源点云的变换矩阵

}

int Saveplyfile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud, std::string pathname)
{
	ofstream corandcolor(pathname);
	corandcolor << "ply"
		<< "\n" << "format ascii 1.0"
		<< "\n" << "comment PCL generated"
		<< "\n" << "element vertex " << result_cloud->size()
		<< "\n" << "property float x"
		<< "\n" << "property float y"
		<< "\n" << "property float z"
		<< "\n" << "property uchar red"
		<< "\n" << "property uchar green"
		<< "\n" << "property uchar blue"
		<< "\n" << "property uchar alpha"
		<< "\n" << "element camera 1"
		<< "\n" << "property float view_px"
		<< "\n" << "property float view_py"
		<< "\n" << "property float view_pz"
		<< "\n" << "property float x_axisx"
		<< "\n" << "property float x_axisy"
		<< "\n" << "property float x_axisz"
		<< "\n" << "property float y_axisx"
		<< "\n" << "property float y_axisy"
		<< "\n" << "property float y_axisz"
		<< "\n" << "property float z_axisx"
		<< "\n" << "property float z_axisy"
		<< "\n" << "property float z_axisz"
		<< "\n" << "property float focal"
		<< "\n" << "property float scalex"
		<< "\n" << "property float scaley"
		<< "\n" << "property float centerx"
		<< "\n" << "property float centery"
		<< "\n" << "property int viewportx"
		<< "\n" << "property int viewporty"
		<< "\n" << "property float k1"
		<< "\n" << "property float k2"
		<< "\n" << "end_header" << endl;
	for (size_t i = 0; i < result_cloud->size(); ++i)
	{
		corandcolor << result_cloud->points[i].x << " " << result_cloud->points[i].y << " " << result_cloud->points[i].z << " " <<
			(int)result_cloud->points[i].r << " " << (int)result_cloud->points[i].g << " " << (int)result_cloud->points[i].b << " " << 255 << endl;
	}
	corandcolor << "0 0 0 1 0 0 0 1 0 0 0 1 0 0 0 0 0 " << result_cloud->size() << " 1 0 0" << endl;
	return 0;
}

int main()
{
	//导入点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_a(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZRGB>);
	int errors = pcl::io::loadPCDFile("F:/DesktopFile_Pointget/1-DATA/2017-10-18/2017-1-after/A2017-1After_2017-7.pcd", *cloud_a);
	int errors2 = pcl::io::loadPCDFile("F:/DesktopFile_Pointget/1-DATA/2017-10-18/2017-1-after/A2017-1After_2017-8.pcd", *cloud_b);
	if (errors == -1)
	{
		cout << "Can not find the file" << endl;
		return -1;
	}
	cout << "loaded" << endl;

	cout << "Set Visualization" << endl;

	////建立左右视窗进行比较
	viewer->createViewPort(0.0, 0, 0.5, 1.0, vp1);
	viewer->createViewPort(0.5, 0, 1.0, 1.0, vp2);
	viewer->initCameraParameters();//设置相机参数，用户从默认的角度和方向观察点云
	viewer->setBackgroundColor(0, 0, 0);
	//int vp1,vp2;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;
	showCloudsLeft(cloud_b, cloud_a);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	//两两配对转换

	pairAlign(cloud_a, cloud_b, temp, pairTransform, true);
	//更新全局变换
	pcl::transformPointCloud(*temp, *result_cloud, GlobalTransform);
	//showCloudsRight(cloud_b, result_cloud);

	GlobalTransform = pairTransform * GlobalTransform;
	//保存配准对，转换到第一个点云框架中
	//std::stringstream pathname;
	//pathname <<  "C:/Users/user/Desktop/test/object2/result2x.ply";
	////pcl::io::savePLYFile(ss.str(), *result_cloud, true);
	//Saveplyfile(result_cloud,pathname.str());
	cout << "result_cloud->size()" << result_cloud->size() << endl;

	//调用spinonce函数给视窗处理时间的时间，允许鼠标键盘等交互操作
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
