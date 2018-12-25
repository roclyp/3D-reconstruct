#include "PCLlibrary.h"
#include <pcl/console/time.h>   // TicToc
#include <pcl/search/kdtree.h>
#include "getfile.h"
//#include "obbBox.h"
#include "triangleinter.h"

using namespace std;

#define Pi 3.141592657;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBPtr;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointXYZPtr;

typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;


//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

//读取配置文件

void downsampling(const PointXYZRGBPtr &cloudin, PointXYZRGBPtr &outpoint, double &leafsize)
{
	pcl::VoxelGrid<pcl::PointXYZRGB> sor1;
	sor1.setInputCloud(cloudin);
	sor1.setLeafSize(leafsize, leafsize, leafsize);
	sor1.filter(*outpoint);
}

fpfhFeature::Ptr compute_fpfh_feature(PointXYZRGBPtr incloud, int ksearchnum, int openmpcores)
{
	//建立kdtree
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	//法向量
	pointnormal::Ptr point_normal(new pointnormal);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> est_normal;
	est_normal.setInputCloud(incloud);
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(ksearchnum);
	est_normal.compute(*point_normal);
	//fpfh 估计
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	//pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_target_fpfh;
	pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(openmpcores); //指定4核计算
	// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZ> ());
	est_fpfh.setInputCloud(incloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	est_fpfh.setKSearch(ksearchnum);
	est_fpfh.compute(*fpfh);

	return fpfh;

}


void ICPregister(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud1_target1, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &align,
	Eigen::Matrix4f &icptransformation, int &ksearchnum, double &MaxCorrespondenceDistance, int &MaximumIterations,
	int &manulIterations, double &TransformationEpsilon, double &EuclideanFitnessEpsilon)
{
	// Compute surface normals and curvature
	PointCloudWithNormals::Ptr points_with_normals_srccloud2(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgtcloud1(new PointCloudWithNormals);

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> norm_est;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(ksearchnum);

	norm_est.setInputCloud(align);
	norm_est.compute(*points_with_normals_srccloud2);
	pcl::copyPointCloud(*align, *points_with_normals_srccloud2);

	norm_est.setInputCloud(cloud1_target1);
	norm_est.compute(*points_with_normals_tgtcloud1);


	//
	// Align

	pcl::IterativeClosestPointWithNormals<PointNormalT, PointNormalT> reg;
	
	// Set the point representation
	reg.setInputSource(points_with_normals_srccloud2);
	reg.setInputTarget(points_with_normals_tgtcloud1);
	reg.setTransformationEpsilon(TransformationEpsilon);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
	reg.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);
	reg.setMaximumIterations(MaximumIterations);
	//	PointCloudWithNormals::Ptr reg_result = points_with_normals_srccloud2;
	PointCloudWithNormals::Ptr reg_result(new PointCloudWithNormals);
	reg.align(*reg_result);
	icptransformation = reg.getFinalTransformation();
	// Run the same optimization in a loop and visualize the results
	//Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, sourceToTarget;// targetToSource;


	//for (int i = 0; i < 30; ++i)
	//{
	//	PCL_INFO("Iteration Nr. %d.\n", i);

	//	// save cloud for visualization purpose
	//	points_with_normals_srccloud2 = reg_result;

	//	// Estimate
	//	reg.setInputSource(points_with_normals_srccloud2);
	//	reg.align(*reg_result);

	//	//accumulate transformation between each Iteration
	//	Ti = reg.getFinalTransformation() * Ti;

	//	//if the difference between this transformation and the previous one
	//	//is smaller than the threshold, refine the process by reducing
	//	//the maximal correspondence distance
	//	if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
	//		reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);
	//	prev = reg.getLastIncrementalTransformation();
	//}
	//
  // Get the transformation from target to source
	//targetToSource = Ti.inverse();
	/*sourceToTarget = Ti;
	icptransformation = sourceToTarget;*/
}

int main()
{
	cout << "Ready.....";
	getchar();
	cout << "Start working" << endl;
	time_t t1 = GetTickCount();
	map<string, string> param;
	ConfigFileRead(param);
	string format = param["format"];
	string filepath1 = param["filepath1"];
	string filepath2 = param["filepath2"];
	string filepathCircle = param["filepath_Circle"];
	string outfilepath = param["outfilepath"];
	string outfilepathCircle = param["outfilepath_Circle"];

	istringstream ksearchnumstr(param["ksearchnum"]), openmpcoresstr(param["openmpcores"]),
		CorrespondenceRandomnessstr(param["CorrespondenceRandomness"]), leafsizestr(param["leafsize"]),
		MaximumIterationsstr(param["MaximumIterations"]), manulIterationsstr(param["manulIterations"]),
		MaxCorrespondenceDistancestr(param["MaxCorrespondenceDistance"]), TransformationEpsilonstr(param["TransformationEpsilon"]),
		EuclideanFitnessEpsilonstr(param["EuclideanFitnessEpsilon"]);
	int ksearchnum, openmpcores, CorrespondenceRandomness, MaximumIterations, manulIterations;
	ksearchnumstr >> ksearchnum;
	openmpcoresstr >> openmpcores;
	CorrespondenceRandomnessstr >> CorrespondenceRandomness;
	MaximumIterationsstr >> MaximumIterations;
	manulIterationsstr >> manulIterations;
	double leafsize, MaxCorrespondenceDistance, TransformationEpsilon, EuclideanFitnessEpsilon;
	leafsizestr >> leafsize;
	MaxCorrespondenceDistancestr >> MaxCorrespondenceDistance;
	EuclideanFitnessEpsilonstr >> EuclideanFitnessEpsilon;
	TransformationEpsilonstr >> TransformationEpsilon;
	cout << "cloud1 Filepath: " << filepath1 << endl;
	cout << "cloud2 FIlepath: " << filepath2 << endl;

	//循环版本
	//vector<string> Allname;
	//vector< pcl::PointCloud<PointXYZRGB>::Ptr> AllCloud;
	//GetAllFiles_CertainFormat(filepathCircle, Allname, format);
	//cout << "File numbers: " << Allname.size() << endl;
	//for (int i = 0; i < Allname.size(); i++)
	//{
	//	pcl::PointCloud<PointXYZRGB>::Ptr cloudtemp(new pcl::PointCloud<PointXYZRGB>);
	//	pcl::io::loadPCDFile(Allname.at(i), *cloudtemp);
	//	AllCloud.push_back(cloudtemp);
	//}

	pcl::PointCloud<PointXYZRGB>::Ptr cloud1_target(new pcl::PointCloud<PointXYZRGB>);
	pcl::PointCloud<PointXYZRGB>::Ptr cloud2_source(new pcl::PointCloud<PointXYZRGB>);
	pcl::PointCloud<PointXYZRGB>::Ptr cloud1_target1(new pcl::PointCloud<PointXYZRGB>);
	pcl::PointCloud<PointXYZRGB>::Ptr cloud2_source2(new pcl::PointCloud<PointXYZRGB>);

	pcl::io::loadPCDFile(filepath1, *cloud1_target);
	pcl::io::loadPCDFile(filepath2, *cloud2_source);
	downsampling(cloud1_target, cloud1_target1, leafsize);
	downsampling(cloud2_source, cloud2_source2, leafsize);

	for (int i = 0; i < cloud2_source2->size(); i++)
	{
		cloud2_source2->at(i).r = 255;
		cloud2_source2->at(i).g = 0;
		cloud2_source2->at(i).b = 0;
	}
	cout << "cloud2 size: " << cloud2_source2->size() << endl;

	//FPFH 粗配准
	fpfhFeature::Ptr cloud2_source_fpfh = compute_fpfh_feature(cloud2_source2, ksearchnum, openmpcores);
	fpfhFeature::Ptr cloud1_target_fpfh = compute_fpfh_feature(cloud1_target1, ksearchnum, openmpcores);
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(cloud2_source2);
	sac_ia.setSourceFeatures(cloud2_source_fpfh);
	sac_ia.setInputTarget(cloud1_target1);
	sac_ia.setTargetFeatures(cloud1_target_fpfh);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr align(new pcl::PointCloud<pcl::PointXYZRGB>);
	//  sac_ia.setNumberOfSamples(20);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
	//sac_ia.setCorrespondenceRandomness(CorrespondenceRandomness); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
	sac_ia.align(*align);
	//align = cloud2_source2;
	Eigen::Matrix4f fpfhtransformation = sac_ia.getFinalTransformation();
	//	pcl::transformPointCloud(*cloud2_source2, *align, finaltrans);
		//精配准
	Eigen::Matrix4f icptransformation;
	ICPregister(cloud1_target1, align, icptransformation, ksearchnum, MaxCorrespondenceDistance,
		MaximumIterations, manulIterations, TransformationEpsilon, EuclideanFitnessEpsilon);
	pcl::PointCloud<PointXYZRGB>::Ptr Final(new pcl::PointCloud<PointXYZRGB>);
	pcl::transformPointCloud(*align, *Final, icptransformation);
	for (int i = 0; i < align->size(); i++)
	{
		align->at(i).r = 0;
		align->at(i).g = 255;
		align->at(i).b = 0;
	}
	for (int i = 0; i < Final->size(); i++)
	{
		Final->at(i).r = 0;
		Final->at(i).g = 0;
		Final->at(i).b = 255;
	}
	//上述点云均为downsampling的结果
	//三角干涉需要处理原始点云以及旋转后的点云
	pcl::PointCloud<PointXYZRGB>::Ptr tricloud2xyzrgb(new pcl::PointCloud<PointXYZRGB>);
	pcl::transformPointCloud(*cloud2_source, *tricloud2xyzrgb, fpfhtransformation*icptransformation);
	cout << "Registeration is over.....";
	time_t t2 = GetTickCount();
	cout << "Registeration Use time: " << ((t2 - t1)*1.0 / 1000) << " s" << endl;
	cout << "--------------------------";
	

	//处理三角干涉问题
	getchar();
	//获取参数
	cout << "Triangle interation emilation is working..." << endl;
	time_t t3 = GetTickCount();
	mypara paraall;
	getPara(paraall);
	int neiborTrinum = paraall.neiborTrinum;
	double neiborTriRadius = paraall.neiborTriRadius;
	string neibormethod = paraall.neibormethod;
	//读取点云
	TrianglePlane::Ptr mytriangles1(new TrianglePlane);
	TrianglePlane::Ptr mytriangles2(new TrianglePlane);
	//pcl::PointCloud<PointXYZ>::Ptr Cloud1xyz(new pcl::PointCloud<PointXYZ>);
	//pcl::PointCloud<PointXYZRGB>::Ptr Cloud1xyzrgb(new pcl::PointCloud<PointXYZRGB>);
	pcl::copyPointCloud(*cloud1_target, *mytriangles1->cloudxyzrgb);
	pcl::copyPointCloud(*cloud1_target, *mytriangles1->cloudxyz);
	pcl::copyPointCloud(*tricloud2xyzrgb, *mytriangles2->cloudxyzrgb);
	pcl::copyPointCloud(*tricloud2xyzrgb, *mytriangles2->cloudxyz);

	//视点旋转
	pcl::PointCloud<pcl::PointXYZ>::Ptr ZeroPoint(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ zerop;
	pcl::PointXYZ zerop_trans;
	zerop.x = 0; zerop.y = 0; zerop.z = 0;
	ZeroPoint->push_back(zerop);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ZeroPoint_Trans(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Matrix4f rotateMatrixinverse = fpfhtransformation * icptransformation;
	pcl::transformPointCloud(*ZeroPoint, *ZeroPoint_Trans, rotateMatrixinverse);
	//添加视点到对应点云的triplane类型中
	mytriangles1->viewpoints = ZeroPoint;
	mytriangles2->viewpoints = ZeroPoint_Trans;

	//pcl::PolygonMesh triangles1;
	//pcl::PolygonMesh triangles2;
	greedy_triangle(mytriangles1->cloudxyz, mytriangles1->triangle, paraall);
	greedy_triangle(mytriangles2->cloudxyz, mytriangles2->triangle, paraall);

	vector<MyTriangles> cloud1_cen_tri;
	vector<MyTriangles> cloud2_cen_tri;
	//计算三角面片中心坐标点然后根据该坐标点重新建立中心的点云，每个中心点与三角面片进行映射，
	//	后续寻找三角面片邻近的三角面片，然后计算交叉干涉情况
	/*map<pcl::PointXYZ, pcl::Vertices> cloud1_cen_tri;
	map<pcl::PointXYZ, pcl::Vertices> cloud2_cen_tri;*/
	pcl::PointCloud<PointXYZ>::Ptr cloud1_cen;
	pcl::PointCloud<PointXYZ>::Ptr cloud2_cen;
	for (int i = 0; i < mytriangles1->triangle.polygons.size(); i++)
	{
		MyTriangles temp(mytriangles1->cloudxyzrgb, mytriangles1->triangle.polygons.at(i),i);
		cloud1_cen_tri.push_back(temp);
		cloud1_cen->push_back(temp.centerPoint);
	}
	for (int i = 0; i < mytriangles2->triangle.polygons.size(); i++)
	{
		MyTriangles temp(mytriangles2->cloudxyzrgb, mytriangles2->triangle.polygons.at(i),i);
		cloud2_cen_tri.push_back(temp);
		cloud2_cen->push_back(temp.centerPoint);
	}

	////存储三角面片干涉关系后续可以通过图的方式来优化缩小存储空间，并进行提速，目前先用链表或vector来进行存储
	map<pcl::Vertices, vector<pcl::Vertices>> triInter_cloud1;
	map<pcl::Vertices, vector<pcl::Vertices>> triInter_cloud2;
	getCenNeiborTri(cloud1_cen, cloud2_cen, cloud1_cen_tri, cloud2_cen_tri);
	getCenNeiborTri(cloud2_cen, cloud1_cen, cloud2_cen_tri, cloud1_cen_tri);

	//mytriangles1->triangle = triangles1;
	//mytriangles2->triangle = triangles2;
	//getspaceplane(*mytriangles1);
	//getspaceplane(*mytriangles2);



	////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(cloud1, 250, 0, 0);
	////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color2(cloud2, 0, 255, 0);
	////viewer->addPointCloud<pcl::PointXYZ>(cloud1, sources_cloud_color,"cloud1");
	////viewer->addPolygonMesh(triangles1,"mesh1");
	////viewer->addPointCloud<pcl::PointXYZ>(cloud2_tran, sources_cloud_color2, "cloud2");
	////viewer->addPolygonMesh(triangles2, "mesh2");
	////viewer->addPointCloud(cloud1_RGB, "cloud1");
	////viewer->addPointCloud(cloud2Tran_RGB, "cloud2");

	//vector<interSituation> interferenceCloud1;
	//vector<interSituation> interferenceCloud2;
	//getInterference(mytriangles1->cloudxyz, mytriangles2, mytriangles2->cloudxyz, ZeroPoint->at(0), interferenceCloud1);
	//cout << "Cloud2 —————————————————————————————— Cloud2" << endl;
	//getInterference(mytriangles2->cloudxyz, mytriangles1, mytriangles1->cloudxyz, ZeroPoint_Trans->at(0), interferenceCloud2);






	//boost::shared_ptr< pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	//viewer->setBackgroundColor(0.5, 0.5, 0.5);
	//viewer->addPointCloud(cloud1_target1, "cloud1");
	//viewer->addPointCloud(cloud2_source2, "cloud2");
	//viewer->addPointCloud(align, "cloud2afterregist");

	//pcl::PolygonMesh mesh1,mesh2,mesh3,mesh4;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr clou1xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::copyPointCloud(*cloud1_target1, *clou1xyz);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr clou2xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::copyPointCloud(*cloud2_source2, *clou2xyz);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr alignxyz(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::copyPointCloud(*align, *alignxyz);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr finalxyz(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::copyPointCloud(*Final, *finalxyz);
	//greedy_triangle(clou1xyz, mesh1, paraall);
	//greedy_triangle(clou2xyz, mesh2, paraall);
	//greedy_triangle(alignxyz, mesh3, paraall);
	//greedy_triangle(finalxyz, mesh4, paraall);
	//pcl::PolygonMesh colormesh2, colormesh3,colormesh4;
	//getcolormesh(mesh2, cloud2_source2,colormesh2);
	//getcolormesh(mesh3, align, colormesh3);
	//getcolormesh(mesh4, Final, colormesh4);
	//viewer->addPolygonMesh(mesh1, "m1");
	////viewer->addPolygonMesh(colormesh2, "cm2");
	//viewer->addPolygonMesh(colormesh3, "cm3");
	//viewer->addPolygonMesh(colormesh4, "cm4");

	//boost::shared_ptr< pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer);
	//viewer2->setBackgroundColor(0.5, 0.5, 0.5);


	//time_t t4 = GetTickCount();
	//cout << "Trianglation interation Use time: " << ((t3 - t4)*1.0 / 1000) << " s" << endl;
	//cout << "Finished!..." << endl;
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	viewer2->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}

	getchar();
}
