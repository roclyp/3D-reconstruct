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
	
	//MLS处理
	time_t t3 = GetTickCount();
	pcl::PointCloud<PointXYZRGB>::Ptr tempfinal(new pcl::PointCloud<PointXYZRGB>);
	*tempfinal = *cloud1_target + *tricloud2xyzrgb;
	pcl::PointCloud<PointXYZRGB>::Ptr filtercloud(new pcl::PointCloud<PointXYZRGB>);

	string PolynomialFit = param["PolynomialFit"];
	istringstream leafsizestr(param["leafsize"]), PolynomialOrderstr(param["PolynomialOrder"]),
		SearchRadiusstr(param["SearchRadius"]);
	double leafsize, PolynomialOrder, SearchRadius;
	PolynomialOrderstr >> PolynomialOrder;
	SearchRadiusstr >> SearchRadius;
	leafsizestr >> leafsize;
	
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(tempfinal);
	sor.setLeafSize(leafsize, leafsize, leafsize);
	sor.filter(*filtercloud);

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search(new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
	search->setInputCloud(filtercloud);

	mls.setInputCloud(filtercloud);
	if (PolynomialFit == "true")
		mls.setPolynomialFit(true);
	else
		mls.setPolynomialFit(false);
	mls.setPolynomialOrder(PolynomialOrder);//一般2-5
	mls.setSearchMethod(search);
	mls.setSearchRadius(SearchRadius);//越大的话平滑力度越大
	mls.process(*result);

	boost::shared_ptr< pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	viewer->setBackgroundColor(0.5, 0.5, 0.5);
	viewer->addPointCloud(cloud1_target,"cloud1_before_adjust");
	viewer->addPointCloud(cloud2_source, "cloud2_before_adjust");
	pcl::PolygonMesh mesh1, mesh2, colormesh1;
	mypara paraall;
	getPara(paraall);
	greedy_triangle(cloud1_target, mesh1, paraall);
	greedy_triangle(cloud2_source, mesh2, paraall);
	getcolormesh(mesh1, cloud1_target, colormesh1);
	viewer->addPolygonMesh(colormesh1, "cloud1mesh_Color");
	viewer->addPolygonMesh(mesh2, "cloud2mesh");

	//处理后两个点云结合在一起
		//pcl::io::savePCDFile("C:\\Users\\zhihong\\Desktop\\cup\\2019.1.7\\after4.pcd", *combineafter);
	//pcl::io::savePCDFile("C:\\Users\\zhihong\\Desktop\\cup\\2019.1.7\\cloud1onlypara.pcd", *mytriangles1->cloudxyzrgb);
	//pcl::io::savePCDFile("C:\\Users\\zhihong\\Desktop\\cup\\2019.1.7\\cloud2adjustonlypara.pcd", *cloud2_adjust);
	//pcl::io::savePCDFile("C:\\Users\\zhihong\\Desktop\\cup\\2019.1.7\\before4.pcd", *combinebefore);

	boost::shared_ptr< pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer);
	viewer2->setBackgroundColor(0.5, 0.5, 0.5);
	viewer->addPointCloud(result, "resultpoint");
	pcl::PolygonMesh resultmesh,resultcolormesh;
	greedy_triangle(result, resultmesh, paraall);
	getcolormesh(resultmesh, result, resultcolormesh);
	viewer2->addPolygonMesh(resultcolormesh, "resultmesh");

	time_t t4 = GetTickCount();
	cout << "MLS Use time: " << ((t4 - t3)*1.0 / 1000) << " s" << endl;
	cout << "Finished!..." << endl;
	while (!viewer->wasStopped() && !viewer2->wasStopped() /*&& !viewer3->wasStopped()*/)
	{
		viewer->spinOnce(100);
		viewer2->spinOnce(100);
		//viewer3->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	getchar();
}
