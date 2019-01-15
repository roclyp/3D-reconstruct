#include "MyClassType.h"

//降采样
void downsampling(const PointCloud<PointXYZRGB>::Ptr &cloudin, PointCloud<PointXYZRGB>::Ptr &outpoint, double &leafsize)
{
	pcl::VoxelGrid<pcl::PointXYZRGB> sor1;
	sor1.setInputCloud(cloudin);
	sor1.setLeafSize(leafsize, leafsize, leafsize);
	sor1.filter(*outpoint);
}
//FPFH
fpfhFeature::Ptr compute_fpfh_feature(PointCloud<PointXYZRGB>::Ptr incloud, int ksearchnum, int openmpcores)
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

//SAC_IA粗配准
void sac_Align(const PointCloud<PointXYZRGB>::Ptr &cloud1, const fpfhFeature::Ptr &fpfh1,
	const PointCloud<PointXYZRGB>::Ptr &cloud2, const fpfhFeature::Ptr &fpfh2, const mypara &para,
	Eigen::Matrix4f &fpfhTransform)
{
	//cloud2是源点云，目标点云是cloud1，因为cloud1是坐标基准，要把cloud2转换到cloud1
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(cloud2);
	sac_ia.setSourceFeatures(fpfh2);
	sac_ia.setInputTarget(cloud1);
	sac_ia.setTargetFeatures(fpfh1);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr align(new pcl::PointCloud<pcl::PointXYZRGB>);
	//sac_ia.setNumberOfSamples(para.sacSamNum);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
	//sac_ia.setCorrespondenceRandomness(para.CorspRn); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
	sac_ia.align(*align);
	//align = cloud2_source2;
	fpfhTransform = sac_ia.getFinalTransformation();
}

//ICP-cloud1是目标，cloud2是源
void ICPregister(const PointCloud<PointXYZRGB>::Ptr &cloud1, const PointCloud<PointXYZRGB>::Ptr &cloud2,
	const mypara &para, Eigen::Matrix4f &icptransformation)
{
	// Compute surface normals and curvature
	PointCloud<PointNormal>::Ptr points_with_normals_cloud2(new PointCloud<PointNormal>);
	PointCloud<PointNormal>::Ptr points_with_normals_cloud1(new PointCloud<PointNormal>);

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> norm_est;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(para.Kicp);

	norm_est.setInputCloud(cloud2);
	norm_est.compute(*points_with_normals_cloud2);
	pcl::copyPointCloud(*cloud2, *points_with_normals_cloud2);

	norm_est.setInputCloud(cloud1);
	norm_est.compute(*points_with_normals_cloud1);
	pcl::copyPointCloud(*cloud1, *points_with_normals_cloud1);

	// Align
	pcl::IterativeClosestPointWithNormals<PointNormal, PointNormal> reg;

	// Set the point representation
	reg.setInputSource(points_with_normals_cloud2);
	reg.setInputTarget(points_with_normals_cloud1);
	reg.setTransformationEpsilon(para.TransformationEpsilon);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance(para.MaxCorrespondenceDistance);
	reg.setEuclideanFitnessEpsilon(para.EuclideanFitnessEpsilon);
	reg.setMaximumIterations(para.MaximumIterations);
	//PointCloudWithNormals::Ptr reg_result = points_with_normals_srccloud2;
	PointCloud<PointNormal>::Ptr reg_result(new PointCloud<PointNormal>);
	reg.align(*reg_result);
	icptransformation = reg.getFinalTransformation();
	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, sourceToTarget;// targetToSource;
	Ti = reg.getFinalTransformation();// .inverse();

	//for (int i = 0; i < para.manulIterations; ++i)
	//{
	//	PCL_INFO("Iteration Nr. %d.\n", i);
	//	// save cloud for visualization purpose
	//	points_with_normals_cloud2 = reg_result;

	//	// Estimate
	//	reg.setInputSource(points_with_normals_cloud1);
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
	
  // Get the transformation from target to source
	//targetToSource = Ti.inverse();
	icptransformation = Ti;
	/*sourceToTarget = Ti;
	icptransformation = sourceToTarget;*/
}

//三角化放在头文件中，因为是模板函数
//获取彩色三角面片
pcl::PolygonMesh getcolormesh(const PointCloud<PointXYZRGB>::Ptr &cloud1_RGB,
	const PolygonMesh &triangles1,int Kcolor)
{
	PolygonMesh meshout = triangles1;
	PointCloud<PointXYZRGB>::Ptr cloud_color_mesh(new PointCloud<PointXYZRGB>);
	pcl::fromPCLPointCloud2(triangles1.cloud, *cloud_color_mesh);
	pcl::KdTreeFLANN<PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloud1_RGB);
	// K nearest neighbor search
	int K = Kcolor;
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
	return meshout;
}
//void trianglulation();

void CloudCenVers2Pcloud(const CloudCenVers &cloudCenVer, PointCloud<PointXYZ>::Ptr &outcloud)
{
	for (int i=0;i<cloudCenVer.interAll.size();i++)
	{
		PointXYZ temp = cloudCenVer.interAll.at(i).cenPoint;
		outcloud->push_back(temp);
	}
}

void getNeiborTriangle(CloudCenVers &cloudCenVer2, const CloudCenVers &cloudCenVer1 ,mypara &para)
{
	//先将自定义类型中的点转化为点云
	PointCloud<PointXYZ>::Ptr cloud2(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud1(new PointCloud<PointXYZ>);
	CloudCenVers2Pcloud(cloudCenVer2, cloud2);
	CloudCenVers2Pcloud(cloudCenVer1, cloud1);
	
	//此时，三角面片中心点在中心点云中的序号与在mesh中的polygon的序号是一致的
	omp_set_num_threads(8);
#pragma omp parallel for
	for (int i = 0; i < cloud2->size(); i++)
	{
		//kdtree搜索点云2中三角面中心点c(i)在点云1中的邻近点，然后将该若干三角面片记录到点云2中
		std::vector<int> kdID;
		std::vector<float> kddis;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud1);
		if (tree->radiusSearch(cloud2->at(i), para.SearchRadius, kdID, kddis) > 0)
		{
			for (int j = 0; j < kdID.size(); j++)
			{
				int triID = kdID.at(j);
				//pcl::PointXYZ tripoint = cloud2->at(triID);
				pcl::Vertices tempvert = cloudCenVer1.interAll.at(triID).vertices;
				cloudCenVer2.interAll.at(i).neiborvers.push_back(tempvert);
				//cloud2_cen_tri[mapcloud];
				//pcl::Vertices tempvert = cloud2_cen_tri[mapcloud];
			}
		}
	}
}

void getCrosss_Parall(CenPointTri &temp, const PointCloud<PointXYZRGB>::Ptr cloud2trans,
	const PointCloud<PointXYZRGB>::Ptr cloud1)
{
	PointXYZRGB cloud2p1 = cloud2trans->at(temp.vertices.vertices[0]);//点云2该三角面片的顶点1
	PointXYZRGB cloud2p2 = cloud2trans->at(temp.vertices.vertices[1]);//点云2该三角面片的顶点2
	PointXYZRGB cloud2p3 = cloud2trans->at(temp.vertices.vertices[2]);//点云2该三角面片的顶点3
	for (int i = 0; i < temp.neiborvers.size; i++)
	{
		PointXYZRGB cloud1p1 = cloud1->at(temp.neiborvers.at(i).vertices[0]);
		PointXYZRGB cloud1p2 = cloud1->at(temp.neiborvers.at(i).vertices[1]);
		PointXYZRGB cloud1p3 = cloud1->at(temp.neiborvers.at(i).vertices[2]);
		Raytri cloudp1p2_c1p(cloud2p1, cloud2p2, cloud1p1, cloud1p2, cloud1p3);//选择cloud2的p1，p2点建立射线，cloud1的三个点建立平面
		Raytri cloudp1p3_c1p(cloud2p1, cloud2p3, cloud1p1, cloud1p2, cloud1p3);//选择cloud2的p1，p3点建立射线，cloud1的三个点建立平面
		Raytri cloudp2p3_c1p(cloud2p2, cloud2p3, cloud1p1, cloud1p2, cloud1p3);//选择cloud2的p2，p3点建立射线，cloud1的三个点建立平面
		//都是0时表示，平行
		if (Cross_triangle(cloudp1p2_c1p) == 0 && Cross_triangle(cloudp1p3_c1p) == 0 && Cross_triangle(cloudp2p3_c1p) == 0)
		{
			temp.Parallitervers.push_back(temp.neiborvers.at(i));
		}
		//其中有一条边是2时，相交，两点在三角面片的两侧
		else if (Cross_triangle(cloudp1p2_c1p) == 2 || Cross_triangle(cloudp1p3_c1p) == 2 || Cross_triangle(cloudp2p3_c1p) == 2)
		{
			temp.Cross2itervers.push_back(temp.neiborvers.at(i));
		}
		//此时全都不为2，此时只要有一条边为1，那么说明相交，且两点在三角面片的两侧
		else if (Cross_triangle(cloudp1p2_c1p) == 1 || Cross_triangle(cloudp1p3_c1p) == 1 || Cross_triangle(cloudp2p3_c1p) == 1)
		{
			temp.Cross1itervers.push_back(temp.neiborvers.at(i));
		}
		//其他情况说明三角面片不存在关系，则不处理
		else
			continue;
	}
}


void TriCross(CloudCenVers &cloudCenVer2, PointCloud<PointXYZRGB>::Ptr &cloud2trans,
	CloudCenVers &cloudCenVer1, PointCloud<PointXYZRGB>::Ptr &cloud1, mypara &para)
{
	//先得到初始的三角面片
	for (int i = 0; i < cloudCenVer2.interAll.size(); i++)
	{
		CenPointTri temp = cloudCenVer2.interAll.at(i);
		getCrosss_Parall(temp, cloud2trans, cloud1);
	}
}

void DealWithTri(CloudCenVers &cloudCenVer2, PointCloud<PointXYZRGB>::Ptr &cloud2trans,
	PointCloud<PointXYZRGB>::Ptr &cloud1, mypara &para)
{
	//处理交叉的且在两边的
	for (int i = 0; i < cloudCenVer2.interAll.size(); i++)
	{
		auto temp = cloudCenVer2.interAll.at(i);
		if (temp.Cross2itervers.size() == 0)
		{
			cout << "Triangle " << i << "has no 2 sides cross triangles" << endl;
			continue;
		}
		else
		{
			for (int j = 0; j < temp.Cross2itervers.size(); j++)
			{
				auto temp2 = temp.Cross2itervers.at(j);
				spaceLine crossLine;
				PointXYZRGB cloud2p1 = cloud2trans->at(temp2.vertices[0]);
				PointXYZRGB cloud2p2 = cloud2trans->at(temp2.vertices[1]);
				PointXYZRGB cloud2p3 = cloud2trans->at(temp2.vertices[2]);
				PointXYZRGB cloud1p1 = cloud1->at(temp2.vertices[0]);
				PointXYZRGB cloud1p2 = cloud1->at(temp2.vertices[1]);
				PointXYZRGB cloud1p3 = cloud1->at(temp2.vertices[2]);
				spacePlane plane2(cloud2p1, cloud2p2, cloud2p3);
				spacePlane plane1(cloud1p1, cloud1p2, cloud1p3);
				double doubleangle = getAngle(plane2, plane1, crossLine);
			}
		}
	}
	
}