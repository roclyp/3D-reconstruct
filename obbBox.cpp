#include "obbBox.h"

void getbox(const pcl::PointCloud<PointXYZRGB>::Ptr &cloudin, pcl::PointCloud<PointXYZ>::Ptr &cloudbox,
	double &xmax, double &xmin, double &ymax, double &ymin, double &zmax, double &zmin,
	pcl::PointCloud<PointXYZRGB>::Ptr cloudplane)
{
	double temxmax = -INT_MAX, temymax = -INT_MAX, temzmax = -INT_MAX, temxmin = INT_MAX, temymin = INT_MAX, temzmin = INT_MAX;
	PointXYZRGB zmaxpoint, zminpoint, xminpoint, xmaxpoint, ymaxpoint, yminpoint;
	omp_set_num_threads(8);
#pragma omp parallel for
	for (int i = 0; i < cloudin->size(); i++) {
		auto point = cloudin->at(i);
		if (point.x == 0 || point.y == 0 || point.z == 0)
			continue;
		else
		{
			if (point.x <= temxmin)
			{
				temxmin = point.x;
				xminpoint = point;
			}
			if (point.x >= temxmax)
			{
				temxmax = point.x;
				xmaxpoint = point;
			}
			if (point.y <= temymin)
			{
				temymin = point.y;
				yminpoint = point;
			}
			if (point.y >= temymax)
			{
				temymax = point.y;
				ymaxpoint = point;
			}
			if (point.z <= temzmin)
			{
				temzmin = point.z;
				zminpoint = point;
			}
			if (point.z >= temzmax)
			{
				temzmax = point.z;
				zmaxpoint = point;
			}
		}
	}

	xmax = temxmax + 0.001; xmin = temxmin + 0.001;
	ymax = temymax + 0.001; ymin = temymin + 0.001;
	zmax = temzmax + 0.001; zmin = temzmin + 0.001;
	PointXYZ point1(xmin, ymin, zmin); PointXYZ point2(xmin, ymax, zmin);
	PointXYZ point3(xmin, ymin, zmax); PointXYZ point4(xmin, ymax, zmax);
	PointXYZ point5(xmax, ymin, zmin); PointXYZ point6(xmax, ymax, zmin);
	PointXYZ point7(xmax, ymin, zmax); PointXYZ point8(xmax, ymax, zmax);

	cloudbox->push_back(point1); cloudbox->push_back(point5);
	cloudbox->push_back(point2); cloudbox->push_back(point6);
	cloudbox->push_back(point3); cloudbox->push_back(point7);
	cloudbox->push_back(point4); cloudbox->push_back(point8);

	cloudplane->push_back(xminpoint); cloudplane->push_back(xmaxpoint);
	cloudplane->push_back(zminpoint); cloudplane->push_back(zmaxpoint);
}


double calCovij(const vector<double> &xall1, const vector<double> &xall2, double ave1, double ave2)
{
	double cov = 0;
	for (int i = 0; i < xall1.size(); i++) {
		double x1 = xall1.at(i) - ave1;
		double x2 = xall2.at(i) - ave2;
		cov += x1 * x2;
	}
	cov = cov / (xall1.size() - 1);
	return cov;
}

void calCov(const pcl::PointCloud<PointXYZRGB>::Ptr &cloudin, const Eigen::Vector4f &centroid, Eigen::Matrix3f &cov_Matrix)
{
	double cenx = centroid(0), ceny = centroid(1), cenz = centroid(2);
	vector<double>xall;
	vector<double>yall;
	vector<double>zall;
	for (int i = 0; i < cloudin->size(); i++) {
		xall.push_back(cloudin->at(i).x);
		yall.push_back(cloudin->at(i).y);
		zall.push_back(cloudin->at(i).z);
	}
	double covxx = calCovij(xall, xall, cenx, cenx);
	double covyy = calCovij(yall, yall, ceny, ceny);
	double covzz = calCovij(zall, zall, cenz, cenz);
	double covxy = calCovij(xall, yall, cenx, ceny);
	double covxz = calCovij(xall, zall, cenx, cenz);
	double covyz = calCovij(yall, zall, ceny, cenz);
	cov_Matrix(0, 0) = covxx; cov_Matrix(0, 1) = covxy; cov_Matrix(0, 2) = covxz;
	cov_Matrix(1, 0) = covxy; cov_Matrix(1, 1) = covyy; cov_Matrix(1, 2) = covyz;
	cov_Matrix(2, 0) = covxz; cov_Matrix(2, 1) = covyz; cov_Matrix(2, 2) = covzz;
}

void getOBBbox(const pcl::PointCloud<PointXYZRGB>::Ptr &cloudin, const Eigen::Vector4f &centroid, pcl::PointCloud<PointXYZRGB>::Ptr &cloudbox)
{
	Eigen::Matrix3f cov_Matrix;
	calCov(cloudin, centroid, cov_Matrix);
	////show box
//viewer->addLine(cloudbox->at(0), cloudbox->at(1), 255, 0, 0, "Line1");
//viewer->addLine(cloudbox->at(0), cloudbox->at(2), 255, 0, 0, "Line2");
//viewer->addLine(cloudbox->at(1), cloudbox->at(3), 255, 0, 0, "Line3");
//viewer->addLine(cloudbox->at(2), cloudbox->at(3), 255, 0, 0, "Line4");

//viewer->addLine(cloudbox->at(4), cloudbox->at(5), 0, 255, 0, "Line5");
//viewer->addLine(cloudbox->at(4), cloudbox->at(6), 0, 255, 0, "Line6");
//viewer->addLine(cloudbox->at(5), cloudbox->at(7), 0, 255, 0, "Line7");
//viewer->addLine(cloudbox->at(6), cloudbox->at(7), 0, 255, 0, "Line8");

//viewer->addLine(cloudbox->at(0), cloudbox->at(4), 0, 0, 255, "Line9");
//viewer->addLine(cloudbox->at(1), cloudbox->at(5), 0, 0, 255, "Line10");
//viewer->addLine(cloudbox->at(2), cloudbox->at(6), 0, 0, 255, "Line11");
//viewer->addLine(cloudbox->at(3), cloudbox->at(7), 0, 0, 255, "Line12");

////show plane
//viewer->addLine(cloudplane->at(0), cloudplane->at(2), 0, 0, 255, "plane1");
//viewer->addLine(cloudplane->at(0), cloudplane->at(1), 0, 255, 0, "plane2");
//viewer->addLine(cloudplane->at(1), cloudplane->at(2), 255, 0, 0, "plane3");
}

vector<PointXYZ> getmaxcoor(const PointXYZ &p1, const PointXYZ &p2)
{
	double xmax = p1.x, ymax = p1.y, zmax = p1.z,
		xmin = p1.x, ymin = p1.y, zmin = p1.z;
	if (p2.x >= xmax)
		xmax = p2.x;
	if (p2.y >= ymax)
		ymax = p2.y;
	if (p2.z >= zmax)
		zmax = p2.z;
	if (p2.x <= xmin)
		xmin = p2.x;
	if (p2.y <= ymin)
		ymin = p2.y;
	if (p2.z <= zmin)
		zmin = p2.z;
	PointXYZ pmin(xmin, ymin, zmin);
	PointXYZ pmax(xmax, ymax, zmax);
	vector<PointXYZ>temp;
	temp.push_back(pmin);
	temp.push_back(pmax);
	return temp;
}

//void getmaxbox(const vector<pcl::PointXYZ> &printPointset, vector<pcl::PointXYZ> &ptemp)
//{
//	PointXYZ pt1 = printPointset.at(0), pt2 = printPointset.at(1), pt3 = printPointset.at(2),
//		pt4 = printPointset.at(3),pt5 = printPointset.at(4),pt6 = printPointset.at(5),
//		pt7 = printPointset.at(6), pt8 = printPointset.at(7);
//	PointXYZ p1 (getmaxcoor(pt1, pt5).at(0).x, getmaxcoor(pt1, pt5).at(1).y, getmaxcoor(pt1, pt5).at(0).z);
//	//PointXYZ p1(xmin, ymin, zmin);
//	//PointXYZ p2(xmin, ymax, zmin);
//	//PointXYZ p3(xmin, ymin, zmax);
//	//PointXYZ p4(xmin, ymax, zmax);
//	//PointXYZ p5(xmax, ymin, zmin);
//	//PointXYZ p6(xmax, ymax, zmin);
//	//PointXYZ p7(xmax, ymin, zmax);
//	//PointXYZ p8(xmax, ymax, zmax);
//	//ptemp.push_back(p1); ptemp.push_back(p2); ptemp.push_back(p3); ptemp.push_back(p4);
//	//ptemp.push_back(p5); ptemp.push_back(p6); ptemp.push_back(p7); ptemp.push_back(p8);
//}

void viewbox(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, vector<pcl::PointXYZ> &printPointset,string strin)
{


	for (int i = 0; i < 5; i = i + 4)
	{
		int allsize = printPointset.size();
		stringstream eagename1, eagename2, eagename3, eagename4;
		eagename1 << strin << i + 1 << "-" << i + 2 << " eage";
		eagename2 << strin << i + 1 << "-" << i + 3 << " eage";
		eagename3 << strin << i + 4 << "-" << i + 2 << " eage";
		eagename4 << strin << i + 4 << "-" << i + 3 << " eage";
		viewer->addLine(printPointset.at(i), printPointset.at(i + 1), 0.0 + i * 1.0 / allsize,
			0.0 + i * 0.5 / allsize, 0.0 + i * 0.25 / allsize, eagename1.str());
		viewer->addLine(printPointset.at(i), printPointset.at(i + 2), 0.0 + i * 1.0 / allsize,
			0.0 + i * 0.5 / allsize, 0.0 + i * 0.25 / allsize, eagename2.str());
		viewer->addLine(printPointset.at(i + 3), printPointset.at(i + 1), 0.0 + i * 1.0 / allsize,
			0.0 + i * 0.5 / allsize, 0.0 + i * 0.25 / allsize, eagename3.str());
		viewer->addLine(printPointset.at(i + 3), printPointset.at(i + 2), 0.0 + i * 1.0 / allsize,
			0.0 + i * 0.5 / allsize, 0.0 + i * 0.25 / allsize, eagename4.str());
	}
	for (int i = 0; i < 4; i++)
	{
		stringstream eagename;
		eagename << strin << i+1 << "-" << i + 5 << " eage";
		viewer->addLine(printPointset.at(i), printPointset.at(i + 4), 0, 1.0, 0, eagename.str());
	}
	//判断端点
	//pcl::PointCloud<PointXYZRGB>::Ptr Pix(new pcl::PointCloud<PointXYZRGB>);
	//for (int i=4;i<7;i++)
	//{
	//	PointXYZRGB temp;
	//	temp.x= printPointset.at(i).x;
	//	temp.y = printPointset.at(i).y;
	//	temp.z = printPointset.at(i).z;
	//	temp.r = 255;
	//	temp.g = 0+50*(i-4);
	//	temp.b = 0 + 100 * (i-4);
	//	Pix->push_back(temp);
	//}
	//viewer->addPointCloud(Pix,"pix");
}

void pclOBBbox(const pcl::PointCloud<PointXYZ>::Ptr &cloudin, OBB_matrix &myobb,vector<pcl::PointXYZ> &printPointset)
{
	auto cloud = cloudin;
	pcl::MomentOfInertiaEstimation<PointXYZ> ObbEsti;
	ObbEsti.setInputCloud(cloud);
	ObbEsti.compute();
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	Eigen::Vector3f mass_center;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;

	ObbEsti.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	ObbEsti.getEigenValues(major_value, middle_value, minor_value);
	ObbEsti.getEigenVectors(major_vector, middle_vector, minor_vector);
	ObbEsti.getMassCenter(mass_center);


	Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
	Eigen::Quaternionf quat(rotational_matrix_OBB);
	//viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
	myobb.max_point_OBB = max_point_OBB;
	myobb.min_point_OBB = min_point_OBB;
	myobb.position_OBB = position_OBB;
	myobb.rotational_matrix_OBB = rotational_matrix_OBB;
	myobb.mass_center = mass_center;

	//可视化工作
	//边框
	Eigen::Vector3f p1(min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
	Eigen::Vector3f p2(min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
	Eigen::Vector3f p3(min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
	Eigen::Vector3f p4(min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
	Eigen::Vector3f p5(max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
	Eigen::Vector3f p6(max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
	Eigen::Vector3f p7(max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
	Eigen::Vector3f p8(max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);

	p1 = rotational_matrix_OBB * p1 + position;
	p2 = rotational_matrix_OBB * p2 + position;
	p3 = rotational_matrix_OBB * p3 + position;
	p4 = rotational_matrix_OBB * p4 + position;
	p5 = rotational_matrix_OBB * p5 + position;
	p6 = rotational_matrix_OBB * p6 + position;
	p7 = rotational_matrix_OBB * p7 + position;
	p8 = rotational_matrix_OBB * p8 + position;

	pcl::PointXYZ pt1(p1(0), p1(1), p1(2));
	pcl::PointXYZ pt2(p2(0), p2(1), p2(2));
	pcl::PointXYZ pt3(p3(0), p3(1), p3(2));
	pcl::PointXYZ pt4(p4(0), p4(1), p4(2));
	pcl::PointXYZ pt5(p5(0), p5(1), p5(2));
	pcl::PointXYZ pt6(p6(0), p6(1), p6(2));
	pcl::PointXYZ pt7(p7(0), p7(1), p7(2));
	pcl::PointXYZ pt8(p8(0), p8(1), p8(2));

	printPointset.push_back(pt1); printPointset.push_back(pt5);
	printPointset.push_back(pt2); printPointset.push_back(pt6);
	printPointset.push_back(pt3); printPointset.push_back(pt7);
	printPointset.push_back(pt4); printPointset.push_back(pt8);

	//坐标轴
	pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
	pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
	pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
	pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
	myobb.center = center;
	myobb.x_axis = x_axis;
	myobb.y_axis = y_axis;
	myobb.z_axis = z_axis;
}

void pclAABBbox(const pcl::PointCloud<PointXYZ>::Ptr &cloudin, AABB_matrix &myaabb, vector<pcl::PointXYZ> &printPointset)
{
	auto cloud = cloudin;
	pcl::MomentOfInertiaEstimation<PointXYZ> aabbEsti;
	aabbEsti.setInputCloud(cloud);
	aabbEsti.compute();
	pcl::PointXYZ min_point_AABB;
	pcl::PointXYZ max_point_AABB;
	pcl::PointXYZ position_AABB;
	Eigen::Matrix3f rotational_matrix_AABB;
	Eigen::Vector3f mass_center;
	aabbEsti.getAABB(min_point_AABB, max_point_AABB);
	printPointset.push_back(min_point_AABB);
	printPointset.push_back(max_point_AABB);

}

//AABBcube可视化
//PointXYZ pt1(min_point_AABB.x, min_point_AABB.y, min_point_AABB.z);
//PointXYZ pt2(min_point_AABB.x, max_point_AABB.y, min_point_AABB.z);
//PointXYZ pt3(min_point_AABB.x, min_point_AABB.y, max_point_AABB.z);
//PointXYZ pt4(min_point_AABB.x, max_point_AABB.y, max_point_AABB.z);
//PointXYZ pt5(max_point_AABB.x, min_point_AABB.y, min_point_AABB.z);
//PointXYZ pt6(max_point_AABB.x, max_point_AABB.y, min_point_AABB.z);
//PointXYZ pt7(max_point_AABB.x, min_point_AABB.y, max_point_AABB.z);
//PointXYZ pt8(max_point_AABB.x, max_point_AABB.y, max_point_AABB.z);
//
//viewer->addLine(pt1, pt2, 0.0, 1.0, 1.0, "1 edge");
//viewer->addLine(pt1, pt4, 0.0, 1.0, 1.0, "2 edge");
//viewer->addLine(pt1, pt5, 0.0, 1.0, 1.0, "3 edge");
//viewer->addLine(pt5, pt6, 0.0, 1.0, 1.0, "4 edge");
//viewer->addLine(pt5, pt8, 0.0, 1.0, 1.0, "5 edge");
//viewer->addLine(pt2, pt6, 0.0, 1.0, 1.0, "6 edge");
//viewer->addLine(pt6, pt7, 0.0, 1.0, 1.0, "7 edge");
//viewer->addLine(pt7, pt8, 0.0, 1.0, 1.0, "8 edge");
//viewer->addLine(pt2, pt3, 0.0, 1.0, 1.0, "9 edge");
//viewer->addLine(pt4, pt8, 0.0, 1.0, 1.0, "10 edge");
//viewer->addLine(pt3, pt4, 0.0, 1.0, 1.0, "11 edge");

////基于pca包围盒与OBB结果一致
//	Eigen::Vector4f pcaCentroid;
//	pcl::compute3DCentroid<PointXYZ>(*cloudCalbox, pcaCentroid);
//	Eigen::Matrix3f covariance;
//	pcl::computeCovarianceMatrixNormalized<PointXYZ>(*cloudCalbox, pcaCentroid, covariance);
//	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
//	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
//	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
//	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
//	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
//	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
//
//	std::cout << "特征值va(3x1):\n" << eigenValuesPCA << std::endl;
//	std::cout << "特征向量ve(3x3):\n" << eigenVectorsPCA << std::endl;
//	std::cout << "质心点(4x1):\n" << pcaCentroid << std::endl;
//	/*
//	// 另一种计算点云协方差矩阵特征值和特征向量的方式:通过pcl中的pca接口，如下，这种情况得到的特征向量相似特征向量
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PCA<pcl::PointXYZ> pca;
//	pca.setInputCloud(cloudSegmented);
//	pca.project(*cloudSegmented, *cloudPCAprojection);
//	std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;//计算特征向量
//	std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;//计算特征值
//	*/
//	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
//	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
//	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
//	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
//	tm_inv = tm.inverse();
//
//	std::cout << "变换矩阵tm(4x4):\n" << tm << std::endl;
//	std::cout << "逆变矩阵tm'(4x4):\n" << tm_inv << std::endl;
//
//	pcl::PointCloud<PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<PointXYZ>);
//	pcl::transformPointCloud<PointXYZ>(*cloudCalbox, *transformedCloud, tm);
//
//	PointXYZ min_p1, max_p1;
//	Eigen::Vector3f c1, c;
//	pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
//	c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());
//
//	std::cout << "型心c1(3x1):\n" << c1 << std::endl;
//
//	Eigen::Affine3f tm_inv_aff(tm_inv);
//	pcl::transformPoint(c1, c, tm_inv_aff);
//
//	Eigen::Vector3f whd, whd1;
//	whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
//	whd = whd1;
//	float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3;  //点云平均尺度，用于设置主方向箭头大小
//
//	std::cout << "width1=" << whd1(0) << endl;
//	std::cout << "heght1=" << whd1(1) << endl;
//	std::cout << "depth1=" << whd1(2) << endl;
//	std::cout << "scale1=" << sc1 << endl;
//
//	const Eigen::Quaternionf bboxQ1(Eigen::Quaternionf::Identity());
//	const Eigen::Vector3f    bboxT1(c1);
//
//	const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
//	const Eigen::Vector3f    bboxT(c);
//
//
//	//变换到原点的点云主方向
//	PointXYZ op;
//	op.x = 0.0;
//	op.y = 0.0;
//	op.z = 0.0;
//	Eigen::Vector3f px, py, pz;
//	Eigen::Affine3f tm_aff(tm);
//	pcl::transformVector(eigenVectorsPCA.col(0), px, tm_aff);
//	pcl::transformVector(eigenVectorsPCA.col(1), py, tm_aff);
//	pcl::transformVector(eigenVectorsPCA.col(2), pz, tm_aff);
//	PointXYZ pcaX;
//	pcaX.x = sc1 * px(0);
//	pcaX.y = sc1 * px(1);
//	pcaX.z = sc1 * px(2);
//	PointXYZ pcaY;
//	pcaY.x = sc1 * py(0);
//	pcaY.y = sc1 * py(1);
//	pcaY.z = sc1 * py(2);
//	PointXYZ pcaZ;
//	pcaZ.x = sc1 * pz(0);
//	pcaZ.y = sc1 * pz(1);
//	pcaZ.z = sc1 * pz(2);
//
//
//	//初始点云的主方向
//	PointXYZ cp;
//	cp.x = pcaCentroid(0);
//	cp.y = pcaCentroid(1);
//	cp.z = pcaCentroid(2);
//	PointXYZ pcX;
//	pcX.x = sc1 * eigenVectorsPCA(0, 0) + cp.x;
//	pcX.y = sc1 * eigenVectorsPCA(1, 0) + cp.y;
//	pcX.z = sc1 * eigenVectorsPCA(2, 0) + cp.z;
//	PointXYZ pcY;
//	pcY.x = sc1 * eigenVectorsPCA(0, 1) + cp.x;
//	pcY.y = sc1 * eigenVectorsPCA(1, 1) + cp.y;
//	pcY.z = sc1 * eigenVectorsPCA(2, 1) + cp.z;
//	PointXYZ pcZ;
//	pcZ.x = sc1 * eigenVectorsPCA(0, 2) + cp.x;
//	pcZ.y = sc1 * eigenVectorsPCA(1, 2) + cp.y;
//	pcZ.z = sc1 * eigenVectorsPCA(2, 2) + cp.z;
//
//
//	//visualization
//	pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> tc_handler(transformedCloud, 0, 255, 0); //转换到原点的点云相关
//	viewer->addPointCloud(transformedCloud, tc_handler, "transformCloud");
//	viewer->addCube(bboxT1, bboxQ1, whd1(0), whd1(1), whd1(2), "bbox1");
//	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox1");
//	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "bbox1");
//
//	viewer->addArrow(pcaX, op, 1.0, 0.0, 0.0, false, "arrow_X");
//	viewer->addArrow(pcaY, op, 0.0, 1.0, 0.0, false, "arrow_Y");
//	viewer->addArrow(pcaZ, op, 0.0, 0.0, 1.0, false, "arrow_Z");
//
//	pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> color_handler(cloudCalbox, 255, 0, 0);  //输入的初始点云相关
//	viewer->addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), "bbox");
//	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
//	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bbox");
//
//	viewer->addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_x");
//	viewer->addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_y");
//	viewer->addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_z");

//void viewbox_old(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, vector<pcl::PointXYZ> &printPointset, string strin)
//{
//
//	double zmax = -INT_MAX, xmax = -INT_MAX, ymax = -INT_MAX,
//		zmin = INT_MAX, xmin = INT_MAX, ymin = INT_MAX;
//	for (int i = 0; i < 5; i = i + 4)
//	{
//		int allsize = printPointset.size();
//		stringstream eagename1, eagename2, eagename3, eagename4;
//		eagename1 << strin << i + 1 << "-" << i + 2 << " eage";
//		eagename2 << strin << i + 1 << "-" << i + 3 << " eage";
//		eagename3 << strin << i + 4 << "-" << i + 2 << " eage";
//		eagename4 << strin << i + 4 << "-" << i + 3 << " eage";
//		viewer->addLine(printPointset.at(i), printPointset.at(i + 1), 0.0 + i * 1.0 / allsize,
//			0.0 + i * 0.5 / allsize, 0.0 + i * 0.25 / allsize, eagename1.str());
//		viewer->addLine(printPointset.at(i), printPointset.at(i + 2), 0.0 + i * 1.0 / allsize,
//			0.0 + i * 0.5 / allsize, 0.0 + i * 0.25 / allsize, eagename2.str());
//		viewer->addLine(printPointset.at(i + 3), printPointset.at(i + 1), 0.0 + i * 1.0 / allsize,
//			0.0 + i * 0.5 / allsize, 0.0 + i * 0.25 / allsize, eagename3.str());
//		viewer->addLine(printPointset.at(i + 3), printPointset.at(i + 2), 0.0 + i * 1.0 / allsize,
//			0.0 + i * 0.5 / allsize, 0.0 + i * 0.25 / allsize, eagename4.str());
//	}
//	for (int i = 0; i < 4; i++)
//	{
//		stringstream eagename;
//		eagename << strin << i + 1 << "-" << i + 5 << " eage";
//		viewer->addLine(printPointset.at(i), printPointset.at(i + 4), 0, 1.0, 0, eagename.str());
//	}
//}