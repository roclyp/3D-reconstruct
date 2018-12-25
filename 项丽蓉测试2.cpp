#include "tchar.h"
#include <iostream>
#include <vector>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
using namespace std;

int main()
{
	pcl::PointCloud<pcl::PointNormal> cloud_filtered;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	/*pcl::PCDReader reader;
	reader.read("C:\\Users\\Zhihong MA\\Desktop\\mls_points.pcd", *cloud_filtered);
	*///pcl::PLYReader reader;
	//reader.read("C:\\Users\\Zhihong MA\\Desktop\\final3.ply", *cloud_filtered);
	pcl::io::loadPCDFile("C:\\Users\\Zhihong MA\\Desktop\\mls_points.pcd", cloud_filtered);
	if (cloud_filtered.empty())
	{
		cout << "wrong" << endl;
		return -1;
	}
	cout << "loaded" << endl;
	cout << cloud_filtered.size() << endl;


	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(cloud_filtered, cloud_filtered, indices);
	std::cout << "size: " << cloud_filtered.points.size() << std::endl;

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud0(&cloud_filtered);
	if (cloud0->empty())
	{
		PCL_ERROR("NO points");
	}
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::copyPointCloud(*cloud0, *normals);
	if (normals->empty())
	{
		PCL_ERROR("NO points2");
	}
	pcl::search::Search<pcl::PointNormal>::Ptr tree2 = boost::shared_ptr<pcl::search::Search<pcl::PointNormal> >(new pcl::search::KdTree<pcl::PointNormal>);

	pcl::RegionGrowing<pcl::PointNormal, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree2);
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(cloud0);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);
	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;

	int temp = 0;
	for (int i = 0; i < clusters.size(); ++i)
	{
		if (clusters[temp].indices.size() < clusters[i].indices.size()) {
			temp = i;
		}

	}

	//std::cout << "the biggest cluster has  " << clusters[temp].indices.size() << " points" << std::endl;
	pcl::copyPointCloud(*cloud0, clusters[temp], *cloud0);

	pcl::PCDWriter writer;
	writer.write<pcl::PointNormal>("C:\\Users\\Zhihong MA\\Desktop\\cloud00.pcd", *cloud0, false);
	getchar();
	return 0;
}



