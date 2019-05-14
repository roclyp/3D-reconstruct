#include "DealwithPointCloud.h"

//处理三角面片DealWithTri三个一起处理
void DealWithTri(CloudCenVers &cloudCenVer2, PointCloud<PointXYZRGB>::Ptr &cloud2trans,
	PointCloud<PointXYZRGB>::Ptr &cloud1, PointCloud<PointXYZRGB>::Ptr &outcloud,
	PointCloud<PointXYZRGB>::Ptr &cross2Cloud, PointCloud<PointXYZRGB>::Ptr &cross1Cloud,
	PointCloud<PointXYZRGB>::Ptr &parallelCloud, mypara &para)
{
	//先处理交叉的且在两边的
	for (int i = 0; i < cloudCenVer2.interAll.size(); i++)
	{
		auto temp = cloudCenVer2.interAll.at(i);
		if (temp.Cross2itervers.size() == 0)
		{
			//cout << "Triangle " << i << "has no 2 sides cross triangles" << endl;
		}
		else
		{
			for (int j = 0; j < temp.Cross2itervers.size(); j++)
			{
				auto temp2 = temp.Cross2itervers.at(j);
				spaceLine crossLine;
				PointXYZRGB cloud2p1 = cloud2trans->at(temp.vertices.vertices[0]);
				PointXYZRGB cloud2p2 = cloud2trans->at(temp.vertices.vertices[1]);
				PointXYZRGB cloud2p3 = cloud2trans->at(temp.vertices.vertices[2]);
				PointXYZRGB cloud1p1 = cloud1->at(temp2.vertices[0]);
				PointXYZRGB cloud1p2 = cloud1->at(temp2.vertices[1]);
				PointXYZRGB cloud1p3 = cloud1->at(temp2.vertices[2]);
				spacePlane plane2(cloud2p1, cloud2p2, cloud2p3);
				spacePlane plane1(cloud1p1, cloud1p2, cloud1p3);
				spacePlane outplane;
				//获得旋转角度的两倍，相交线，以及中值平面
				double doubleangle = getAngle(plane2, plane1, outplane, crossLine);
				//将点云2投影至中值平面
				PointXYZRGB adjustC2p1, adjustC2p2, adjustC2p3;
				if (getAdjustPoint(cloud2p1, outplane, adjustC2p1, para.Tthrehold) == true)
					outcloud->push_back(adjustC2p1);
				if (getAdjustPoint(cloud2p2, outplane, adjustC2p2, para.Tthrehold) == true)
					outcloud->push_back(adjustC2p2);
				if (getAdjustPoint(cloud2p3, outplane, adjustC2p3, para.Tthrehold) == true)
					outcloud->push_back(adjustC2p3);
			}
		}
		if (temp.Cross1itervers.size() == 0)
		{
			//cout << "Triangle " << i << "has no 1 sides cross triangles" << endl;
		}
		else
		{
			for (int j = 0; j < temp.Cross1itervers.size(); j++)
			{
				auto temp2 = temp.Cross1itervers.at(j);
				spaceLine crossLine;
				PointXYZRGB cloud2p1 = cloud2trans->at(temp.vertices.vertices[0]);
				PointXYZRGB cloud2p2 = cloud2trans->at(temp.vertices.vertices[1]);
				PointXYZRGB cloud2p3 = cloud2trans->at(temp.vertices.vertices[2]);
				PointXYZRGB cloud1p1 = cloud1->at(temp2.vertices[0]);
				PointXYZRGB cloud1p2 = cloud1->at(temp2.vertices[1]);
				PointXYZRGB cloud1p3 = cloud1->at(temp2.vertices[2]);
				spacePlane plane2(cloud2p1, cloud2p2, cloud2p3);
				spacePlane plane1(cloud1p1, cloud1p2, cloud1p3);
				spacePlane outplane;
				//获得旋转角度的两倍，相交线，以及中值平面
				double doubleangle = getAngle(plane2, plane1, outplane, crossLine);
				//将点云2投影至中值平面
				PointXYZRGB adjustC2p1, adjustC2p2, adjustC2p3;
				if (getAdjustPoint(cloud2p1, outplane, adjustC2p1, para.Tthrehold) == true)
					outcloud->push_back(adjustC2p1);
				if (getAdjustPoint(cloud2p2, outplane, adjustC2p2, para.Tthrehold) == true)
					outcloud->push_back(adjustC2p2);
				if (getAdjustPoint(cloud2p3, outplane, adjustC2p3, para.Tthrehold) == true)
					outcloud->push_back(adjustC2p3);
			}
		}
		if (temp.Parallitervers.size() == 0)
		{
			//cout << "Triangle " << i << "has no parallel triangles" << endl;
		}
		else
		{
			for (int j = 0; j < temp.Parallitervers.size(); j++)
			{
				auto temp2 = temp.Parallitervers.at(j);
				spaceLine crossLine;
				PointXYZRGB cloud2p1 = cloud2trans->at(temp.vertices.vertices[0]);
				PointXYZRGB cloud2p2 = cloud2trans->at(temp.vertices.vertices[1]);
				PointXYZRGB cloud2p3 = cloud2trans->at(temp.vertices.vertices[2]);
				PointXYZRGB cloud1p1 = cloud1->at(temp2.vertices[0]);
				PointXYZRGB cloud1p2 = cloud1->at(temp2.vertices[1]);
				PointXYZRGB cloud1p3 = cloud1->at(temp2.vertices[2]);
				spacePlane plane2(cloud2p1, cloud2p2, cloud2p3);
				spacePlane plane1(cloud1p1, cloud1p2, cloud1p3);
				spacePlane outplane;
				//获得平行片面的中直面
				outplane.A = (plane1.A + plane2.A)*1.0 / 2;
				outplane.B = (plane1.B + plane2.B)*1.0 / 2;
				outplane.C = (plane1.C + plane2.C)*1.0 / 2;
				outplane.D = (plane1.D + plane2.D)*1.0 / 2;
				//将点云2投影至中值平面
				PointXYZRGB adjustC2p1, adjustC2p2, adjustC2p3;
				if (getAdjustPoint(cloud2p1, outplane, adjustC2p1, para.Tthrehold) == true)
					outcloud->push_back(adjustC2p1);
				if (getAdjustPoint(cloud2p2, outplane, adjustC2p2, para.Tthrehold) == true)
					outcloud->push_back(adjustC2p2);
				if (getAdjustPoint(cloud2p3, outplane, adjustC2p3, para.Tthrehold) == true)
					outcloud->push_back(adjustC2p3);
			}
		}
	}

}


//给邻近三角面片排序
void sortDueDis(vector<pcl::Vertices> &vers, vector<double> &dis)
{
	for (int i = 0; i < vers.size(); i++)
	{
		for (int j = i + 1; j < vers.size(); j++)
		{
			if (dis.at(i) >= dis.at(j))
			{
				swap(vers.at(i), vers.at(j));
				swap(dis.at(i), dis.at(j));
			}
		}
	}
}

//分别处理，处理相交两侧的
void DealCross2cLoud(CloudCenVers &cloudCenVer2, PointCloud<PointXYZRGB>::Ptr &cloud2trans,
	PointCloud<PointXYZRGB>::Ptr &cloud1, PointCloud<PointXYZRGB>::Ptr &outcloud,
	PointCloud<PointXYZRGB>::Ptr &cross2Cloud, mypara &para)
{
	//auto temp = cloudCenVer2.interAll.at(0);
	for (int i = 0; i < cloudCenVer2.interAll.size(); i++)
	{

		auto temp = cloudCenVer2.interAll.at(i);
		if (temp.Cross2itervers.size() == 0)
		{
			//cout << "Triangle " << i << "has no 2 sides cross triangles" << endl;
		}
		else
		{
			//处理前先根据欧氏距离对每个三角名片的邻近三角面片进行排序
			sortDueDis(temp.Cross2itervers, temp.Cross2dis);
			int dealnum;
			if (temp.Cross2itervers.size() >= para.dealnum)
				dealnum = para.dealnum;
			else
				dealnum = temp.Cross2itervers.size();
			//开始处理
			for (int j = 0; j < dealnum; j++)
			{
				auto temp2 = temp.Cross2itervers.at(j);
				spaceLine crossLine;
				PointXYZRGB cloud2p1 = cloud2trans->at(temp.vertices.vertices[0]);
				PointXYZRGB cloud2p2 = cloud2trans->at(temp.vertices.vertices[1]);
				PointXYZRGB cloud2p3 = cloud2trans->at(temp.vertices.vertices[2]);
				PointXYZRGB cloud1p1 = cloud1->at(temp2.vertices[0]);
				PointXYZRGB cloud1p2 = cloud1->at(temp2.vertices[1]);
				PointXYZRGB cloud1p3 = cloud1->at(temp2.vertices[2]);
				spacePlane plane2(cloud2p1, cloud2p2, cloud2p3);
				spacePlane plane1(cloud1p1, cloud1p2, cloud1p3);
				spacePlane outplane;
				//获得旋转角度的两倍，相交线，以及中值平面
				double doubleangle = getAngle(plane2, plane1, outplane, crossLine);
				//将点云2投影至中值平面
				PointXYZRGB adjustC2p1, adjustC2p2, adjustC2p3;
				if (getAdjustPoint(cloud2p1, outplane, adjustC2p1, para.Tthrehold) == true)
				{
					outcloud->push_back(adjustC2p1);
					cross2Cloud->push_back(adjustC2p1);
				}
				if (getAdjustPoint(cloud2p2, outplane, adjustC2p2, para.Tthrehold) == true)
				{
					outcloud->push_back(adjustC2p2);
					cross2Cloud->push_back(adjustC2p2);
				}
				if (getAdjustPoint(cloud2p3, outplane, adjustC2p3, para.Tthrehold) == true)
				{
					outcloud->push_back(adjustC2p3);
					cross2Cloud->push_back(adjustC2p3);
				}
			}
		}
	}
}

//分别处理，处理相交同侧的
void DealCross1cLoud(CloudCenVers &cloudCenVer2, PointCloud<PointXYZRGB>::Ptr &cloud2trans,
	PointCloud<PointXYZRGB>::Ptr &cloud1, PointCloud<PointXYZRGB>::Ptr &outcloud,
	PointCloud<PointXYZRGB>::Ptr &cross1Cloud, mypara &para)
{
	for (int i = 0; i < cloudCenVer2.interAll.size(); i++)
	{
		auto temp = cloudCenVer2.interAll.at(i);
		if (temp.Cross1itervers.size() == 0)
		{
			//cout << "Triangle " << i << "has no 1 sides cross triangles" << endl;
		}
		else
		{
			sortDueDis(temp.Cross1itervers, temp.Cross1dis);
			int dealnum;
			if (temp.Cross1itervers.size() >= para.dealnum)
				dealnum = para.dealnum;
			else
				dealnum = temp.Cross1itervers.size();
			for (int j = 0; j < dealnum; j++)
			{
				auto temp2 = temp.Cross1itervers.at(j);
				spaceLine crossLine;
				PointXYZRGB cloud2p1 = cloud2trans->at(temp.vertices.vertices[0]);
				PointXYZRGB cloud2p2 = cloud2trans->at(temp.vertices.vertices[1]);
				PointXYZRGB cloud2p3 = cloud2trans->at(temp.vertices.vertices[2]);
				PointXYZRGB cloud1p1 = cloud1->at(temp2.vertices[0]);
				PointXYZRGB cloud1p2 = cloud1->at(temp2.vertices[1]);
				PointXYZRGB cloud1p3 = cloud1->at(temp2.vertices[2]);
				spacePlane plane2(cloud2p1, cloud2p2, cloud2p3);
				spacePlane plane1(cloud1p1, cloud1p2, cloud1p3);
				spacePlane outplane;
				//获得旋转角度的两倍，相交线，以及中值平面
				double doubleangle = getAngle(plane2, plane1, outplane, crossLine);
				//将点云2投影至中值平面
				PointXYZRGB adjustC2p1, adjustC2p2, adjustC2p3;
				if (getAdjustPoint(cloud2p1, outplane, adjustC2p1, para.Tthrehold) == true)
				{
					outcloud->push_back(adjustC2p1);
					cross1Cloud->push_back(adjustC2p1);
				}
				if (getAdjustPoint(cloud2p2, outplane, adjustC2p2, para.Tthrehold) == true)
				{
					outcloud->push_back(adjustC2p2);
					cross1Cloud->push_back(adjustC2p2);
				}
				if (getAdjustPoint(cloud2p3, outplane, adjustC2p3, para.Tthrehold) == true)
				{
					outcloud->push_back(adjustC2p3);
					cross1Cloud->push_back(adjustC2p3);
				}
			}
		}

	}
}

//分别处理，处理平行的
void DealParalledcLoud(CloudCenVers &cloudCenVer2, PointCloud<PointXYZRGB>::Ptr &cloud2trans,
	PointCloud<PointXYZRGB>::Ptr &cloud1, PointCloud<PointXYZRGB>::Ptr &outcloud,
	PointCloud<PointXYZRGB>::Ptr &parallelCloud, mypara &para)
{
	for (int i = 0; i < cloudCenVer2.interAll.size(); i++)
	{
		auto temp = cloudCenVer2.interAll.at(i);
		if (temp.Parallitervers.size() == 0)
		{
			//cout << "Triangle " << i << "has no parallel triangles" << endl;
		}
		else
		{
			sortDueDis(temp.Parallitervers, temp.Paralledldis);
			int dealnum;
			if (temp.Parallitervers.size() >= para.dealnum)
				dealnum = para.dealnum;
			else
				dealnum = temp.Parallitervers.size();
			for (int j = 0; j < dealnum; j++)
			{
				auto temp2 = temp.Parallitervers.at(j);
				spaceLine crossLine;
				PointXYZRGB cloud2p1 = cloud2trans->at(temp.vertices.vertices[0]);
				PointXYZRGB cloud2p2 = cloud2trans->at(temp.vertices.vertices[1]);
				PointXYZRGB cloud2p3 = cloud2trans->at(temp.vertices.vertices[2]);
				PointXYZRGB cloud1p1 = cloud1->at(temp2.vertices[0]);
				PointXYZRGB cloud1p2 = cloud1->at(temp2.vertices[1]);
				PointXYZRGB cloud1p3 = cloud1->at(temp2.vertices[2]);
				spacePlane plane2(cloud2p1, cloud2p2, cloud2p3);
				spacePlane plane1(cloud1p1, cloud1p2, cloud1p3);
				spacePlane outplane;
				//获得平行片面的中直面
				outplane.A = (plane1.A + plane2.A)*1.0 / 2;
				outplane.B = (plane1.B + plane2.B)*1.0 / 2;
				outplane.C = (plane1.C + plane2.C)*1.0 / 2;
				outplane.D = (plane1.D + plane2.D)*1.0 / 2;
				//将点云2投影至中值平面
				PointXYZRGB adjustC2p1, adjustC2p2, adjustC2p3;
				if (getAdjustPoint(cloud2p1, outplane, adjustC2p1, para.Tthrehold) == true)
				{
					outcloud->push_back(adjustC2p1);
					parallelCloud->push_back(adjustC2p1);
				}
				if (getAdjustPoint(cloud2p2, outplane, adjustC2p2, para.Tthrehold) == true)
				{
					outcloud->push_back(adjustC2p2);
					parallelCloud->push_back(adjustC2p2);
				}
				if (getAdjustPoint(cloud2p3, outplane, adjustC2p3, para.Tthrehold) == true)
				{
					outcloud->push_back(adjustC2p3);
					parallelCloud->push_back(adjustC2p3);
				}
			}
		}
	}
}

bool getAdjustPoint(PointXYZRGB &p1, spacePlane &sp, PointXYZRGB &outpoint, double Tthrehold)
{
	spaceLine nor;
	nor.dx = sp.A; nor.dy = sp.B; nor.dz = sp.C;
	nor.x0 = p1.x; nor.y0 = p1.y; nor.z0 = p1.z;
	if (nor.dx == 0 && nor.dy == 0 && nor.dz == 0)
	{
		//cout << "The Normal of median Plane is Nan, Something is wrong!" << endl;
		return false;
	}
	//x'=x0+dx*t,y'=y0+dy*t,z'=z0+dz*t, A*x'+B*y'+C*z'+D=0;
	double standard = sp.A*sp.A + sp.B*sp.B + sp.C*sp.C;
	double t = -(sp.A*nor.x0 + sp.B*nor.y0 + sp.C*nor.z0 + sp.D)*1.0 / standard;
	if (t <= Tthrehold || t >= -Tthrehold)
	{
		outpoint.x = nor.x0 + nor.dx*t;
		outpoint.y = nor.y0 + nor.dy*t;
		outpoint.z = nor.z0 + nor.dz*t;
		outpoint.r = p1.r;
		outpoint.g = p1.g;
		outpoint.b = p1.b;
		return true;
	}
	else
	{
		//cout << "Projected distance is over the threshold" << endl;
		return false;
	}
}


template<typename T>
double computedis(const T &cloud1, const T &cloud2 )
{
	double x2 = (cloud1.x - cloud2.x)*(cloud1.x - cloud2.x);
	double y2 = (cloud1.y - cloud2.y)*(cloud1.y - cloud2.y);
	double z2 = (cloud1.z - cloud2.z)*(cloud1.z - cloud2.z);
	double dis = sqrt(x2 + y2 + z2);
	return dis;
}

void DealCross2cLoud_new(CloudCenVers &cloudCenVer2, PointCloud<PointXYZRGB>::Ptr &cloud2trans,
	PointCloud<PointXYZRGB>::Ptr &cloud1, PointCloud<PointXYZRGB>::Ptr &outcloud,
	PointCloud<PointXYZRGB>::Ptr &cross2Cloud, mypara &para)
{
	//auto temp = cloudCenVer2.interAll.at(0);
	for (int i = 0; i < cloudCenVer2.interAll.size(); i++)
	{

		auto temp = cloudCenVer2.interAll.at(i);
		if (temp.Cross2itervers.size() == 0)
		{
			//cout << "Triangle " << i << "has no 2 sides cross triangles" << endl;
		}
		else
		{
			//处理前先根据欧氏距离对每个三角名片的邻近三角面片进行排序
			sortDueDis(temp.Cross2itervers, temp.Cross2dis);
			int dealnum;
			if (temp.Cross2itervers.size() >= para.dealnum)
				dealnum = para.dealnum;
			else
				dealnum = temp.Cross2itervers.size();
			//开始处理
			PointXYZRGB cenpoint;
			pcl::copyPoint(temp.cenPoint, cenpoint);
			for (int j = 0; j < dealnum; j++)
			{
				auto temp2 = temp.Cross2itervers.at(j);
				spaceLine crossLine;
				PointXYZRGB cloud2p1 = cloud2trans->at(temp.vertices.vertices[0]);
				PointXYZRGB cloud2p2 = cloud2trans->at(temp.vertices.vertices[1]);
				PointXYZRGB cloud2p3 = cloud2trans->at(temp.vertices.vertices[2]);
				PointXYZRGB cloud1p1 = cloud1->at(temp2.vertices[0]);
				PointXYZRGB cloud1p2 = cloud1->at(temp2.vertices[1]);
				PointXYZRGB cloud1p3 = cloud1->at(temp2.vertices[2]);
				/*	spacePlane plane2(cloud2p1, cloud2p2, cloud2p3);
					spacePlane plane1(cloud1p1, cloud1p2, cloud1p3);
					spacePlane outplane;*/
				double doubleangle = 100.0;
				int samevalue = 0;
				int maxitration = 0;
				PointXYZRGB adjustC2p1 = cloud2p1, adjustC2p2 = cloud2p2, adjustC2p3 = cloud2p3;
				do
				{
					double anglebefore = doubleangle;
					spacePlane plane2(adjustC2p1, adjustC2p2, adjustC2p3);
					spacePlane plane1(cloud1p1, cloud1p2, cloud1p3);
					spacePlane outplane;
					doubleangle = fabs(getAngle(plane2, plane1, outplane, crossLine));
					//将点云2投影至中值平面
					PointXYZRGB out1, out2, out3;
					if (getAdjustPoint(adjustC2p1, outplane, out1, para.Tthrehold) == true
						&& computedis(cenpoint, out1) <= para.SearchRadius)
						adjustC2p1 = out1;
					if (getAdjustPoint(adjustC2p2, outplane, out2, para.Tthrehold) == true
						&& computedis(cenpoint, out2) <= para.SearchRadius)
						adjustC2p2 = out2;
					if (getAdjustPoint(adjustC2p3, outplane, out3, para.Tthrehold) == true
						&& computedis(cenpoint, out3) <= para.SearchRadius)
						adjustC2p3 = out3;
					if (fabs(doubleangle - anglebefore) <= 0.001)
						samevalue++;
					if (samevalue == 10 || maxitration == 100)
						break;
					maxitration++;
				} while (doubleangle >= (10.0 / 180 * 3.141592657));

				if (doubleangle <= (10.0 / 180 * 3.141592657))
				{
					if (adjustC2p1.x != 0 && adjustC2p1.y != 0 && adjustC2p1.z != 0)
					{
						outcloud->push_back(adjustC2p1);
						cross2Cloud->push_back(adjustC2p1);
					}
					if (adjustC2p2.x != 0 && adjustC2p2.y != 0 && adjustC2p2.z != 0)
					{
						outcloud->push_back(adjustC2p2);
						cross2Cloud->push_back(adjustC2p2);
					}
					if (adjustC2p3.x != 0 && adjustC2p3.y != 0 && adjustC2p3.z != 0)
					{
						outcloud->push_back(adjustC2p3);
						cross2Cloud->push_back(adjustC2p3);
					}
				}
			}
		}
	}
}

//分别处理，处理相交同侧的
void DealCross1cLoud_new(CloudCenVers &cloudCenVer2, PointCloud<PointXYZRGB>::Ptr &cloud2trans,
	PointCloud<PointXYZRGB>::Ptr &cloud1, PointCloud<PointXYZRGB>::Ptr &outcloud,
	PointCloud<PointXYZRGB>::Ptr &cross1Cloud, mypara &para)
{
	for (int i = 0; i < cloudCenVer2.interAll.size(); i++)
	{
		auto temp = cloudCenVer2.interAll.at(i);
		if (temp.Cross1itervers.size() == 0)
		{
			//cout << "Triangle " << i << "has no 1 sides cross triangles" << endl;
		}
		else
		{
			sortDueDis(temp.Cross1itervers, temp.Cross1dis);
			int dealnum;
			if (temp.Cross1itervers.size() >= para.dealnum)
				dealnum = para.dealnum;
			else
				dealnum = temp.Cross1itervers.size();
			PointXYZRGB cenpoint;
			pcl::copyPoint(temp.cenPoint, cenpoint);
			for (int j = 0; j < dealnum; j++)
			{
				auto temp2 = temp.Cross1itervers.at(j);
				
				spaceLine crossLine;
				PointXYZRGB cloud2p1 = cloud2trans->at(temp.vertices.vertices[0]);
				PointXYZRGB cloud2p2 = cloud2trans->at(temp.vertices.vertices[1]);
				PointXYZRGB cloud2p3 = cloud2trans->at(temp.vertices.vertices[2]);
				PointXYZRGB cloud1p1 = cloud1->at(temp2.vertices[0]);
				PointXYZRGB cloud1p2 = cloud1->at(temp2.vertices[1]);
				PointXYZRGB cloud1p3 = cloud1->at(temp2.vertices[2]);
				/*spacePlane plane2(cloud2p1, cloud2p2, cloud2p3);
				spacePlane plane1(cloud1p1, cloud1p2, cloud1p3);
				spacePlane outplane;*/
				//获得旋转角度的两倍，相交线，以及中值平面
				double doubleangle = 100.0;
				int samevalue = 0;
				int maxitration = 0;
				PointXYZRGB adjustC2p1 = cloud2p1, adjustC2p2 = cloud2p2, adjustC2p3 = cloud2p3;
				do
				{
					double anglebefore = doubleangle;
					spacePlane plane2(adjustC2p1, adjustC2p2, adjustC2p3);
					spacePlane plane1(cloud1p1, cloud1p2, cloud1p3);
					spacePlane outplane;
					doubleangle = fabs(getAngle(plane2, plane1, outplane, crossLine));
					//将点云2投影至中值平面
					PointXYZRGB out1, out2, out3;
					if (getAdjustPoint(adjustC2p1, outplane, out1, para.Tthrehold) == true
						&& computedis(cenpoint,out1)<=para.SearchRadius)
						adjustC2p1 = out1;
					if (getAdjustPoint(adjustC2p2, outplane, out2, para.Tthrehold) == true
						&& computedis(cenpoint, out2) <= para.SearchRadius)
						adjustC2p2 = out2;
					if (getAdjustPoint(adjustC2p3, outplane, out3, para.Tthrehold) == true
						&& computedis(cenpoint, out3) <= para.SearchRadius)
						adjustC2p3 = out3;
					if (fabs(doubleangle - anglebefore) <= 0.001)
						samevalue++;
					if (samevalue == 10 || maxitration == 100)
						break;
					maxitration++;
				} while (doubleangle >= (10.0 / 180 * 3.141592657));

				if (doubleangle <= (10.0 / 180 * 3.141592657))
				{
					if (adjustC2p1.x != 0 && adjustC2p1.y != 0 && adjustC2p1.z != 0)
					{
						outcloud->push_back(adjustC2p1);
						cross1Cloud->push_back(adjustC2p1);
					}
					if (adjustC2p2.x != 0 && adjustC2p2.y != 0 && adjustC2p2.z != 0)
					{
						outcloud->push_back(adjustC2p2);
						cross1Cloud->push_back(adjustC2p2);
					}
					if (adjustC2p3.x != 0 && adjustC2p3.y != 0 && adjustC2p3.z != 0)
					{
						outcloud->push_back(adjustC2p3);
						cross1Cloud->push_back(adjustC2p3);
					}
				}
			}
		}

	}
}

//分别处理，处理平行的
void DealParalledcLoud_new(CloudCenVers &cloudCenVer2, PointCloud<PointXYZRGB>::Ptr &cloud2trans,
	PointCloud<PointXYZRGB>::Ptr &cloud1, PointCloud<PointXYZRGB>::Ptr &outcloud,
	PointCloud<PointXYZRGB>::Ptr &parallelCloud, mypara &para)
{
	for (int i = 0; i < cloudCenVer2.interAll.size(); i++)
	{
		auto temp = cloudCenVer2.interAll.at(i);
		if (temp.Parallitervers.size() == 0)
		{
			//cout << "Triangle " << i << "has no parallel triangles" << endl;
		}
		else
		{
			sortDueDis(temp.Parallitervers, temp.Paralledldis);
			int dealnum;
			if (temp.Parallitervers.size() >= para.dealnum)
				dealnum = para.dealnum;
			else
				dealnum = temp.Parallitervers.size();
			PointXYZRGB cenpoint;
			pcl::copyPoint(temp.cenPoint, cenpoint);
			for (int j = 0; j < dealnum; j++)
			{
				auto temp2 = temp.Parallitervers.at(j);
				spaceLine crossLine;
				PointXYZRGB cloud2p1 = cloud2trans->at(temp.vertices.vertices[0]);
				PointXYZRGB cloud2p2 = cloud2trans->at(temp.vertices.vertices[1]);
				PointXYZRGB cloud2p3 = cloud2trans->at(temp.vertices.vertices[2]);
				PointXYZRGB cloud1p1 = cloud1->at(temp2.vertices[0]);
				PointXYZRGB cloud1p2 = cloud1->at(temp2.vertices[1]);
				PointXYZRGB cloud1p3 = cloud1->at(temp2.vertices[2]);
				/*spacePlane plane2(cloud2p1, cloud2p2, cloud2p3);
				spacePlane plane1(cloud1p1, cloud1p2, cloud1p3);
				spacePlane outplane;*/
				//获得平行片面的中直面
				spacePlane plane1(cloud1p1, cloud1p2, cloud1p3);
				double dis=10;
				double doubleangle = 100.0;
				int samevalue = 0;
				int maxitration = 0;
				PointXYZRGB adjustC2p1 = cloud2p1, adjustC2p2 = cloud2p2, adjustC2p3 = cloud2p3;
				do
				{
					double disbefore = dis;
					spacePlane plane2(adjustC2p1, adjustC2p2, adjustC2p3);
					
					spacePlane outplane;
					outplane.A = (plane1.A + plane2.A)*1.0 / 2;
					outplane.B = (plane1.B + plane2.B)*1.0 / 2;
					outplane.C = (plane1.C + plane2.C)*1.0 / 2;
					outplane.D = (plane1.D + plane2.D)*1.0 / 2;
					dis = fabs(plane1.D - plane2.D);
					//将点云2投影至中值平面
					PointXYZRGB out1, out2, out3;
					if (getAdjustPoint(adjustC2p1, outplane, out1, para.Tthrehold) == true)
						adjustC2p1 = out1;
					if (getAdjustPoint(adjustC2p2, outplane, out2, para.Tthrehold) == true)
						adjustC2p2 = out2;
					if (getAdjustPoint(adjustC2p3, outplane, out3, para.Tthrehold) == true)
						adjustC2p3 = out3;
					if (fabs(dis - disbefore) <= 10e-10)
						samevalue++;
					if (samevalue == 10 || maxitration == 100)
						break;
					maxitration++;
				} while ( dis > 5*10e-7);

				if ( dis <= 5*10e-7)
				{
					outcloud->push_back(adjustC2p1);
					outcloud->push_back(adjustC2p2);
					outcloud->push_back(adjustC2p3);

					parallelCloud->push_back(adjustC2p1);
					parallelCloud->push_back(adjustC2p2);
					parallelCloud->push_back(adjustC2p3);
				}
			}
		}
	}
}

