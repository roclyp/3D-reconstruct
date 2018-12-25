#include "PCLlibrary.h"
#include <iostream>
#include <string>
#include <pcl/console/time.h>   // TicToc
#include <time.h>

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBPtr;
//穿件PCLFile结构
struct PCLFile
{
	PointXYZRGBPtr cloud;
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

//配准的具体步骤
void AlignCloud(const PointXYZRGBPtr cloud_first, const PointXYZRGBPtr &cloud_second,
	PointXYZRGBPtr &cloud_afterTrans, PointXYZRGBPtr &cloud_after_add, Eigen::Matrix4f &final_transform, bool downsample = false)
{
	PointXYZRGBPtr First_could(new pcl::PointCloud<pcl::PointXYZRGB>);
	PointXYZRGBPtr Second_could(new pcl::PointCloud<pcl::PointXYZRGB>);

	//下采样
	pcl::VoxelGrid<PointT> grid;//建立体素格，将点云分配给局部三维网格中，对网格中的点云数据进行下采样
	if (downsample==true)
	{
		grid.setLeafSize(0.2, 0.2, 0.2);
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
	}
	//计算从源点云到目标点云
	sourceToTarget = MatrixTran;
	pcl::transformPointCloud(*cloud_afterTrans, *cloud_first, sourceToTarget);
	*cloud_after_add = *cloud_afterTrans + *cloud_second;
	final_transform = sourceToTarget;
}

//从一串字符串（包含英文与数字，甚至可能还有中文）获取指定位数的数字
int getNumberInString(string istring,bool &hasnumbr)
{
	int number = 0;
	string filterstring;
	for (int i = istring.size(); i > istring.size() - 10 && istring[i] != '_'; i--)
	{
		filterstring += istring[i];
	}
	for (int i = filterstring.size(); i >0; i--)
	{
		if (filterstring[i] >= '0'&&filterstring[i] <= '9')
		{
			number = number * 10 + filterstring[i] - '0';
		}
	}
	if (number == 0)
		hasnumbr = false;
	return number;
}

//根据文件名字中指定数字大小从小到大排序
template <class T>
int reRangeFileName(vector<T>& files, vector<int> &SerialNumber)
{
	if (files.size() != SerialNumber.size())
	{
		cout << "The number of Files and Serial Number is wrong" << endl;
		return -1;
	}
	for (int i = 0; i<files.size(); i++)
	{
		for (int j = i; j < files.size(); j++)
		{
			if (SerialNumber[i] >= SerialNumber[j])
			{
				//swap(SerialNumber[i], SerialNumber[j]);
			
				int tmp;
				tmp = SerialNumber[j];
				SerialNumber[j] = SerialNumber[i];
				SerialNumber[i] = tmp;
				//swap(files[i], files[j]);
				T tmpname;
				tmpname = files[j];
				files[j] = files[i];
				files[i] = tmpname;
			}
		}

	}
	return 0;
}

//获取指定文件夹下特定格式的文件
int GetAllFiles_CertainFormat(string path,vector<string>& files, string format)
{
	intptr_t hFile = 0;//_findnext返回类型为intprt_t,而非long类型，从intptr_t转换到long类型丢失数据
	struct  _finddata_t fileinfo;
	vector<int> SerialNumber;//用于存储序号
	string serialnumber;//用于排序
	bool hasnumber = true;//用于判断文件名中是否有数字
	int nonum = 0;//用于在没有数字时排序
	string p; 
	if ((hFile = _findfirst(p.assign(path).append("/*"+format).c_str(), &fileinfo)) != -1)
	{
		do
		{
			if ((fileinfo.attrib & _A_SUBDIR))//判断是否为文件夹
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, ".") != 0)
				{
					GetAllFiles_CertainFormat((p.assign(path).append("/")).append(fileinfo.name), files, format);

				}
			}
			else
			{
				serialnumber.append(fileinfo.name);
				int num = getNumberInString(serialnumber,hasnumber);
				//cout << "number:" << num << endl;
				if(hasnumber==true)
					SerialNumber.push_back(num);
				else
				{
					SerialNumber.push_back(nonum);
				}
				p.assign(path).append("/").append(fileinfo.name);
				files.push_back(p);
			}
			nonum++;
			serialnumber.clear();

		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
	reRangeFileName(files, SerialNumber);
	cout << "-------------------------------------------------" << endl;
	return 0;
}

//最后一次配准
int FinalRegistration(vector<PointXYZRGBPtr> Cloud_Temp_Sort1, PointXYZRGBPtr &Cloud_Temp_Last)
{
	PointXYZRGBPtr Cloud_Temp;
	vector<PointXYZRGBPtr> Cloud_temp_vector;
	Cloud_Temp = Cloud_Temp_Sort1[0];
	for (int i = 0; i<Cloud_Temp_Sort1.size(); i++)
		Cloud_temp_vector.push_back(Cloud_Temp_Sort1[i]);
	
	if (Cloud_Temp_Sort1.size() != 1)
	{
		for (int i = 0; i < Cloud_temp_vector.size()-1; i++)//for (int i = 0; i < PointAll.size(); i++)
		{
			cout << i << endl;
			cout << Cloud_temp_vector[i + 1]->size() << endl;
			DWORD PerStart = ::GetTickCount();
			ostringstream Registration_of;
			Registration_of << "Registration of " << i + 1 << " and " << i + 2 << endl;
			cout << Registration_of.str();
			PointXYZRGBPtr Cloud_after_Reg(new pcl::PointCloud<pcl::PointXYZRGB>);
			PointXYZRGBPtr Cloud_comblined(new pcl::PointCloud<pcl::PointXYZRGB>);
			Eigen::Matrix4f Trans_matrix_global = Eigen::Matrix4f::Identity(), Trans_matrix;
			AlignCloud(Cloud_Temp, Cloud_temp_vector[i + 1], Cloud_after_Reg, Cloud_comblined, Trans_matrix, true);
			cout << Cloud_temp_vector[i + 1]->size() << endl;
			*Cloud_Temp = *Cloud_comblined;
			DWORD PerEnd = ::GetTickCount();
			cout << "The time taken for PerRegistration is: " << PerEnd - PerStart << endl;
			ofstream fout("C:/Users/Zhihong MA/Desktop/data/1.txt");
			fout << Trans_matrix << endl;
		}

		*Cloud_Temp_Last = *Cloud_Temp;
	}
	else
		*Cloud_Temp_Last = *Cloud_Temp;
	return 0;
}

//两两配准的前步骤包括点云重排序
int PerRegistration(vector<PointXYZRGBPtr> PointAll, vector<PointXYZRGBPtr>  &Cloud_Temp_Sort1)
{
	int threadNumber = 4;
	vector<int> PointNumber;
	vector<vector<int>>threadnum(threadNumber);
	vector<vector<PointXYZRGBPtr>>OpenMpPointSet(threadNumber);
	vector<PointXYZRGBPtr> PointTemp;
#pragma omp parallel for num_threads(threadNumber)
	for (int i = 0; i < PointAll.size(); i = i + 2)
	{
		DWORD PerStart = ::GetTickCount();
		ostringstream Registration_of;
		Registration_of << "Registration of " << i + 1 << " and " << i + 2 << endl;//这种写法保证打印在同一行
		cout << Registration_of.str();
		PointXYZRGBPtr Cloud_after_Reg(new pcl::PointCloud<pcl::PointXYZRGB>);
		PointXYZRGBPtr Cloud_comblined(new pcl::PointCloud<pcl::PointXYZRGB>);
		Eigen::Matrix4f Trans_matrix_global = Eigen::Matrix4f::Identity(), Trans_matrix;
		AlignCloud(PointAll[i], PointAll[i + 1], Cloud_after_Reg, Cloud_comblined, Trans_matrix, false);//配准后两个点云数据统一在Cloud_comblined中
		int k = omp_get_thread_num();
		threadnum[k].push_back(i*11+1);
		OpenMpPointSet[k].push_back(Cloud_comblined);
		//Cloud_Temp_Sort1[i] = Cloud_comblined;
		//PointNumber.push_back(i);
		ofstream fout("C:/Users/Zhihong MA/Desktop/data/3/1.txt");
		fout << Trans_matrix << endl;

		DWORD PerEnd = ::GetTickCount();
		ostringstream TakenTime;
		TakenTime << "The time taken for PerRegistration is: " << PerEnd - PerStart << endl;
		cout << TakenTime.str();
	}
	for (int i = 0; i < 4; i++)
	{
		if (threadnum[i].size() != OpenMpPointSet[i].size())
		{
			cout << "The number is wrong!" << endl;
			return -1;
		}
		for (int j = 0; j < threadnum[i].size(); j++)
		{
			PointNumber.push_back(threadnum[i][j]);
			PointTemp.push_back(OpenMpPointSet[i][j]);
		}
	}
	for (int i = 0; i < PointTemp.size(); i++)
	{
		Cloud_Temp_Sort1.push_back(PointTemp[i]);
	}
	if (Cloud_Temp_Sort1.size() == 1)
	{
		return 0;
	}
	//重排序，依照点云标号从小到大排
	reRangeFileName(Cloud_Temp_Sort1, PointNumber);
	//if (PointNumber.size() != Cloud_Temp_Sort1.size())
	//{
	//	cout << "The number of PointNumber and Cloud_Temp_Sort1 is wrong" << endl;
	//	return -1;
	//}
	//for (int i = 0; i < PointNumber.size(); i++)
	//{
	//	for (int j = i; j < PointNumber.size(); j++)
	//	{
	//		if (PointNumber[i] >= PointNumber[j])
	//		{
	//			swap(PointNumber[i], PointNumber[j]);
	//			swap(Cloud_Temp_Sort1[i], Cloud_Temp_Sort1[j]);
	//		}
	//	}
	//}
	return 0;
}

//判断是否进行下一次配准并进行
int SortAndContinue(vector<PointXYZRGBPtr> Cloud_Temp_Sort1, vector<PointXYZRGBPtr> &Cloud_Temp_Sort2,
	vector<PointXYZRGBPtr>&PointLeftAll, PointXYZRGBPtr &Cloud_Temp_Last)
{
	PointXYZRGBPtr Cloud_Final_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	//先判断是个数为奇数并小于4的情况6
	if (Cloud_Temp_Sort1.size() % 2 == 1 && Cloud_Temp_Sort1.size() < 4)
	{
		if (!PointLeftAll.empty())
		{
			for (int i= PointLeftAll.size()-1;i>=0 ;i--)
				Cloud_Temp_Sort1.push_back(PointLeftAll[i]);
		}
		cout << "Cloud_Temp_Sort1.size(): " << Cloud_Temp_Sort1.size() << endl;
		FinalRegistration(Cloud_Temp_Sort1, Cloud_Final_temp);
		*Cloud_Temp_Last = *Cloud_Final_temp;
	}
	//然后判断个数是为奇数但是大于5 的情况
	else if (Cloud_Temp_Sort1.size() % 2 == 1 && Cloud_Temp_Sort1.size() >=4 )
	{
		PointLeftAll.push_back(Cloud_Temp_Sort1[Cloud_Temp_Sort1.size()-1]);
		Cloud_Temp_Sort1.pop_back();
		PerRegistration(Cloud_Temp_Sort1, Cloud_Temp_Sort2);
	}
	else
		PerRegistration(Cloud_Temp_Sort1, Cloud_Temp_Sort2);
	return 0;
}

int main_1()//单循环1先和2配准，配准结果在和3配准，一直到最后
{

	string pathdir = "F:/1-DesktopFile_Pointget/1-DATA/2017.11.11/50_5_2After";
	vector<string> FilesDirName;
	string format = ".pcd";
	GetAllFiles_CertainFormat(pathdir, FilesDirName, format);
	for (auto c : FilesDirName)
		cout << c << endl;
	cout << "------------" << endl;

	vector<PointXYZRGBPtr> PointAll;
	
	for (int i = 0; i < FilesDirName.size(); i++)
	{
		PointXYZRGBPtr cloud_IN(new pcl::PointCloud<pcl::PointXYZRGB>);
		int error=pcl::io::loadPCDFile(FilesDirName[i], *cloud_IN);
		if (error == -1)
		{
			PCL_WARN("Haven't load the Cloud First(The source one)!");
			return -1;
		}
		PointAll.push_back(cloud_IN);
	}

	PCL_INFO("Loaded");
	vector<PointXYZRGBPtr> PointAfterRegAll;
	PointXYZRGBPtr Cloud_Temp_Sort1(new pcl::PointCloud<pcl::PointXYZRGB>);
	Cloud_Temp_Sort1 = PointAll[0];
	DWORD Start = ::GetTickCount();
	for (int i = 0; i < 5; i++)//for (int i = 0; i < PointAll.size(); i++)
	{
		DWORD PerStart = ::GetTickCount();
		cout << "Registration of " << i + 1 << " and " << i + 2<< endl;
		PointXYZRGBPtr Cloud_after_Reg(new pcl::PointCloud<pcl::PointXYZRGB>);
		PointXYZRGBPtr Cloud_comblined(new pcl::PointCloud<pcl::PointXYZRGB>);
		Eigen::Matrix4f Trans_matrix_global = Eigen::Matrix4f::Identity(), Trans_matrix;
		AlignCloud(Cloud_Temp_Sort1, PointAll[i + 1], Cloud_after_Reg, Cloud_comblined, Trans_matrix, false);
		*Cloud_Temp_Sort1 = *Cloud_comblined;
		PointAfterRegAll.push_back(Cloud_after_Reg);
		DWORD PerEnd = ::GetTickCount();
		cout << "The time taken for PerRegistration is: " << PerEnd - PerStart << endl;
	}
	DWORD End = ::GetTickCount();
	cout << "The time taken for test is: " << End - Start << endl;
	string saveName = "C:/Users/Zhihong MA/Desktop/The_whole2.ply";
	pcl::io::savePLYFile(saveName, *Cloud_Temp_Sort1);
	return 0;
}

int main()//分80组，先1和2配准，3和4配准，4和6配准。。。然后结果12和34配准，56与78配准。。一直到最后位置，存在多个循环
{
	//string pathdir = "F:/1-DesktopFile_Pointget/1-DATA/2017.11.11/50_5_2After";
	//string pathdir = "F:/1-DesktopFile_Pointget/1-DATA/xlr data/New folder";
	string pathdir = "C:/Users/Zhihong MA/Desktop/data/3";
	vector<string> FilesDirName;
	string format = ".pcd";
	vector<int> SerialNumber;//用于存储序号
	GetAllFiles_CertainFormat(pathdir, FilesDirName, format);
	for (auto c : FilesDirName)
		cout << c << endl;
	cout << "------------" << endl;
	vector<PointXYZRGBPtr> PointAll;
	for (int i = 0; i < FilesDirName.size(); i++)
	{
		PointXYZRGBPtr cloud_IN(new pcl::PointCloud<pcl::PointXYZRGB>);
		int error = pcl::io::loadPCDFile(FilesDirName[i], *cloud_IN);
		if (error == -1)
		{
			PCL_WARN("Haven't load the Cloud First(The source one)!");
			return -1;
		}
		PointAll.push_back(cloud_IN);
	}
	PCL_INFO("Loaded");
	cout << endl;
	////用于测试预,先减少数量
	int savenum = 60;
	while (PointAll.size() > savenum)
	{
		PointAll.pop_back();
	}

	vector<PointXYZRGBPtr> Cloud_Temp_Sort1;
	vector<PointXYZRGBPtr> Cloud_Temp_Sort2;
	vector<PointXYZRGBPtr> Cloud_Temp_Sort3;
	vector<PointXYZRGBPtr> Cloud_Temp_Sort4;
	vector<PointXYZRGBPtr> Cloud_Temp_Sort5;
	vector<PointXYZRGBPtr> Cloud_Temp_Sort6;
	vector<PointXYZRGBPtr> Cloud_Temp_Sort7;
	vector<PointXYZRGBPtr> Cloud_Temp_Sort8;
	vector<PointXYZRGBPtr> Cloud_Temp_Sort9;
	PointXYZRGBPtr Cloud_Temp_Last(new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<PointXYZRGBPtr >PointLeftAll;

	DWORD Start = ::GetTickCount();
	//第1次分组配准
	if (PointAll.size() % 2 == 1)
	{
		PointLeftAll.push_back(PointAll[PointAll.size()-1]);
		PointAll.pop_back();
	}
	PerRegistration(PointAll, Cloud_Temp_Sort1);
	//第2次分组配准
	cout << "The Second  registration " << endl;
	SortAndContinue(Cloud_Temp_Sort1, Cloud_Temp_Sort2, PointLeftAll, Cloud_Temp_Last);
	//第3次分组配准
	cout << "The Third  registration " << endl;
	SortAndContinue(Cloud_Temp_Sort2, Cloud_Temp_Sort3, PointLeftAll, Cloud_Temp_Last);
	//第4次分组配准
	cout << "The Fourth  registration " << endl;
	SortAndContinue(Cloud_Temp_Sort3, Cloud_Temp_Sort4, PointLeftAll, Cloud_Temp_Last);
	//第5次分组配准
	cout << "The Fifth  registration " << endl;
	SortAndContinue(Cloud_Temp_Sort4, Cloud_Temp_Sort5, PointLeftAll, Cloud_Temp_Last);
	//第6次分组配准
	cout << "The Sixth  registration " << endl;
	SortAndContinue(Cloud_Temp_Sort5, Cloud_Temp_Sort6, PointLeftAll, Cloud_Temp_Last);
	//第7次分组配准
	cout << "The Seventh  registration " << endl;
	SortAndContinue(Cloud_Temp_Sort6, Cloud_Temp_Sort7, PointLeftAll, Cloud_Temp_Last);
	//第8次分组配准
	cout << "The Eighth  registration " << endl;
	SortAndContinue(Cloud_Temp_Sort7, Cloud_Temp_Sort8, PointLeftAll, Cloud_Temp_Last);
	//第9次分组配准
	cout << "The Nineth  registration " << endl;
	SortAndContinue(Cloud_Temp_Sort8, Cloud_Temp_Sort9, PointLeftAll, Cloud_Temp_Last);

	DWORD End = ::GetTickCount();
	cout << "The time taken for test is: " << End - Start << endl;
	ostringstream saveName;
	saveName << "C:/Users/Zhihong MA/Desktop/data/" << savenum << ".pcd";
	//saveName << "C:/Users/Zhihong MA/Desktop/The_whole" << savenum << ".ply";
	pcl::io::savePCDFile(saveName.str(), *Cloud_Temp_Last);
	return 0;
}