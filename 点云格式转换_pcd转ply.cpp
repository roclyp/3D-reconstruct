#include "PCLlibrary.h"
#include <iostream>
#include <string>
#include <pcl/console/time.h>   // TicToc
#include <io.h>  
#include <direct.h>  
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBPtr;

//从一串字符串（包含英文与数字，甚至可能还有中文）获取指定位数的数字
int getNumberInString(string istring, bool &hasnumbr)
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
int GetAllFiles_CertainFormat(string path, vector<string>& files, string format)
{
	intptr_t hFile = 0;//_findnext返回类型为intprt_t,而非long类型，从intptr_t转换到long类型丢失数据
	struct  _finddata_t fileinfo;
	vector<int> SerialNumber;//用于存储序号
	string serialnumber;//用于排序
	bool hasnumber = true;//用于判断文件名中是否有数字
	int nonum = 0;//用于在没有数字时排序
	string p;
	if ((hFile = _findfirst(p.assign(path).append("/*" + format).c_str(), &fileinfo)) != -1)
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
				int num = getNumberInString(serialnumber, hasnumber);
				//cout << "number:" << num << endl;
				if (hasnumber == true)
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



int main()
{
	//输入
	string pathdir = "F:/1-DesktopFile_Pointget/1-DATA/2017.11.11/50_2_2After";
	pathdir = "C:\\Users\\zhihong\\Desktop\\hupilan";
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

	string dirName;
	dirName.append(pathdir);
	dirName.append("/PLY2");
	if (_access(dirName.c_str(), 0) == -1)
	{
		int i=_mkdir(dirName.c_str());
	}
	//输出保存
	string saveformat = ".ply";
	vector<string> SaveFilesDirName;
	for (int i = 0; i < FilesDirName.size(); i++)
	{
		SaveFilesDirName.push_back(FilesDirName[i]);
		SaveFilesDirName[i].erase(FilesDirName[i].size() - 4, 4);
		SaveFilesDirName[i].replace(0, pathdir.size(),dirName);
		SaveFilesDirName[i].append(saveformat);
	}
	cout << SaveFilesDirName.size() << endl;

	int threadNumber = 4;
#pragma omp parallel for num_threads(threadNumber)
	for (int i = 0; i < SaveFilesDirName.size(); i++)
	{
		//pcl::io::savePLYFileBinary(SaveFilesDirName[i], *PointAll[i]);
		pcl::io::savePLYFile(SaveFilesDirName[i], *PointAll[i]);
	}

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_first(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_second(new pcl::PointCloud<pcl::PointXYZRGB>);
	///*string filepath1 = "F:/DesktopFile_Pointget/1-DATA/2017-10-18/2017-1-after/A2017-1After_2017-7.pcd";
	//string filepath2 = "F:/DesktopFile_Pointget/1-DATA/2017-10-18/2017-1-after/A2017-1After_2017-8.pcd";*/
	//string filepath1 = "F:/1-DesktopFile_Pointget/1-DATA/2017.11.11/50_2_2After/After_2017.11.11_1.pcd";
	//string filepath2 = "F:/1-DesktopFile_Pointget/1-DATA/2017.11.11/50_2_2After/After_2017.11.11_2.pcd";

	//int error1 = pcl::io::loadPCDFile(filepath1, *cloud_first);
	//int error2 = pcl::io::loadPCDFile(filepath2, *cloud_second);
	//if (error1 == -1)
	//{
	//	PCL_WARN("Haven't load the Cloud First(The source one)!");
	//	return -1;
	//}
	//if (error2 == -1)
	//{
	//	PCL_WARN("Haven't load the Cloud Second(The Target one)!");
	//	return -1;
	//}
	//PCL_INFO("Loaded");
	//string saveName1 = "C:/Users/Zhihong MA/Desktop/After_2017.11.11_1.obj";
	//string saveName2 = "C:/Users/Zhihong MA/Desktop/After_2017.11.11_2.obj";
	//pcl::io::savePLYFileBinary(saveName1, *cloud_first);
	//pcl::io::savePLYFileBinary(saveName2, *cloud_second);
	return 0;
}