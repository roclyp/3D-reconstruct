#include "PCLlibrary.h"
#include <iostream>
#include <string>
#include <pcl/console/time.h>   // TicToc
#include <time.h>
#include <pcl/search/kdtree.h>
#include <direct.h> 
#include "MyPointType.h"
using namespace std;

#define Pi 3.141592657;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointXYZRGBPtr;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointXYZPtr;


void ConfigFileRead(map<string, string>& m_mapConfigInfo)
{
	ifstream configFile;
	string path = "./聚类.ini";
	configFile.open(path.c_str());
	string str_line;
	if (configFile.is_open())
	{
		while (!configFile.eof())
		{
			getline(configFile, str_line);
			if (str_line.find('#') == 0) //过滤掉注释信息，即如果首个字符为#就过滤掉这一行
			{
				continue;
			}
			size_t pos = str_line.find('=');
			string str_key = str_line.substr(0, pos);
			string str_value = str_line.substr(pos + 1);
			m_mapConfigInfo.insert(pair<string, string>(str_key, str_value));
		}
	}
	else
	{
		cout << "Cannot open config file setting.ini, path: ";
		exit(-1);
	}
}

//计算两个点云之间距离
template <class MyPoint>
inline double getdis(MyPoint &clouda, MyPoint &cloudb) {
	double x = clouda.x - cloudb.x;
	double y = clouda.y - cloudb.y;
	double z = clouda.z - cloudb.z;
	double dis = sqrt(x * x + y * y + z * z);
	return dis;
}

//判断在那个区间，输入是距离，步长距离，总步长
int inwhichPart(double &dis, double &aveerange,int &k)
{
	int part = 0;
	for (int i = 0; i < k - 1; i++)
	{
		double tempmin = i * aveerange;
		if (dis >= (i * aveerange) && dis < ((i + 1)* aveerange))
		{
			part = i;
			break;
		}
		else
		{
			part = k - 1;
		}
	}
	return part;
}

//获取初始半径
double geter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &Cloud,int segparts)
{
	int k = segparts;
	double maxdis = 0;
	double mindis = 10000;
	omp_set_num_threads(8);
#pragma omp parallel for
	for (int i = 0; i < Cloud->size(); i++)
	{
		for (int j = i+1; j < Cloud->size(); j++)
		{
			if(i==j)
				continue;
			double dis = getdis(Cloud->at(i), Cloud->at(j));
			if (dis >= maxdis)
				maxdis = dis;
			if (dis <= mindis)
				mindis = dis;
		}
	}
	double disrange = maxdis - mindis;
	double aveerange = disrange / k;

	vector<int> rate(k,0);	
	for (int i = 0; i < Cloud->size(); i++) {
		for (int j = i+1; j < Cloud->size(); j++)
		{
			double dis = getdis(Cloud->at(i), Cloud->at(j));
			int finalpart = inwhichPart(dis,aveerange,segparts);
			rate.at(finalpart)++;
		}
	}
	int maxrate = rate.at(0);
	int maxid = 0;
	for (int i = 1; i < k; i++)
	{
		if (rate.at(i) >= maxrate)
		{
			maxrate = rate.at(i);
			maxid = i;
		}
	}
	//获得中值
	vector<double>partdis;
	for (int i = 0; i < Cloud->size(); i++) {
		for (int j = i+1; j < Cloud->size(); j++)
		{
			double dis = getdis(Cloud->at(i), Cloud->at(j));
			int finalpart = inwhichPart(dis, aveerange, segparts);
			if (finalpart == maxid)
				partdis.push_back(dis);
		}
	}
	sort(partdis.begin(), partdis.end());
	int pos = partdis.size() / 2;
	double erange = partdis.at(pos);
	return erange;
}

int getminpts(map<int, int> &pnumimap, int cloudsize) {

	//将Map存入vector中，提高速度
	vector<int>pnumi;
	for (int j = 0; j < pnumimap.size(); j++) {
		int c = pnumimap[j];
		pnumi.push_back(c);
	}
	map<int, int> pnum_minpts;
	for (int minpts = 1; minpts <= cloudsize * 0.95; minpts++)
	{
		int temcount = 0;
		for (int j = 0; j < pnumi.size(); j++) {
			int c = pnumi[j];
			if (c >= minpts)
				temcount++;
		}
		pnum_minpts[minpts] = temcount;
	}
	vector<int> pos;
	vector<int> pos2;
	vector<int> pos3;
	for (int i = pnum_minpts.size()*0.1; i < pnum_minpts.size()*0.9; i++)
	{
		int step = pnum_minpts.size()*0.075;
		int orix = i;
		int oriy = pnum_minpts[i];
		int lastx = i - 1;
		int lasty = pnum_minpts[lastx];
		int nextx = i + 1;
		int nexty = pnum_minpts[nextx];
		double slope = (lasty - nexty)*1.0 / 2;
		int slopelasty = lasty - oriy;
		int slopenexty = oriy - nexty;
		if (slope == 1)
		{
			pos.push_back(i);
		}
		else if (slope == 2)
		{
			pos2.push_back(i);
		}
		else if (slope == 3)
		{
			pos3.push_back(i);
		}
	}
	int minpts = pos2.at(pos2.size() - 1)*(1 - 1.1 - 0.9) + pos.at(pos.size() - 1)*0.9 + pos3.at(pos3.size() - 1)*1.1;
	return minpts;
}

//是否是中心点
bool isCenter(pcl::PointCloud<myPointCloud>::Ptr &tempcloud, int &id, double &iniRadius, int &minpts,vector<int> &outnerid)
{
	//原始点
	myPointCloud oripoint = tempcloud->at(id);
	vector<int> tempoutID;
	int counts = 0;
	for (int i= 0; i < tempcloud->size(); i++) {
		double dist;
		if (i == id)
			continue;
		else
		{
			dist = getdis(oripoint, tempcloud->at(i));
		}
		if (dist <= iniRadius)
		{
			tempoutID.push_back(i);
			counts++;
		}
	}
	//只有是中心点的时候，该点的领域点才会加入到outneipoint中
	if (counts >= minpts)
	{
		tempcloud->at(id).istaged = true;
		for (int i = 0; i < tempoutID.size(); i++)
		{
			//如果为在neibor容器中找到序号i，则在该容器中存入序列i，否则不存
			if (find(outnerid.begin(), outnerid.end(), tempoutID.at(i)) == outnerid.end())
			{
				int idd = tempoutID.at(i);
				outnerid.push_back(idd);
			/*	sort(outnerid.begin(), outnerid.end());
				outnerid.erase(unique(outnerid.begin(), outnerid.end()), outnerid.end());*/
			}
		}
		return true;
	}
	else
		return false;
}

//分割点云
void segPointCloud(pcl::PointCloud<myPointCloud>::Ptr &tempcloud, int &id, double &iniRadius, int &minpts,int &ClusterId, pcl::PointCloud<myPointCloud>::Ptr &outPoint) {
	myPointCloud oripoint = tempcloud->at(id);
	vector<int>neiPointId;
	set<int> neiPointSet;
	if (isCenter(tempcloud, id, iniRadius, minpts, neiPointId) == true)//判断中心点
	{
		tempcloud->at(id).istaged = true;//做标记
		tempcloud->at(id).clusterId = ClusterId;//赋予ID
		for (int i = 0; i < neiPointId.size(); i++) {
			tempcloud->at(neiPointId.at(i)).istaged = true;//领域点做标记
			//tempcloud->at(neiPointId.at(i)).clusterId = ClusterId;//领域点赋予ID
		}
	}
	else
	{
		//如果不是中心点那么为边界点，那么ID为0
		tempcloud->at(id).istaged = true;//做标记
		tempcloud->at(id).clusterId = 0;//id赋值为0，表示为边界点，直接退出
		return;
	}
	int iniNeiSize = neiPointId.size();
	omp_set_num_threads(8);
#pragma omp parallel for
	//搜索所有领域点，并进行判断标记
	for (int i = 0; i < neiPointId.size(); i++)
	{
		cout << "Point: " << id << " neiborid: " << i << " Size: " << neiPointId.size() << endl;
		myPointCloud tempneipoint = tempcloud->at(neiPointId.at(i));
		if (isCenter(tempcloud, id, iniRadius, minpts, neiPointId) == true)
		{
			//如果领域点也是中心点，那么将领域点的领域点加入id该点的领域集合中，否则不变化，领域点的clusterID仍与id该点一致
			tempcloud->at(neiPointId.at(i)).clusterId = ClusterId;//领域点赋予ID
			for (int i = iniNeiSize; i < neiPointId.size(); i++) {
				if(tempcloud->at(neiPointId.at(i)).istaged != true)
					tempcloud->at(neiPointId.at(i)).istaged = true;
				else
					continue;
			}
		}
		sort(neiPointId.begin(), neiPointId.end());
		neiPointId.erase(unique(neiPointId.begin(), neiPointId.end()), neiPointId.end());
		iniNeiSize = neiPointId.size();
	}
	outPoint->push_back(oripoint);
	for (int i = 0; i < neiPointId.size(); i++)
	{
		outPoint->push_back(tempcloud->at(neiPointId.at(i)));
	}
	return ;
}

inline void transformRGBtoMytype(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudin, pcl::PointCloud <myPointCloud>::Ptr &tempcloud)
{
	for (int i = 0; i < cloudin->size(); i++)
	{
		myPointCloud temp;
		temp.x = cloudin->at(i).x;
		temp.y = cloudin->at(i).y;
		temp.z = cloudin->at(i).z;
		temp.r = cloudin->at(i).r;
		temp.g = cloudin->at(i).g;
		temp.b = cloudin->at(i).b;
		tempcloud->push_back(temp);
	}
}

int main()
{

	cout << "Ready.....";
	//getchar();
	cout << "Start working" << endl;
	time_t t1 = GetTickCount();

	//获取文件路径与参数配置
	map<string, string> param;
	ConfigFileRead(param);
	string filepath = param["filepath"], outfilepath = param["outfilepath"];
	istringstream segpartsstr(param["segparts"]);
	int segparts;
	segpartsstr >> segparts;

	//读取点云文件
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<myPointCloud>::Ptr cloud(new pcl::PointCloud<myPointCloud>);
	int errors = pcl::io::loadPCDFile(filepath, *cloudin);
	if (errors == -1)
	{
		cout << "Can't find file" << endl;
		return -1;
	}

	//Kdtree计算临近点
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree1(new pcl::search::KdTree<pcl::PointXYZRGB>);
	kdtree1->setInputCloud(cloudin);

	//获取自适应半径
	map<int, int>pnumimap;
	vector<int> erangenum(segparts);
	double er = geter(cloudin,20);//获取erang

	//获取自适应最小聚类点
	for (int i = 0; i < cloudin->size(); i++)
	{
		//double er = geter(cloudin, i, 20);//获取id为i的点的erang
		//获取半径内全部的id
		vector<int>kdpointID;
		vector<float>kdpointDis;
		kdtree1->radiusSearch(cloudin->at(i), er, kdpointID, kdpointDis, 0);
		//pnumi.push_back(kdpointID.size() - 1);//获取这个点对应的erang范围内的点的数量
		pnumimap[i] = kdpointID.size() - 1;
	}
	//大约估计拐点
	int cloudsize = cloudin->size();
	int minpts = getminpts(pnumimap, cloudsize);


	//密度聚类-生长算法
	vector<pcl::PointCloud<myPointCloud>::Ptr> segPointCloudAll;
	pcl::PointCloud <myPointCloud>::Ptr tempcloud(new pcl::PointCloud <myPointCloud>);
	transformRGBtoMytype(cloudin,tempcloud);
	int ClusterId = 1;
	for (int i = 0; i < tempcloud->size(); i++)
	{
		if(tempcloud->at(i).istaged==true)
			continue;
		pcl::PointCloud<myPointCloud>::Ptr tempsegcloud(new pcl::PointCloud<myPointCloud>);
		segPointCloud(tempcloud,i,er,minpts, ClusterId, tempsegcloud);
		if(tempsegcloud->size()!=0)
			segPointCloudAll.push_back(tempsegcloud);
	}
	pcl::PointCloud <myPointCloud>::Ptr outcloud(new pcl::PointCloud <myPointCloud>);
	for (int i = 0; i < segPointCloudAll.size(); i++)
	{
		auto tempa = segPointCloudAll.at(i);
		for (int j = 0; j < tempa->size(); j++)
		{
			tempa->at(j).r = 255 - 30 * i;
			tempa->at(j).g = 255-30*i;
			tempa->at(j).b = 10 + 30 * i;
			outcloud->push_back(tempa->at(j));
		}
	}
	pcl::io::savePCDFileBinary(outfilepath,*outcloud);
}