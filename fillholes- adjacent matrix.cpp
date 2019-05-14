#include "fillholes.h"
#include <vector>
#include <algorithm>

//void getedge(int p1id,int p2id, myedge &edge) 
//{
//	edge=myedge (p1id, p2id);
//}

//建立图
//图类型
#define Maxsize 200000;
typedef struct Gnode *PtrGnode;
struct Gnode
{
	int Nv; //顶点数
	int Ne; //边数
	int G[15000][15000];
};
typedef PtrGnode Mgraph;

typedef int Vertex;
//初始化得到只有点没有边的图
Mgraph CreatGraph(int vertexnum)
{
	Vertex v, w;
	Mgraph mg;

	mg = (Mgraph)malloc(sizeof(Gnode));
	mg->Ne = vertexnum;
	mg->Ne = 0;
	for (v = 0; v < mg->Nv; v++)
	{
		for (w = 0; w < mg->Nv; w++)
		{
			mg->G[v][w] = 0;
		}
	}
	return mg;
}

//边类型
typedef struct  Enode *PtrEnode;
struct Enode
{
	Vertex V1, V2;
	int power;
};
typedef PtrEnode Edge;

void InsertEdge(Mgraph mg, myedge halfEdges)
{
	mg->G[halfEdges.p1id][halfEdges.p2id] = 1;
	mg->G[halfEdges.p2id][halfEdges.p1id] = 1;
}


bool isSameEdge(myedge e1, myedge e2)
{
	if (e1.p1id == e2.p1id&&e1.p2id == e2.p2id)
		return true;
	else
		return false;
}

bool isInfront(myedge e1, myedge e2)
{
	if (e1.p1id == e2.p1id)
	{
		if (e1.p2id <= e2.p2id)
			return true;
		else
			return false;
	}
	else if (e1.p1id < e2.p1id)
		return true;
	else
		return false;
}

void getHalfEdge(PolygonMesh &mesh1, std::vector<myedge> &halfedges_out)
{
	// get halfedge;
	std::vector<myedge> halfedges;
	for (int i = 0; i < mesh1.polygons.size(); i++)
	{
		int p1 = mesh1.polygons.at(i).vertices[0];
		int p2 = mesh1.polygons.at(i).vertices[1];
		int p3 = mesh1.polygons.at(i).vertices[2];
		myedge e1(p1, p2);
		myedge e2(p2, p3);
		myedge e3(p3, p1);
		if (e1.edge_valid == 0)
			halfedges.push_back(e1);
		if (e2.edge_valid == 0)
			halfedges.push_back(e2);
		if (e3.edge_valid == 0)
			halfedges.push_back(e3);
	}
	std::vector<myedge>::iterator begin_pos = halfedges.begin();
	std::vector<myedge>::iterator end_pos = halfedges.end();
	std::sort(halfedges.begin(), halfedges.end(), isInfront);
	// above process is to get all edges(includeing halfedges and whole edges) and take them in order
	// if the edgd showed twice, it is not the halfedges

	std::vector<myedge> halfedges_copy;
	halfedges_copy = halfedges;

	std::vector<myedge>::iterator new_end = std::unique(halfedges.begin(), halfedges.end(), isSameEdge);
	halfedges.erase(new_end, end_pos);

	// Counts
	for (int i = 0; i < halfedges.size() - 1; i++)
	{
		auto begin_temp =
			find(halfedges_copy.begin(), halfedges_copy.end(), halfedges.at(i));
		auto end_temp =
			find(halfedges_copy.begin(), halfedges_copy.end(), halfedges.at(i + 1));
		halfedges.at(i).edge_times = end_temp - begin_temp;
	}

	// get halfedges
	//std::vector<myedge> halfedges_out;
	for (int i = 0; i < halfedges.size() - 1; i++)
	{
		if (halfedges.at(i).edge_times == 1)
			halfedges_out.push_back(halfedges.at(i));
	}
	cout << halfedges_out.size() << endl;
}

struct PL //PointList
{
	int id;
	PL *nest;
};

//// map图，起点与终点id，当前搜索的数组头id，输出数组，数组内点数
//void gethole(std::vector<std::vector<int>> &holemap,int sameid, int stID, std::vector<int>outhole, int hole_pts)
//{
//	//如果邻接表的某一项符合sameid，那么判断outhole里面的点数是否大于3，若大于3，则判断是hole；
//	std::vector<int> temp = holemap.at(stID);
//	for (int i=0;i<temp.size();i++)
//	{
//		std::vector<int> tempout;
//		if (temp.at(i) == sameid && outhole.size() >= 3)
//			return;
//		tempout.push_back(temp.at(i));
//		hole_pts++;
//		gethole(holemap, sameid, temp.at(i), tempout, hole_pts);
//	}
//
//}


//int total = sizeof(ph->G);
//int col = sizeof(ph->G[0]) / sizeof(int);
//int row = total / col;
//深度优先搜索得到孔洞
//邻接矩阵方式
void DeleteEndPoint(Mgraph &Ph,int col,int row)
{
	
	std::vector<int> delete_pts; //获取要删除的点
	for (int i=0;i<row;i++) // 邻接矩循环
	{
		int pts = 0;
		for (int j=0;j<col;j++)
		{
			if (Ph->G[i][j] == 1)
				pts++;
		}
		if (pts==1) //当数量为1的时候说明这个点只有一个邻接点相连，则为该点为端点
		{
			delete_pts.push_back(i);
		}
	}

	if (delete_pts.size()!=0)//删除端点
	{
		for (int i = 0; i < delete_pts.size(); i++)
		{
			for (int j = 0; j < row; j++)
			{
				Ph->G[delete_pts.at(i)][j] = 0;
				Ph->G[j][delete_pts.at(i)] = 0;
			}
		}
		DeleteEndPoint(Ph, col, row);// 递归
	}
	else
		return;
}

void getHolesmap(std::vector<myedge> &halfedges, Mgraph &ph, std::vector<std::vector<int>> &holemap2)
{
	std::vector<int>pts;
	for (int i = 0; i < halfedges.size(); i++)
	{
		pts.push_back(halfedges.at(i).p1id);
		pts.push_back(halfedges.at(i).p2id);
	}
	sort(pts.begin(), pts.end());
	pts.erase(unique(pts.begin(), pts.end()), pts.end());

	Mgraph graph;
	graph = CreatGraph(pts.size());
	graph->Ne = halfedges.size();
	if (graph->Ne!=0)
	{
		for (int i = 0; i < graph->Ne; i++)
		{
			InsertEdge(graph, halfedges.at(i));
		}
	}
	int total = sizeof(ph->G) / sizeof(int);
	int col = sizeof(ph->G[0]) / sizeof(int);
	int row = total / col;
	DeleteEndPoint(graph, col, row);
	ph=graph;
	
	std::vector<int> gsize;
	for (int i=0;i<row;i++)
	{
		int ptssize = 0;
		for (int j=0;j<col;j++)
		{
			if (ph->G[i][j] == 1)
				ptssize++;
		}
		if (ptssize!=0)
		{
			gsize.push_back(ptssize);
		}
	}
	//上面过程应该是正确的

	std::vector<std::vector<int>> holemap(pts.size());
	for (int i=0;i< pts.size();i++)
	{
		for (int j=0;j< pts.size(); j++)
		{
			if (ph->G[i][j]==1)
			{
				holemap.at(i).push_back(j);
			}
		}
	}

	std::vector<myedge> halfedges_holes;
	for (int i=0;i<holemap.size();i++)
	{
		if (holemap.at(i).size() != 0)
		{
			for (int j=0;j< holemap.at(i).size();j++)
			{
				myedge e(i, holemap.at(i).at(j));
				halfedges_holes.push_back(e);
			}
		}
	}

	holemap2 = holemap;

}



//void getHoles(std::vector<myedge> &halfedges, std::vector<std::vector<myedge>> &holes)
//{
//	std::vector<int>pts;
//	for (int i=0;i<halfedges.size();i++)
//	{
//		pts.push_back(halfedges.at(i).p1id);
//		pts.push_back(halfedges.at(i).p2id);
//	}
//	sort(pts.begin(), pts.end());
//	pts.erase(unique(pts.begin(), pts.end()),pts.end());
//	std::vector<std::vector<int>> holemap(pts.size());
//	for (int i=0;i<pts.size();i++)
//	{
//		for (int j=i+1;j< pts.size(); j++)
//		{
//			if (find(halfedges.begin(), halfedges.end(), myedge(i, j, 1)) != halfedges.end())
//			{
//				holemap.at(i).push_back(j);
//				holemap.at(j).push_back(i);
//			}
//		}
//	}
//	// the above is to get hole map	
//	std::vector<std::vector<int>>tempholeall;
//	for (int i=0;i<holemap.size();i++)
//	{
//		if(holemap.at(i).size()<2|| holemap.at(i).size()%2!=0)
//			continue;
//		std::vector<int>temphole;
//		//gethole()
//	}
//}

void fillholes(PolygonMesh &mesh1, PointCloud<PointXYZRGB>::Ptr &cloud, PolygonMesh &meshout)
{
	std::vector<myedge> halfedges;
	getHalfEdge(mesh1, halfedges);

	//  std::vector<std::vector<myedge>> holes;
	//	getHoles(halfedges,holes);
	Mgraph ph;
	std::vector<std::vector<int>> holemap;
	getHolesmap(halfedges, ph, holemap);


}

