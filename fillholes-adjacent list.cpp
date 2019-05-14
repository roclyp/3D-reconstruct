#pragma once
#include "fillholes.h"
#include <vector>
#include <algorithm>
#include "hole_Graph_type.h"

//void getedge(int p1id,int p2id, myedge &edge) 
//{
//	edge=myedge (p1id, p2id);
//}


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
	for (int i = 0; i < halfedges.size() - 1; i++)
	{
		if (halfedges.at(i).edge_times == 1)
			halfedges_out.push_back(halfedges.at(i));
	}
	cout << halfedges_out.size() << endl;
}

int FindMaxValue(std::vector<int>pts)
{
	int max = pts.at(0);
	for (int i = 1; i < pts.size(); i++)
	{
		if (pts.at(i) >= max)
			max = pts.at(i);
	}
	return max;
}

////深度优先搜索得到孔洞
//void DFS_getHole(PtrToAdjVnode stnode, Lgraph ph)
//{
//	//从节点stnode开始进行DFS搜索
//	PtrToAdjVnode v;
//	//stnode
//	//if (stnode->Visited==0)
//	//{
//	//	
//	//}
//
//
//}

//标记含有复杂点的闭环圈，即圈内存在被共用次数＞2的点
void dfs_complex_holes(Vnode v, Lgraph listin)
{
	//因为进入这个递归过程中的点，都是复杂情况下的闭环的点，因此标记这些点为复杂闭环圈；
	//标记完后递归
	PtrToAdjVnode w;
	if (v.isComplex==0)
	{
		listin->G[v.id].isComplex = 1;// v.isComplex = 1;
	}
	else
	{
		return;
	}
	for (w = v.FirstEdge; w!=NULL; )
	{
		auto temp = listin->G[w->AdjV];
		if ((listin->G[w->AdjV].isComplex == 0))
		{
			dfs_complex_holes(listin->G[w->AdjV], listin);
			if (w->Next == NULL)
			{
				break;
			}
			else
			{
				w = w->Next;
			}
		}
		else
		{
			if (w->Next == NULL)
			{
				break;
			}
			else
			{
				w = w->Next;
				continue;
			}
		}
	}
}
//获取复杂点
void get_complex_holes(Lgraph listin, int searchtimes)
{

	for(int i = 0; i < searchtimes; i++)
	{
		int len = getlen(listin->G[i]);
		if (len == 0)
			continue;
		else
		{
			//将遍历点都从复杂点开始，即被共用次数为4,6等，除0外
			if (len != 2)
				dfs_complex_holes(listin->G[i], listin);
		}
	}
}

//获取闭环
void DFS_hole(Vnode v, Lgraph listin, std::vector<Vnode> &out,
	std::vector<std::vector<Vnode>> &holes)//此处仅考虑len=2的情况
{
	//如果该节点是复杂点，则不进入搜索闭环的环节；
	if (v.isComplex==1)
	{
		return;
	}
	int len = getlen(v); //获取节点后面的长度，长度为2,4,6
	if (len == 2)//若这个节点长度是2，说明该节点被4条边共用，属于情况1
	{
		PtrToAdjVnode w;
		if (v.Visited==0)
		{
			listin->G[v.id].Visited = 1;
			out.push_back(v);
		}
		for (w = v.FirstEdge; w != NULL; )
		{
			if (listin->G[w->AdjV].Visited == 0)
			{
				DFS_hole(listin->G[w->AdjV], listin, out, holes);
			}
			if (w->Next == NULL)
			{
				break;
			}
			else
			{
				w = w->Next;
			}
		}
				//visit=0说明该节点还未访问过，添加到输出vector out中这个是第一个节点
				/*out.push_back(v);
				listin->G[v.id].Visited += 1;
				w= listin->G[v.FirstEdge->AdjV].FirstEdge;*/
				//if (listin->G[v.FirstEdge->AdjV].Visited == 0)
				//{
				//	v = listin->G[v.FirstEdge->AdjV];
				//}
				//else if (listin->G[v.FirstEdge->AdjV].Visited == 1
				//	&& listin->G[vnext.FirstEdge->AdjV].Visited == 0)
				//{
				//	v = listin->G[vnext.FirstEdge->AdjV];
				//}
				//else
				//{
				//	return;
				//}
		//		DFS_hole(v, listin, out, holes);
		//	}
		//}
		//// 由于length=2，则所有点只有3个节点，即除本身外还有两个节点，因此这两个节点都要考虑进去
		//auto vnext = listin->G[v.FirstEdge->AdjV];
		//auto vnext_temp = vnext.FirstEdge;
		//auto vnext_next = listin->G[vnext.FirstEdge->AdjV];
		////visit=0说明该节点还未访问过，添加到输出vector out中这个是第一个节点
		//if (v.Visited == 0)
		//{
		//	out.push_back(v);
		//	listin->G[v.id].Visited+=1;
		//	if (listin->G[v.FirstEdge->AdjV].Visited == 0)
		//	{
		//		v = listin->G[v.FirstEdge->AdjV];
		//	}
		//	else if(listin->G[v.FirstEdge->AdjV].Visited == 1
		//		&& listin->G[vnext.FirstEdge->AdjV].Visited == 0)
		//	{
		//		v = listin->G[vnext.FirstEdge->AdjV];
		//	}
		//	else
		//	{
		//		return;
		//	}
		//	DFS_hole(v, listin, out, holes);
		//}
		////如果visit=1，说明该节点已经被访问，则访问该节点的下一节点
		////visit=1说明该节点已经被访问，且又回到该节点，那说明该节点是起点且又回到起点，则out里一定有一个闭环，则holes中添加该闭环，并退出递归
		//else if (v.Visited == 1 && vnext.Visited == 0)
		//{
		//	//out.push_back(listin->G[vnext->AdjV]);
		//	//listin->G[vnext->AdjV].Visited += 1;
		//	v = listin->G[v.FirstEdge->AdjV];
		//	DFS_hole(v, listin, out, holes);
		//}
		//else if(v.Visited == 1 && vnext.Visited == 1
		//	&& vnext_next.Visited == 0)
		//{
		//	//out.push_back(listin->G[vnext_next->AdjV]);
		//	//listin->G[vnext_next->AdjV].Visited += 1;
		//	v = listin->G[vnext.FirstEdge->AdjV];
		//	DFS_hole(v, listin, out, holes);
		//}
		//else
		//{
		//	return;
		//}
		////如果访问≥1次，在len=2时不存在这种情况，所以直接return；
		////不存在的原因时候len=4或len=6时会在生成一个闭环时终止递归，因此对于所有len=2的点最多只可能变量2次
	}
}

void get_holes(Lgraph ph, std::vector<std::vector<Vnode>> &holes)
{
	for (int i = 0; i < MaxVertexuUm; i++)
	{
		std::vector<Vnode> temp;
		DFS_hole(ph->G[i], ph, temp, holes);
		if (temp.size()!=0)
		{
			holes.push_back(temp);
		}
	}
}

int recursion_times = 0;
void DeleteEndPoint(Lgraph gp, int searchsize)
{
	cout << "Recursion times is: " << recursion_times << endl;
	std::vector<int> deleteID;
	// 由于find的得到的是第几个节点，因此得到n还需+1，因此链表下标从0开始
	for (int i1 = 0; i1 < searchsize; i1++)
	{
		int len = getlen(gp->G[i1]);
		if (len==1)
		{
			deleteID.push_back(i1);
		}
	}
	if (deleteID.size() != 0)
	{
		for (int i = 0; i < deleteID.size(); i++)
		{
			for (int j = 5485; j < searchsize; j++)
			{
				int len = getlen(gp->G[j]);//当邻接表表长为0的时候说明是空表，
				if(len==0)
					continue;
				auto temp = delete_Vertex(deleteID.at(i), gp->G[j].FirstEdge);
				gp->G[j].FirstEdge = temp;
				gp->G[deleteID.at(i)].FirstEdge = NULL;
				/*if (gp->G[j].FirstEdge->Next == NULL)
					gp->G[j].FirstEdge = NULL;*/
			}
		}
		recursion_times++;
		DeleteEndPoint(gp, searchsize);
		
	}
	else
		return;

}

void getHolesmap(std::vector<myedge> &halfedges, Lgraph &ph, 
	std::vector<std::vector<int>> &holemap2, int &seaerchtimes)
{
	std::vector<int>pts;
	for (int i = 0; i < halfedges.size(); i++)
	{
		pts.push_back(halfedges.at(i).p1id);
		pts.push_back(halfedges.at(i).p2id);
	}
	sort(pts.begin(), pts.end());
	pts.erase(unique(pts.begin(), pts.end()), pts.end());
	//上边是对的 保留半边的端点便于后语建立邻接表
	
	int searchSize = FindMaxValue(pts);
	seaerchtimes = searchSize;

	Lgraph graph;
	graph = CreatGraph(pts.size());
	graph->Ne = halfedges.size();
	if (graph->Ne != 0)
	{
		for (int i = 0; i < graph->Ne; i++)
		{
			InsertEdge(graph, halfedges.at(i));
		}
	}

	DeleteEndPoint(graph, searchSize);
	ph = graph;

	for (int i = 0; i < MaxVertexuUm; i++)
	{
		if (getlen(ph->G[i]) != 0)
			ph->G[i].id = i;
	}

	//std::vector<std::vector<int>> holemap(pts.size());
	std::vector<int> lens;
	int len4 = 0;
	int len6 = 0;
	for (int i = 0; i < searchSize; i++)
	{
		int lentep = getlen(ph->G[i]);
		if (lentep != 0)
		{
			lens.push_back(lentep);
		}
		else
			continue;;
		if (lentep == 2)
		{
			ph->G[i].Min_visited_Times = 1;
			ph->G[i].Max_visited_Times = 2;
		}
		if (lentep == 4)
		{
			len4++;
			ph->G[i].Min_visited_Times = 2;
			ph->G[i].Max_visited_Times = 4;
		}
		if (lentep == 6)
		{
			len6++;
			ph->G[i].Min_visited_Times = 3;
			ph->G[i].Max_visited_Times = 6;
		}
	}
	cout << "Number of points have 4 adjacent points: " << len4 << endl;
	cout << "Number of points have 6 adjacent points: " << len6 << endl;
}

void fillholes(PolygonMesh &mesh1, PointCloud<PointXYZRGB>::Ptr &cloud, 
	std::vector<PointCloud<PointXYZRGB>::Ptr> &outcloud, PolygonMesh &meshout)
{
	std::vector<myedge> halfedges;
	getHalfEdge(mesh1, halfedges);

	//  std::vector<std::vector<myedge>> holes;
	//	getHoles(halfedges,holes);
	Lgraph ph;
	std::vector<std::vector<int>> holemap;
	int seaerchtimes = 0;
	getHolesmap(halfedges, ph, holemap, seaerchtimes);

	get_complex_holes(ph, seaerchtimes);

	std::vector<std::vector<Vnode>> holes;

	int allsize = 0;
	int simplesize = 0;
	int complexsize = 0;
	for (int i=0;i<MaxVertexuUm;i++)
	{
		if (getlen(ph->G[i]) != 0)
		{
			allsize++;
			if (ph->G[i].isComplex == 1)
				complexsize++;
			else
				simplesize++;
		}
	}
	cout << "All hole pts size is: " << allsize << endl;
	cout << "Simple hole pts size is: " << simplesize << endl;
	cout << "Complexed hole pts size is: " << complexsize << endl;

	get_holes(ph, holes);
	auto it = holes.begin();
	while (it!= holes.end())
	{
		if (it->size() == 0)
		{
			holes.erase(it);
		}
		else
		{
			it++;
		}
	}
	int allpts = 0;
	for (int i = 0; i < holes.size(); i++)
	{
		allpts += holes.at(i).size();
		//for (int j=0;j< holes.at(i).size();j++)
		//{
		//	cout << holes.at(i).at(j).id << " ";
		//}
		//cout << endl;
	}
	cout << "Holes' size is: " << holes.size() << endl;
	cout << "Use pts: " << allpts << endl;
	
	for (int i = 0; i < holes.size(); i++)
	{
		PointCloud<PointXYZRGB>::Ptr temp(new PointCloud<PointXYZRGB>);	
		int cr = (rand() % 255);
		int cg = (rand() % 255);
		int cb = (rand() % 255);
		for (int j=0;j<holes.at(i).size();j++)
		{
			temp->push_back(cloud->at(holes.at(i).at(j).id));
			cloud->at(holes.at(i).at(j).id).r = /*255; */cr;
			cloud->at(holes.at(i).at(j).id).g = /*0;*/ cg;
			cloud->at(holes.at(i).at(j).id).b = /*0;*/ cb;
		}
		outcloud.push_back(temp);
	}
}

