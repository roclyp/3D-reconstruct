#include "hole_Graph_type.h"


//初始化得到只有点没有边的图
Lgraph CreatGraph(int vertexnum)
{
	Vertex v;// , w;
	Lgraph mg;

	mg = (Lgraph)malloc(sizeof(Gnode));
	mg->Ne = vertexnum;
	mg->Ne = 0;
	for (v = 0; v < mg->Nv; v++)
	{
		mg->G[v].FirstEdge = NULL;//初始化所有节点的边为空
	}
	return mg;
}

//向图里插入边
void InsertEdge(Lgraph mg, myedge halfEdges)
{
	//有向图这部分就够了插入边<v1,v2>
	//建立V2的新节点
	PtrToAdjVnode newNode;
	newNode = (PtrToAdjVnode)malloc(sizeof(struct AdjVnode));
	newNode->AdjV = halfEdges.p2id;
	newNode->power = 1;
	//将v2插入v1表头
	newNode->Next = mg->G[halfEdges.p1id].FirstEdge;//新节点指向g[p1id]的第一条边，
	mg->G[halfEdges.p1id].FirstEdge = newNode;//g[p1id]的第一条边指向新节点pi2d，


	//因为本图是无向图，因此还需插入边<v2,v1>
	PtrToAdjVnode newNode2;
	newNode2 = (PtrToAdjVnode)malloc(sizeof(struct AdjVnode));
	newNode2->AdjV = halfEdges.p1id;
	newNode2->power = 1;
	//将v2插入v1表头
	newNode2->Next = mg->G[halfEdges.p2id].FirstEdge;//新节点指向g[p1id]的第一条边，
	mg->G[halfEdges.p2id].FirstEdge = newNode2;//g[p1id]的第一条边指向新节点pi2d，
}

//求表长
int getlen(Vnode adv)
//int getlen(PtrToAdjVnode adjvlist)
{
	PtrToAdjVnode temp;
	if (adv.FirstEdge == NULL)
		return 0;
	else
		temp = adv.FirstEdge->Next;
	
	int i = 1;
	while (temp)
	{
		temp = temp->Next;
		i++;
	}
	return i;
}

//查找-按值查找 输入的是要找的值，链表，输出该节点的是第几个节点,返回的是该节点位置
PtrToAdjVnode FindID(Vertex id, PtrToAdjVnode advjList, int &outid)
{
	PtrToAdjVnode p = advjList;
	int i = 1;
	while (p != NULL && p->AdjV != id) {
		p = p->Next;
		i++;
	}
	outid = i;
	return p;
}

//按序列查找
//输入的是第几个节点，链表
PtrToAdjVnode FindNumInList(int num_in_list, PtrToAdjVnode advjlist)
{
	PtrToAdjVnode p = advjlist;
	int i = 1;
	while (p != NULL && i < num_in_list)
	{
		p = p->Next;
		i++;
	}
	if (i == num_in_list)
		return p;
	else
		return NULL;
}


//表尾插入
void insert(Vertex id, PtrToAdjVnode adjlist)
{

	PtrToAdjVnode newnode;
	newnode = (PtrToAdjVnode)malloc(sizeof(struct AdjVnode));
	newnode->AdjV = id;
	newnode->Next = NULL;
	if (adjlist == NULL)
	{
		adjlist = newnode;
		return;
	}
	PtrToAdjVnode temp = adjlist;
	while (temp->Next!=NULL)
	{
		temp = temp->Next;
	}
	temp->Next = newnode;
}

//删除节点
PtrToAdjVnode  delete_Vertex(Vertex id, PtrToAdjVnode advlist)
{
	PtrToAdjVnode p, s;
	int num_in_list=0;
	FindID(id, advlist, num_in_list);
	if (num_in_list == 1)
	{
		s = advlist;
		if (advlist != NULL)
			advlist = advlist->Next;
		else
			return NULL;
		free(s);
		return advlist;
	}

	p = FindNumInList((num_in_list)-1, advlist);
	//因为要删除该节点，因此要找到该节点的前一节点
	if (p==NULL)//该节点的前一个节点不存在
	{
		//cout << "No " << id-1 << " vertex" << endl;
		return advlist;
	}
	else if(p->Next==NULL)//该节点的前一节点不存在是
	{
		//cout << "No " << id << " vertex" << endl;
		return advlist;
	}
	else
	{
		s = p->Next;
		p->Next = s->Next;
		free(s);
		return advlist;
	}
}