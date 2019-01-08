#include "Ray_Trianglation.h"
#define EPSILON 0.00000001
#define TRiERROR 1
//叉乘
#define CROSS(dest,v1,v2) \
	dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
	dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
	dest[2]=v1[0]*v2[1]-v1[1]*v2[0]; 
//点乘
#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])
//向量
#define SUB(dest,v1,v2) \
	dest[0]=v1[0]-v2[0]; \
	dest[1]=v1[1]-v2[1]; \
	dest[2]=v1[2]-v2[2]; 
    //dest[0]=dest[0]/dest[2]; \
    //dest[1]=dest[1]/dest[2]; \
    //dest[2]=dest[2]/dest[2]; 
//double DOTT(const double v1[], const double v2[])
//{
//	return (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]);
//}
//
//void SUB(double dest[], const double v1[], const double v2[])
//{
//	dest[0] = v1[0] - v2[0];
//	dest[1] = v1[1] - v2[1];
//	dest[2] = v1[2] - v2[2];
//}
//
//void CROSS(double dest[], const double v1[], const double v2[])
//{
//	dest[0] = v1[1] * v2[2] - v1[2] * v2[1];
//	dest[1] = v1[2] * v2[0] - v1[0] * v2[2];
//	dest[2] = v1[0] * v2[1] - v1[1] * v2[0];
//}

int intersect_triangle(const Raytri& rt)
{

	//Moller写法
	double u1, v1;
	double edge11[3], edge21[3], tvec[3], pvec[3], Q1[3], qvec[3];
	double det1, inv_det1, det12;
	double t1;
	double ptest1[3];
	SUB(edge11, rt.v1, rt.v0);
	SUB(edge21, rt.v2, rt.v0);
	CROSS(pvec, rt.Vdirection, edge21);
	det1 = DOT(pvec, edge11);
	if (det1 < EPSILON && det1 > EPSILON)
		return 0;//return 0表示该射线与三角面片平行
	inv_det1 = 1.0 / det1;
	SUB(tvec, rt.P0, rt.v0);
	u1 = DOT(tvec, pvec)*inv_det1;
	if (u1 < 0 || u1> 1)
		return 3;//return 3表示该线段与三角面片平面相交，但是不在三角面片内
	CROSS(qvec, tvec, edge11);
	v1 = DOT(rt.Vdirection, qvec)*inv_det1;
	if (v1 < 0 || u1 + v1 > 1)
		return 3;//return 3表示该线段与三角面片平面相交，但是不在三角面片内
	t1 = DOT(qvec, edge21)*inv_det1;
	if (t1 >= 0 && t1 <= 1)
		return 2;  //return2 表示两点在三角面片两侧
	else
		return 1;  //return1 表示两点在三角面片同侧

	////我的写法：
	////cout << "Vdirection:" << rt.Vdirection[0] << "," << rt.Vdirection[1]
	////	<< "," << rt.Vdirection[2] << endl;
	//double u, v;
	//double edge1[3], edge2[3], Ntri[3], T[3], Q[3], P[3];
	//double det, inv_det, det2;
	//double t;
	//double ptest[3];

	//SUB(edge1, rt.v1, rt.v0);//求三角面片向量v1 v0->v1
	//SUB(edge2, rt.v2, rt.v0);//求三角面片向量v2 v0->v2

	////这个部分是判断该射线是否与三角面片平行或近似平行
	//CROSS(Ntri, edge1, edge2);//先求取与三角面片法向量Ntri
	////然后进行法向量与射线向量点乘，若点乘结果小于设定阈值，则可以判断该射线为平行
	////程序结束，该处也可以帮助判断求取附近平行面片
	//det = DOT(rt.Vdirection, Ntri);//这个det也表示了[−D (B−A) (C−A)]该矩阵的行列式的值，对后续克莱默法则求解有用
	//
	///*cout << "det= " << det << "; det2=" << det2 << endl;
	//cout << "========" << endl;*/
	//							   //说明公式在
	////http://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection

	//if (det > -EPSILON && det < EPSILON)
	//	return 0;//return 0表示该射线与三角面片平行
	////若不平行，则可建立tuv坐标系进行计算
	//inv_det = 1.0 / det;
	////此处用于计算射线与平面相交点p'的坐标，用的公式为
	//SUB(T, rt.P0, rt.v0);
	//CROSS(Q, T, edge1);
	//CROSS(P, T, edge2);

	//计算P点是否在三角面片内
	//计算u
	//u = DOT(P, T) * inv_det;
	//if (u < 0.0 || u > 1.0)
	//	return 3;//return 3表示该线段与三角面片平面相交，但是不在三角面片内
	////计算v
	//v = DOT(Q, rt.Vdirection) * inv_det;
	//if (v < 0.0 || u + v > 1.0)
	//	return 3;//return 3表示该线段与三角面片平面相交，但是不在三角面片内
	////计算t的目的是判断交点在线段两个点的中间还是两侧，即判断线段两点在三角面片平面的同侧还是两侧
	//t = DOT(Q, edge2) * inv_det;
	//if (t >= 0 && t <= 1)
	//	return 2;  //return2 表示两点在三角面片两侧
	//else
	//	return 1;  //return1 表示两点在三角面片同侧
}

int getCrossPoint(const Raytri& rt,pcl::PointXYZRGB &CrossPoint)
{
	//Moller写法
	double u1, v1;
	double edge11[3], edge21[3], tvec[3], pvec[3], Q1[3], qvec[3];
	double det1, inv_det1, det12;
	double t1;
	double ptest1[3];
	SUB(edge11, rt.v1, rt.v0);
	SUB(edge21, rt.v2, rt.v0);
	CROSS(pvec, rt.Vdirection, edge21);
	det1 = DOT(pvec, edge11);
	if (det1 < EPSILON && det1 > EPSILON)
	{
		cout << "ViewPoint direction is paralle to triangle!" << endl;
		return TRiERROR;
	}
	inv_det1 = 1.0 / det1;
	SUB(tvec, rt.P0, rt.v0);
	u1 = DOT(tvec, pvec)*inv_det1;
	if (u1 < 0 || u1> 1)
		return -1;
	CROSS(qvec, tvec, edge11);
	v1 = DOT(rt.Vdirection, qvec)*inv_det1;
	if (v1 < 0 || u1 + v1 > 1)
		return -1;
	t1 = DOT(qvec, edge21)*inv_det1;
	return 0;
//	double edge1[3], edge2[3],  Ntri[3], pcross[3];
//	double det, inv_det;
//	double t;
//
//	SUB(edge1, rt.v1, rt.v0);//求三角面片向量v1 v0->v1
//	SUB(edge2, rt.v2, rt.v0);//求三角面片向量v2 v0->v2
//
//	//这个部分是判断该射线是否与三角面片平行或近似平行
//	CROSS(Ntri, edge1, edge2);//先求取与三角面片法向量Ntri
//
//	//然后进行法向量与射线向量点乘，若点乘结果小于设定阈值，则可以判断该射线为平行
//	//程序结束，该处也可以帮助判断求取附近平行面片
//	det = DOT(rt.Vdirection, Ntri);
//
//	if (det > -EPSILON && det < EPSILON)
//	{
//		cout << "ViewPoint direction is paralle to triangle!" << endl;
//		return TRiERROR;
//	}
//		//return 0表示该射线与三角面片平行
//	inv_det = 1.0 / det;
//	//此处用于计算射线与平面相交点p'的坐标，用的公式为
//	//p'=P0+Vdirectiona*t，此处为求t
//	//推导过程在
////http://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/ray-triangle-intersection-geometric-solution
//	t = -(DOT(Ntri, rt.P0) + rt.plane.d)*1.0 / DOT(Ntri, rt.Vdirection);
//	//得到p点坐标
//	pcross[0] = rt.P0[0] + t * rt.Vdirection[0];
//	pcross[1] = rt.P0[1] + t * rt.Vdirection[1];
//	pcross[2] = rt.P0[2] + t * rt.Vdirection[2];
//	CrossPoint.x = pcross[0];
//	CrossPoint.y = pcross[1];
//	CrossPoint.z = pcross[2];

	//pcl::PointXYZRGB CrossPoint2;
	CrossPoint.x = rt.P0[0] + t1 * rt.Vdirection[0];
	CrossPoint.y = rt.P0[1] + t1 * rt.Vdirection[1];
	CrossPoint.z = rt.P0[2] + t1 * rt.Vdirection[2];
	return 0;
}

int getParallPoint(const Raytri& rt, const pcl::PointXYZRGB &ParallPoint, pcl::PointXYZRGB &outpoint)
{
	double edge1[3], edge2[3], Ntri[3], pcross[3];
	double det, inv_det;
	double t;

	SUB(edge1, rt.v1, rt.v0);//求三角面片向量v1 v0->v1
	SUB(edge2, rt.v2, rt.v0);//求三角面片向量v2 v0->v2

	//这个部分是判断该射线是否与三角面片平行或近似平行
	CROSS(Ntri, edge1, edge2);//先求取与三角面片法向量Ntri
	//推导公式：
	//平面法向量：(a,b,c)，平面上一点v(x0，y0，z0)，空间点坐标(x,y,z),空间在平面投影点坐标(x',y',z');
	//x′−xa=y′−yb=z′−zc=t ⇒  {x′=x+at;y′=y+bt;z′=z+ct}
	//a(x′−x0)+b(y′−y0)+c(z′−z0)=0  ⇒ ax′+by′+cz′=ax0+by0+cz0
	//t=(ax0+by0+cz0−(ax+by+cz))/a2+b2+c2
	t = (Ntri[0] * rt.v1[0] + Ntri[1] * rt.v1[1] + Ntri[2] * rt.v1[2]
		- (Ntri[0] * ParallPoint.x + Ntri[1] * ParallPoint.y + Ntri[2] * ParallPoint.z))*1.0 /
		(Ntri[0] * Ntri[0] + Ntri[1] * Ntri[1] + Ntri[2] * Ntri[2]);
	outpoint.x = ParallPoint.x + Ntri[0] * t;
	outpoint.y = ParallPoint.y + Ntri[1] * t;
	outpoint.z = ParallPoint.z + Ntri[2] * t;
	return 0;
}