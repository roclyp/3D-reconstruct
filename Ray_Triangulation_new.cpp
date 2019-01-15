#include "MyClassType.h"
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

//计算相交关系
int Cross_triangle(const Raytri& rt)
{
	//Moller写法
	//建立edge1和edge2边的坐标系
	//其中u表示在edge1边的坐标的值，v表示在edge2边的坐标的值
	//t表示在射线p0-pend上的坐标的值
	double u, v, t;
	double edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
	double det, inv_det;
	SUB(edge1, rt.v1, rt.v0);
	SUB(edge2, rt.v2, rt.v0);
	//向量叉乘得到两个向量的法向量（射线与坐标系一边edge2的法向量）
	CROSS(pvec, rt.Vdirection, edge2);
	//若该法向量与另一条边点乘小于EPSILON，则说明该法向量与另一边垂直
	//由于法向量垂直edge1,edge2,和射线，那么三线共平面，此时返回0
	det = DOT(pvec, edge1);
	if (det < EPSILON && det > EPSILON)
		return 0;
	//否则,用点乘的结果的倒数计算u，v，t的值
	inv_det = 1.0 / det;
	SUB(tvec, rt.P0, rt.v0);
	u = DOT(tvec, pvec)*inv_det;
	if (u < 0 || u> 1)
		return 3;//return 3表示该线段与三角面片平面相交，但是不在三角面片内
	CROSS(qvec, tvec, edge1);
	v = DOT(rt.Vdirection, qvec)*inv_det;
	if (v < 0 || u + v > 1)
		return 3;//return 3表示该线段与三角面片平面相交，但是不在三角面片内
	t = DOT(qvec, edge2)*inv_det;
	if (t >= 0 && t <= 1)
		return 2;  //return2 表示两点在三角面片两侧
	else
		return 1;  //return1 表示两点在三角面片同侧

}

//获取三角面中直面角度
double getAngle(spacePlane &sp1, spacePlane &sp2, spaceLine &crossLine)
{
	double norPlane1[3], norPlane2[3];
	double crossx, crossy, crossz = 0;
	double crossLinetemp[3];
	norPlane1[0] = sp1.A; norPlane1[1] = sp1.B; norPlane1[2] = sp1.C;
	norPlane2[0] = sp2.A; norPlane2[1] = sp2.B; norPlane2[2] = sp2.C;
	double absNor1 = sqrt(sp1.A*sp1.A + sp1.B*sp1.B + sp1.C*sp1.C);
	double absNor2 = sqrt(sp2.A*sp2.A + sp2.B*sp2.B + sp2.C*sp2.C);
	double abdot = abs(DOT(norPlane1, norPlane2)*1.0/(absNor1*absNor2));
	double angle = acos(abdot);
	crossx = (sp1.B*sp2.D - sp2.B*sp1.D)*1.0 / (sp2.B*sp1.A - sp1.B*sp2.A);
	crossy = (sp1.A*sp2.D - sp2.A*sp1.D)*1.0 / (sp2.A*sp1.B - sp1.A*sp2.B);
	CROSS(crossLinetemp, norPlane1, norPlane2);
	crossLine.dx = crossLinetemp[0]; crossLine.dy = crossLinetemp[1]; crossLine.dz = crossLinetemp[2];
	crossLine.x0 = crossx; crossLine.y0 = crossy; crossLine.z0 = crossz;
	return angle;
}