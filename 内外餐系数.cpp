#include "PCLlibrary.h"
#include <iostream>
#include <string>
#include <pcl/console/time.h>   // TicToc
#include <io.h>  
#include <direct.h> 
#include <stdio.h>
#include <Kinect.h>
#include <Windows.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>  
#include <opencv2/opencv.hpp>
#include <cv.h>
#include <iostream>
#include <GL/glut.h>
#include "GLFW/glfw3.h"
#include <time.h>
#include <thread>


using namespace cv;
using namespace std;
using namespace pcl;


//RGB相机内参矩阵
double inmatrix_RGB[3][3] =
{ { 1032.709231,0.000000,981.428774 },
{ 0.000000, 1030.974347, 538.918598 },
{ 0.000000, 0.000000, 1.000000 } };
//RGB相机畸变系数
double Distortion_RGB[4][1] =
{ 0.054988, - 0.059714, - 0.003015, - 0.002555 };
//RGB相机外参矩阵
double exmatrix_RGB[4][4] =
{
	{ 0.818792, -0.555380, -0.145371, 70.101161 },
	{ -0.573744, -0.800422, -0.173615, 262.096973 },
	{ -0.019936,  0.225560, -0.974025, 357.542419 },
	{ 0.000000, 0.000000, 0.000000, 1.000000 }
};
//近红外相机内参矩阵
double inmatrix_Infared[3][3] =
{ { 363.160085, 0.000000, 258.046828 },
{ 0.000000, 363.064039, 209.335001 },
{ 0.000000, 0.000000, 1.000000 } };
double Distortion_Infared[4][1] =
{ 0.100285, - 0.240013, - 0.004493, - 0.003895 };
//近红外相机外参矩阵
double exmatrix_Infared[4][4] =
{
	{ 0.817328, -0.556828, -0.148043, 111.823330 },
	{ -0.575742, -0.799226, -0.172509, 233.837522 },
	{ -0.022262, 0.226231, -0.973819, 363.986017 },
	{ 0.000000, 0.000000, 0.000000, 1.000000 }
};
double R_calculate[3][3] =
{
	{ 0.999993921369000, -0.00246100194499998, -0.00230707843499998 },
	{ 0.00246283082899992, 0.999996741455000, 0.000762005131000021 },
	{ 0.00230520838700001, -0.000767345323000019, 0.999996531067000 },
};
double T_calculate[3][1] = { -40.3062703803215,
27.7074518129964,
-6.52067730622377 };
