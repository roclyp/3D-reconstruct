#include <stdio.h>
#include <Kinect.h>
#include <Windows.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>  
#include <opencv2/opencv.hpp>
#include <cv.h>
#include <iostream>
#include "PCllibrary.h"
#include "GLFW/glfw3.h"
#include <time.h>
#include <vector>
#include <opencv2\imgproc\imgproc.hpp> 
#include <iostream>
#include <time.h>
#include <vector> 
#include <omp.h>
#include<algorithm>

using namespace cv;
using namespace std;
using namespace pcl;

#define png ".png"
#define jpg ".jpg"

vector<Mat> depthall;
vector<Mat> RGBall;

int main()
{
	
	
	int framesnum = 10;
	string path = "C:\\Users\\zhihong\\Desktop\\hpl3\\1_";
	for (int i = 1; i <= framesnum; i++)
	{
		Mat temprgb(1080, 1920, CV_8UC3);
		Mat tempgrey(424, 512, CV_16UC1);
		stringstream rgb_name,depth_name;
		rgb_name << path << i << ".jpg";
		depth_name << path << i << ".png";
		tempgrey = imread(depth_name.str(), CV_16UC1);
		temprgb = imread(rgb_name.str());
		depthall.push_back(tempgrey);
		RGBall.push_back(temprgb);
	}
	//namedWindow("Depthpic1", CV_WINDOW_NORMAL);
	//imshow("Depthpic1", depthall.at(0));
	//waitKey(0);
	//namedWindow("color", CV_WINDOW_NORMAL);
	//imshow("color", RGBall.at(0));
	//waitKey(0);
	Mat DepthOut(424, 512, CV_16UC1);
	Mat i_depthshow(424, 512, CV_8UC1);
	for (int row = 0; row < 424; ++row)
	{
		for (int col = 0; col < 512; ++col)
		{
			unsigned short depthValue;
			double depthaverage;
			for (int i = 0; i < framesnum; i++)
			{
				depthaverage += depthall.at(i).data[row * 512 + col];
			}
			depthaverage = depthaverage*1.0 / (framesnum * 1.0);
			double minusValue;
			for (int i = 0; i < framesnum; i++)
			{
				depthValue = depthall.at(i).data[row * 512 + col];
				minusValue += abs(depthValue - depthaverage);
			}
			minusValue = minusValue / framesnum;
			if (minusValue <= 70)
			{
				DepthOut.data[row * 512 + col] = (unsigned short)depthaverage;
			}
			else{
				DepthOut.data[row * 512 + col] = -std::numeric_limits<unsigned short>::infinity();
				//DepthOut.data[row * 512 + col] = 65535;
			}
		}
	}
	for (int row = 0; row < 424; ++row)
	{
		for (int col = 0; col < 512; ++col)
		{
			//0-255深度图，为了显示名单，只取深度数据的低8位；
			BYTE intensity = static_cast<BYTE>(DepthOut.data[row * 512 + col] % 256);
			reinterpret_cast<BYTE*>(i_depthshow.data)[row * 512 + col] = intensity;
		}
	}
	namedWindow("Depthpic", CV_WINDOW_NORMAL);
	imshow("Depthpic", DepthOut);
	waitKey(0);
	imwrite("C:/Users/zhihong/Desktop/hpl3/outdepth/1.png", DepthOut);
	imwrite("C:/Users/zhihong/Desktop/hpl3/outdepth/2.png", i_depthshow);
	return 0;
}