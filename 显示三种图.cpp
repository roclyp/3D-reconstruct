#include <stdio.h>
#include <Kinect.h>
#include <Windows.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>  
#include <opencv2/opencv.hpp>
#include <cv.h>
#include <iostream>
#include "PCllibrary.h"
#include <GL/glut.h>
//#include <GLFW/glfw3.h>
#define GL_DISPLAY

using namespace cv;
using namespace std;

IMultiSourceFrameReader* m_pMultiFrameReader = nullptr;//建立多源数据读取器对象，Kinect获取的多源数据由该读取器读取


// 安全释放指针
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

//获取连接Kinect2.0
int ConnectionKin(IKinectSensor* m_pKinectSensor, HRESULT hr)
{
	if (FAILED(hr))
	{
		cout << "Can't find the Kinect" << endl;
		return hr;
	}
	if (m_pKinectSensor)
	{
		hr = m_pKinectSensor->Open();
		if (SUCCEEDED(hr))
		{
			//读取器获取多源（RGB，近红外，深度）数据,
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Color |
				FrameSourceTypes::FrameSourceTypes_Infrared |
				FrameSourceTypes::FrameSourceTypes_Depth,
				&m_pMultiFrameReader);
		}
	}
	if (!m_pKinectSensor || FAILED(hr))
	{
		cout << "Warning: No Connection!/NO Source" << endl;
		return E_FAIL;
	}
	return 0;
}


int main()
{

	//获取Kinect设备
	IKinectSensor* m_pKinectSensor;
	HRESULT hr;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	//IMultiSourceFrameReader* m_pMultiFrameReader = nullptr;//建立多源数据读取器对象，Kinect获取的多源数据由该读取器读取
	//连接Kinect2.0
	ConnectionKin(m_pKinectSensor, hr);

	//从读取器中分别获取三种格式的数据
	IColorFrameReference* m_pColorFrameReference = nullptr;
	IInfraredFrameReference* m_pInfraredFrameReference = nullptr;
	IDepthFrameReference* m_pDepthFrameReference = nullptr;  //三种格式数据源帧引用
	IColorFrame* m_pColorFrame = nullptr;
	IInfraredFrame* m_pInfraredFrame = nullptr;
	IDepthFrame* m_pDepthFrame = nullptr;  //三种格式数据源帧

	//预设三种格式图片		PS：这里必须为4通道的图，Kinect的数据只能以BGRA的格式输出
	Mat i_rgb(1080, 1920, CV_8UC4);
	Mat i_depth(424, 512, CV_8UC1);
	Mat i_infrared(424, 512, CV_16UC1);

	//初始化多源数据帧
	UINT16 *depthData = new UINT16[424 * 512];
	IMultiSourceFrame* m_pMultiFrame = nullptr;
	
	while (true)
	{
		//获取新的多源数据帧
		hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);//这里取址,目的使变量m_pMultiFrame的地址为新帧的地址；
		if (FAILED(hr)||!m_pMultiFrame)
		{
			//cout << "Can't get the latest frame!" << endl;
			continue;
		}
		//获取RGB数据源并拷贝至图像
		if (SUCCEEDED(hr))
		{
			hr = m_pMultiFrame->get_ColorFrameReference(&m_pColorFrameReference);
			if (SUCCEEDED(hr))
			{
				//拷贝RGB数据到图片
				hr = m_pColorFrameReference->AcquireFrame(&m_pColorFrame);
				UINT nColorBufferSize = 1920 * 1080 * 4; //设置缓冲大小
				if (SUCCEEDED(hr))
				{
					hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize,
						reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);//CopyConvertedFrameDataToArray(容量，输出，格式)
				}
			}
		}



		//获取深度源并拷贝至图像
		if (SUCCEEDED(hr))
		{
			hr = m_pMultiFrame->get_DepthFrameReference(&m_pDepthFrameReference);
			if (SUCCEEDED(hr))
			{
				hr = m_pDepthFrameReference->AcquireFrame(&m_pDepthFrame);
				if (SUCCEEDED(hr))
				{
					hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, depthData);
					for (int i = 0; i < 512 * 424; ++i)
					{
						//0-255深度图，为了显示名单，只取深度数据的低8位；
						BYTE intensity = static_cast<BYTE>(depthData[i] % 256);
						reinterpret_cast<BYTE*>(i_depth.data)[i] = intensity;
					}
					//hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_depth.data));
				}
			}
		}


		//获取近红外源
		if (SUCCEEDED(hr))
		{
			hr = m_pMultiFrame->get_InfraredFrameReference(&m_pInfraredFrameReference);
			if (SUCCEEDED(hr))
			{
				hr = m_pInfraredFrameReference->AcquireFrame(&m_pInfraredFrame);
				if (SUCCEEDED(hr))
				{
					hr = m_pInfraredFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_infrared.data));
				}
			}

		}
		
		//显示
		namedWindow("RGBpic", CV_WINDOW_NORMAL);
		imshow("RGBpic", i_rgb);

		namedWindow("Depthpic", CV_WINDOW_NORMAL);
		imshow("Depthpic", i_depth);
		
		namedWindow("Infraredpic", CV_WINDOW_NORMAL);
		imshow("Infraredpic", i_infrared);
		if (waitKey(1) == VK_ESCAPE)
			break;


		//深度图映射到彩色图
		//获取坐标映射器
		ICoordinateMapper* m_pCoordinateMapper;
		ColorSpacePoint* m_pColorCoordinates = new ColorSpacePoint[512 * 424];//建立颜色空间点数组，用于之后的映射；
		CameraSpacePoint* m_pCameraCoordinates = new CameraSpacePoint[512 * 424];
		hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		//进行映射
		hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(512 * 424, depthData, 512 * 424, m_pColorCoordinates);
		Mat i_DepthToRgb(424, 512, CV_8UC4);
		//注意：424位行，512为列，函数中用上“*”符号都是列×行（不过可能只取相乘后的结果），用上“,”分割都是“行,列”（这个部分一定是这么写）
		if (SUCCEEDED(hr))
		{
			for (int i = 0; i < 424 * 512; i++)
			{
				ColorSpacePoint p = m_pColorCoordinates[i];
				if (p.X != -numeric_limits<float>::infinity() && p.Y != -numeric_limits<float>::infinity())
				{
					int colorX = static_cast<int>(p.X + 0.5f);
					int colorY = static_cast<int>(p.Y + 0.5f);
					//static_cast<int>强制转换作用
					if ((colorX >= 0 && colorX < 1920) && (colorY >= 0 && colorY < 1080))
					{
						i_DepthToRgb.data[i * 4] = i_rgb.data[(colorY * 1920 + colorX) * 4];
						i_DepthToRgb.data[i * 4+1] = i_rgb.data[(colorY * 1920 + colorX) * 4+1];
						i_DepthToRgb.data[i * 4+2] = i_rgb.data[(colorY * 1920 + colorX) * 4+2];
						i_DepthToRgb.data[i * 4+3] = i_rgb.data[(colorY * 1920 + colorX) * 4+3];
					}
				}
			}
		}
		imshow("DepthToRgbpic", i_DepthToRgb);
		if (waitKey(1) == VK_ESCAPE)
			break;

//#ifdef GL_DISPLAY
//		/// [gl display]
//		// Open a GLFW window to display our output
//		int a = glfwInit();
//		GLFWwindow * win = glfwCreateWindow(1024, 768, "gl_win", nullptr, nullptr);
//		glfwSetCursorPosCallback(win, on_cursor_pos);
//		glfwSetMouseButtonCallback(win, on_mouse_button);
//		glfwMakeContextCurrent(win);
//#endif
//
//#ifdef GL_DISPLAY
//		glfwPollEvents();
//		// Set up a perspective transform in a space that we can rotate by clicking and dragging the mouse
//		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//		glMatrixMode(GL_PROJECTION);
//		glLoadIdentity();
//		gluPerspective(60, (float)1280 / 960, 0.01f, 20.0f);
//		glMatrixMode(GL_MODELVIEW);
//		glLoadIdentity();
//		gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);
//		glTranslatef(0, 0, +0.5f);
//		glRotated(pitch, 1, 0, 0);
//		glRotated(yaw, 0, 1, 0);
//		glTranslatef(0, 0, -0.5f);
//
//		// We will render our depth data as a set of points in 3D space
//		glPointSize(2);
//		glEnable(GL_DEPTH_TEST);
//		glBegin(GL_POINTS);
//#endif

		//深度图像映射到相机空间
		if (SUCCEEDED(hr))
		{
			hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(512 * 424, depthData, 512 * 424, m_pCameraCoordinates);
			if (SUCCEEDED(hr))
			{
				for (int i = 0; i < 512 * 424; i++)
				{
					CameraSpacePoint p = m_pCameraCoordinates[i];
					if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity() && p.Z != -std::numeric_limits<float>::infinity())
					{
						float cameraX = static_cast<float>(p.X);
						float cameraY = static_cast<float>(p.Y);
						float cameraZ = static_cast<float>(p.Z);

						//cout << "x: " << cameraX << "y: " << cameraY << "z: " << cameraZ << endl;
						GLubyte *rgb = new GLubyte();
						rgb[2] = i_DepthToRgb.data[i * 4 + 0];
						rgb[1] = i_DepthToRgb.data[i * 4 + 1];
						rgb[0] = i_DepthToRgb.data[i * 4 + 2];
						// 显示点
						glColor3ubv(rgb);
						glVertex3f(cameraX, -cameraY, cameraZ);
					}
				}
			}
		}
		


		//namedWindow("映射后结果", CV_WINDOW_NORMAL);
		//imshow("映射后结果", rgb);
		//if (waitKey(1) == VK_ESCAPE)
		//	break;


		// 释放资源
		SafeRelease(m_pColorFrame);
		SafeRelease(m_pDepthFrame);
		SafeRelease(m_pInfraredFrame);
		SafeRelease(m_pColorFrameReference);
		SafeRelease(m_pDepthFrameReference);
		SafeRelease(m_pInfraredFrameReference);
		SafeRelease(m_pMultiFrame);
		SafeRelease(m_pCoordinateMapper);
	}

	//关闭设备和窗口	
	destroyAllWindows();
	m_pKinectSensor->Close();
	return 0;
}
