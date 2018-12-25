#include <stdio.h>
#include <Kinect.h>
#include <Windows.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>  
#include <opencv2/opencv.hpp>
#include <cv.h>
#include <iostream>
#include "PCllibrary.h"
//#include <GL/glut.h>
#include "GLFW/glfw3.h"
#include <time.h>
#include <vector>
#define SAVE_IMG
#define SAVE_Point
#define GL_DISPLAY
double yaw, pitch, lastX, lastY; int ml;


using namespace cv;
using namespace std;

IMultiSourceFrameReader* m_pMultiFrameReader = nullptr;//建立多源数据读取器对象，Kinect获取的多源数据由该读取器读取

//
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

void saveRGBDfile(string &file_name, vector<Point3DRGBAXYZ> points, int &depthheight, int &depthwidth)
{
	ofstream corandcolor;
	corandcolor.open(file_name, std::ios::out | std::ios::app);
	if (corandcolor.is_open())
	{
		for (auto c : points)
		{
			ostringstream outformate; 
			outformate << (double)c.r << " " << (double)c.g << " " << (double)c.b << " " << (double)c.depth << endl;
			string outstr= outformate.str();
			char outchar[100];
			strcpy(outchar,outstr.c_str());
			corandcolor.write(outchar,  strlen(outchar));
		}
		corandcolor.close();
	}
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

	//从读取器中分别获取2种格式的数据
	IColorFrameReference* m_pColorFrameReference = nullptr;
	IDepthFrameReference* m_pDepthFrameReference = nullptr;  //2种格式数据源帧引用
	IColorFrame* m_pColorFrame = nullptr;
	IDepthFrame* m_pDepthFrame = nullptr;  //2种格式数据源帧

	//预设2种格式图片		PS：这里必须为4通道的图，Kinect的数据只能以BGRA的格式输出
	Mat i_rgb(1080, 1920, CV_8UC4);
	Mat i_depth(424, 512, CV_8UC1);
	//初始化多源数据帧
	UINT16 *depthData = new UINT16[424 * 512];
	IMultiSourceFrame* m_pMultiFrame = nullptr;

	int fps = 0;
	int saveline = 0;

	while (true)
	{
		clock_t timeStart;
		timeStart = clock();
		cout << timeStart << endl;
		//获取新的多源数据帧
		hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);//这里取址,目的使变量m_pMultiFrame的地址为新帧的地址；
		if (FAILED(hr) || !m_pMultiFrame)
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
		clock_t timegainRGB = clock();
		cout << "获取RGB耗时" << timegainRGB - timeStart << endl;
		cout << "至获取RGB总耗时" << timegainRGB - timeStart << endl;

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
		clock_t timegainDepth = clock();
		cout << "获取Depth耗时" << timegainDepth - timegainRGB << endl;
		cout << "至获取Depth总耗时" << timegainDepth - timeStart << endl;

		//显示
		namedWindow("RGBpic", CV_WINDOW_NORMAL);
		imshow("RGBpic", i_rgb);
		namedWindow("Depthpic", CV_WINDOW_NORMAL);
		imshow("Depthpic", i_depth);
		if (waitKey(1) == VK_ESCAPE)
			break;

		clock_t timeshow = clock();
		cout << "图像显示耗时" << timeshow - timegainDepth << endl;
		cout << "至显示图像总耗时" << timeshow - timeStart << endl;

		//深度图映射到彩色图
		//获取坐标映射器
		ICoordinateMapper* m_pCoordinateMapper;
		ColorSpacePoint* m_pColorCoordinates = new ColorSpacePoint[512 * 424];//建立颜色空间点数组，用于之后的映射；
		CameraSpacePoint* m_pCameraCoordinates = new CameraSpacePoint[512 * 424];
		hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		//进行映射深度点到彩色坐标系下，m_pColorCoordinates中每个点彩色相机坐标下的点含有深度信息
		hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(512 * 424, depthData, 512 * 424, m_pColorCoordinates);
		Mat i_DepthToRgb(424, 512, CV_8UC4);
		int depthheight = 424, depthwidth = 512;
		const int sum = depthheight*depthwidth;
		vector<Point3DRGBAXYZ> points(sum);
		//注意：424为行，512为列，用上“,”分割都是“行,列”（这个部分一定是这么写）
		clock_t timepremap = clock();
		cout << "premap耗时" << timepremap - timeshow << endl;
		cout << "至premap总耗时" << timepremap - timeStart << endl;

		if (SUCCEEDED(hr))
		{
			for (int i = 0; i < 424 * 512; i++)
			{
				ColorSpacePoint p = m_pColorCoordinates[i];//这个步骤是获取这些在彩色相机坐标系下深度点的坐标X，Y
				if (p.X != -numeric_limits<float>::infinity() && p.Y != -numeric_limits<float>::infinity())
				{
					int colorX = static_cast<int>(p.X + 0.5f);//坐标四舍五入
					int colorY = static_cast<int>(p.Y + 0.5f);
					//static_cast<int>强制转换作用
					if ((colorX >= 0 && colorX < 1920) && (colorY >= 0 && colorY < 1080))
					{

						i_DepthToRgb.data[i * 4] = i_rgb.data[(colorY * 1920 + colorX) * 4];
						i_DepthToRgb.data[i * 4 + 1] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 1];
						i_DepthToRgb.data[i * 4 + 2] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 2];
						//上述三者分别为RGB三个通道，或者是BGR通道，下面为透明度
						i_DepthToRgb.data[i * 4 + 3] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 3];
						points[i].r = i_DepthToRgb.data[i * 4];
						points[i].g = i_DepthToRgb.data[i * 4 + 1];
						points[i].b = i_DepthToRgb.data[i * 4 + 2];
						points[i].a = i_DepthToRgb.data[i * 4 + 3];
						points[i].depth = depthData[i];
					}
				}
			}
			clock_t timecoloremap = clock();
			cout << "coloremap耗时" << timecoloremap - timepremap << endl;
			cout << "至coloremap总耗时" << timecoloremap - timeStart << endl;
#ifdef SAVE_Point
		fps++;
		if (fps % 1 == 0)
		{
			saveline++;
			string RGBDsaveadd = "C:/Users/zhihong/Desktop/2/radius_test/5/";
			ostringstream RGBDsaveAdd;
			RGBDsaveAdd << RGBDsaveadd << "RGBDpoint" << saveline << ".rgbd";
			saveRGBDfile(RGBDsaveAdd.str(), points, depthheight, depthwidth);
		}
		clock_t timesave = clock();
		cout << "save耗时" << timesave-timecoloremap << endl;
		cout << "至save总耗时" << timesave - timeStart << endl;
		//cout << "<<------------------------------------------------->>" << endl;
		//cout << endl;
#endif
		}
		// 释放资源
		SafeRelease(m_pColorFrame);
		SafeRelease(m_pDepthFrame);
		SafeRelease(m_pColorFrameReference);
		SafeRelease(m_pDepthFrameReference);
		SafeRelease(m_pMultiFrame);
		//SafeRelease(m_pCoordinateMapper);
		clock_t timeend;
		timeend = clock();
		cout << "总耗时" << timeend - timeStart << endl;
		cout << "<<------------------------------------------------->>" << endl;
		cout << endl;
	}
	//关闭设备和窗口	
	destroyAllWindows();
	m_pKinectSensor->Close();
	return 0;
}



