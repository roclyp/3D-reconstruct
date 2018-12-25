#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv.hpp>
#include <fstream>
#include <vector>
#include<opencv\cv.hpp>
#include <string>
#include <iostream>
#include<io.h>
//#include <iostream>  

using namespace std;
using namespace cv;


int getNumberInString(string istring, bool &hasnumbr)
{
	int number = 0;
	string filterstring;
	for (int i = istring.size(); i > istring.size() - 10 && istring[i] != '_'; i--)
	{
		filterstring += istring[i];
	}
	for (int i = filterstring.size(); i >0; i--)
	{
		if (filterstring[i] >= '0'&&filterstring[i] <= '9')
		{
			number = number * 10 + filterstring[i] - '0';
		}
	}
	if (number == 0)
		hasnumbr = false;
	return number;
}

int reRangeFileName(vector<string>& files, vector<int> &SerialNumber)
{
	if (files.size() != SerialNumber.size())
	{
		cout << "The number of Files and Serial Number is wrong" << endl;
		return -1;
	}
	for (int i = 0; i<files.size(); i++)
	{
		for (int j = i; j < files.size(); j++)
		{
			if (SerialNumber[i] >= SerialNumber[j])
			{
				int tmp;
				tmp = SerialNumber[j];
				SerialNumber[j] = SerialNumber[i];
				SerialNumber[i] = tmp;
				string tmpname;
				tmpname = files[j];
				files[j] = files[i];
				files[i] = tmpname;
			}
		}

	}
	return 0;
}

int GetAllFiles_CertainFormat(string path, vector<string>& files, string format)
{
	intptr_t hFile = 0;//_findnext返回类型为intprt_t,而非long类型，从intptr_t转换到long类型丢失数据
	struct  _finddata_t fileinfo;
	vector<int> SerialNumber;//用于存储序号
	string serialnumber;//用于排序
	bool hasnumber = true;//用于判断文件名中是否有数字
	int nonum = 0;//用于在没有数字时排序
	string p;
	if ((hFile = _findfirst(p.assign(path).append("/*" + format).c_str(), &fileinfo)) != -1)
	{
		do
		{
			if ((fileinfo.attrib & _A_SUBDIR))//判断是否为文件夹
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, ".") != 0)
				{
					GetAllFiles_CertainFormat((p.assign(path).append("/")).append(fileinfo.name), files, format);

				}
			}
			else
			{
				serialnumber.append(fileinfo.name);
				int num = getNumberInString(serialnumber, hasnumber);
				//cout << "number:" << num << endl;
				if (hasnumber == true)
					SerialNumber.push_back(num);
				else
				{
					SerialNumber.push_back(nonum);
				}
				p.assign(path).append("/").append(fileinfo.name);
				files.push_back(p);
			}
			nonum++;
			serialnumber.clear();

		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
	cout << "-------------------------------------------------" << endl;
	reRangeFileName(files, SerialNumber);//重新排序
	return 0;
}

int main()
{
	double time0 = static_cast<double>(getTickCount());
	ofstream fout("caliberation_result.txt");  /**    保存定标结果的文件     **/
											   /************************************************************************
											   读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化
											   *************************************************************************/

	cout << "开始提取角点………………" << endl;
	int image_count = 14;                    /****    图像数量     ****/
	int corner_count;						 /****    定义角点数量     ****/
	Size image_size;                         /****    图像的尺寸      ****/
	Size board_size = Size(14, 11);            /****    定标板上每行、列的角点数     ****/
	vector<Point2f> corners;                 /****    缓存每幅图像上检测到的角点    ****/
	vector<vector<Point2f>>  corners_Seq;    /****    保存检测到的所有角点     ****/
	vector<Mat>  image_Seq;
	string DirAddress = "C:/Users/user/Desktop/Kinect2.0/CalibrationPictures";
	string format = ".png";
	stringstream address1;
	address1 << DirAddress << "/";
	vector<string> FilesName;
	GetAllFiles_CertainFormat(DirAddress, FilesName, format);
	vector<Mat>imageALL;
	for (int i = 0; i < FilesName.size(); i++)
	{
		imageALL[i] = imread(FilesName[i], 1);
	}

	int count = 0;
	for (int i = 0; i != image_count; i++)
	{
		/*读取文件*/
		stringstream filename, numoffile;
		filename << "C:/Users/user/Desktop/Kinect2.0/CalibrationPictures/Depth_" << i + 1 << ".png";
		numoffile << "图像" << i + 1 << endl;								//输入图像窗口命名，后面会用到
		string imageFileName, numoffilesrt;									//定义文件名和文件数量（字符串）
		filename >> imageFileName;											//把字符串流信息传递到字符串中——文件名
		cout << imageFileName << endl;										//输出文件名
		numoffile >> numoffilesrt;											//把字符串流信息传递到
		Mat image = imread(imageFileName, 1);								//读取文件，rgb格式,0为灰度格式
		if (image.data == NULL)												//如果未检测到该文件，则提示错误
		{
			cout << "文件读取错误/未找到该文件..." << endl;
			return -1;
		}
		image_size = image.size();											//获取图像尺寸
		imshow("", image);
		waitKey(1);
		/* 提取角点 */
		Mat imageGray;														//定义mat对象灰度图
		cvtColor(image, imageGray, CV_RGB2GRAY);							// RGB图像转换为灰度图
		bool patternfound = findChessboardCorners(image, board_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		//布尔类型，判断是否发现所有角点，发现了，findchessboardcorners返回1，bool判断为true，否则为false
		if (!patternfound)													//如果patternfound为fasle，那么提示错误，结束程序并返回给系统1
		{
			cout << "can not find chessboard corners!\n";
			continue;
			exit(1);
		}
		else																//如果patternfound为true，执行以下程序
		{	/* 亚像素精确化*/  //发现亚像素级角点位置
						 // 输入图像，角点，搜索窗口，死区，迭代函数（方法，次数，精度）
			cornerSubPix(imageGray, corners, Size(19, 13), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			/* 绘制检测到的角点并保存 */
			Mat imageTemp = image.clone();									//复制图像，用于进行角点提取
			for (int j = 0; j < corners.size(); j++)
			{
				//circle(imageTemp, corners[j], 7, Scalar(0, 0, 255),1.8,8, 0);
				drawChessboardCorners(imageTemp, board_size, corners, patternfound);
				//绘制检测到的棋盘角点
				//输入图像，每行每列角点数目，检测到的角点数组，指示完整的期盼是否被发现
			}

			string imageFileName2;                                          //输出标记角点并连线后的图像
			std::stringstream StrStm;
			StrStm << address1.str() << i + 1;
			StrStm >> imageFileName2;
			imageFileName2 += "-角点.jpg";
			/*			namedWindow(numoffilesrt, CV_WINDOW_NORMAL);
			imshow(numoffilesrt, imageTemp);
			waitKey(10);		          */                                   //可理解为刷新
			imwrite(imageFileName2, imageTemp);		                         //输出图像
			cout << "图像" << i + 1 << "角点提取完成" << endl;
			count = count + corners.size();									 //count记录角点总数，Corners.size为每幅图检测到的角点数
			corners_Seq.push_back(corners);									 //缓存检测到的角点到容器vector<Point2f>中

		}
		image_Seq.push_back(image);											 //缓存图像到mat容器中
	}
	cout << "角点提取完成！\n";
	//getchar();
	//getchar();
	/************************************************************************
	摄像机定标
	*************************************************************************/
	cout << "开始定标………………" << endl;
	Size square_size = Size(1, 1);											  /**** 实际测量得到的定标板上每个棋盘格的大小 ****/
	vector<vector<Point3f>>  object_Points;                                   /****  保存定标板上角点的三维坐标   ****/
	Mat image_points = Mat(1, count, CV_32FC2, Scalar::all(0));				  /*****   保存提取的所有角点   *****/
	vector<int>  point_counts;												  /*****    每幅图像中角点的数量    ****/
	Mat intrinsic_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));               /*****    摄像机内参数矩阵    ****/
	Mat distortion_coeffs = Mat(1, 4, CV_32FC1, Scalar::all(0));              /* 摄像机的4个畸变系数：k1,k2,p1,p2 */
	vector<cv::Mat> rotation_vectors;										  /* 每幅图像的旋转向量 */
	vector<cv::Mat> translation_vectors;									  /* 每幅图像的平移向量 */

																			  /* 初始化定标板上角点的三维坐标 */
	for (int t = 0; t<image_count; t++)
	{
		vector<Point3f> tempPointSet;										   //缓存三维向量点
		for (int i = 0; i<board_size.height; i++)                              //对每个点进行初始化处理
		{
			for (int j = 0; j<board_size.width; j++)
			{
				/* 假设定标板放在世界坐标系中z=0的平面上 */
				Point3f tempPoint;
				tempPoint.x = i*square_size.width;
				tempPoint.y = j*square_size.height;
				tempPoint.z = 0;
				tempPointSet.push_back(tempPoint);
				//cout << "三维向量点" << tempPoint << endl;
			}
		}
		object_Points.push_back(tempPointSet);                                   // 保存定标板上角点的三维坐标
	}

	/* 初始化每幅图像中的角点数量，这里我们假设每幅图像中都可以看到完整的定标板 */
	for (int i = 0; i< image_count; i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}

	/* 开始定标 */
	calibrateCamera(object_Points, corners_Seq, image_size, intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors, 0);

	//cout << "每幅图像的定标误差：" << endl;

	cout << "定标完成！\n";
	/************************************************************************
	对定标结果进行评价
	*************************************************************************/
	cout << "开始评价定标结果………………" << endl;
	double total_err = 0.0;                   /* 所有图像的平均误差的总和 */
	double err = 0.0;                        /* 每幅图像的平均误差 */
	vector<Point2f>  image_points2;             /****   保存重新计算得到的投影点    ****/

	cout << "每幅图像的定标误差：" << endl;
	//cout << "每幅图像的定标误差：" << endl << endl;
	ofstream Internalparameterscout("C:/Users/user/Desktop/Kinect2.0/CalibrationPictures/ Depth内参系数(包括内参矩阵).txt");
	for (int i = 0; i<image_count; i++)
	{
		vector<Point3f> tempPointSet = object_Points[i];
		/****    通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点     ****/
		projectPoints(tempPointSet, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs, image_points2);
		/* 计算新的投影点和旧的投影点之间的误差*/
		vector<Point2f> tempImagePoint = corners_Seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (size_t i = 0; i != tempImagePoint.size(); i++)
		{
			image_points2Mat.at<Vec2f>(0, i) = Vec2f(image_points2[i].x, image_points2[i].y);
			tempImagePointMat.at<Vec2f>(0, i) = Vec2f(tempImagePoint[i].x, tempImagePoint[i].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);//对应差值=参数1对应元素-参数2对应元素，L2求差值平方的累加和后开根号，欧几里得范数
		total_err += err /= point_counts[i];
		cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
		fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
		Internalparameterscout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	}
	cout << "总体平均误差：" << total_err / image_count << "像素" << endl;
	fout << "总体平均误差：" << total_err / image_count << "像素" << endl << endl;
	cout << "评价完成！" << endl;
	//getchar();
	/************************************************************************
	保存定标结果
	*************************************************************************/
	cout << "开始保存定标结果………………" << endl;
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */

	cout << "相机内参数矩阵：" << endl;
	cout << intrinsic_matrix << endl;
	cout << "畸变系数：\n";
	cout << distortion_coeffs << endl;
	fout << "相机内参数矩阵：" << endl;
	fout << intrinsic_matrix << endl;
	fout << "畸变系数：\n";
	fout << distortion_coeffs << endl;
	//输出内参系数
	//ofstream Internalparameterscout("C:/Users/user/Desktop/毕设/毕设相关程序/grabcut/内参系数(包括内参矩阵).txt");

	Internalparameterscout << "总体平均误差：" << total_err / image_count << "像素" << endl << endl;
	Internalparameterscout << "相机内参数矩阵：" << endl;
	Internalparameterscout << intrinsic_matrix << endl;
	Internalparameterscout << "畸变系数：\n";
	Internalparameterscout << distortion_coeffs << endl;





	for (int i = 0; i<image_count; i++)
	{
		fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
		fout << rotation_vectors[i] << endl;

		/* 将旋转向量转换为相对应的旋转矩阵 */
		Rodrigues(rotation_vectors[i], rotation_matrix);
		fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
		fout << rotation_matrix << endl;
		fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
		fout << translation_vectors[i] << endl;
		//输出到文件中
		Internalparameterscout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
		Internalparameterscout << rotation_matrix << endl;
		Internalparameterscout << "第" << i + 1 << "幅图像的平移向量：" << endl;
		Internalparameterscout << translation_vectors[i] << endl;
	}
	Internalparameterscout.close();
	cout << "完成保存" << endl;
	fout << endl;
	getchar();
	/************************************************************************
	显示定标结果
	*************************************************************************/
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);
	cout << "保存矫正图像" << endl;
	for (int i = 0; i != image_count; i++)
	{
		cout << "Frame #" << i + 1 << "..." << endl;
		Mat newCameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
		initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image_size, CV_32FC1, mapx, mapy);
		Mat t = image_Seq[i].clone();
		cv::remap(image_Seq[i], t, mapx, mapy, INTER_LINEAR);
		string imageFileName;
		std::stringstream StrStm;
		StrStm << address1.str() << i + 1;
		StrStm >> imageFileName;
		imageFileName += "_d.jpg";
		imwrite(imageFileName, t);
	}
	cout << "保存结束" << endl;

	time0 = ((double)getTickCount() - time0) / getTickFrequency();
	cout << "标定用时:" << time0 << "秒" << endl;

	/////************************************************************************
	////测试一张图片z`z
	////*************************************************************************/
	//double time1 = static_cast<double>(getTickCount());
	//	cout << "TestImage ..." << endl;
	//	Mat newCameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
	//	/*Mat testImage = imread("C:\\Users\\user\\Desktop\\1\\1.jpg", 1);*/
	//	for (int i = 0; i < 8; ++i)
	//	{
	//		stringstream testname,testasvename;
	//		//testname << "C:/Users/user/Desktop/test/1" << i << ".jpg" << endl;
	//		//testasvename << "C:/Users/user/Desktop/test/1" << i << "-矫正.jpg" << endl;
	//		testname << "C:/Users/user/Desktop/pointcloudshow/pic2/" << i+1 << ".png";
	//		testasvename << "C:/Users/user/Desktop/pointcloudshow/pic2/" << i+1 << "-矫正.jpg";
	//		cout << testasvename.str() << endl;
	//		Mat testImage = imread(testname.str(), 1);
	//		if (&(testImage.data) == NULL)
	//		{
	//			cout << "无法找到文件" << endl;
	//			return -1;
	//		}
	//		//image_size.height = MAX(image_size.height,image_size.width);
	//		//image_size.width = MAX(image_size.height,image_size.width);
	//		initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image_size, CV_32FC1, mapx, mapy);
	//		Mat t = testImage.clone();
	//		cv::remap(testImage, t, mapx, mapy, INTER_LINEAR);
	//		imwrite(testasvename.str(), t);
	//		
	//		cout << "保存结束" << endl;
	//	}

	//time1 = ((double)getTickCount() - time1) / getTickFrequency();
	//cout << "校正用时:" << time1 << "秒" << endl;
	getchar();
	return 0;
}



