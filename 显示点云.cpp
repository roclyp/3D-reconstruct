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
#include "GLFW/glfw3.h"
#include <time.h>
#include <omp.h>
#define SAVE_IMG
#define SAVE_Point
#define GL_DISPLAY
double yaw, pitch, lastX, lastY; int ml;
static void on_mouse_button(GLFWwindow * win, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT) ml = action == GLFW_PRESS;
}
static double clamp(double val, double lo, double hi) { return val < lo ? lo : val > hi ? hi : val; }
static void on_cursor_pos(GLFWwindow * win, double x, double y)
{
	if (ml)
	{
		yaw = clamp(yaw - (x - lastX), -120, 120);
		pitch = clamp(pitch + (y - lastY), -80, 80);
	}
	lastX = x;
	lastY = y;
}


using namespace cv;
using namespace std;
using namespace pcl;


int main()
{
	//œ‘ æ”≥…‰ÕºœÒ
#ifdef GL_DISPLAY
	/// [gl display]
	// Open a GLFW window to display our output
	int a = glfwInit();
	GLFWwindow * win = glfwCreateWindow(1024, 768, "gl_win", nullptr, nullptr);
	glfwSetCursorPosCallback(win, on_cursor_pos);
	glfwSetMouseButtonCallback(win, on_mouse_button);
	glfwMakeContextCurrent(win);
#endif

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Incloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	//int errors = pcl::io::loadPCDFile<pcl::PointXYZRGBA>("F:/DesktopFile_Pointget/1-DATA/2017-10-18/2017-1/2017-1.pcd", *Incloud);
	int errors = pcl::io::loadPCDFile<pcl::PointXYZRGBA>("F:\\1-DesktopFile_Pointget\\1-DATA\\2017.11.11\\50_5_2After_2\\After_2017.11.11_1.pcd", *Incloud);
	if (errors == -1)
	{
		cout << "Can not find the file!" << endl;
		return -1;
	}
	cout << "Loaded" << endl;
	
#ifdef GL_DISPLAY
	for (auto i = Incloud->begin(); i < Incloud->end(); i++)
	{
		//cout << "x: " << cameraX << "y: " << cameraY << "z: " << cameraZ << endl;
		GLubyte *rgb = new GLubyte();
		rgb[2] = i->r;
		rgb[1] = i->g;
		rgb[0] = i->b;
		// œ‘ æµ„
		glColor3ubv(rgb);
		glVertex3f(i->x, i->y, i->z);
	}
#endif


#ifdef GL_DISPLAY		
		glfwPollEvents();
		// Set up a perspective transform in a space that we can rotate by clicking and dragging the mouse
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(60, (float)1280 / 960, 0.01f, 20.0f);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);
		glTranslatef(0, 0, +0.5f);
		glRotated(pitch, 1, 0, 0);
		glRotated(yaw, 0, 1, 0);
		glTranslatef(0, 0, -0.5f);

		// We will render our depth data as a set of points in 3D space
		glPointSize(2);
		glEnable(GL_DEPTH_TEST);
		glBegin(GL_POINTS);
#endif

//#ifdef GL_DISPLAY
//		glEnd();
//		glfwSwapBuffers(win);
//#endif
		getchar();
	return 0;
}



