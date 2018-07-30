#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <vector>

//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/opencv.hpp>
#include <highgui.h>
//#include <imgproc.hpp>
//#include <opencv.h>
#include "opencv2/opencv.hpp"
#include "GPIOlib.h"

#define PI 3.1415926

//Uncomment this line at run-time to skip GUI rendering
#define _DEBUG

using namespace cv;
using namespace GPIO;
using namespace std;

const string CAM_PATH="/dev/video1";
const string MAIN_WINDOW_NAME="Processed Image";
const string CANNY_WINDOW_NAME="Canny";

const int CANNY_LOWER_BOUND=50;
const int CANNY_UPPER_BOUND=250;
const int HOUGH_THRESHOLD=160;

const double INIT_SPEED=6;

double distanceLR;
double speedLeft = INIT_SPEED;
double speedRight = INIT_SPEED;
int seeEndLine=0;




int main()
{
	init();
	VideoCapture capture(CAM_PATH);
	//If this fails, try to open as a video camera, through the use of an integer param
	if (!capture.isOpened())
	{
		clog<<"open way2"<<endl;
		capture.open(atoi(CAM_PATH.c_str()));
	}
	capture.set(CV_CAP_PROP_FRAME_WIDTH,320);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,240);
	double dWidth=capture.get(CV_CAP_PROP_FRAME_WIDTH);			//the width of frames of the video
	double dHeight=capture.get(CV_CAP_PROP_FRAME_HEIGHT);		//the height of frames of the video
	clog<<"Frame Size: "<<dWidth<<"x"<<dHeight<<endl;
	double t=0;
	double fps;
	Mat image;

	// perspective transform
	cv::Point2f objectivePoints[4], imagePoints[4];

	// original image points.
	imagePoints[0].x = 0; imagePoints[0].y = 80-1;
	imagePoints[1].x = 105; imagePoints[1].y = 0;
	imagePoints[2].x = 215; imagePoints[2].y = 0;
	imagePoints[3].x = 320-1; imagePoints[3].y = 80-1;

	// objective points of perspective image.
	// move up the perspective image : objectivePoints.y - value .
	// move left the perspective image : objectivePoints.x - value.
	double moveValueX = 0.0;
	double moveValueY = 0.0;

	objectivePoints[0].x = 100 + moveValueX; objectivePoints[0].y = 400 + moveValueY;
	objectivePoints[1].x = 100 + moveValueX; objectivePoints[1].y = 0 + moveValueY;
	objectivePoints[2].x = 200 + moveValueX; objectivePoints[2].y = 0 + moveValueY;
	objectivePoints[3].x = 200 + moveValueX; objectivePoints[3].y = 400 + moveValueY;

	cv::Mat transform = cv::getPerspectiveTransform(objectivePoints, imagePoints);

	while(true)
	{
	        t = (double)cv::getTickCount();
		capture>>image;
		if(image.empty())
		{
			clog<<"image empty"<<endl;
			break;
	//		return 0;
		}
		Mat imaget;
		
		//Set the ROI for the image
//		resize(image,imaget,Size(320,240),0,0,INTER_LINEAR);
	//	clog<<"image.cols*rows: "<<imaget.cols<<"x"<<imaget.rows<<endl;
		Rect roi(0,image.rows/3*2,image.cols,image.rows/3);
		Mat imgROI=image(roi);
//		Mat imgROI=imaget;
//		Mat imgROI=image;

		cvtColor(imgROI,imgROI,CV_RGB2GRAY);
		imshow("gray_image",imgROI);
		adaptiveThreshold(imgROI,imgROI,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,31,2);
		Mat element = getStructuringElement(MORPH_RECT, Size(5,5));
		dilate(imgROI,imgROI,element);
		erode(imgROI,imgROI,element);

		#ifdef _DEBUG
//		imshow(CANNY_WINDOW_NAME,imgROI);
		#endif
		bitwise_not(imgROI,imgROI);
		Mat imgtrans;
		// perspective.
		cv::warpPerspective(imgROI, imgtrans, transform,
//			cv::Size(imgROI.rows*2, imgROI.cols*2),
			cv::Size(300,300),
			cv::INTER_LINEAR| cv::WARP_INVERSE_MAP);

		vector<Vec2f> lines;
		HoughLines(imgtrans,lines,1,PI/6,HOUGH_THRESHOLD);
		Point* line_p;
		double distance=0;
		double temp_max=0;
		float rho_max,theta_max=0;
		//Draw the lines and judge the slope
		int i=0;
		for(vector<Vec2f>::const_iterator it=lines.begin();it!=lines.end();++it,++i)
		{
			float rho=(*it)[0];			//First element is distance rho
			float theta=(*it)[1];		//Second element is angle theta £¬thetaÊÇ¦È
			line_p =(Point*)cvGetSeqElem(lines,i);
			distance = sqrt((line_p[1].x-line_p[0].x)^2+(line_p[1].y-line_p[0].y)^2);
			if(distance>temp_max){
				temp_max=distance;
				rho_max=rho;
				theta_max=theta;
			}
		}
		if(distance!=0)
		{
			#ifdef _DEBUG
			//point of intersection of the line with first row
			Point pt1(rho_max/cos(theta_max),0);
			//point of intersection of the line with last row
			Point pt2((rho_max-imgtrans.rows*sin(theta_max))/cos(theta_max),imgtrans.rows);
			//Draw a line
			line(imgtrans,pt1,pt2,Scalar(0,255,255),3,CV_AA);
	 		imshow("perspective image", imgtrans);
			#endif

			#ifdef _DEBUG
			clog<<"Line: ("<<rho_max<<","<<theta_max<<")\n";
			#endif
		}

		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
            	fps = 1.0 / t;
		clog<<"FPS: "<<fps<<endl;


		waitKey(1);
	}
	return 0;
}
