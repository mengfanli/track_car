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
		capture.open(atoi(CAM_PATH.c_str()));
	}
	capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);
	double dWidth=capture.get(CV_CAP_PROP_FRAME_WIDTH);			//the width of frames of the video
	double dHeight=capture.get(CV_CAP_PROP_FRAME_HEIGHT);		//the height of frames of the video
	clog<<"Frame Size: "<<dWidth<<"x"<<dHeight<<endl;
	double t=0;
	double fps;
	Mat image;
	while(true)
	{
	        t = (double)cv::getTickCount();
		capture>>image;
		if(image.empty())
			break;
		Mat imaget;
		
		//Set the ROI for the image
		resize(image,imaget,Size(image.cols/4,image.rows/4),0,0,INTER_LINEAR);
		clog<<"image.rows*cols: "<<imaget.rows<<"x"<<imaget.cols<<endl;
		Rect roi(0,image.rows/5*4,image.cols,image.rows/5);
		//Rect roi(0,image.rows/4-1,image.cols-1,image.rows-1);
		Mat imgROI=image(roi);

		

		cvtColor(imgROI,imgROI,CV_RGB2GRAY);
		imshow("gray_image",imgROI);
		adaptiveThreshold(imgROI,imgROI,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,31,2);
		Mat element = getStructuringElement(MORPH_RECT, Size(9,9));
		dilate(imgROI,imgROI,element);
		erode(imgROI,imgROI,element);

		#ifdef _DEBUG
		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
            	fps = 1.0 / t;
		clog<<"FPS: "<<fps<<endl;
		imshow(CANNY_WINDOW_NAME,imgROI);
		#endif
		bitwise_not(imgROI,imgROI);
		HoughLines(imgROI,lines,1,PI/6,HOUGH_THRESHOLD);
		//Draw the lines and judge the slope
		for(vector<Vec2f>::const_iterator it=lines.begin();it!=lines.end();++it)
		{
			float rho=(*it)[0];			//First element is distance rho
			float theta=(*it)[1];		//Second element is angle theta £¬thetaÊÇŠÈ

			//Filter to remove vertical and horizontal lines,
			//and atan(0.09) equals about 5 degrees.
			if((theta>0.09&&theta<1.48)||(theta>1.62&&theta<3.05)) 
			{
				if(theta>maxRad)
					maxRad=theta;
				if(theta<minRad)
					minRad=theta;

				if(theta>PI/2) countRight++;
				else countLeft++;
				
				#ifdef _DEBUG
				//point of intersection of the line with first row
				Point pt1(rho/cos(theta),0);
				//point of intersection of the line with last row
				Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
				//Draw a line
				line(result,pt1,pt2,Scalar(0,255,255),3,CV_AA);
				#endif
			}
			else if(theta>1.48&&theta<1.62)
			{
				
			}

			#ifdef _DEBUG
			clog<<"Line: ("<<rho<<","<<theta<<")\n";
			#endif
		}


		waitKey(1);
	}
	return 0;
}
