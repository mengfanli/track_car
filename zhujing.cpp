#include <cstdlib>
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


void stop()
{
	clog<<"stop!"<<endl;
	controlLeft(FORWARD, (int)INIT_SPEED);
	controlRight(FORWARD, (int)INIT_SPEED);
	delay(3000);

	stopLeft();
	stopRight();
	delay(1000);

}


void move(float angle)
{
	resetCounter();
	controlLeft(FORWARD, (int)speedLeft);
	controlRight(FORWARD, (int)speedRight);
	//delay(200);

	if(angle>-5 || angle<5 ) 
	{
		int readingLeft=0,readingRight=0;
		getCounter(&readingLeft, &readingRight);
		if(readingLeft==-1||readingRight==-1)
		{
			clog<<"Error!\n";
			return;
		}

		double distanceLeft=readingLeft*63.4*M_PI/390;
		double distanceRight=1*readingRight*63.4*M_PI/390;//1是右轮修正系数
		distanceLR += distanceLeft - distanceRight;

		//100是距离差与速度差的比值(不精确),除以2是平均修正差异到左右轮
		double speedToChange=speedLeft-(distanceLR/100/2)>INIT_SPEED+4 || 
			speedLeft-(distanceLR/100/2)<INIT_SPEED-4 ? 0:(distanceLR/100/2);
		speedLeft-=speedToChange;
		speedRight+=speedToChange;
	}
	else 
	{
		resetCounter();
	}
	
}


void safeTurnTo(float angle)
{
	if(angle>30) angle=30;
	if(angle<-30) angle=-30;
	turnTo((int)angle);
	clog<<"turnTo: "<< angle <<endl;
	move(angle);
}


void ctrlDirection(float rho1,float theta1,float rho2,float theta2,float a)
{
	clog<<"rho1="<<rho1<<",theta1="<<theta1<<",rho2="<<rho2<<",theta2="<<theta2<<",a="<<a<<endl;
	float angle=0;
	
	if(theta1==0 && theta2!=0)
	{
		angle=-30;
	}
	else if(theta1!=0 && theta2==0)
	{
		angle=30;
	}
	else if(theta1==0 && theta2==0)
	{
		clog<<"error: no lines. \n";
		stop();
		return;
	}
	else
	{
		
		if(rho1>rho2)
		{
			
			clog<<"exchange l1,l2 "<<endl;
			float temp=rho1;
			rho1=rho2;
			rho2=temp;
			float temp2=theta1;
			theta1=theta2;
			theta2=temp2;
		}

		if(-tan(theta1)>tan(theta2))
		{
			
			angle=(PI-theta1)-theta2;
		}
		else
		{
			
			angle=(PI-theta1)-theta2;
		}

	}
	safeTurnTo(angle);
} 

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

	Mat image;
	while(true)
	{
		capture>>image;
		if(image.empty())
			break;

		clog<<"image.rows*cols: "<<image.rows<<"x"<<image.cols<<endl;
		//Set the ROI for the image
		//Rect roi(0,image.rows/3,image.cols,image.rows/3);
		Rect roi(0,0,image.cols-1,image.rows/4);
		Mat imgROI=image(roi);

		//Canny algorithm
		Mat contour;
		//Canny(imgROI,contours,CANNY_LOWER_BOUND,CANNY_UPPER_BOUND);
		cvtColor(imgROI,contour,CV_RGB2GRAY);
		Mat contours;
		adaptiveThreshold(contour,contours,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,55,2);
		Mat element = getStructuringElement(MORPH_RECT, Size(12,12));
		dilate(contours,contour,element);
		erode(contour,contours,element);
		#ifdef _DEBUG
		imshow(CANNY_WINDOW_NAME,contours);
		#endif

		vector<Vec2f> lines;
		bitwise_not(contours,contours);
		HoughLines(contours,lines,1,PI/12,HOUGH_THRESHOLD);
		//Mat result(imgROI.size(),CV_8U,Scalar(255));
		Mat result;
		imgROI.copyTo(result);
		if(lines.size()!=0) clog<<"lines.size():"<<lines.size()<<endl;
		
		float maxRad=-2*PI;
		float minRad=2*PI;
		float rho1=0,theta1=0,rho2=0,theta2=0;
		int countLeft=0,countRight=0;//识别了几条左线和右线
		//Draw the lines and judge the slope
		for(vector<Vec2f>::const_iterator it=lines.begin();it!=lines.end();++it)
		{
			float rho=(*it)[0];			//First element is distance rho
			float theta=(*it)[1];		//Second element is angle theta ，theta是θ

			//Filter to remove vertical and horizontal lines,
			//and atan(0.09) equals about 5 degrees.
			if((theta>0.09&&theta<1.48)||(theta>1.62&&theta<3.05)) //保留 5°<θ<85°或 93°<θ<175°的向量
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

		float angle=0;
		if(maxRad>PI/2)
		{
			angle=-20;
		}
		else if(maxRad>0)
		{
			angle=20;
		}
		safeTurnTo(angle);
		if(angle!=0) clog<<" countLeft:"<<countLeft<<" countRight:"<<countRight<<endl;
		move(angle);

		#ifdef _DEBUG
		stringstream overlayedText;
		overlayedText<<"Lines: "<<lines.size();
		putText(result,overlayedText.str(),Point(10,result.rows-10),2,0.8,Scalar(0,0,255),0);
		imshow(MAIN_WINDOW_NAME,result);
		#endif
		lines.clear();

		waitKey(1);
	}
	return 0;
}
