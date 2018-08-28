#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <highgui.h>
#include "opencv2/opencv.hpp"
#include "GPIOlib.h"

#define PI 3.1415926

//Uncomment this line at run-time to skip GUI rendering
//#define _DEBUG
#define _MOVE
using namespace cv;
using namespace GPIO;
using namespace std;

const string CAM_PATH="/dev/video1";
const string outFile = "/home/pi/meng/recoder.avi";
const string MAIN_WINDOW_NAME="Processed Image";
const string CANNY_WINDOW_NAME="Canny";

const int CANNY_LOWER_BOUND=50;
const int CANNY_UPPER_BOUND=250;
const int HOUGH_THRESHOLD=38;

const double INIT_SPEED=6;
const double FAST_SPEED=9;
const double SLOW_SPEED=4;

double distanceLR;
double speedLeft = INIT_SPEED;
double speedRight = INIT_SPEED;
int seeEndLine=0;

int position_pid(float error)
{
	return 0;
}
void stop()
{
	clog<<"stop!"<<endl;
	stopLeft();
	stopRight();
	//delay(1000);

}

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
	VideoWriter write;

	capture.set(CV_CAP_PROP_FRAME_WIDTH,320);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,240);
	double dWidth=capture.get(CV_CAP_PROP_FRAME_WIDTH);			//the width of frames of the video
	double dHeight=capture.get(CV_CAP_PROP_FRAME_HEIGHT);		//the height of frames of the video
	clog<<"Frame Size: "<<dWidth<<"x"<<dHeight<<endl;
	double t=0;
	double fps;
	Mat image;

//	Size S(50,50);
//	double r = capture.get(CV_CAP_PROP_FPS);
//	write.open(outFile,-1,r,S,true);
//	if(!capture.isOpened())
//	{
//		clog<<"write open fail"<<endl;
//		return 1;
//	}

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

	objectivePoints[0].x = 50 + moveValueX; objectivePoints[0].y = 200 + moveValueY;
	objectivePoints[1].x = 50 + moveValueX; objectivePoints[1].y = 0 + moveValueY;
	objectivePoints[2].x = 100 + moveValueX; objectivePoints[2].y = 0 + moveValueY;
	objectivePoints[3].x = 100 + moveValueX; objectivePoints[3].y = 200 + moveValueY;

	cv::Mat transform = cv::getPerspectiveTransform(objectivePoints, imagePoints);
	
	double dt = (double)cv::getTickCount();
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
	//	imshow("original_image",image);
		Rect roi(0,image.rows/3*2,image.cols,image.rows/3);
		Mat imgROI=image(roi);
//		Mat imgROI=imaget;
//		Mat imgROI=image;

		cvtColor(imgROI,imgROI,CV_RGB2GRAY);
#ifdef _DEBUG
		imshow("gray_image",imgROI);
#endif
		adaptiveThreshold(imgROI,imgROI,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,31,2);
		Mat element = getStructuringElement(MORPH_RECT, Size(5,5));
		dilate(imgROI,imgROI,element);
		erode(imgROI,imgROI,element);

		bitwise_not(imgROI,imgROI);
		Mat imgtrans;
		// perspective.
		cv::warpPerspective(imgROI, imgtrans, transform,
			cv::Size(150,150),
			cv::INTER_LINEAR| cv::WARP_INVERSE_MAP);

		vector<Vec4i> lines;
		vector<Vec2f> linef;
		resize(imgtrans,imgtrans,Size(50,50),0,0,INTER_LINEAR);
	//	Canny(imgtrans,imgtrans,CANNY_LOWER_BOUND,CANNY_UPPER_BOUND);	
		HoughLines(imgtrans,linef,1,PI/180,HOUGH_THRESHOLD);
//		HoughLinesP(imgtrans,lines,1,PI/180,25,12,1);
		Point line_p[2];
		float rho,theta;
		double distance=0;
		double temp_max=0;
		double rho_max,theta_max=0;
		float rho_in=0,theta_in=0;
		float rho_old,theta_old;
#ifdef _DEBUG
      	cvtColor(imgtrans,imgtrans,CV_GRAY2RGB);
		for(int j=0;j<linef.size();j++)
		{
			rho=linef[j][0];
			theta=linef[j][1];
			clog<<"line"<<j<<" :"<<"rho="<<linef[j][0]<<"theta"<<linef[j][1]*180.0/PI<<endl;   
  			Point pt1(rho/cos(theta),0);
           		 //point of intersection of the line with last row
           		 Point pt2((rho-imgtrans.rows*sin(theta))/cos(theta),imgtrans.rows);
			line(imgtrans,pt1,pt2,Scalar(0,255,255),1,CV_AA);
		}
#endif
		if(linef.size()>5||linef.size()==0)
		{
#ifdef _MOVE
			controlLeft(FORWARD, (int)SLOW_SPEED);
			controlRight(FORWARD, (int)SLOW_SPEED);
#endif
		}
		else{
			int trust[5]={0,0,0,0,0};
			int max_index=0;
			double LIMIT = 0.08;//6'
			for(int i=0;i<linef.size();i++)
			{
				for (int j=i+1;j<linef.size();j++){
					if (fabs(linef[i][1]-linef[j][1]<LIMIT))
					{
						trust[j]+=1;
					}
				}
			}
			int tsum_temp=0;
			for(int i=0;i<linef.size();i++)
			{
				if(trust[i]>tsum_temp)
				{
					max_index=i;
					tsum_temp=trust[i];
				}	
			}
			theta_in=linef[max_index][1];
			theta_in=theta_in*180/PI;

			float kp=2;
			float ki=0;
			float kd=0.04;
			int angle=0;
			theta_in=theta_in > 90? theta_in-180:theta_in; 
			dt = ((double)cv::getTickCount() - dt) / cv::getTickFrequency();
			angle = theta_in*kp+theta_in*ki*dt+(theta_in-theta_old)*kd/dt;
			theta_old=theta_in;
			rho_old=rho_in;
			turnTo(angle);
			clog<<"theta_in="<<theta_in<<"turn to angle="<<angle<<endl;
#ifdef _MOVE
			if(angle<10)
			{
			controlLeft(FORWARD, (int)FAST_SPEED);
			controlRight(FORWARD, (int)FAST_SPEED);
			}
			else{
			controlLeft(FORWARD, (int)INIT_SPEED);
			controlRight(FORWARD, (int)INIT_SPEED);
			}
#endif
		}
#ifdef _DEBUG
 		imshow("perspective image", imgtrans);
#endif

		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
            	fps = 1.0 / t;
		clog<<"FPS: "<<fps<<endl;


		waitKey(1);
	}
	return 0;
}
