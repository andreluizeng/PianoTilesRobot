// pianotilesplayer.cpp : main project file.

#include "stdafx.h"

#include <opencv2\highgui\highgui.hpp>
#include <opencv2\opencv.hpp>

#include <windows.h>

using namespace System;
using namespace cv;


#include "serial.h"

Rect tablet_boundingbox;

bool blob_filter_by_area = 1; //default: disabled
							  //int blob_filter_min_area = 5000;
							  //int blob_filter_max_area = 8000;
int blob_filter_min_area = 500;
int blob_filter_max_area = 5000;

bool blob_filter_by_color = 0; // default: disabled
int blob_color_filter = 255; // ligther blobs, default (darker blobs = 0);

bool blob_filter_by_circularity = 0; // default: disabled
float blob_filter_min_circularity = 0.1;
float blob_filter_max_circularity = 1.0;

bool blob_filter_by_convexity = 0; // default: disabled
float blob_filter_min_convexity = 0.81;
float blob_filter_max_convexity = 1.0;

bool blob_filter_by_intertia = 0; // default: disabled
float blob_filter_min_inertia_ratio = 0.01;
float blob_filter_max_inertia_ratio = 1.0;

bool motor_1 = false;
bool motor_2 = false;
bool motor_3 = false;
bool motor_4 = false;

unsigned char var = 0;

Mat frame;

#define TIMEOUT 100
int timeout = TIMEOUT;
bool need_restart = 1;

Mat Thresh(Mat bin_img)
{
	uchar *ptr = (uchar *)bin_img.data;
	uchar v;


	for (int i = 0; i < bin_img.rows * bin_img.cols; i++)
	{
		v = ptr[i];

		if (v > 70)
		{
			ptr[i] = 0;
		}

		else
		{
			ptr[i] = 255;
		}

	}

	return bin_img;
}

void BlobDetectionRun(Mat gray_img, Mat output_img)
{
	static int var_ant;

	vector <KeyPoint> keypoints;

	SimpleBlobDetector::Params params;

	params.minThreshold = 10;
	params.maxThreshold = 200;

	if (blob_filter_by_color)
	{
		params.filterByColor = true;
		params.blobColor = blob_color_filter;
	}
	else
		params.filterByColor = false;

	if (blob_filter_by_area)
	{
		params.filterByArea = true;
		params.minArea = blob_filter_min_area;
		params.maxArea = blob_filter_max_area;
	}

	if (blob_filter_by_circularity)
	{
		params.filterByCircularity = true;
		params.minCircularity = blob_filter_min_circularity;
		params.maxCircularity = blob_filter_max_circularity;
	}
	else
		params.filterByCircularity = false;


	if (blob_filter_by_convexity)
	{
		params.filterByConvexity = true;
		params.minConvexity = blob_filter_min_convexity;
		params.maxConvexity = blob_filter_max_convexity;
	}
	else
		params.filterByConvexity = false;

	if (blob_filter_by_intertia)
	{
		params.filterByInertia = true;
		params.minInertiaRatio = blob_filter_min_inertia_ratio;
		params.maxInertiaRatio = blob_filter_max_inertia_ratio;
	}
	else
		params.filterByInertia = false;

	SimpleBlobDetector detector(params);

	detector.detect(gray_img, keypoints);

	//drawKeypoints(gray_img, keypoints, output_img, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	if (keypoints.size() > 0)
	{
	//printf("found %d keypoints\n", keypoints.size());

		Point2f pt;
		motor_1 = 0;
		motor_2 = 0;
		motor_3 = 0;
		motor_4 = 0;

		var_ant = var;

		var = 0x00;

		if (keypoints.size() <= 0)
			var = 0x00;

		else
		{


			for (int i = 0; i < keypoints.size(); i++)
			{

				circle(output_img, keypoints.at(i).pt, 30/2, Scalar(0, 255, 0), -1, 8, 0);

				pt = keypoints.at(i).pt;

				//printf("key %d: %.1f %.1f\n", i + 1, keypoints.at(i).pt.x, keypoints.at(i).pt.y);

				// 20 
				if ((pt.x < 70/2))
					var = var | 0x08;

				// 90
				if ((pt.x > 70/2) && (pt.x < 120/2))
					var = var | 0x04;

				// 160
				if ((pt.x > 120/2) && (pt.x < 200/2))
					var = var | 0x02;
				// 230
				if (pt.x > 200/2)
					var = var | 0x01;
			}
		}
	}

//	printf("\nvar: 0x%x - timeout %d", var, timeout);
	/*
	if (var_ant == var)
	{
		char msg[1];
		if (!timeout)
		{
			need_restart = true;
			timeout = TIMEOUT;
			var = 0x00;
			msg[0] = var;
			EscrevePorta(msg);

		}
		else
		{
			timeout--;
			need_restart = false;
		}
	}
	else
	{
		var_ant = var;
		need_restart = false;
		timeout = TIMEOUT;
	}

	*/
	return;
}



void BlobDetectionRunBonus(Mat gray_img, Mat output_img)
{

	vector <vector<Point> > contours; // Vector for storing contour
	vector<Vec4i> hierarchy;
	vector<Point> approx;

	vector<vector<Point> >contours_poly(contours.size());
	vector<Point2f>center(contours.size());
	vector<float>radius(contours.size());

	findContours(gray_img, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	for (unsigned int i = 0; i < contours.size(); i++)
	{
		float area = contourArea(contours[i], false);

		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);

		if (area > 10)
		{
			char msg[1];
			var = 0x0f;
			msg[0] = var;
			EscrevePorta(msg);
			waitKey(50);
			var = 0x00;
			msg[0] = var;
			EscrevePorta(msg);
			waitKey(10);
		}
	}


	return;
}

static void onMouse(int event, int x, int y, int, void*);


Mat ColorSeg(Mat rgb, int low_h, int high_h)
{
	Mat hsv_img;
	Mat bin_img(rgb.rows, rgb.cols, CV_8UC1, Scalar(0));
	Mat bin;

	cvtColor(rgb, hsv_img, CV_BGR2HSV);
	uchar *ptr = (uchar *)hsv_img.data;
	uchar *ptr_bin = (uchar *)bin_img.data;
	uchar h, s, v;


	for (int i = 0, j = 0; i < rgb.rows * rgb.cols * 3; i += 3, j++)
	{
		h = ptr[i];
		s = ptr[i + 1];
		v = ptr[i + 2];

		if (low_h < 0) // red
		{
			int angle = 180 + low_h;

			if ((h >= angle) || (h <= high_h))
			{
				ptr_bin[j] = 255;
			}

			else
			{
				ptr_bin[j] = 0;
			}
		}

		else
		{
			if ((h >= low_h) && (h <= high_h))
			{
				ptr_bin[j] = 255;
			}

			else
			{
				ptr_bin[j] = 0;
			}
		}
	}

	return bin_img;
}

#define NFRAMES 200
string filename = "play.xml";
FileStorage fs(filename, FileStorage::WRITE);

Mat gray;
int main(array<System::String ^> ^args)
{
	char key = 0;
	VideoCapture video;

	Mat bin;
	Mat mask;

	tablet_boundingbox.x = 230/2;
	tablet_boundingbox.y = 250/2;
	tablet_boundingbox.width = 260/2;
	tablet_boundingbox.height = 80/2;

	char msg[1];
	AbrePorta (L"COM4");
	msg[0] = var;
	EscrevePorta(msg);

	
	Mat Frames[NFRAMES];
	
	video.open(0);
	if (!video.isOpened())
	{
		return 0;
	}

	video.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	video.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	video.set(CV_CAP_PROP_BRIGHTNESS, 100);
	//video.set(CAP_PROP_AUTOFOCUS, 15);
	video.set(CV_CAP_PROP_EXPOSURE, -7);
	//video.set(CV_CAP_PROP_GAIN, 0.0);
	video.set(CV_CAP_PROP_SETTINGS, 0.0);


	mask.create(480/2, 640/2, CV_8UC3);
	mask = Scalar(255, 255, 255);

	//rectangle(mask, tablet_boundingbox, Scalar(0, 0, 0), -1, 8);
	
	

	//namedWindow("gray", CV_WINDOW_AUTOSIZE);
	//setMouseCallback("gray", onMouse, 0);

	int flag = 0;
	int i = 1;

	Mat img;

	bool first_frame = 1;
	char tag_name[50];
	int ii = 0;

	while (key != 27)
	{
		double tt = (double)cvGetTickCount();
		video >> frame;
		if (frame.empty())
			break;

		/*	if (ii < NFRAMES)
			{
				sprintf(tag_name, "frame_%02d", ii);
				fs << tag_name << frame;
				ii++;
			}
			*/

			//bitwise_or(frame, mask, frame);
		img = frame(tablet_boundingbox);
		//imshow("Video", img);

	/*	if (need_restart)
		{

			printf("\nRestart the kinets board");

			if (LePorta())
			{
				need_restart = 0;
				printf("\nKinets restarted");
			}
				
		}
		*/
		//else
		//{

	/*		Mat yellow_mask;
			yellow_mask = ColorSeg(img, -30, 40);
			//dilate(yellow_mask, yellow_mask, Mat(), Point(-1, -1), 4);
			erode (yellow_mask, yellow_mask, Mat(), Point(-1, -1), 5);
			imshow("yellow", yellow_mask);
			BlobDetectionRunBonus(yellow_mask, img);
			*/
			//Mat blue_mask_rgb;

			//cvtColor(blue_mask, blue_mask_rgb, CV_GRAY2BGR);
			//bitwise_or(img, blue_mask_rgb, img);


			//make it grayscale


			cvtColor(img, gray, CV_BGR2GRAY);



			medianBlur(gray, gray, 3);
			//imshow("gray", gray);

			bin = Thresh(gray);
			//threshold(gray, bin, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
			//threshold(gray, bin, 100, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

			//adaptiveThreshold(gray, bin, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 11, 2);

			

			erode(bin, bin, Mat(), Point(-1, -1), 3);
			//dilate(bin, bin, Mat(), Point(-1, -1), 1);
			//dilate(bin, bin, Mat(), Point(-1, -1), 3);

	//		bitwise_or(bin, yellow_mask, bin);

			imshow("bin", bin);


			//imshow("bin", bin);
			//equalizeHist(bin, bin);
			//imshow("histogram", bin);

			//bitwise_not(bin, bin);

			//var = 0x00;
			BlobDetectionRun(bin, img);
			//printf("\nSending var: 0x%x", var);
			/*
			var = 0x0f;
			msg[0] = var;
			EscrevePorta(msg);
			key = waitKey(500);

			var = 0x00;
			msg[0] = var;
			EscrevePorta(msg);
			key = waitKey(500);
			*/

			msg[0] = var;
			EscrevePorta(msg);
			key = waitKey(15);
			var = 0;
			EscrevePorta(msg);
			key = waitKey(5);
		//}

		line(img, Point(0, img.rows / 2), Point(img.cols, img.rows / 2), Scalar (0,0, 255), 4, 8, 0);
		imshow("frame", frame);



		
		tt = (double)cvGetTickCount() - tt;
		double value = tt / (cvGetTickFrequency()*1000.);
		value = 1000 / value;



		printf("\ntime =  %.2lf FPS", value);


	}

	gray.release();
	bin.release();
	destroyAllWindows();

	FechaPorta();

	fs.release();
	
    return 0;
}

static void onMouse(int event, int x, int y, int, void*)
{

	switch (event)
	{

		// for drawing.. just test.
	case CV_EVENT_RBUTTONDOWN:

		break;
	case CV_EVENT_RBUTTONUP:

		break;

	case CV_EVENT_MOUSEMOVE:
		//if (drawing)
		//{
		//circle(frame, Point(x, y), 2, (b_slider, g_slider, r_slider), -1);
		//imshow("frame", frame);
		//}

		break;

	case CV_EVENT_LBUTTONDOWN:

		break;


	case CV_EVENT_LBUTTONUP:

		// JUST TO CHECK PIXEL COLOR IN HSV AT CLICK POSITION
		Mat hsv_a;
		//cvtColor(frame, hsv_a, CV_BGR2HSV);
		cvtColor(gray, hsv_a, CV_GRAY2RGB);
		printf("\nPixel: %d %d %d", hsv_a.at<cv::Vec3b>(y, x)[0], hsv_a.at<cv::Vec3b>(y, x)[1], hsv_a.at<cv::Vec3b>(y, x)[2]);
		//hsv_a.release();




		break;

	}
}