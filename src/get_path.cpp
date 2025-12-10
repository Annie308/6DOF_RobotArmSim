#include <iostream>
#include <cmath>
#include <random>
#include <chrono>
#include <thread>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp> 
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "render.h"
#include "calculations.h"
#include "arm_attributes.h"
#include "get_path.h"

//for math constants
#define _USE_MATH_DEFINES

using namespace cv;

Mat src, src_gray;
Mat dst, detected_edges;
int lowThreshold = 0, highThreshold = 100;
const int ratio = 3;
const int kernel_size = 3;
const char* window_name = "test edges";

static void CannyThreshold(int, void*) {
	//blur with filter of kernel size 3
	blur(src_gray, detected_edges, Size(3, 3));
	//apply canny
	Canny(detected_edges, detected_edges, lowThreshold, highThreshold * ratio, kernel_size);

	//create a black image (backdrop)
	dst = Scalar::all(0);

	//copyy the edges onto the black image
	src.copyTo(dst, detected_edges);

	imshow(window_name, dst);
}

void find_target() {
	// Check if the image was loaded successfully
	Mat img = imread("C:\\Users\\annie\\Pictures\\Screenshots\\test2.png");
	
	if (img.empty()) {
		std::cerr << "Error: Could not open or find the image.\n";
		waitKey(0);
		return;
	}

	resize(img, src, Size(400,400));

	//create matrix of same type and size (the black image)
	dst.create(src.size(), src.type());

	//convert to grayscale
	cvtColor(src, src_gray, COLOR_BGR2GRAY);

	namedWindow(window_name, WINDOW_AUTOSIZE);

	//add trackbar for user to enter threshold
	createTrackbar("Min Threshold:", window_name, &lowThreshold, highThreshold, CannyThreshold);
	CannyThreshold(0, 0);

	waitKey(0);
}