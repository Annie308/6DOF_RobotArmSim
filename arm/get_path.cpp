#include <iostream>
#include <cmath>
#include <random>
#include <chrono>
#include <thread>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp> 

#include "render.h"
#include "calculations.h"
#include "arm_attributes.h"
#include "get_path.h"

//for math constants
#define _USE_MATH_DEFINES

using namespace std;

void find_target() {
	// Check if the image was loaded successfully
	cv::Mat image = cv::imread("C:\\Users\\annie\\Downloads\\APSC Toothpaste CAD Prototype.png");
	if (image.empty()) {
		std::cerr << "Error: Could not open or find the image." << endl;
		return;
	}

	// Display the image in a window
	// 
	//cv::imshow("My Image", image);

	// Wait for a key press before closing the window
	cv::waitKey(0);
}