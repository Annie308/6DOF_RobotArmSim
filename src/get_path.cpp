#include <opencv2/opencv.hpp>
#include <iostream>

void find_target() {
    // Paths to your images
    std::string left_path = "C:\\Users\\annie\\Documents\\CppProjects\\learnOpenCV\\data\\imgL.png";
    std::string right_path = "C:\\Users\\annie\\Documents\\CppProjects\\learnOpenCV\\data\\imgR.png";

    // Load images
    cv::Mat imgL = cv::imread(left_path);
    cv::Mat imgR = cv::imread(right_path);

    if (imgL.empty() || imgR.empty()) {
        std::cerr << "Cannot load images!" << std::endl;
        return;
    }

    // Resize images to half their size
    cv::Mat imgL_resized, imgR_resized;
    cv::resize(imgL, imgL_resized, cv::Size(480, 600), 0, 0, cv::INTER_LINEAR);
    cv::resize(imgR, imgR_resized, cv::Size(480, 600), 0, 0, cv::INTER_LINEAR);

    // Convert to grayscale
    cv::Mat grayL, grayR;
    cv::cvtColor(imgL_resized, grayL, cv::COLOR_BGR2GRAY);
    cv::cvtColor(imgR_resized, grayR, cv::COLOR_BGR2GRAY);

    // Create StereoBM object
    int numDisparities = 16; // must be multiple of 16
    int blockSize = 15;      // odd number
    cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(numDisparities, blockSize);

    // Compute disparity
    cv::Mat disp16S, disp32F;
    stereo->compute(grayL, grayR, disp16S);
    disp16S.convertTo(disp32F, CV_32F, 1.0 / 16.0);

    // Normalize for display
    cv::Mat dispNormalized;
    cv::normalize(disp32F, dispNormalized, 0, 255, cv::NORM_MINMAX, CV_8U);

    // Show images
    cv::imshow("Left", grayL);
    cv::imshow("Right", grayR);
    cv::imshow("Disparity", dispNormalized);
    cv::waitKey(0);
}
