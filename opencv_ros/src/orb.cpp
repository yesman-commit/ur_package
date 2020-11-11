#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#define _CRT_SECURE_NO_WARNINGS

int main()
{
    cv::Mat img_src1, img_src2, img_dst;
    std::vector<cv::KeyPoint> kpts1, kpts2;
    cv::Mat desc1, desc2;

    img_src1 = cv::imread("image.jpg", 0);
    img_src2 = cv::imread("image_1.jpg", 0);
    cv::Mat img_gray1, img_gray2;
    cv::cvtColor(img_src1, img_gray1, cv::COLOR_RGB2GRAY);
    cv::cvtColor(img_src2, img_gray2, cv::COLOR_RGB2GRAY);








    


    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    detector->detectAndCompute(img_src1, cv::noArray(), kpts1, desc1);
    detector->detectAndCompute(img_src2, cv::noArray(), kpts2, desc2);
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce");
    std::vector<cv::DMatch> matches;
    matcher->match(desc1, desc2, matches);
    cv::drawMatches(img_src1, kpts1, img_src2, kpts2, matches, img_dst);
    cv::imwrite("./im_2.jpg", img_dst);

    return 0;
}