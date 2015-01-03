/**
 * \file main.cpp
 *
 *  \date Aug 4, 2013
 *  \author Michał Orynicz
 */

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "ConvenienceFunctions.hpp"

int main() {
    cv::initModule_nonfree();
    cv::VideoCapture capL, capR;

    capL.open(0);
    capR.open(1);

    char c = ' ';

    std::cerr << capL.get(CV_CAP_PROP_FRAME_WIDTH) << std::endl;
    std::cerr << capL.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
    std::cerr << capR.get(CV_CAP_PROP_FRAME_WIDTH) << std::endl;
    std::cerr << capR.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;

    cv::Size imageSize(capL.get(CV_CAP_PROP_FRAME_WIDTH),
            capL.get(CV_CAP_PROP_FRAME_HEIGHT));

    std::cerr << imageSize << std::endl;

    cv::Mat disp(imageSize.height, imageSize.width * 2, CV_8UC3);
    std::cerr << disp.size() << std::endl;

    cv::Rect leftRoi(cv::Point(0, 0), imageSize);
    cv::Rect rightRoi(
            cv::Point(capL.get(CV_CAP_PROP_FRAME_WIDTH), 0),
            imageSize);

    cv::namedWindow("left", cv::WINDOW_NORMAL);
    cv::namedWindow("right", cv::WINDOW_NORMAL);
    cv::namedWindow("main", cv::WINDOW_NORMAL);

    cv::Ptr<cv::FeatureDetector> detector =
            cv::FeatureDetector::create("FAST");
    cv::Ptr<cv::DescriptorExtractor> descriptor =
            cv::DescriptorExtractor::create("FREAK");

    std::vector<cv::KeyPoint> keyPointsL, keyPointsR;
    std::vector<cv::Point2f> keyPointsLS, keyPointsRS;
    std::vector<cv::DMatch> matches;
    cv::Mat descriptionL, descriptionR;

    cv::BFMatcher matcher(cv::NORM_L2, true);
    std::vector<cv::Vec4d> homogenousPoints;

    cv::Mat projMatL, projMatR,r,t,rectL,rectR,disparityToDepthMap;

    cv::stereoRectify(cameraMatrixL,distCoeffsL,cameraMatrixR,distCoeffsR,imageSize,r,t,rectL,rectR,projMatL,projMatR,disparityToDepthMap);

    do {
        cv::Mat mL = disp(leftRoi);
        cv::Mat mR = disp(rightRoi);
        cv::Mat tL, tR;

        capL.grab();
        capR.grab();
        capL.retrieve(tL);
        capR.retrieve(tR);
        tL.copyTo(mL);
        tR.copyTo(mR);
        printMatrix(disp);
        printMatrix(mL);
        printMatrix(mR);

        {
            cv::Mat gL, gR;
            cv::cvtColor(tL, gL, cv::COLOR_RGB2GRAY);
            detector->detect(gL, keyPointsL);
            descriptor->compute(gL, keyPointsL, descriptionL);

            cv::cvtColor(tR, gR, cv::COLOR_RGB2GRAY);
            detector->detect(gR, keyPointsR);
            descriptor->compute(gR, keyPointsR, descriptionR);

            matcher.match(descriptionL, descriptionR, matches);
            keyPointsLS.resize(matches.size());
            keyPointsRS.resize(matches.size());

            for (unsigned int i = 0; i < matches.size(); ++i) {
                keyPointsLS[i] = keyPointsL[matches[i].queryIdx].pt;
                keyPointsRS[i] = keyPointsR[matches[i].trainIdx].pt;
            }

            cv::triangulatePoints(projMatL, projMatR, keyPointsLS,
                    keyPointsRS, homogenousPoints);

        }

        cv::imshow("main", disp);
        cv::imshow("left", mL);
        cv::imshow("right", mR);
        c = cv::waitKey(1);
    } while ('q' != c);

    return 0;
}

