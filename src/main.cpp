/**
 * \file main.cpp
 *
 *  \date Aug 4, 2013
 *  \author Michał Orynicz
 */

#include <boost/program_options.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <iostream>

#include "ConvenienceFunctions.hpp"
#include "StorageFunctions.hpp"

const int USER_TRIGGERED_EXIT = 0;

using std::cerr;
using std::endl;

void parseCommandline(const int &argc, char **argv, std::string &input,
        int &leftCapture, int &rightCapture) {

    boost::program_options::options_description desc;

    desc.add_options()("help,h", "this help message")("left,l",
            boost::program_options::value<int>(),
            "number of /dev/videoX device used as left camera")("right,r",
            boost::program_options::value<int>(),
            "number of /dev/videoX device used as left camera")("input,i",
            boost::program_options::value<std::string>(),
            "input file with calibration parameters");
    boost::program_options::variables_map vm;
    boost::program_options::store(
            boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        throw USER_TRIGGERED_EXIT;
    }

    if (vm.count("input")) {
        input = vm["input"].as<std::string>();
    }

    if (vm.count("left")) {
        leftCapture = vm["left"].as<int>();
    }
    if (vm.count("right")) {
        rightCapture = vm["right"].as<int>();
    }
}

int main(int argc, char **argv) {
    cv::initModule_nonfree();
    cv::VideoCapture capL, capR;

    std::string path;
    int leftDevice, rightDevice;

    cv::Mat projMatL, projMatR, rectL, rectR, disparityToDepthMap, lCM, rCM,
            lDC, rDC, R, T, E, F, lR, rR, lP, rP, Q;

    cv::Mat lMap1, lMap2, rMap1, rMap2;
    double sideLength;
    cv::Size chessboardSize, imageSize;

    parseCommandline(argc, argv, path, leftDevice, rightDevice);

    capL.open(leftDevice);
    capR.open(rightDevice);

    cv::waitKey(100);

    char c = ' ';

    loadCalibParameters(path, lCM, rCM, lDC, rDC, R, T, E, F, chessboardSize,
            imageSize, sideLength);

//    cv::Size imageSize(capL.get(CV_CAP_PROP_FRAME_WIDTH),
//            capL.get(CV_CAP_PROP_FRAME_HEIGHT));

    std::cerr << imageSize << std::endl;

    cv::Mat disp(imageSize.height, imageSize.width * 2, CV_8UC3);
    std::cerr << disp.size() << std::endl;

    cv::Rect leftRoi(cv::Point(0, 0), imageSize);
    cv::Rect rightRoi(cv::Point(imageSize.width, 0),
            imageSize);

    cv::namedWindow("left", cv::WINDOW_NORMAL);
    cv::namedWindow("right", cv::WINDOW_NORMAL);
    cv::namedWindow("main", cv::WINDOW_NORMAL);
    cv::namedWindow("matches", cv::WINDOW_NORMAL);
    cv::namedWindow("disparity", cv::WINDOW_NORMAL);

    cv::Ptr<cv::FeatureDetector> detector =
            cv::FeatureDetector::create("FAST");
    cv::Ptr<cv::DescriptorExtractor> descriptor =
            cv::DescriptorExtractor::create("FREAK");

    std::vector<cv::KeyPoint> keyPointsL, keyPointsR;
    std::vector<cv::Point2f> keyPointsLS, keyPointsRS;
    std::vector<cv::KeyPoint> goodKeyPointsL, goodKeyPointsR;

    std::vector<cv::DMatch> matches, goodMatches;
    cv::Mat descriptionL, descriptionR;

    cv::BFMatcher matcher(cv::NORM_L2, true);
    cv::Mat homogenousPoints;

    cv::stereoRectify(lCM, lDC, rCM, rDC, imageSize, R, T, lR, rR, lP, rP, Q,
            0);
    cv::initUndistortRectifyMap(lCM, lDC, lR, lP, imageSize, CV_16SC2, lMap1,
            lMap2);
    cv::initUndistortRectifyMap(rCM, rDC, rR, rP, imageSize, CV_16SC2, rMap1,
            rMap2);


    capL.set(CV_CAP_PROP_FRAME_WIDTH, imageSize.width);
    capL.set(CV_CAP_PROP_FRAME_HEIGHT, imageSize.height);
    capR.set(CV_CAP_PROP_FRAME_WIDTH, imageSize.width);
    capR.set(CV_CAP_PROP_FRAME_HEIGHT, imageSize.height);

    std::cerr << capL.get(CV_CAP_PROP_FRAME_WIDTH) << std::endl;
    std::cerr << capL.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
    std::cerr << capR.get(CV_CAP_PROP_FRAME_WIDTH) << std::endl;
    std::cerr << capR.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;


    do {
        cv::Mat mL = disp(leftRoi);
        cv::Mat mR = disp(rightRoi);
        cv::Mat tL, tR;
        cv::Mat matched;

        capL.grab();
        capR.grab();
        capL.retrieve(tL);
        capR.retrieve(tR);
        tL.copyTo(mL);
        tR.copyTo(mR);

        {
            cv::Mat gL, gR, uL, uR;
            cv::cvtColor(tL, gL, cv::COLOR_RGB2GRAY);
            cv::remap(gL, uL, lMap1, lMap2, cv::INTER_LINEAR,
                    cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
            cv::cvtColor(tR, gR, cv::COLOR_RGB2GRAY);
            cv::remap(gR, uR, rMap1, rMap2, cv::INTER_LINEAR,
                    cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
            keyPointsL.clear();
            keyPointsR.clear();
            goodMatches.clear();
            detector->detect(uL, keyPointsL);
            descriptor->compute(uL, keyPointsL, descriptionL);

            detector->detect(uR, keyPointsR);
            descriptor->compute(uR, keyPointsR, descriptionR);
            //TODO need to add filtering function like RANSAC or STH for the matches
            matcher.match(descriptionL, descriptionR, matches);

            {
                double max_dist = 0, min_dist = 1000;
                for (int i = 0; i < matches.size(); ++i) {
                    double dist = matches[i].distance;
                    if (dist > max_dist)
                        max_dist = dist;
                    if (dist < min_dist)
                        min_dist = dist;
                }

                for (int i = 0; i < matches.size(); ++i) {
                    if (matches[i].distance < min_dist + 0.3 * (max_dist - min_dist))
                        goodMatches.push_back(matches[i]);

                }
            }

            keyPointsLS.resize(matches.size());
            keyPointsRS.resize(matches.size());

            for (unsigned int i = 0; i < matches.size(); ++i) {
                keyPointsLS[i] = keyPointsL[matches[i].queryIdx].pt;
                keyPointsRS[i] = keyPointsR[matches[i].trainIdx].pt;
            }

            cv::triangulatePoints(lP, rP, keyPointsLS, keyPointsRS,
                    homogenousPoints);


            if (!keyPointsL.empty() && !keyPointsR.empty()
                    && !goodMatches.empty()) {
                cv::drawMatches(uL, keyPointsL, uR, keyPointsR, goodMatches,
                        matched);
                cv::imshow("matches", matched);
            }


            {
                cv::Mat disparity;
                cv::Mat shown;
                cv::StereoBM sbm;

                sbm(uL, uR, disparity);
                if (!disparity.empty()) {
                    disparity.convertTo(shown, CV_8U, 255 / (32 * 16.));
                    imshow("disparity", shown);
                }

                cv::Mat xyz;
                cv::reprojectImageTo3D(disparity, xyz, Q, true);

            }
            cv::imshow("left", uL);
            cv::imshow("right", uR);

        }
        cv::imshow("main", disp);
        c = cv::waitKey(1);
    } while ('q' != c);

    return 0;
}
