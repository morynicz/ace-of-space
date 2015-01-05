/**
 * \file stereoCalibPrep.cpp
 *
 *  \date Aug 6, 2013
 *  \author Michał Orynicz
 */

#include <boost/program_options.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <iostream>
#include <list>

#include "ConvenienceFunctions.hpp"
#include "StorageFunctions.hpp"

const int USER_TRIGGERED_EXIT = 0;

void parseCommandline(const int &argc, char **argv, std::string &target,
        int &leftCapture, int &rightCapture, cv::Size &chessboardSize,
        double & length) {

    boost::program_options::options_description desc;

    desc.add_options()("help,h", "this help message")("left,l",
            boost::program_options::value<int>(),
            "number of /dev/videoX device used as left camera")("right,r",
            boost::program_options::value<int>(),
            "number of /dev/videoX device used as left camera")("output,o",
            boost::program_options::value<std::string>(),
            "output file for list of pictures")("length,L",
            boost::program_options::value<double>(),
            "Legth of side of a single chessboard field in chosen units."
                    "These units will be used later as measure unit for all calculations")(
            "width,W", boost::program_options::value<int>(),
            "Width of calibration chessboard in number of inner corners")(
            "height,H", boost::program_options::value<int>(),
            "Height of calibration chessboard in number of inner corners");
    boost::program_options::variables_map vm;
    boost::program_options::store(
            boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        throw USER_TRIGGERED_EXIT;
    }
//TODO Make some assertion for missing arguments
    if (vm.count("left")) {
        leftCapture = vm["left"].as<int>();
    }
    if (vm.count("right")) {
        rightCapture = vm["right"].as<int>();
    }
    if (vm.count("output")) {
        target = vm["output"].as<std::string>();
    }
    if (vm.count("width")) {
        chessboardSize.width = vm["width"].as<int>();
    }
    if (vm.count("height")) {
        chessboardSize.height = vm["height"].as<int>();
    }
    if (vm.count("length")) {
        length = vm["length"].as<double>();
    }
}

cv::Size normalizeCaptureAndGetSize(cv::VideoCapture &leftCapture,
        cv::VideoCapture &rightCapture) {
    cv::Size leftImageSize(leftCapture.get(CV_CAP_PROP_FRAME_WIDTH),
            leftCapture.get(CV_CAP_PROP_FRAME_HEIGHT));

    cv::Size rightImageSize(rightCapture.get(CV_CAP_PROP_FRAME_WIDTH),
            rightCapture.get(CV_CAP_PROP_FRAME_HEIGHT));

    cv::Size result = leftImageSize;

    if (leftImageSize.height < rightImageSize.height) {
        rightCapture.set(CV_CAP_PROP_FRAME_HEIGHT, leftImageSize.height);
    } else if (leftImageSize.height > rightImageSize.height) {
        leftCapture.set(CV_CAP_PROP_FRAME_HEIGHT, rightImageSize.height);
        result.height = rightImageSize.height;
    }

    if (leftImageSize.width < rightImageSize.width) {
        rightCapture.set(CV_CAP_PROP_FRAME_WIDTH, leftImageSize.width);
    } else if (leftImageSize.width > rightImageSize.width) {
        leftCapture.set(CV_CAP_PROP_FRAME_WIDTH, rightImageSize.width);
        result.width = rightImageSize.width;
    }

    return result;
}

int main(int argc, char **argv) {
    std::string target;
    int leftDevice;
    int rightDevice;
    cv::Size chessboardSize;
    double sideLenght;

    cv::VideoCapture leftCapture;
    cv::VideoCapture rightCapture;

    parseCommandline(argc, argv, target, leftDevice, rightDevice,chessboardSize,sideLenght);

    leftCapture.open(leftDevice);
    if (!leftCapture.isOpened()) {
        cv::Exception ex(0, "Didn't open left device", __func__, __FILE__,
        __LINE__);
        throw ex;
    }
    rightCapture.open(rightDevice);
    if (!rightCapture.isOpened()) {
        cv::Exception ex(0, "Didn't open right device", __func__, __FILE__,
        __LINE__);
        throw ex;
    }
    cv::Size imageSize = normalizeCaptureAndGetSize(leftCapture, rightCapture);

    cv::Mat display(imageSize.height, imageSize.width * 2, CV_8UC3);

    char c = ' ';

    cv::Rect leftRoi(cv::Point(imageSize.width, 0), imageSize);
    cv::Rect rightRoi(cv::Point(0, 0), imageSize);
    std::cerr << imageSize << std::endl;
    std::cerr << display.size() << std::endl;
    std::cerr << leftRoi << std::endl;
    std::cerr << rightRoi << std::endl;
    cv::namedWindow("main", cv::WINDOW_NORMAL);

    int delay = 30;
    int dc = 0;
    cv::waitKey(1000);

    std::list<std::pair<cv::Mat, cv::Mat>> imageList;
    do {
        std::vector<cv::Mat> tmp(2);
        cv::Mat l = display(leftRoi);
        cv::Mat r = display(rightRoi);
        leftCapture.grab();
        rightCapture.grab();
        leftCapture.retrieve(tmp[0]);
        rightCapture.retrieve(tmp[1]);
        tmp[0].copyTo(l);
        tmp[1].copyTo(r);
        cv::imshow("main", display);
        c = cv::waitKey(1);
        if (dc >= delay) {
            std::pair<cv::Mat, cv::Mat> pair;
            pair.first = tmp[0].clone();
            pair.second = tmp[1].clone();

            imageList.push_back(pair);
            dc = 0;
        }
        std::cerr << dc << std::endl;
        ++dc;
    } while ('q' != c);

    saveImageList(target, imageSize, chessboardSize, sideLenght, imageList);
}
