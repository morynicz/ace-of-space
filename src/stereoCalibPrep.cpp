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

#include "ConvenienceFunctions.hpp"

const int USER_TRIGGERED_EXIT = 0;

void parseCommandline(const int &argc, char **argv, std::string &target,
        int &leftCapture, int &rightCapture) {

    boost::program_options::options_description desc;

    desc.add_options()("help,h", "this help message")("left,l",
            boost::program_options::value<int>(),
            "number of /dev/videoX device used as left camera")("right,r",
            boost::program_options::value<int>(),
            "number of /dev/videoX device used as left camera")("output,o",
            boost::program_options::value<std::string>(),
            "output file for list of pictures");
    boost::program_options::variables_map vm;
    boost::program_options::store(
            boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        throw USER_TRIGGERED_EXIT;
    }

    if (vm.count("left")) {
        leftCapture = vm["left"].as<int>();
    }

    if (vm.count("right")) {
        rightCapture = vm["right"].as<int>();
    }

    if (vm.count("output")) {
        target = vm["output"].as<std::string>();
    }
}

int main(int argc, char **argv) {
    std::string target;
    int leftDevice;
    int rightDevice;

    cv::VideoCapture leftCapture;
    cv::VideoCapture rightCapture;

    parseCommandline(argc, argv, target, leftDevice, rightDevice);

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
    cv::Size imageSize(leftCapture.get(CV_CAP_PROP_FRAME_WIDTH),
            leftCapture.get(CV_CAP_PROP_FRAME_HEIGHT));

    cv::Mat display(imageSize.height, imageSize.width * 2, CV_8UC3);

    char c = ' ';

    cv::Rect leftRoi(cv::Point(imageSize.width, 0), imageSize);
    cv::Rect rightRoi(cv::Point(0, 0), imageSize);
    std::cerr << imageSize << std::endl;
    std::cerr << display.size() << std::endl;
    std::cerr << leftRoi << std::endl;
    std::cerr << rightRoi << std::endl;
    cv::namedWindow("main", cv::WINDOW_NORMAL);

    cv::FileStorage fs(target + "/imageList.xml", cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        cv::Exception ex(-1, "Could not open FileStorage", __func__, __FILE__,
                __LINE__);
    }

    fs << "images" << "[";
    int counter = 0;
    int delay = 300;
    int dc = 0;
    cv::waitKey(1000);
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
            std::string leftPath = target + "/left" + std::to_string(counter)
                    + ".png";
            std::string rightPath = target + "/right" + std::to_string(counter)
                    + ".png";
            cv::imwrite(leftPath, l);
            cv::imwrite(rightPath, r);
            fs << leftPath;
            fs << rightPath;
            dc = 0;
        }
        ++dc;
    } while ('q' != c);
    fs << "]";
    fs.release();
}