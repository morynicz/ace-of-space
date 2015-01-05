/*
 * editStereoList.cpp
 *
 *  Created on: 3 Jan 2015
 *      Author: link
 */

#include <boost/program_options.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <list>

#include "ConvenienceFunctions.hpp"
#include "StorageFunctions.hpp"

const int USER_TRIGGERED_EXIT = 0;

void parseCommandline(const int &argc, char **argv, std::string &input,
        std::string &output) {

    boost::program_options::options_description desc;

    desc.add_options()("help,h", "this help message")("input,i",
            boost::program_options::value<std::string>(),
            "input directory containing imageList.xml")("output,o",
            boost::program_options::value<std::string>(),
            "output directory to which purged imageList.xml will be written");
    boost::program_options::variables_map vm;
    boost::program_options::store(
            boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        throw USER_TRIGGERED_EXIT;
    }

    if (vm.count("output")) {
        output = vm["output"].as<std::string>();
    }

    if (vm.count("input")) {
        input = vm["input"].as<std::string>();
    }
}

int main(int argc, char **argv) {
    std::string input, output;

    parseCommandline(argc, argv, input, output);

    cv::Size imageSize, chessboardSize;
    double sideLength;
    std::list<std::pair<cv::Mat, cv::Mat>> imageList;

    loadImageList(input, imageSize, chessboardSize, sideLength, imageList);

    cv::Mat display(imageSize.height, imageSize.width * 2, CV_8UC1);

    cv::Rect leftRoi(cv::Point(imageSize.width, 0), imageSize);
    cv::Rect rightRoi(cv::Point(0, 0), imageSize);
    std::cerr << imageSize << std::endl;
    std::cerr << display.size() << std::endl;
    std::cerr << leftRoi << std::endl;
    std::cerr << rightRoi << std::endl;
    cv::namedWindow("main", cv::WINDOW_NORMAL);

    char c = ' ';

    auto bIt = imageList.begin();
    int cnt = 0;
    do {
        cv::Mat left = display(leftRoi);
        cv::Mat right = display(rightRoi);
        ((std::pair<cv::Mat, cv::Mat>) *bIt).first.copyTo(left);
        ((std::pair<cv::Mat, cv::Mat>) *bIt).second.copyTo(right);
        cv::imshow("main", display);
        c = cv::waitKey(0);
        if('d' == c) {
            bIt = imageList.erase(bIt);
        }
        ++bIt;
        std::cerr << cnt++ << std::endl;
    } while (imageList.end() != bIt && 'q' != c);

    saveImageList(output, imageSize, chessboardSize, sideLength, imageList);
}

