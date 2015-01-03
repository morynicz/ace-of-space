/*
 * ImageListIO.cpp
 *
 *  Created on: 3 Jan 2015
 *      Author: Michał Orynicz
 */

#include <iostream>

#include <opencv2/highgui/highgui.hpp>

#include "ImageListIO.hpp"

void saveImageList(const std::string &path, const cv::Size &size,
        const std::list<std::pair<cv::Mat, cv::Mat>> &imageList) {
    cv::FileStorage fs(path + "/imageList.xml", cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        cv::Exception ex(-1, "Could not open FileStorage", __func__, __FILE__,
        __LINE__);
    }

    fs << "image_size" << size;

    fs << "images" << "[";
    int counter = 0;
    for (auto it = imageList.begin(); imageList.end() != it; ++it, ++counter) {
        std::string leftPath = path + "/left" + std::to_string(counter)
                + ".png";
        std::string rightPath = path + "/right" + std::to_string(counter)
                + ".png";
        std::pair<cv::Mat, cv::Mat> pair = *it;
        cv::Mat l = pair.first;
        cv::Mat r = pair.second;

        cv::imwrite(leftPath, l);
        cv::imwrite(rightPath, r);

        fs << "{" << "left" << leftPath;
        fs << "right" << rightPath << "}";
        std::cerr << leftPath << " " << rightPath << std::endl;
    }

    fs << "]";
    fs.release();
}

void loadImageList(const std::string &path, cv::Size &imageSize,
        std::list<std::pair<cv::Mat, cv::Mat>> &imageList) {
    cv::FileStorage fs(path + "/imageList.xml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        cv::Exception ex(-1, "Could not open FileStorage", __func__, __FILE__,
        __LINE__);
    }

    fs["image_size"] >> imageSize;

    cv::FileNode images = fs["images"];

    for (cv::FileNodeIterator it = images.begin(); images.end() != it; ++it) {
        std::pair<cv::Mat, cv::Mat> pair;
        std::string leftString = (*it)["left"];
        std::string rightString = (*it)["right"];
        pair.first = cv::imread(leftString);
        pair.second = cv::imread(rightString);
        imageList.push_back(pair);
        std::cerr<<pair.first.size()<<" "<<pair.second.size()<<std::endl;
    }

    fs.release();
}

