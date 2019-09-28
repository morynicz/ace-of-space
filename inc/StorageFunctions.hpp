/*
 * ImageListIO.hpp
 *
 *  Created on: 3 Jan 2015
 *      Author: Michał Orynicz
 */

#pragma once

#include <list>
#include <string>

#include <opencv2/core/core.hpp>

/// Load image list from directory given by path
/**
 * Function loading image list to list container based on imageList.xml
 * file located in directory pointed by path
 * @param path - location of imageList.xml
 * @param imageSize - width and heigth of the images
 * @param chessboardSize - size in number of inner corners.
 * @param sideLength - side length of single chesboard field in chosen units.
 * @param imageList - loaded pairs of images
 */

void loadImageList(const std::string& path,
                   cv::Size& imageSize,
                   cv::Size& chessboardSize,
                   double& sideLength,
                   std::list<std::pair<cv::Mat, cv::Mat>>& imageList);

/// Save list of images to directory given by path
/**
 * Function saves images from imageList to directory given by path,
 * It writes the list of the images paths  and single image size to
 * imageList.xml file in target directory.
 * @param path - target directory for images and imageList.txt
 * @param imageSize - geometric size of single image.
 * @param chessboardSize - size in number of inner corners.
 * @param sideLength - side length of single chesboard field in chosen units.
 * @param imageList - list conatining pairs of images to save
 */
void saveImageList(const std::string& path,
                   const cv::Size& imageSize,
                   const cv::Size& chessboardSize,
                   const double& sideLength,
                   const std::list<std::pair<cv::Mat, cv::Mat>>& imageList);

void loadCalibParameters(const std::string& path,
                         cv::Mat& leftCameraMatrix,
                         cv::Mat& rightCameraMatrix,
                         cv::Mat& leftDistCoeffs,
                         cv::Mat& rightDistCoeffs,
                         cv::Mat& R,
                         cv::Mat& T,
                         cv::Mat& E,
                         cv::Mat& F,
                         cv::Size& chessboardSize,
                         cv::Size& imageSize,
                         double& sideLength);

void saveCalibParameters(const std::string& path,
                         const cv::Mat& leftCameraMatrix,
                         const cv::Mat& rightCameraMatrix,
                         const cv::Mat& leftDistCoeffs,
                         const cv::Mat& rightDistCoeffs,
                         const cv::Mat& R,
                         const cv::Mat& T,
                         const cv::Mat& E,
                         const cv::Mat& F,
                         const cv::Size& chessboardSize,
                         const cv::Size& imageSize,
                         const double& sideLength);
