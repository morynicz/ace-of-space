/*
 * ImageListIO.cpp
 *
 *  Created on: 3 Jan 2015
 *      Author: Michał Orynicz
 */

#include <iostream>

#include "StorageFunctions.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ConvenienceFunctions.hpp"

void assureGray(cv::Mat& img)
{
  if (CV_8UC1 != img.type())
  {
    cv::Mat tmp = img.clone();
    cv::cvtColor(tmp, img, cv::COLOR_BGR2GRAY);
  }
}

void saveImageList(const std::string& path,
                   const cv::Size& imageSize,
                   const cv::Size& chessboardSize,
                   const double& sideLength,
                   const std::list<std::pair<cv::Mat, cv::Mat>>& imageList)
{
  cv::FileStorage fs(path + "/imageList.xml", cv::FileStorage::WRITE);
  if (!fs.isOpened())
  {
    cv::Exception ex(
      -1, "Could not open FileStorage", __func__, __FILE__, __LINE__);
  }

  fs << "image_size" << imageSize;
  fs << "chessboard_size" << chessboardSize;
  fs << "side_length" << sideLength;

  fs << "images"
     << "[";
  int counter = 0;
  for (auto it = imageList.begin(); imageList.end() != it; ++it, ++counter)
  {
    std::string leftPath = path + "/left" + std::to_string(counter) + ".png";
    std::string rightPath = path + "/right" + std::to_string(counter) + ".png";
    std::pair<cv::Mat, cv::Mat> pair = *it;
    cv::Mat l = pair.first;
    cv::Mat r = pair.second;

    assureGray(l);
    assureGray(r);

    cv::imwrite(leftPath, l);
    cv::imwrite(rightPath, r);

    fs << "{"
       << "left" << leftPath;
    fs << "right" << rightPath << "}";
  }

  fs << "]";
  fs.release();
}

void loadImageList(const std::string& path,
                   cv::Size& imageSize,
                   cv::Size& chessboardSize,
                   double& sideLength,
                   std::list<std::pair<cv::Mat, cv::Mat>>& imageList)
{
  cv::FileStorage fs(path + "/imageList.xml", cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    cv::Exception ex(
      -1, "Could not open FileStorage", __func__, __FILE__, __LINE__);
  }

  fs["image_size"] >> imageSize;
  fs["chessboard_size"] >> chessboardSize;
  fs["side_length"] >> sideLength;

  cv::FileNode images = fs["images"];

  for (cv::FileNodeIterator it = images.begin(); images.end() != it; ++it)
  {
    std::pair<cv::Mat, cv::Mat> pair;
    std::string leftString = (*it)["left"];
    std::string rightString = (*it)["right"];
    pair.first = cv::imread(leftString);
    pair.second = cv::imread(rightString);
    assureGray(pair.first);
    assureGray(pair.second);
    imageList.push_back(pair);
  }

  fs.release();
}

void saveCalibParameters(const std::string& path,
                         const cv::Mat& lCM,
                         const cv::Mat& rCM,
                         const cv::Mat& lDC,
                         const cv::Mat& rDC,
                         const cv::Mat& R,
                         const cv::Mat& T,
                         const cv::Mat& E,
                         const cv::Mat& F,
                         const cv::Size& chessboardSize,
                         const cv::Size& imageSize,
                         const double& sideLength)
{
  cv::FileStorage fs(path, cv::FileStorage::WRITE);
  if (!fs.isOpened())
  {
    cv::Exception ex(
      -1, "Could not open FileStorage", __func__, __FILE__, __LINE__);
  }

  fs << "image_size" << imageSize;
  fs << "chessboard_size" << chessboardSize;
  fs << "side_length" << sideLength;

  fs << "left_camera_matrix" << lCM;
  fs << "right_camera_matrix" << rCM;
  fs << "left_dist_coeffs" << lDC;
  fs << "right_dist_coeffs" << rDC;

  fs << "R" << R;
  fs << "T" << T;
  fs << "E" << E;
  fs << "F" << F;
  fs.release();
}

void loadCalibParameters(const std::string& path,
                         cv::Mat& lCM,
                         cv::Mat& rCM,
                         cv::Mat& lDC,
                         cv::Mat& rDC,
                         cv::Mat& R,
                         cv::Mat& T,
                         cv::Mat& E,
                         cv::Mat& F,
                         cv::Size& chessboardSize,
                         cv::Size& imageSize,
                         double& sideLength)
{
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    cv::Exception ex(
      -1, "Could not open FileStorage", __func__, __FILE__, __LINE__);
  }

  fs["image_size"] >> imageSize;
  fs["chessboard_size"] >> chessboardSize;
  fs["side_length"] >> sideLength;

  fs["left_camera_matrix"] >> lCM;
  fs["right_camera_matrix"] >> rCM;
  fs["left_dist_coeffs"] >> lDC;
  fs["right_dist_coeffs"] >> rDC;

  fs["R"] >> R;
  fs["T"] >> T;
  fs["E"] >> E;
  fs["F"] >> F;
  fs.release();
}
