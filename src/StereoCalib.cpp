/*
 * StereoCalib.cpp
 *
 *  Created on: 3 Jan 2015
 *      Author: link
 */

#include <iostream>
#include <list>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <StorageFunctions.hpp>
#include <boost/program_options.hpp>

#include "ConvenienceFunctions.hpp"

const int USER_TRIGGERED_EXIT = 0;

void parseCommandline(const int& argc,
                      char** argv,
                      std::string& input,
                      std::string& output)
{

  boost::program_options::options_description desc;

  desc.add_options()("help,h", "this help message")(
    "input,i",
    boost::program_options::value<std::string>(),
    "Input directory containing imageList.xml")(
    "output,o",
    boost::program_options::value<std::string>(),
    "Output file with calibration values for cameras")(
    "length,l",
    boost::program_options::value<double>(),
    "Legth of side of a single chessboard field in chosen units."
    "These units will be used later as measure unit for all calculations");
  boost::program_options::variables_map vm;
  boost::program_options::store(
    boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    throw USER_TRIGGERED_EXIT;
  }

  if (vm.count("output"))
  {
    output = vm["output"].as<std::string>();
  }

  if (vm.count("input"))
  {
    input = vm["input"].as<std::string>();
  }
}

bool getCorners(const cv::Mat& image,
                const cv::Size& chessboardSize,
                std::vector<cv::Point2f>& corners)
{

  if (cv::findChessboardCorners(image, chessboardSize, corners, 0))
  {
    cv::cornerSubPix(
      image,
      corners,
      cv::Size(11, 11),
      cv::Size(-1, -1),
      cv::TermCriteria(
        cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
    return true;
  }
  else
  {
    return false;
  }
}

int main(int argc, char** argv)
{
  std::string input, output;
  double sideLength;

  parseCommandline(argc, argv, input, output);

  cv::Size imageSize, chessboardSize;
  std::list<std::pair<cv::Mat, cv::Mat>> imageList;

  loadImageList(input, imageSize, chessboardSize, sideLength, imageList);

  std::vector<std::vector<cv::Point3f>> modelPoints(1);

  for (int i = 0; i < chessboardSize.height; ++i)
  {
    for (int j = 0; j < chessboardSize.width; ++j)
    {
      modelPoints[0].push_back(sideLength * cv::Point3f(j, i, 0));
    }
  }

  std::vector<std::vector<cv::Point2f>> leftPoints, rightPoints;
  {
    int i = 0;
    for (auto it = imageList.begin(); imageList.end() != it; ++it, ++i)
    {
      std::pair<cv::Mat, cv::Mat> pair = *it;
      std::cerr << i << std::endl;
      std::vector<cv::Point2f> lP, rP;
      if (getCorners(pair.first, chessboardSize, lP) &&
          getCorners(pair.second, chessboardSize, rP))
      {
        leftPoints.push_back(lP);
        rightPoints.push_back(rP);
      }
      else
      {
        it = imageList.erase(it);
        continue;
      }
    }
  }

  modelPoints.resize(leftPoints.size(), modelPoints[0]);

  cv::Mat lCM, rCM, lDC, rDC, R, T, E, F;

  std::vector<cv::Mat> rvecs, tvecs;

  double leftReprojectionError = cv::calibrateCamera(
    modelPoints, leftPoints, imageSize, lCM, lDC, rvecs, tvecs);
  rvecs.clear();
  tvecs.clear();
  double rightReprojectionError = cv::calibrateCamera(
    modelPoints, rightPoints, imageSize, rCM, rDC, rvecs, tvecs);

  std::cerr << "left rms" << leftReprojectionError << std::endl;
  std::cerr << "right rms" << rightReprojectionError << std::endl;

  double reprojectionError = cv::stereoCalibrate(
    modelPoints,
    leftPoints,
    rightPoints,
    lCM,
    lDC,
    rCM,
    rDC,
    imageSize,
    R,
    T,
    E,
    F,
    cv::CALIB_FIX_INTRINSIC,
    cv::TermCriteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, 1e-6));
  std::cerr << reprojectionError << std::endl;

  saveCalibParameters(output,
                      lCM,
                      rCM,
                      lDC,
                      rDC,
                      R,
                      T,
                      E,
                      F,
                      chessboardSize,
                      imageSize,
                      sideLength);
}
