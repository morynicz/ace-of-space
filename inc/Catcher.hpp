/// \file
/// \brief Header file for class Catcher
/// \author Michał Orynicz
#pragma once

#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

///\brief Error code for inability to open a device
static const int CATCH_CANNOT_OPEN_DEVICE = -1;
///\brief Error code for inability to open a fil
static const int CATCH_CANNOT_OPEN_FILE = -2;

///\brief Class providing ability to catch the most recent frame from
/// video stream

class Catcher
{
  std::vector<cv::Mat> _fr; ///< Most recent frame
  std::mutex* _mut;
  std::mutex* _fin; ///< finish mutex
  bool _finish;     ///< finish flag
  ///\brief Class used to ensure, that video streams buffer is empty
  class Camera
  {
    std::vector<cv::VideoCapture> _cam; ///< video stream
    std::vector<cv::Mat>* _fr;          ///< Most recent frame
    std::mutex* _mut;                   ///< Mutex
    std::mutex* _fin;                   ///< Finish mutex
    bool* _finish;                      ///< Finish flag
  public:
    ///\brief Constructor, initializes mutex pointer
    Camera()
      : _fr(nullptr)
      , _mut(nullptr)
      , _fin(nullptr)
      , _finish(nullptr)
    {
    }
    ///\brief Constructor for video streams from a device
    Camera(const std::vector<int>& nr,
           std::vector<cv::Mat>* mat,
           std::mutex* mtx,
           std::mutex* fMtx,
           bool* finish);
    ///\brief Function used to start emptying of the buffer
    void operator()();
    ///\brief Destructor
    ~Camera();
    ///\brief Method for acquiring cv::VideoCapture properties
    double get(const int& propId, const int& channel = 0);
    ///\brief Method for setting cv::VideoCapture properties
    bool set(const int& propId, const double& value, const int& channel = 0);
    ///\brief Method checks if cv::VideoCapture is opened
    bool isOpened(const int& channel) const;
    ///\brief Method checks if all cv::VideoCaptures are opened
    bool isOpened(void) const;
  };
  Camera _cam;       ///< Object providing the most recent frame
  std::thread* _thr; ///< thread receiving all the images from camera

public:
  ///\brief Constructor initialising pointer fields
  Catcher();
  ///\brief Constructor for stream from video device
  Catcher(const int& nr);
  ///\brief Constructor for stream from video devices
  Catcher(const std::vector<int>& nrs);
  ///\brief initialization method for streams from a device
  bool open(const int& nr);
  ///\brief initialization method for streams from a devices
  bool open(const std::vector<int>& nrs);
  ///\brief Destructor removing mutex and thread objects
  ~Catcher();
  ///\brief Method for retrieving the most recent frame
  virtual bool read(cv::Mat& frame, const int& channel);
  ///\brief Method for retrieving the most recent frames
  virtual bool read(std::vector<cv::Mat>& frames);
  ///\brief Method calls read method
  virtual Catcher& operator>>(cv::Mat& frame);
  ///\brief Method for acquiring cv::VideoCapture properties
  virtual double get(const int& propId, const int& channel = 0);
  ///\brief Method for setting cv::VideoCapture properties
  virtual bool set(const int& propId,
                   const double& value,
                   const int& channel = 0);
};
