///\file
///\brief File containing implementation of class Catcher
///\author Michał Orynicz
#include "Catcher.hpp"
#include "ConvenienceFunctions.hpp"
/**
 * Constructor for video streams originating from a device
 * \param nrs - numbers of the devices
 * \param mat - pointer to vector of cv::Mat objects which
 * will contain the most recent frame
 * \param mtx - pointer to mutex
 */
Catcher::Camera::Camera(const std::vector<int>& nr,
                        std::vector<cv::Mat>* mat,
                        std::mutex* mtx,
                        std::mutex* fMtx,
                        bool* finish)
  : _fr(mat)
  , _mut(mtx)
  , _fin(fMtx)
  , _finish(finish)
{

  _cam.resize(nr.size());
  _fr->resize(nr.size());
  for (unsigned i = 0; i < _cam.size(); ++i)
  {
    std::cerr << "nri" << nr[i] << std::endl;
    _cam[i].open(nr[i]);
    if (!_cam[i].isOpened())
    {
      cv::Exception err(CATCH_CANNOT_OPEN_DEVICE,
                        "file cannot be opened",
                        __func__,
                        __FILE__,
                        __LINE__);
      throw err;
    }
  }
}

/// Empty destructor
Catcher::Camera::~Camera() {}

/// Method which reads subsequent frames from stream. Ensures that frame
/// buffer of stream is empty
void Catcher::Camera::operator()()
{
  bool run = true;
  cv::waitKey(1000);
  while (run)
  {
    _mut->lock();
    for (register unsigned i = 0; i < _cam.size(); ++i)
    {
      _cam[i].grab();
    }
    for (unsigned i = 0; i < _cam.size(); ++i)
    {
      _cam[i].retrieve((*_fr)[i]);
    }
    _mut->unlock();
    cv::waitKey(10);
    _fin->lock();
    run = !*_finish;
    _fin->unlock();
  }
}
/**
 * Method for acquiring cv::VideoCapture properties
 * \param propId - property id. To get possible values, see
 * cv::VideoCapture::get documentation
 */
double Catcher::Camera::get(const int& propId, const int& channel)
{
  return _cam[channel].get(propId);
}
/// Method for setting cv::VideoCapture properties
///\param propId - property id. To get possible values, see
/// cv::VideoCapture::set documentation
///\param value - value to which property @propId will be set
bool Catcher::Camera::set(const int& propId,
                          const double& value,
                          const int& channel)
{
  return _cam[channel].set(propId, value);
}

/// Method checking if underlying cv::VideoCapture channel is opened
/// \retval true if capture is opened
/// \retval false if it is not
bool Catcher::Camera::isOpened(const int& channel) const
{
  _mut->lock();
  bool result = _cam[channel].isOpened();
  _mut->unlock();
  return result;
}

/// Method checking if underlying cv::VideoCapture is opened
/// \retval true if capture is opened
/// \retval false if it is not
bool Catcher::Camera::isOpened() const
{
  _mut->lock();
  bool result = true;
  for (unsigned i = 0; i < _cam.size(); ++i)
  {
    result = _cam[i].isOpened() && result;
  }
  _mut->unlock();
  return result;
}

/// Initializes mut and thr pointers
Catcher::Catcher()
  : _mut(nullptr)
  , _fin(nullptr)
  , _finish(false)
  , _thr(nullptr)
{
}

/// Opens video devices
Catcher::Catcher(const std::vector<int>& nrs)
  : _mut(new std::mutex)
  , _fin(new std::mutex)
  , _finish(false)
  , _cam(Camera(nrs, &_fr, _mut, _fin, &_finish))
  , _thr(new std::thread(std::ref(_cam)))
{
}

/// Opens video device
Catcher::Catcher(const int& nr)
  : _mut(new std::mutex)
  , _fin(new std::mutex)
  , _finish(false)
  , _cam(Camera(std::vector<int>(1, nr), &_fr, _mut, _fin, &_finish))
  , _thr(new std::thread(std::ref(_cam)))
{
}

/// Method initializing the object for drawing video stream from devices
/// with given numbers
/// \param nrs - numbers of devices from which the video stream will be drawn
bool Catcher::open(const std::vector<int>& nrs)
{
  if (_thr != nullptr)
  {
    _fin->lock();
    _finish = true;
    _fin->unlock();
    _thr->join();
    delete _thr;
    _thr = nullptr;
  }
  if (_mut != nullptr)
  {
    delete _mut;
  }
  _mut = new std::mutex;
  _fin = new std::mutex;
  _finish = false;
  _cam = Camera(nrs, &_fr, _mut, _fin, &_finish);
  if (_cam.isOpened())
  {
    _thr = new std::thread(std::ref(_cam));
    return true;
  }
  else
  {
    return false;
  }
}

/// Method initializing the object for drawing video stream from a device
/// with given number
/// \param nr - number of device from which the video stream will be drawn
bool Catcher::open(const int& nr)
{
  return open(std::vector<int>(1, nr));
}

/// Destructor, ensures that all threads and dynamic objects will be
/// disposed properly
Catcher::~Catcher()
{
  if (_thr)
  {
    _fin->lock();
    _finish = true;
    _fin->unlock();
    _thr->join();
    delete _thr;
  }
  if (_mut)
  {
    delete _mut;
  }
  if (_fin)
  {
    delete _fin;
  }
}

/// Method which makes a deep copy of most recent frame, and returns it
/// outside
/// \param frame - object in which the deep copy of most recent frame will
/// be placed
bool Catcher::read(cv::Mat& frame, const int& channel)
{
  _mut->lock();
  frame = _fr[channel].clone();
  _mut->unlock();
  return !frame.empty(); // need to correct this
}

/// Method which makes a deep copy of most recent frame, and returns it
/// outside
/// \param frame - object in which the deep copy of most recent frame will
/// be placed
bool Catcher::read(std::vector<cv::Mat>& frame)
{
  frame.resize(_fr.size());
  _mut->lock();
  for (unsigned i = 0; i < _fr.size(); ++i)
  {
    if (_fr[i].empty())
    {
      cv::Exception ex(0, "Empty matrix", __func__, __FILE__, __LINE__);
      throw ex;
    }
    frame[i] = _fr[i].clone();
  }
  _mut->unlock();
  return !frame.empty(); // need to correct this
}

/// Method for acquiring cv::VideoCapture properties
///\param propId - property id. To get possible values, see
/// cv::VideoCapture::get documentation
double Catcher::get(const int& propId, const int& channel)
{
  double result;
  _mut->lock();
  result = _cam.get(propId, channel);
  _mut->unlock();
  return result;
}
/// Method for setting cv::VideoCapture properties
///\param propId - property id. To get possible values, see
/// cv::VideoCapture::set documentation
///\param value - value to which property propId will be set
bool Catcher::set(const int& propId, const double& value, const int& channel)
{
  bool result;
  _mut->lock();
  result = _cam.set(propId, value);
  _mut->unlock();
  return result;
}

Catcher& Catcher::operator>>(cv::Mat& frame)
{
  read(frame, 0);
  return *this;
}
