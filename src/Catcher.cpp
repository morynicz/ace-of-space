///\file
///\brief File containing implementation of class Catcher
///\author Michał Orynicz
#include "Catcher.hpp"
//using namespace std;

/// Constructor for video streams originating from a device
/// \param nr - number of the device
/// \param mat - pointer to cv::Mat object which will contain the most
/// recent frame
/// \param mtx - pointer to mutex 
Catcher::Camera::Camera(const int &nr, cv::Mat *mat, std::mutex *mtx,
        std::mutex *fMtx, bool *finish) :
        _frameRate(0), _cam(nr), _fr(mat), _mut(mtx), _fin(fMtx), _finish(
                finish) {

    if (!_cam.isOpened()) {
        cv::Exception err(CATCH_CANNOT_OPEN_DEVICE,
                "file cannot be opened", __func__, __FILE__,
                __LINE__);
        throw err;
    }
}

/// Constructor for video streams originating from a device
/// \param name - name of video file
/// \param mat - pointer to cv::Mat object which will contain the most
/// recent frame
/// \param mtx - pointer to mutex 
Catcher::Camera::Camera(const std::string &name, cv::Mat *mat,
        std::mutex *mtx, std::mutex *fMtx, bool *finish) :
        _cam(name), _fr(mat), _mut(mtx), _fin(fMtx), _finish(finish) {

    if (!_cam.isOpened()) {
        cv::Exception err(CATCH_CANNOT_OPEN_FILE,
                "file cannot be opened", __func__, __FILE__,
                __LINE__);
        throw err;
    }
    _frameRate = _cam.get(cv::CAP_PROP_FPS);
    if (_frameRate <= 0) {
        _frameRate = 30;
    }
}

/// Empty destructor
Catcher::Camera::~Camera() {
}

/// Method which reads subsequent frames from stream. Ensures that frame 
/// buffer of stream is empty
void Catcher::Camera::operator()() {
    bool run = true;
    if (_frameRate > 0) {
        while (run) {
            _mut->lock();
            _cam >> (*_fr);
            _mut->unlock();
            cv::waitKey(10);
            _fin->lock();
            run = !*_finish;
            _fin->unlock();
        }
    } else {
        while (run) {
            _mut->lock();
            _cam >> (*_fr);
            _mut->unlock();
            cv::waitKey(10);
            _fin->lock();
            run = !*_finish;
            _fin->unlock();
        }
    }
}
/**
 * Method for acquiring cv::VideoCapture properties
 * \param propId - property id. To get possible values, see
 * cv::VideoCapture::get documentation
 */
double Catcher::Camera::get(const int &propId) {
    return _cam.get(propId);
}
///Method for setting cv::VideoCapture properties
///\param propId - property id. To get possible values, see
///cv::VideoCapture::set documentation
///\param value - value to which property @propId will be set
bool Catcher::Camera::set(const int &propId, const double &value) {
    return _cam.set(propId, value);
}

/// Method checking if underlying cv::VideoCapture is opened
/// \retval true if capture is opened
/// \retval false if it is not
bool Catcher::Camera::isOpened() {
    _mut->lock();
    bool result = _cam.isOpened();
    _mut->unlock();
    return result;
}

/// Initializes mut and thr pointers
Catcher::Catcher() :
        _mut(nullptr), _thr(nullptr),_fin(nullptr), _finish(false) {

}

/// Opens video device
Catcher::Catcher(const int &nr) :
        _mut(new std::mutex), _fin(new std::mutex), _finish(false), _cam(
                Camera(nr, &_fr, _mut, _fin, &_finish)), _thr(
                new std::thread(std::ref(_cam))) {
}

/// Opens video stream
Catcher::Catcher(const std::string &name) :
        _mut(new std::mutex), _fin(new std::mutex), _finish(false), _cam(
                Camera(name, &_fr, _mut, _fin, &_finish)), _thr(
                new std::thread(std::ref(_cam))) {
}

/// Method initializing the object for drawing video stream from a device 
/// with given number
/// \param nr - number of device from which the video stream will be drawn
bool Catcher::open(const int &nr) {
    if (_thr != nullptr) {
        _fin->lock();
        _finish = true;
        _fin->unlock();
        _thr->join();
        delete _thr;
        _thr = nullptr;
    }
    if (_mut != nullptr) {
        delete _mut;
    }
    _mut = new std::mutex;
    _fin = new std::mutex;
    _finish = false;
    _cam = Camera(nr, &_fr, _mut, _fin, &_finish);
    if (_cam.isOpened()) {
        _thr = new std::thread(std::ref(_cam));
        return true;
    } else {
        return false;
    }
}

/// Method initializing the object for drawing video stream from a video
/// file
/// \param name - name of file from which the video stream will be read
bool Catcher::open(const std::string& name) {
    if (_thr != nullptr) {
        _fin->lock();
        _finish = true;
        _fin->unlock();
        _thr->join();
        delete _thr;
        _thr = nullptr;
    }
    if (_mut != nullptr) {
        delete _mut;
    }
    _mut = new std::mutex;
    _fin = new std::mutex;
    _finish = false;
    _cam = Camera(name, &_fr, _mut, _fin, &_finish);
    if (_cam.isOpened()) {
        _thr = new std::thread(std::ref(_cam));
        return true;
    } else {
        return false;
    }
}

/// Destructor, ensures that all threads and dynamic objects will be
/// disposed properly
Catcher::~Catcher() {
    if (_thr) {
        _fin->lock();
        _finish = true;
        _fin->unlock();
        _thr->join();
        delete _thr;
    }
    if (_mut) {
        delete _mut;
    }
    if (_fin) {
        delete _fin;
    }
}

/// Method which makes a deep copy of most recent frame, and returns it
/// outside
/// \param frame - object in which the deep copy of most recent frame will
/// be placed
bool Catcher::read(cv::Mat& frame) {
    _mut->lock();
    frame = _fr.clone();
    _mut->unlock();
    return !frame.empty(); //need to correct this
}

///Method for acquiring cv::VideoCapture properties
///\param propId - property id. To get possible values, see
///cv::VideoCapture::get documentation
double Catcher::get(const int &propId) {
    double result;
    _mut->lock();
    result = _cam.get(propId);
    _mut->unlock();
    return result;
}
///Method for setting cv::VideoCapture properties
///\param propId - property id. To get possible values, see
///cv::VideoCapture::set documentation
///\param value - value to which property propId will be set
bool Catcher::set(const int &propId, const double &value) {
    bool result;
    _mut->lock();
    result = _cam.set(propId, value);
    _mut->unlock();
    return result;
}

Catcher& Catcher::operator>>(cv::Mat &frame) {
    read(frame);
    return *this;
}
