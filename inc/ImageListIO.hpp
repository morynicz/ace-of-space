/*
 * ImageListIO.hpp
 *
 *  Created on: 3 Jan 2015
 *      Author: Michał Orynicz
 */

#ifndef IMAGELISTIO_HPP_
#define IMAGELISTIO_HPP_

#include <string>
#include <list>

#include <opencv2/core/core.hpp>

///Load image list from directory given by path
/**
 * Function loading image list to list container based on imageList.xml
 * file located in directory pointed by path
 * @param path - location of imageList.xml
 * @param size - width and heigth of the images
 * @param imageList - loaded pairs of images
 */

void loadImageList(const std::string &path, cv::Size &imageSize,
		std::list<std::pair<cv::Mat, cv::Mat>> &imageList);

/// Save list of images to directory given by path
/**
 * Function saves images from imageList to directory given by path,
 * It writes the list of the images paths  and single image size to
 * imageList.xml file in target directory.
 * @param path - target directory for images and imageList.txt
 * @param size - geometric size of single image.
 * @param imageList - list conatining pairs of images to save
 */
void saveImageList(const std::string &path, const cv::Size &size,
		const std::list<std::pair<cv::Mat, cv::Mat>> &imageList);

#endif /* IMAGELISTIO_HPP_ */
