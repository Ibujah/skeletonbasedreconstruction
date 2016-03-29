/*
Copyright (c) 2016 Bastien Durix

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


/**
 *  \file CameraFile.cpp
 *  \brief Defines camera file reader and writer
 *  \author Bastien Durix
 */

#include "CameraFile.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <mathtools/affine/Frame.h>
#include <camera/PinHole.h>

camera::Camera::Ptr fileio::ReadCamera(const std::string &filename)
{
	cv::FileStorage fscam(filename.c_str(),cv::FileStorage::READ);
	const cv::FileNode &node(fscam["Camera"]);
	
	cv::Mat pmat, tmat;
	cv::FileNode trmat = node["TransformationMatrix"];
	if(!trmat.empty()) trmat >> tmat;
	node["ProjectionMatrix"] >> pmat;
	
	int width, height;
	node["width"] >> width;
	node["height"] >> height;
	
	Eigen::Matrix<double,3,4> projmat;
	for(unsigned int l=0;l<3;l++)
	{
		for(unsigned int c=0;c<4;c++)
		{
			projmat(l,c) = pmat.at<double>(l,c);
		}
	}
	
	double u0, v0, ku, kv;

	ku = projmat(0,0);
	kv = projmat(1,1);
	u0 = projmat(0,2);
	v0 = projmat(1,2);

	camera::Intrinsics::Ptr intrinsics(new camera::PinHole(width,height,u0,v0,ku,kv));
	
	Eigen::Matrix<double,4,4> tramat = Eigen::Matrix4d::Identity();
	if(!trmat.empty())
	{
		for(unsigned int l=0;l<4;l++)
		{
			for(unsigned int c=0;c<4;c++)
			{
				tramat(l,c) = tmat.at<double>(l,c);
			}
		}
	}
	
	mathtools::affine::Frame<3>::Ptr frametr =
		mathtools::affine::Frame<3>::CreateFrame(
				tramat.block<3,1>(0,3),
				mathtools::vectorial::Basis<3>::CreateBasis(tramat.block<3,3>(0,0)))->getFrameInverse();

	camera::Extrinsics::Ptr extrinsics(new camera::Extrinsics(frametr));

	return camera::Camera::Ptr(new camera::Camera(intrinsics,extrinsics));
}
