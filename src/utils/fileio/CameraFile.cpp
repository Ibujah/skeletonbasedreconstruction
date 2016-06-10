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

void fileio::WriteCamera(const camera::Camera::Ptr cam, const std::string &filename)
{
	cv::FileStorage filestor(filename,cv::FileStorage::WRITE);

	cv::Mat pmat(3,4,CV_64F);
	pmat.at<double>(0,0) = cam->getIntrinsics()->getFrame()->getBasis()->getMatrix()(0,0);
	pmat.at<double>(1,0) = cam->getIntrinsics()->getFrame()->getBasis()->getMatrix()(1,0);
	pmat.at<double>(2,0) = 0.0;

	pmat.at<double>(0,1) = cam->getIntrinsics()->getFrame()->getBasis()->getMatrix()(0,1);
	pmat.at<double>(1,1) = cam->getIntrinsics()->getFrame()->getBasis()->getMatrix()(1,1);
	pmat.at<double>(2,1) = 0.0;

	pmat.at<double>(0,2) = cam->getIntrinsics()->getFrame()->getOrigin().x();
	pmat.at<double>(1,2) = cam->getIntrinsics()->getFrame()->getOrigin().y();
	pmat.at<double>(2,2) = 1.0;

	pmat.at<double>(0,3) = 0.0;
	pmat.at<double>(1,3) = 0.0;
	pmat.at<double>(2,3) = 0.0;

	cv::Mat tmat = cv::Mat::eye(4,4,CV_64F);
	Eigen::Matrix3d mat3d = cam->getExtrinsics()->getFrame()->getFrameInverse()->getBasis()->getMatrix();
	Eigen::Vector3d ori3d = cam->getExtrinsics()->getFrame()->getFrameInverse()->getOrigin();
	for(unsigned int i = 0; i < 3; i++)
	{
		for(unsigned int j = 0; j < 3; j++)
		{
			tmat.at<double>(i,j) = mat3d(i,j);
		}
		tmat.at<double>(i,3) = ori3d(i);
	}
	
	filestor << "TransformationMatrix" << tmat;
	filestor << "ProjectionMatrix" << pmat;
	filestor << "width" << (int)cam->getIntrinsics()->getWidth();
	filestor << "height" << (int)cam->getIntrinsics()->getHeight();
}
