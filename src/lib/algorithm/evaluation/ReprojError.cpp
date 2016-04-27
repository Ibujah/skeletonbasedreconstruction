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
 *  \file ReprojError.cpp
 *  \brief Computes the reprojection error of a 3d reconstructed skeleton
 *  \author Bastien Durix
 */

#include "ReprojError.h"
#include <algorithm/projection/SkeletonProjection.h>
#include <algorithm/skinning/Filling.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

double algorithm::evaluation::HausDist(const std::vector<shape::DiscreteShape<2>::Ptr> vecproj, const std::vector<shape::DiscreteShape<2>::Ptr> &vecshape)
{
	double dist = 0.0;
	for(unsigned int i = 0; i < vecproj.size(); i++)
	{
		cv::Mat img;
		cv::Mat im_shape(vecshape[i]->getHeight(),vecshape[i]->getWidth(),CV_8U,&vecshape[i]->getContainer()[0]);
		cv::Mat im_proj(vecproj[i]->getHeight(),vecproj[i]->getWidth(),CV_8U,&vecproj[i]->getContainer()[0]);
		cv::absdiff(im_shape,im_proj,img);

		double dist_i = cv::sum(img)(0)/cv::sum(im_shape)(0);

		dist += dist_i;
	}
	
	return dist/(double)vecproj.size();
}

double algorithm::evaluation::HausDist(const skeleton::BranchContSkel3d::Ptr contbr, const std::vector<shape::DiscreteShape<2>::Ptr> &vecshape, const std::vector<camera::Camera::Ptr> &veccam)
{
	std::vector<shape::DiscreteShape<2>::Ptr> vecproj(veccam.size());
	
	for(unsigned int i = 0; i < veccam.size(); i++)
	{
		skeleton::BranchContProjSkel::Ptr projbr = algorithm::projection::SkeletonProjection(contbr,veccam[i]);
		vecproj[i] = shape::DiscreteShape<2>::Ptr(
				new shape::DiscreteShape<2>(
					veccam[i]->getIntrinsics()->getWidth(), 
					veccam[i]->getIntrinsics()->getHeight(), 
					veccam[i]->getIntrinsics()->getFrame()));
		algorithm::skinning::Filling(vecproj[i],projbr);
	}
	
	return HausDist(vecproj,vecshape);
}

double algorithm::evaluation::HausDist(const skeleton::CompContSkel3d::Ptr contskl, const std::vector<shape::DiscreteShape<2>::Ptr> &vecshape, const std::vector<camera::Camera::Ptr> &veccam)
{
	std::vector<shape::DiscreteShape<2>::Ptr> vecproj(veccam.size());
	
	for(unsigned int i = 0; i < veccam.size(); i++)
	{
		skeleton::CompContProjSkel::Ptr projskl = algorithm::projection::SkeletonProjection(contskl,veccam[i]);
		vecproj[i] = shape::DiscreteShape<2>::Ptr(
				new shape::DiscreteShape<2>(
					veccam[i]->getIntrinsics()->getWidth(), 
					veccam[i]->getIntrinsics()->getHeight(), 
					veccam[i]->getIntrinsics()->getFrame()));
		algorithm::skinning::Filling(vecproj[i],projskl);
	}
	
	return HausDist(vecproj,vecshape);
}
