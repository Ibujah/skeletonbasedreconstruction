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
 *  \file DisplaySkeletonOCV.cpp
 *  \brief Displays skeleton with opencv
 *  \author Bastien Durix
 */

#include <opencv2/imgproc/imgproc.hpp>
#include "DisplaySkeletonOCV.h"

void displayopencv::DisplayGraphSkeleton(const skeleton::GraphSkel2d::Ptr grskel, cv::Mat &img, const mathtools::affine::Frame<2>::Ptr frame, const cv::Scalar &color)
{
	std::list<std::pair<unsigned int,unsigned int> > edges;
	grskel->getAllEdges(edges);
	for(std::list<std::pair<unsigned int,unsigned int> >::iterator it = edges.begin(); it != edges.end(); it++)
	{
		Eigen::Vector2d vec1 = grskel->getNode<mathtools::affine::Point<2> >(it->first).getCoords(frame);
		Eigen::Vector2d vec2 = grskel->getNode<mathtools::affine::Point<2> >(it->second).getCoords(frame);
		
		cv::Point pt1(vec1.x(),vec1.y());
		cv::Point pt2(vec2.x(),vec2.y());

		cv::line(img,pt1,pt2,color);
	}
}
