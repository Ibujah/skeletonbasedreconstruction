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
 *  \brief Draw shape test
 *  \author Bastien Durix
 */

#include <time.h>
#include <iostream>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <shape/DiscreteShape.h>
#include <boundary/DiscreteBoundary.h>
#include <skeleton/Skeletons.h>

#include <userinput/DrawShape.h>

#include <algorithm/extractboundary/MarchingSquares.h>
#include <algorithm/skeletonization/VoronoiSkeleton2D.h>

#include <displayopencv/DisplayShapeOCV.h>
#include <displayopencv/DisplayBoundaryOCV.h>
#include <displayopencv/DisplaySkeletonOCV.h>

int main()
{
	time_t start,end;
	double diff;

	shape::DiscreteShape<2>::Ptr dissh = userinput::DrawShape(640,480);
	
	time(&start);
	boundary::DiscreteBoundary<2>::Ptr disbnd = algorithm::extractboundary::MarchingSquare(dissh,4);
	time(&end);
	diff = difftime(end,start);
	std::cout << "Boundary computation: " << std::setprecision(2) << diff << "s." << std::endl;
	
	time(&start);
	skeleton::GraphSkel2d::Ptr grskel = algorithm::skeletonization::VoronoiSkeleton2d(disbnd);
	time(&end);
	diff = difftime(end,start);
	std::cout << "Skeleton computation: " << std::setprecision(2) << diff << "s." << std::endl;
	
	cv::Mat image(480,640,CV_8UC3,cv::Scalar(0,0,0));

	time(&start);
	displayopencv::DisplayDiscreteShape(dissh,image,dissh->getFrame(),cv::Scalar(255,255,255));
	time(&end);
	diff = difftime(end,start);
	std::cout << "Shape display: " << std::setprecision(2) << diff << "s." << std::endl;
	
	time(&start);
	displayopencv::DisplayDiscreteBoundary(disbnd,image,dissh->getFrame(),cv::Scalar(0,0,255));
	time(&end);
	diff = difftime(end,start);
	std::cout << "Boundary display: " << std::setprecision(2) << diff << "s." << std::endl;
	
	time(&start);
	displayopencv::DisplayGraphSkeleton(grskel,image,dissh->getFrame(),cv::Scalar(255,0,0));
	time(&end);
	diff = difftime(end,start);
	std::cout << "Skeleton display: " << std::setprecision(2) << diff << "s." << std::endl;
	
	cv::imwrite("res.png",image);
	std::cout << "Saved image at res.png" << std::endl;

	return 0;
}



