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
 *  \brief Compute Bspline demo
 *  \author Bastien Durix
 */

#include <time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iomanip>

#include <mathtools/affine/Frame.h>
#include <mathtools/application/Bspline.h>

#include <userinput/ClickWindow.h>

#include <algorithm/fitbspline/ComputeNodeVector.h>
#include <algorithm/fitbspline/FitBspline.h>

#include <displayopencv/DisplayBsplineOCV.h>

int main()
{
	time_t start,end;
	double diff;
	
	cv::Mat img(480,640,CV_8UC3,cv::Scalar(0,0,0));
	
	mathtools::affine::Frame<2>::Ptr frame =
		mathtools::affine::Frame<2>::CreateFrame(
			Eigen::Vector2d(320.0/320.0, 240.0/320.0),
			Eigen::Vector2d(1.0/320.0, 0.0),
			Eigen::Vector2d(0.0, -1.0/320.0));
	
	std::vector<mathtools::affine::Point<2> > pt_approx = userinput::ClickPoints(img,10,frame);
	std::vector<Eigen::Vector2d> pt_coords(pt_approx.size());

	for(unsigned int i = 0; i < pt_approx.size(); i++)
	{
		pt_coords[i] = pt_approx[i].getCoords();
	}

	time(&start);
	Eigen::Matrix<double,1,Eigen::Dynamic> vec_approx_nod = algorithm::fitbspline::ComputeApproxNodeVec<2>(pt_coords);
	time(&end);
	diff = difftime(end,start);
	std::cout << "Approximative node vector computation: " << std::setprecision(2) << diff << "s." << std::endl;
	
	time(&start);
	Eigen::Matrix<double,1,Eigen::Dynamic> vec_nod = algorithm::fitbspline::ComputeNodeVec(vec_approx_nod,3,4);
	time(&end);
	diff = difftime(end,start);
	std::cout << "Node vector computation: " << std::setprecision(2) << diff << "s." << std::endl;
	
	time(&start);
	mathtools::application::Bspline<2> bspline = algorithm::fitbspline::FitBspline<2>(pt_coords,vec_approx_nod,vec_nod,3);
	time(&end);
	diff = difftime(end,start);
	std::cout << "Fitting bspline computation: " << std::setprecision(2) << diff << "s." << std::endl;
    
	displayopencv::DisplayBspline(bspline,img,frame,cv::Scalar(255,0,0));
	
	cv::imshow("Res",img);

	cv::imwrite("res.png",img);
	std::cout << "Saved image at res.png" << std::endl;

	return 0;
}



