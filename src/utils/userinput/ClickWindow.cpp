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
 *  \file ClickWindow.cpp
 *  \brief Click points in a window
 *  \author Bastien Durix
 */

#include "ClickWindow.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

struct DatImg
{
	std::vector<cv::Point> *vec_pt_cv;
	std::vector<mathtools::affine::Point<2> > *vec_pt;
	mathtools::affine::Frame<2>::Ptr frame;
};

void clickwinonMouse( int event, int xc, int yc, int, void* data )
{
	DatImg *datim = (DatImg*)data;
    if( event == cv::EVENT_LBUTTONDOWN )
    {
		mathtools::affine::Point<2> pt(xc,yc,datim->frame);
		datim->vec_pt->push_back(pt);
		datim->vec_pt_cv->push_back(cv::Point(xc,yc));
	}
}


std::vector<mathtools::affine::Point<2> > userinput::ClickPoints(cv::Mat &img, unsigned int nbpts, const mathtools::affine::Frame<2>::Ptr frame)
{
	std::vector<cv::Point> vec_pt_cv(nbpts);
	std::vector<mathtools::affine::Point<2> > vec_pt(nbpts);
	vec_pt_cv.resize(0);
	vec_pt.resize(0);

	DatImg datim;
	datim.vec_pt = &vec_pt;
	datim.vec_pt_cv= &vec_pt_cv;
	datim.frame = frame;

	cv::namedWindow("Click", CV_WINDOW_AUTOSIZE);
    cv::setMouseCallback("Click",clickwinonMouse,&datim);
	cv::imshow("Click",img);
	
	unsigned int nb_draw = 0;
	
	do
	{
		for(;nb_draw<vec_pt_cv.size();nb_draw++)
		{
			cv::circle(img,vec_pt_cv[nb_draw],10,cv::Scalar(0,255,0),-1);
		}
		cv::imshow("Click",img);
		cv::waitKey(10);
	}while(vec_pt.size() != nbpts);

	for(;nb_draw<vec_pt_cv.size();nb_draw++)
	{
		cv::circle(img,vec_pt_cv[nb_draw],10,cv::Scalar(0,255,0),-1);
	}

	cv::setMouseCallback("Click",NULL);
	cv::destroyWindow("Click");

	return vec_pt;
}

