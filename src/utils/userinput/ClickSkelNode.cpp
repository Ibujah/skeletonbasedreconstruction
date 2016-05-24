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
 *  \file ClickSkelNode.cpp
 *  \brief Allows user to click nodes on skeleton
 *  \author Bastien Durix
 */

#include "ClickSkelNode.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

struct DatImg
{
	std::map<unsigned int, Eigen::Vector2d> m_nod;
	std::vector<unsigned int> clicknod;
	cv::Mat &img;
	cv::Scalar col;
	
	DatImg(cv::Mat &im) : img(im){};
};

void extclickonMouse( int event, int xc, int yc, int, void* data )
{
	DatImg *datim = (DatImg*)data;
    if( event == cv::EVENT_LBUTTONDOWN )
    {
		Eigen::Vector2d pt(xc,yc);
		unsigned int ind = 0;
		Eigen::Vector2d nodcoords;
		double dist_min=-1.0;
		for(std::map<unsigned int,Eigen::Vector2d>::iterator it = datim->m_nod.begin(); it != datim->m_nod.end(); it++)
		{
			double dist = (pt-it->second).squaredNorm();
			if(dist_min == -1.0 || dist < dist_min)
			{
				dist_min = dist;
				ind=it->first;
				nodcoords = it->second;
			}
		}
		datim->clicknod.push_back(ind);

		cv::circle(datim->img,cv::Point(nodcoords.x(),nodcoords.y()),2,datim->col,-1);
	}
}

template<typename Model>
std::vector<unsigned int> ClickSkelNodes_helper(cv::Mat &img, const typename skeleton::GraphCurveSkeleton<Model>::Ptr grskel, unsigned int nbnod, const mathtools::affine::Frame<2>::Ptr frame, const bool ext, const cv::Scalar &col)
{
	std::list<unsigned int> l_allnodes;
	grskel->getAllNodes(l_allnodes);
	
	DatImg datim(img);
	datim.col = col;
	
	for(std::list<unsigned int>::iterator it = l_allnodes.begin(); it != l_allnodes.end(); it++)
	{
		if(grskel->getNodeDegree(*it) != 2 && !ext)
			datim.m_nod.insert(std::pair<unsigned int,Eigen::Vector2d>(*it,grskel->template getNode<mathtools::affine::Point<2> >(*it).getCoords(frame)));
		if(grskel->getNodeDegree(*it) <= 1 && ext)
			datim.m_nod.insert(std::pair<unsigned int,Eigen::Vector2d>(*it,grskel->template getNode<mathtools::affine::Point<2> >(*it).getCoords(frame)));
	}
	
	
	datim.clicknod.reserve(nbnod);
	datim.clicknod.resize(0);
	
	cv::namedWindow("Click", CV_WINDOW_AUTOSIZE);
    cv::setMouseCallback("Click",extclickonMouse,&datim);
    cv::imshow("Click",img);
	
	while(datim.clicknod.size() != nbnod)
	{
		cv::imshow("Click",img);
		cv::waitKey(10);
	}

	cv::setMouseCallback("Click",NULL);
	cv::destroyWindow("Click");

	return datim.clicknod;
}

std::vector<unsigned int> userinput::ClickSkelNodes(cv::Mat &img, const skeleton::GraphSkel2d::Ptr grskel, unsigned int nbnod, const mathtools::affine::Frame<2>::Ptr frame, const bool ext, const cv::Scalar &col)
{
	return ClickSkelNodes_helper<skeleton::model::Classic<2> >(img,grskel,nbnod,frame,ext,col);
}

std::vector<unsigned int> userinput::ClickSkelNodes(cv::Mat &img, const skeleton::GraphProjSkel::Ptr grskel, unsigned int nbnod, const mathtools::affine::Frame<2>::Ptr frame, const bool ext, const cv::Scalar &col)
{
	return ClickSkelNodes_helper<skeleton::model::Projective>(img,grskel,nbnod,frame,ext,col);
}
