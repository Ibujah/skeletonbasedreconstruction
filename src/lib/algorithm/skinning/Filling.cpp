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
 *  \file Filling.h
 *  \brief Computes a discrete shape associated to a continuous skeleton
 *  \author Bastien Durix
 */

#include "Filling.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void algorithm::skinning::Filling(shape::DiscreteShape<2>::Ptr shape, const skeleton::BranchContSkel2d::Ptr contbr, const OptionsFilling &options)
{
	cv::Mat im_shape(shape->getHeight(),shape->getWidth(),CV_8U,&shape->getContainer()[0]);
	
	for(unsigned int i = 0; i < options.nbcer; i++)
	{
		double t = (double)i/(double)(options.nbcer-1);
		mathtools::geometry::euclidian::HyperSphere<2> sph = contbr->getNode<mathtools::geometry::euclidian::HyperSphere<2> >(t);
		
    	cv::circle(im_shape,cv::Point2i(sph.getCenter().getCoords(shape->getFrame()).x(),sph.getCenter().getCoords(shape->getFrame()).y()),sph.getRadius(),255,-1);
	}
}

void algorithm::skinning::Filling(shape::DiscreteShape<2>::Ptr shape, const skeleton::CompContSkel2d::Ptr contskl, const OptionsFilling &options)
{
	std::vector<unsigned int> edge(0);
	contskl->getAllEdges(edge);
	
	for(unsigned int i=0;i<edge.size();i++)
	{
		std::pair<unsigned int,unsigned int> ext = contskl->getExtremities(edge[i]);
		
		Filling(shape,contskl->getBranch(ext.first,ext.second),options);
	}
}

void algorithm::skinning::Filling(shape::DiscreteShape<2>::Ptr shape, const skeleton::BranchContProjSkel::Ptr contbr, const OptionsFilling &options)
{
	cv::Mat im_shape(shape->getHeight(),shape->getWidth(),CV_8U,&shape->getContainer()[0]);
	
	for(unsigned int i = 0; i < options.nbcer; i++)
	{
		double t = (double)i/(double)(options.nbcer-1);
		mathtools::geometry::euclidian::HyperEllipse<2> ell = contbr->getNode<mathtools::geometry::euclidian::HyperEllipse<2> >(t);
		
		cv::RotatedRect rect(cv::Point2f(ell.getCenter().getCoords().x(),ell.getCenter().getCoords().y()),
							 cv::Size2f(1.0/ell.getAxes().block<2,1>(0,0).norm(),1.0/ell.getAxes().block<2,1>(0,1).norm()),
							 atan2(ell.getAxes()(1,0),ell.getAxes()(0,0)));

		cv::ellipse(im_shape,rect,255,-1);
	}
}

void algorithm::skinning::Filling(shape::DiscreteShape<2>::Ptr shape, const skeleton::CompContProjSkel::Ptr contskl, const OptionsFilling &options)
{
	std::vector<unsigned int> edge(0);
	contskl->getAllEdges(edge);
	
	for(unsigned int i=0;i<edge.size();i++)
	{
		std::pair<unsigned int,unsigned int> ext = contskl->getExtremities(edge[i]);
		
		Filling(shape,contskl->getBranch(ext.first,ext.second),options);
	}
}
