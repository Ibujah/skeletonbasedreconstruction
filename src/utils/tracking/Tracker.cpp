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
 *  \file Tracker.cpp
 *  \brief Provides methods to acquire calibrated images
 *  \author Bastien Durix
 */


#include "Tracker.h"
#include <iostream>

tracking::Tracker::Tracker() : m_arhandle(NULL), m_arparamlt(NULL)
{
}

tracking::Tracker::~Tracker()
{
	clean();
}

void tracking::Tracker::init(const camera::Camera::Ptr cam)
{
	clean();
	ARParam arparam;
	arparam.xsize = cam->getIntrinsics()->getWidth();
	arparam.ysize = cam->getIntrinsics()->getHeight();

	arparam.mat[0][0] = cam->getIntrinsics()->getFrame()->getBasis()->getMatrix()(0,0);
	arparam.mat[0][1] = cam->getIntrinsics()->getFrame()->getBasis()->getMatrix()(0,1);
	arparam.mat[0][2] = cam->getIntrinsics()->getFrame()->getOrigin().x();
	arparam.mat[0][3] = 0.0;

	arparam.mat[1][0] = cam->getIntrinsics()->getFrame()->getBasis()->getMatrix()(1,0);
	arparam.mat[1][1] = cam->getIntrinsics()->getFrame()->getBasis()->getMatrix()(1,1);
	arparam.mat[1][2] = cam->getIntrinsics()->getFrame()->getOrigin().y();
	arparam.mat[1][3] = 0.0;

	arparam.mat[2][0] = 0.0; 
	arparam.mat[2][1] = 0.0; 
	arparam.mat[2][2] = 1.0;
	arparam.mat[2][3] = 0.0;

	arparam.dist_factor[0] = 0.0;
	arparam.dist_factor[1] = 0.0;
	arparam.dist_factor[2] = 0.0;
	arparam.dist_factor[3] = 0.0;
	arparam.dist_factor[4] = 675;
	arparam.dist_factor[5] = 673;
	arparam.dist_factor[6] = 331;
	arparam.dist_factor[7] = 233;
	arparam.dist_factor[8] = 1;
    arparam.dist_function_version = 4;

	arParamDisp(&arparam);

	m_arparamlt = arParamLTCreate(&arparam,AR_PARAM_LT_DEFAULT_OFFSET);
	
	m_arhandle = arCreateHandle(m_arparamlt);
	
	arSetPixelFormat(m_arhandle,AR_PIXEL_FORMAT_BGR);
	arSetPatternDetectionMode(m_arhandle,AR_MATRIX_CODE_DETECTION);
	arSetMatrixCodeType(m_arhandle,AR_MATRIX_CODE_3x3);
	arSetDebugMode(m_arhandle,AR_DEBUG_DISABLE);
}

void tracking::Tracker::clean()
{
	if(m_arhandle)
		arDeleteHandle(m_arhandle);
	m_arhandle = NULL;

	if(m_arparamlt)
    	arParamLTFree(&m_arparamlt);
	m_arparamlt = NULL;
}

void tracking::Tracker::detect(cv::Mat &img)
{
	ARUint8 *dataPtr = img.ptr();

	arDetectMarker(m_arhandle,dataPtr);
	
	if(m_arhandle->marker_num)
	{
		std::cout << m_arhandle->marker_num << std::endl;
	}
	//cv::Mat bwimg(img.rows,img.cols,CV_8UC1,m_arhandle->labelInfo.bwImage);
	//bwimg.copyTo(img);
}
