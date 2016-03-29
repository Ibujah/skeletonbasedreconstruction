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
 *  \file ClickSkelNode.h
 *  \brief Allows user to click nodes on skeleton
 *  \author Bastien Durix
 */

#ifndef _CLICKSKELNODE_H_
#define _CLICKSKELNODE_H_

#include <opencv2/core/core.hpp>

#include <skeleton/Skeletons.h>
#include <mathtools/affine/Frame.h>
#include <mathtools/affine/Point.h>

/**
 *  \brief User input functions
 */
namespace userinput
{
	/**
	 *  \brief Click skeleton nodes on 2d graph skeleton
	 *
	 *  \param img     input image displaying the skeleton/output image displaying the clickes nodes
	 *  \param grskel  2d graph skeleton
	 *  \param nbnod   number of nodes to click
	 *  \param frame   frame of the image
	 *  \param col     color of the clickes nodes
	 *
	 *  \return id of clicked nodes
	 */
	std::vector<unsigned int> ClickSkelNodes(cv::Mat &img, const skeleton::GraphSkel2d::Ptr grskel, unsigned int nbnod, const mathtools::affine::Frame<2>::Ptr frame, const cv::Scalar &col = cv::Scalar(0,255,0));

	/**
	 *  \brief Click skeleton nodes on projective graph skeleton
	 *
	 *  \param img     input image displaying the skeleton/output image displaying the clickes nodes
	 *  \param grskel  projective graph skeleton
	 *  \param nbnod   number of nodes to click
	 *  \param frame   frame of the image
	 *  \param col     color of the clickes nodes
	 *
	 *  \return id of clicked nodes
	 */
	std::vector<unsigned int> ClickSkelNodes(cv::Mat &img, const skeleton::GraphProjSkel::Ptr grskel, unsigned int nbnod, const mathtools::affine::Frame<2>::Ptr frame, const cv::Scalar &col = cv::Scalar(0,255,0));
}

#endif //_CLICKSKELNODE_H_
