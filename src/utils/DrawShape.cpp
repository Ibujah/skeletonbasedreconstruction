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
 *  \file  DrawShape.cpp
 *  \brief Defines function to quickly draw a shape
 *  \author Bastien Durix
 */

#include "DrawShape.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace utils;

/**
 *  \brief Defines drawing image data
 */
struct DatImg
{
	/**
	 *  \brief Pointer to the image to draw
	 */
	cv::Mat *im_shape;

	/**
	 *  \brief States if the mouse is clicked
	 */
	bool lclick;

	/**
	 *  \brief Circle radius
	 */
	unsigned int radius;
};

/**
 *  \brief Callback function to draw circles on image
 */
void shapeacqonMouse( int event, int xc, int yc, int, void* data )
{
	DatImg *datim = (DatImg*)data;
    if( event == cv::EVENT_LBUTTONDOWN )
        datim->lclick = true;
    if( event == cv::EVENT_LBUTTONUP )
        datim->lclick = false;

    if(datim->lclick)
    {
    	cv::circle(*(datim->im_shape),cv::Point2i(xc,yc),datim->radius,255,-1);
    }
}

shape::DiscreteShape<2>::Ptr DrawShape(unsigned int width, unsigned int height, const mathtools::affine::Frame<2>::Ptr frame)
{
	shape::DiscreteShape<2>::Ptr disshape(new shape::DiscreteShape<2>(width, height, frame));
	
	cv::Mat im_shape(height,width,CV_8U,&disshape->getContainer());

	DatImg datim;
	datim.lclick=false;
	datim.im_shape = &im_shape;
	datim.radius = 40;
	
	std::string winname = "Shape acquisition";

	cv::namedWindow(winname, CV_WINDOW_AUTOSIZE);
    cv::setMouseCallback(winname,shapeacqonMouse,&datim);
    cv::imshow(winname,im_shape);

	bool loop = true;
	std::cout << "Radius: " << datim.radius << "px" << std::endl;
	std::cout << "Press a/A to decrease radius" << std::endl;
	std::cout << "Press z/Z to increase radius" << std::endl;
	std::cout << "Press any key to finish" << std::endl;
    while(loop)
    {
        cv::imshow(winname,im_shape);

        char key = cv::waitKey(10);
        if(key=='a' || key=='A')
		{
			datim.radius -= 10;
			if(datim.radius < 10) datim.radius = 10;
			else
			{
				std::cout << "radius: " << datim.radius << "px" << std::endl;
				std::cout << "press a/a to decrease radius" << std::endl;
				std::cout << "press z/z to increase radius" << std::endl;
				std::cout << "press any key to finish" << std::endl;
			}
		}
		else if(key=='z' || key=='Z')
		{
			datim.radius += 10;
			if(datim.radius > 200) datim.radius = 200;
			else
			{
				std::cout << "radius: " << datim.radius << "px" << std::endl;
				std::cout << "press a/a to decrease radius" << std::endl;
				std::cout << "press z/z to increase radius" << std::endl;
				std::cout << "press any key to finish" << std::endl;
			}
		}
        else if(key!=-1)
            loop=false;
    }

	cv::setMouseCallback(winname,NULL);
	cv::destroyWindow(winname);
	
	return disshape;
}
