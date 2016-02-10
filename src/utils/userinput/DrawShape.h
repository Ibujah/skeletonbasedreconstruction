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
 *  \file  DrawShape.h
 *  \brief Defines function to quickly draw a shape
 *  \author Bastien Durix
 */

#ifndef _DRAWSHAPE_H_
#define _DRAWSHAPE_H_

#include <shape/DiscreteShape.h>
#include <mathtools/affine/Frame.h>

/**
 *  \brief User input functions
 */
namespace userinput
{
	/**
	 *  \brief Opens a window and draw a shape
	 *
	 *  \param width  width of the window
	 *  \param height height of the window
	 *  \param frame  frame of the shape
	 *
	 *  \return shared pointer to the shape
	 */
	shape::DiscreteShape<2>::Ptr DrawShape(unsigned int width, unsigned int height, const mathtools::affine::Frame<2>::Ptr frame = mathtools::affine::Frame<2>::CanonicFrame());
}

#endif
