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
 *  \file DisplayBoundary.h
 *  \brief Provides method to display 3d boundary
 *  \author Bastien Durix
 */

#ifndef _DISPLAYBOUNDARY_H_
#define _DISPLAYBOUNDARY_H_

#include "DisplayClass.h"
#include <boundary/DiscreteBoundary3.h>

/**
 *  \brief 3D displaying functions
 */
namespace display3d
{
	/**
	 *  \brief Displays a 3d wire boundary
	 *
	 *  \param disclass  display class containing the opengl context
	 *  \param disbnd    boundary to display
	 *  \param red       red color
	 *  \param green     green color
	 *  \param blue      blue color
	 *
	 *  \return index of the list containing the boundary
	 */
	unsigned int DisplayBoundary_Wired(DisplayClass &disclass, const boundary::DiscreteBoundary<3>::Ptr disbnd, float red = 0.0, float green = 0.0, float blue = 1.0);

	/**
	 *  \brief Displays a 3d filled boundary
	 *
	 *  \param disclass  display class containing the opengl context
	 *  \param disbnd    boundary to display
	 *  \param red       red color
	 *  \param green     green color
	 *  \param blue      blue color
	 *
	 *  \return index of the list containing the boundary
	 */
	unsigned int DisplayBoundary_Filled(DisplayClass &disclass, const boundary::DiscreteBoundary<3>::Ptr disbnd, float red = 0.0, float green = 0.0, float blue = 1.0);
};


#endif // _DISPLAYBOUNDARY_H_
