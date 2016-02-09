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
 *  \file VoronoiSkeleton2D.h
 *  \brief Defines functions to compute 2d skeleton with voronoi algorithm
 *  \author Bastien Durix
 */

#ifndef _VORONOISKELETON2D_H_
#define _VORONOISKELETON2D_H_

#include <skeleton/Skeletons.h>
#include <boundary/DiscreteBoundary.h>

/**
 *  \brief Lots of algorithms
 */
namespace algorithm
{
	/**
	 *  \brief skeletonization algorithms
	 */
	namespace skeletonization
	{
		/**
		 *  \brief 2D classical skeletonization
		 *
		 *  \param disbnd discrete boundary of the shape
		 *
		 *  \return pointer to the computed 2d graph skeleton
		 */
		skeleton::GraphSkel2d::Ptr VoronoiSkeleton2d(const boundary::DiscreteBoundary<2>::Ptr disbnd);
	}
}

#endif //_VORONOISKELETON2D_H_
