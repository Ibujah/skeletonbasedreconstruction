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
 *  \file SkeletonProjection.h
 *  \brief Computes the 2d projection of a 3D skeleton
 *  \author Bastien Durix
 */

#ifndef _SKELETONPROJECTION_H_
#define _SKELETONPROJECTION_H_

#include <skeleton/Skeletons.h>
#include <camera/Camera.h>

/**
 *  \brief Lots of algorithms
 */
namespace algorithm
{
	/**
	 *  \brief Skeleton projection
	 */
	namespace projection 
	{
		/**
 		 *  \brief Associates a projective branch to a 3d skeletal branch
 		 *
 		 *  \param contbr  continuous branch to project
 		 *  \param cam     camera used to project
 		 *
 		 *  \return projective branch associated to contbr
 		 */
		skeleton::BranchContProjSkel::Ptr SkeletonProjection(const skeleton::BranchContSkel3d::Ptr contbr, const camera::Camera::Ptr camera);
		
		/**
 		 *  \brief Associates a projective skeleton to a 3d skeleton
 		 *
 		 *  \param contskl continuous skeleton to project
 		 *  \param cam     camera used to project
 		 *
 		 *  \return projective branch associated to contskl
 		 */
		skeleton::CompContProjSkel::Ptr SkeletonProjection(const skeleton::CompContSkel3d::Ptr contskl, const camera::Camera::Ptr camera);
	}
}

#endif //_SKELETONPROJECTION_H_
