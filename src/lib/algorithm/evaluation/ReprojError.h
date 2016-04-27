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
 *  \file ReprojError.h
 *  \brief Computes the reprojection error of a 3d reconstructed skeleton
 *  \author Bastien Durix
 */

#ifndef _REPROJERROR_H_
#define _REPROJERROR_H_

#include <skeleton/Skeletons.h>
#include <shape/DiscreteShape.h>
#include <camera/Camera.h>

/**
 *  \brief Lots of algorithms
 */
namespace algorithm
{
	/**
	 *  \brief Evaluation algorithms
	 */
	namespace evaluation
	{
		/**
		 *  \brief Computes Hausdorff distance between projection of a 3d skeleton branch and images used for its reconstruction
		 *
		 *  \param vecproj  set of projected 3d skeleton
		 *  \param vecshape set of shapes used for reconstruction
		 *
		 *  \return Hausdorff distance between projections of skeleton and shapes
		 */
		double HausDist(const std::vector<shape::DiscreteShape<2>::Ptr> vecproj, const std::vector<shape::DiscreteShape<2>::Ptr> &vecshape);

		/**
		 *  \brief Computes Hausdorff distance between projection of a 3d skeleton branch and images used for its reconstruction
		 *
		 *  \param contbr   continuous branch to evaluate
		 *  \param vecshape set of shapes used for reconstruction
		 *  \param veccam   set of camera associated to shapes
		 *
		 *  \return Hausdorff distance between projections of skeleton and shapes
		 */
		double HausDist(const skeleton::BranchContSkel3d::Ptr contbr, const std::vector<shape::DiscreteShape<2>::Ptr> &vecshape, const std::vector<camera::Camera::Ptr> &veccam);

		/**
		 *  \brief Computes Hausdorff distance between projection of a 3d skeleton and images used for its reconstruction
		 *
		 *  \param contskl  continuous skeleton to evaluate
		 *  \param vecshape set of shapes used for reconstruction
		 *  \param veccam   set of camera associated to shapes
		 *
		 *  \return Hausdorff distance between projections of skeleton and shapes
		 */
		double HausDist(const skeleton::CompContSkel3d::Ptr contskl, const std::vector<shape::DiscreteShape<2>::Ptr> &vecshape, const std::vector<camera::Camera::Ptr> &veccam);
	}
}

#endif //_REPROJERROR_H_
