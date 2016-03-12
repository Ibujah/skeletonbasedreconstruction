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
 *  \file ContinuousSkinning.h
 *  \brief Computes a discrete 3D boundary associatd to a skeleton
 *  \author Bastien Durix
 */

#ifndef _CONTINUOUSSKINNING_H_
#define _CONTINUOUSSKINNING_H_

#include <skeleton/Skeletons.h>
#include <boundary/DiscreteBoundary3.h>

/**
 *  \brief Lots of algorithms
 */
namespace algorithm
{
	/**
	 *  \brief Skeleton skinning operation
	 */
	namespace skinning
	{
		/**
		 *  \brief Skinning options structure
		 */
		struct OptionsContSkinning
		{
			/**
 			 *  \brief Number of circles on each branch
 			 */
			unsigned int nbcer;

			/**
 			 *  \brief Number of points on each circle
 			 */
			unsigned int nbpt;

			/**
			 *  \brief Default constructor
			 */
			OptionsContSkinning(unsigned int nbcer_ = 20, unsigned int nbpt_ = 16) :
				nbcer(nbcer_), nbpt(nbpt_) {}
		};
		
		/**
 		 *  \brief Computes continuous skinning on a branch
 		 *
 		 *  \param contbr  continuous branch to skin
 		 *  \param options skinning options
 		 *
 		 *  \returns boundary corresponding to the branch
 		 */
		boundary::DiscreteBoundary<3>::Ptr ContinuousSkinning(const skeleton::BranchContSkel3d::Ptr contbr, const OptionsContSkinning &options = OptionsContSkinning());

		/**
 		 *  \brief Computes continuous skinning on a skeleton
 		 *
 		 *  \param contskl continuous skeleton to skin
 		 *  \param options skinning options
 		 *
 		 *  \returns boundary corresponding to the skeleton
 		 */
		boundary::DiscreteBoundary<3>::Ptr ContinuousSkinning(const skeleton::CompContSkel3d::Ptr contskl, const OptionsContSkinning &options = OptionsContSkinning());
	}
}

#endif //_CONTINUOUSSKINNING_H_
