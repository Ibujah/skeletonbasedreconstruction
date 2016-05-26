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
 *  \file SkelMatching.h
 *  \brief Matches multiple skeletal branches
 *  \author Bastien Durix
 */

#ifndef _SKELMATCHING_H_
#define _SKELMATCHING_H_

#include <skeleton/Skeletons.h>

/**
 *  \brief Lots of algorithms
 */
namespace algorithm
{
	/**
	 *  \brief Matching and triangulation skeleton algorithms
	 */
	namespace matchskeletons
	{
		/**
		 *  \brief Match options structure
		 */
		struct OptionsMatch
		{
			/**
			 *  \brief Several matching methods
			 */
			enum enum_methodmatch
			{
				ode = 0
			};
			
			/**
			 *  \brief Matching method
			 */
			enum_methodmatch methodmatch;
			
			/**
			 *  \brief Min weight of the distance function between the lines (>0)
			 */
			double lambdamin;
			
			/**
			 *  \brief Max weight of the distance function between the lines (>0)
			 */
			double lambdamax;
			
			/**
			 *  \brief Discretization step between min and max lambda
			 */
			double lambdastep;
			
			/**
			 *  \brief Time between two step for ODE
			 */
			double deltat;
			
			/**
			 *  \brief Default constructor
			 */
			OptionsMatch(enum_methodmatch methodmatch_ = ode, double lambdamin_ = 0.1, double lambdamax_ = 10.0, double lambdastep_ = 0.1, double deltat_ = 0.01) :
				methodmatch(methodmatch_), lambdamin(lambdamin_), lambdamax(lambdamax_), lambdastep(lambdastep_), deltat(deltat_) {}
		};
		
		/**
		 *  \brief Multiple branches matching algorithm
		 *
		 *  \param recbranch branch containing reconstruction data
		 *  \param projbr    projective branches
		 *  \param options   algorithm options
		 */
		void BranchMatching(
				skeleton::ReconstructionBranch::Ptr recbranch,
				const std::vector<skeleton::BranchContProjSkel::Ptr> projbr,
				const OptionsMatch &options = OptionsMatch());
		
		/**
		 *  \brief Two skeletons matching algorithm
		 *
		 *  \param recskel  skeleton containing reconstruction data
		 *  \param projskel projective skeletons
		 *  \param options  algorithm options
		 */
		void ComposedMatching(
				skeleton::ReconstructionSkeleton::Ptr recskel,
				const std::vector<skeleton::CompContProjSkel::Ptr> projskel,
				const OptionsMatch &options = OptionsMatch());
	}
}


#endif //_SKELMATCHING_H_
