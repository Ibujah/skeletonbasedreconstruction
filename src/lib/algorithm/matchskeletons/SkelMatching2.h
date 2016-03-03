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
 *  \file SkelMatching2.h
 *  \brief Matches two skeletal branches
 *  \author Bastien Durix
 */

#ifndef _SKELMATCHING2_H_
#define _SKELMATCHING2_H_

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
				graph = 0
			};
			
			/**
			 *  \brief Matching method
			 */
			enum_methodmatch methodmatch;
			
			/**
			 *  \brief Number of matching points
			 */
			unsigned int nb_triang;
			
			/**
			 *  \brief Weight of the distance function between the lines (>0)
			 */
			double lambda;
			
			/**
			 *  \brief Default constructor
			 */
			OptionsMatch(enum_methodmatch methodmatch_ = graph, unsigned int nb_triang_ = 100, double lambda_ = 1.0) :
				methodmatch(methodmatch_), nb_triang(nb_triang_), lambda(lambda_) {}
		};
		
		/**
		 *  \brief Two branches matching algorithm
		 *
		 *  \param recbranch branch containing reconstruction data
		 *  \param projbr1   first projective branch
		 *  \param projbr2   second projective branch
		 *  \param options   algorithm options
		 */
		void BranchMatching(
				skeleton::ReconstructionBranch::Ptr recbranch,
				const skeleton::BranchContProjSkel::Ptr projbr1,
				const skeleton::BranchContProjSkel::Ptr projbr2,
				const OptionsMatch &options = OptionsMatch());
		
		/**
		 *  \brief Two skeletons matching algorithm
		 *
		 *  \param recskel   skeleton containing reconstruction data
		 *  \param projskel1 first projective skeleton
		 *  \param projskel2 second projective skeleton
		 *  \param options   algorithm options
		 */
		void ComposedMatching(
				skeleton::ReconstructionSkeleton::Ptr recskel,
				const skeleton::CompContProjSkel::Ptr projskel1,
				const skeleton::CompContProjSkel::Ptr projskel2,
				const OptionsMatch &options = OptionsMatch());
	}
}


#endif //_SKELMATCHING2_H_
