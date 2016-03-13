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
 *  \file SkelTriangulation.h
 *  \brief Triangulates skeleton from projective skeletons
 *  \author Bastien Durix
 */

#ifndef _SKELETONTRIANGULATION_H_
#define _SKELETONTRIANGULATION_H_

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
		 *  \brief Triangulation options structure
		 */
		struct OptionsTriang 
		{
			/**
			 *  \brief B-Spline degree
			 */
			unsigned int degree;
			
			/**
			 *  \brief Default constructor
			 */
			OptionsTriang(unsigned int degree_ = 3) :
				degree(degree_) {};
		};

		/**
		 *  \brief Two branches matching algorithm
		 *
		 *  \param recbranch branch containing reconstruction data
		 *  \param projbr    projective branch
		 *  \param model     future branch model
		 *  \param options   algorithm options
		 *
		 *  \return triangulated branch
		 */
		skeleton::BranchContSkel3d::Ptr BranchTriangulation(
				skeleton::ReconstructionBranch::Ptr recbranch,
				const std::vector<skeleton::BranchContProjSkel::Ptr> projbr,
				const skeleton::model::Classic<3>::Ptr model = skeleton::model::Classic<3>::Ptr(new skeleton::model::Classic<3>()),
				const OptionsTriang &options = OptionsTriang());
		
		/**
		 *  \brief Two skeletons matching algorithm
		 *
		 *  \param recskel   skeleton containing reconstruction data
		 *  \param projskel  projective skeleton
		 *  \param model     future skeleton model
		 *  \param options   algorithm options
		 *
		 *  \return triangulated skeleton
		 */
		skeleton::CompContSkel3d::Ptr ComposedTriangulation(
				skeleton::ReconstructionSkeleton::Ptr recskel,
				const std::vector<skeleton::CompContProjSkel::Ptr> projskel,
				const skeleton::model::Classic<3>::Ptr model = skeleton::model::Classic<3>::Ptr(new skeleton::model::Classic<3>()),
				const OptionsTriang &options = OptionsTriang());
	}
}

#endif //_SKELETONTRIANGULATION_H_
