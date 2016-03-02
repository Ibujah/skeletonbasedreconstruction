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
 *  \file Graph2Bspline.h
 *  \brief Converts graph skeleton to bspline continuous skeleton
 *  \author Bastien Durix
 */

#ifndef _GRAPH2BSPLINE_H_
#define _GRAPH2BSPLINE_H_

#include <skeleton/Skeletons.h>

/**
 *  \brief Lots of algorithms
 */
namespace algorithm
{
	/*
	 *  \brief Defines tools to compute Bsplines
	 */
	namespace fitbspline
	{
		/**
		 *  \brief Converts a graph branch to a bspline continuous branch
		 *
		 *  \param grbr     graph branch to convert
		 *  \param degree   bspline degree
		 *  \param propctrl control points proportion to keep
		 *
		 *  \return pointer to bspline continuous branch
		 */
		skeleton::BranchContSkel2d::Ptr Graph2Bspline(const skeleton::BranchGraphSkel2d::Ptr grskel, unsigned int degree = 3, double propctrl = 0.1);
		
		/**
		 *  \brief Converts a graph skeleton to a bspline continuous skeleton
		 *
		 *  \param grskel   graph skeleton to convert
		 *  \param degree   bspline degree
		 *  \param propctrl control points proportion to keep
		 *
		 *  \return pointer to bspline continuous skeleton
		 */
		skeleton::CompContSkel2d::Ptr Graph2Bspline(const skeleton::CompGraphSkel2d::Ptr grskel, unsigned int degree = 3, double propctrl = 0.1);
		

		/**
		 *  \brief Converts a graph branch to a bspline continuous branch
		 *
		 *  \param grbr     graph branch to convert
		 *  \param degree   bspline degree
		 *  \param propctrl control points proportion to keep
		 *
		 *  \return pointer to bspline continuous branch
		 */
		skeleton::BranchContSkel3d::Ptr Graph2Bspline(const skeleton::BranchGraphSkel3d::Ptr grskel, unsigned int degree = 3, double propctrl = 0.1);
		
		/**
		 *  \brief Converts a graph skeleton to a bspline continuous skeleton
		 *
		 *  \param grskel   graph skeleton to convert
		 *  \param degree   bspline degree
		 *  \param propctrl control points proportion to keep
		 *
		 *  \return pointer to bspline continuous skeleton
		 */
		skeleton::CompContSkel3d::Ptr Graph2Bspline(const skeleton::CompGraphSkel3d::Ptr grskel, unsigned int degree = 3, double propctrl = 0.1);

		
		/**
		 *  \brief Converts a graph branch to a bspline continuous branch
		 *
		 *  \param grbr     graph branch to convert
		 *  \param degree   bspline degree
		 *  \param propctrl control points proportion to keep
		 *
		 *  \return pointer to bspline continuous branch
		 */
		skeleton::BranchContProjSkel::Ptr Graph2Bspline(const skeleton::BranchGraphProjSkel::Ptr grskel, unsigned int degree = 3, double propctrl = 0.1);
		
		/**
		 *  \brief Converts a graph skeleton to a bspline continuous skeleton
		 *
		 *  \param grskel   graph skeleton to convert
		 *  \param degree   bspline degree
		 *  \param propctrl control points proportion to keep
		 *
		 *  \return pointer to bspline continuous skeleton
		 */
		skeleton::CompContProjSkel::Ptr Graph2Bspline(const skeleton::CompGraphProjSkel::Ptr grskel, unsigned int degree = 3, double propctrl = 0.1);
	}
}

#endif //_GRAPH2BSPLINE_H_
