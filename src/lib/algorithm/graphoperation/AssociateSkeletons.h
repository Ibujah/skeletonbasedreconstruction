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
 *  \file AssociateSkeletons.h
 *  \brief Defines algorithm to associate branches of multiple skeletons
 *  \author Bastien Durix
 */

#ifndef _ASSOCIATESKELETONS_H_
#define _ASSOCIATESKELETONS_H_

#include <skeleton/Skeletons.h>

/**
 *  \brief Lots of algorithms
 */
namespace algorithm
{
	/**
	 *  \brief Operations on graphs
	 */
	namespace graphoperation
	{
		/**
		 *  \brief Associates skeletons using extremities informations (genius-0)
		 *
		 *  \param vec_skel    input projective skeletons
		 *  \param assoc_ext   input extremities (first indice : skeleton number)
		 *
		 *  \return output reconstruction skeleton
		 */
		skeleton::ReconstructionSkeleton::Ptr TopoMatch(const std::vector<skeleton::GraphProjSkel::Ptr > &vec_skel, const std::vector<std::vector<unsigned int> > &assoc_ext);
	}
}

#endif //_ASSOCIATESKELETONS_H_
