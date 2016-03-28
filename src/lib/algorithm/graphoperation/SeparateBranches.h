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
 *  \file SeparateBranches.h
 *  \brief Defines function to separate skeleton in different branches
 *  \author Bastien Durix
 */


#ifndef _SEPARATEBRANCHES_H_
#define _SEPARATEBRANCHES_H_

#include <skeleton/Skeletons.h>
#include <list>

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
		 *  \brief Separates skeleton into several branches
		 *
		 *  \param grskel  Skeleton to divide
		 *  
		 *  \return Composed skeleton corresponding to grskel
		 */
		typename skeleton::CompGraphSkel2d::Ptr SeparateBranches(const typename skeleton::GraphSkel2d::Ptr grskel);

		/**
		 *  \brief Separates skeleton into several branches
		 *
		 *  \param grskel  Skeleton to divide
		 *  
		 *  \return Composed skeleton corresponding to grskel
		 */
		typename skeleton::CompGraphSkel3d::Ptr SeparateBranches(const typename skeleton::GraphSkel3d::Ptr grskel);

		/**
		 *  \brief Separates skeleton into several branches
		 *
		 *  \param grskel  Skeleton to divide
		 *  
		 *  \return Composed skeleton corresponding to grskel
		 */
		typename skeleton::CompGraphProjSkel::Ptr SeparateBranches(const typename skeleton::GraphProjSkel::Ptr grskel);

		/**
		 *  \brief Gets a skeletal branch between two nodes
		 *
		 *  \param grskel  Skeleton in which get the branch
		 *  \param first   Branch first node 
		 *  \param last    Branch last node 
		 *
		 *  \return List of discrete branches between the nodes
		 */
		std::list<typename skeleton::BranchContSkel2d::Ptr> SeparateBranch(const typename skeleton::GraphSkel2d::Ptr grskel, unsigned int first, unsigned int last);

		/**
		 *  \brief Gets a skeletal branch between two nodes
		 *
		 *  \param grskel  Skeleton in which get the branch
		 *  \param first   Branch first node 
		 *  \param last    Branch last node 
		 *
		 *  \return List of discrete branches between the nodes
		 */
		std::list<typename skeleton::BranchGraphSkel3d::Ptr> SeparateBranch(const typename skeleton::GraphSkel3d::Ptr grskel, unsigned int first, unsigned int last);

		/**
		 *  \brief Gets a skeletal branch between two nodes
		 *
		 *  \param grskel  Skeleton in which get the branch
		 *  \param first   Branch first node 
		 *  \param last    Branch last node 
		 *
		 *  \return List of discrete branches between the nodes
		 */
		std::list<typename skeleton::BranchGraphProjSkel::Ptr> SeparateBranch(const typename skeleton::GraphProjSkel::Ptr grskel, unsigned int first, unsigned int last);
	}
}

#endif //_SEPARATEBRANCHES_H_
