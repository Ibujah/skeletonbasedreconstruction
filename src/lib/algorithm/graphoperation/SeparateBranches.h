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
		 *  \brief Gets the nodes indices between two nodes
		 *
		 *  \param grskel  Skeleton in which get the branch
		 *  \param first   Branch first node 
		 *  \param last    Branch last node 
		 *
		 *  \return List of node indices between the nodes
		 */
		std::list<std::vector<unsigned int> > GetNodes(const typename skeleton::GraphSkel2d::Ptr grskel, unsigned int first, unsigned int last);

		/**
		 *  \brief Gets the nodes indices between two nodes
		 *
		 *  \param grskel  Skeleton in which get the branch
		 *  \param first   Branch first node 
		 *  \param last    Branch last node 
		 *
		 *  \return List of node indices between the nodes
		 */
		std::list<std::vector<unsigned int> > GetNodes(const typename skeleton::GraphSkel3d::Ptr grskel, unsigned int first, unsigned int last);

		/**
		 *  \brief Gets the nodes indices between two nodes
		 *
		 *  \param grskel  Skeleton in which get the branch
		 *  \param first   Branch first node 
		 *  \param last    Branch last node 
		 *
		 *  \return List of node indices between the nodes
		 */
		std::list<std::vector<unsigned int> > GetNodes(const typename skeleton::GraphProjSkel::Ptr grskel, unsigned int first, unsigned int last);

		/**
		 *  \brief Gets the nodes indices between two nodes
		 *
		 *  \param grskel  Composed skeleton in which get the branch
		 *  \param first   Branch first node 
		 *  \param last    Branch last node 
		 *
		 *  \return List of node indices between the nodes
		 */
		std::list<std::vector<unsigned int> > GetNodes(const typename skeleton::ReconstructionSkeleton::Ptr grskel, unsigned int first, unsigned int last);

		/**
		 *  \brief Gets a skeletal branch between two nodes
		 *
		 *  \param grskel  Skeleton in which get the branch
		 *  \param first   Branch first node 
		 *  \param last    Branch last node 
		 *
		 *  \return List of discrete branches between the nodes
		 */
		std::list<typename skeleton::BranchGraphSkel2d::Ptr> GetBranch(const typename skeleton::GraphSkel2d::Ptr grskel, unsigned int first, unsigned int last);

		/**
		 *  \brief Gets a skeletal branch between two nodes
		 *
		 *  \param grskel  Skeleton in which get the branch
		 *  \param first   Branch first node 
		 *  \param last    Branch last node 
		 *
		 *  \return List of discrete branches between the nodes
		 */
		std::list<typename skeleton::BranchGraphSkel3d::Ptr> GetBranch(const typename skeleton::GraphSkel3d::Ptr grskel, unsigned int first, unsigned int last);

		/**
		 *  \brief Gets a skeletal branch between two nodes
		 *
		 *  \param grskel  Skeleton in which get the branch
		 *  \param first   Branch first node 
		 *  \param last    Branch last node 
		 *
		 *  \return List of discrete branches between the nodes
		 */
		std::list<typename skeleton::BranchGraphProjSkel::Ptr> GetBranch(const typename skeleton::GraphProjSkel::Ptr grskel, unsigned int first, unsigned int last);

		/**
		 *  \brief Get composed skeletons from graph skeleton and reconstruction skeleton
		 *  \details Skeletons have to have no cycles
		 *
		 *  \param recskel     Reconstruction skeleton
		 *  \param vec_prskel  Graph projective skeletons associated to reconstruction skeleton
		 *
		 *  \return Computed composed skeletons
		 *
		 *  \throws std::logic_error if a skeleton contains a cycle, or if a branch does not exist
		 */
		std::vector<typename skeleton::CompGraphProjSkel::Ptr> GetComposed(const typename skeleton::ReconstructionSkeleton::Ptr recskel, const std::vector<skeleton::GraphProjSkel::Ptr> &vec_prskel);

		/**
		 *  \brief Delete branch from an extremity
		 *
		 *  \param grskel  Skeleton in which delete the branch
		 *  \param ext     Extremity from which start
		 */
		void DeleteExtremity(skeleton::GraphSkel2d::Ptr grskel, unsigned int ext);

		/**
		 *  \brief Delete branch from an extremity
		 *
		 *  \param grskel  Skeleton in which delete the branch
		 *  \param ext     Extremity from which start
		 */
		void DeleteExtremity(skeleton::GraphSkel3d::Ptr grskel, unsigned int ext);

		/**
		 *  \brief Delete branch from an extremity
		 *
		 *  \param grskel  Skeleton in which delete the branch
		 *  \param ext     Extremity from which start
		 */
		void DeleteExtremity(skeleton::GraphProjSkel::Ptr grskel, unsigned int ext);
	}
}

#endif //_SEPARATEBRANCHES_H_
