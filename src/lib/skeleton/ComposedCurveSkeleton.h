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
 *  \file ComposedCurveSkeleton.h
 *  \brief Defines composed curve skeleton
 *  \author Bastien Durix
 */

#ifndef _COMPOSEDCURVESKELETON_H_
#define _COMPOSEDCURVESKELETON_H_

#include <memory>
#include <boost/graph/adjacency_list.hpp>

/**
 *  \brief Skeleton representations
 */
namespace skeleton
{
	/**
	 *  \brief Describes composed curve skeleton
	 *
	 *  \tparam BranchType Branch class composing the skeleton
	 */
	template<typename BranchType>
	class ComposedCurveSkeleton
	{
		public:
			/**
			 *  \brief Composed curve skeleton shared pointer
			 */
			using Ptr = std::shared_ptr<ComposedCurveSkeleton<BranchType> >;

		protected:
			/**
			 *  \brief Edge property
			 */
			struct EdgeProperty 
			{
				/**
				 *  \brief Edge index
				 */
				unsigned int index;

				/**
 				 *  \brief Branch associated to the edge
 				 */
				typename BranchType::Ptr branch;
			};
			
			/**
			 *  \brief Graph type
			 *
			 *  \details Creates a undirected graph with vertices using VertexProperty
			 */
			using GraphType = boost::adjacency_list<boost::listS,boost::listS,boost::directedS,boost::no_property>;

			/**
			 *  \brief Graph of the composed skeleton
			 */
			GraphType m_graph;

	};
}


#endif //_COMPOSEDCURVESKELETON_H_
