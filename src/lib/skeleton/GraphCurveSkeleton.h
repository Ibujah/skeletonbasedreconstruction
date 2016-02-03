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
 *  \file GraphCurveSkeleton.h
 *  \brief Defines graph curve skeleton
 *  \author Bastien Durix
 */

#ifndef _GRAPHCURVESKELETON_H_
#define _GRAPHCURVESKELETON_H_

#include <map>
#include <Eigen/Dense>
#include <boost/graph/adjacency_list.hpp>
#include "model/MetaModel.h"


/**
 *  \brief Skeleton representations
 */
namespace skeleton
{
	/**
	 *  \brief Describes graph curve skeleton
	 *
	 *  \tparam Model : Class giving a meaning to the skeleton (dimensions, geometric interpretation...)
	 */
	template<typename Model>
	class GraphCurveSkeleton
	{
		protected:
			/**
			 *  \brief All nodes in the graph curve skeleton
			 */
			std::map<unsigned int,Eigen::Matrix<double,model::meta<Model>::stordim,1> > m_node;
			
			/**
			 *  \brief Graph of the curve skeleton
			 */
			boost::adjacency_list<> m_graph;

	};
}


#endif //_GRAPHCURVESKELETON_H_

