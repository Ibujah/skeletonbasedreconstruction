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

#include <memory>
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
			 *  \brief Graph curve skeleton shared pointer
			 */
			using Ptr = std::shared_ptr<GraphCurveSkeleton<Model> >;
			
			/**
			 *  \brief Storage type of the objects in the skeleton
			 */
			using Stor = Eigen::Matrix<double,model::meta<Model>::stordim,1>;

			/**
			 *  \brief Model used to give a meaning to the skeleton
			 *
			 *  \details This class will not be modified inside the skeleton:
			 *			 once it is created, it should not be changed
			 */
			typename Model::Ptr m_model;
			
			/**
			 *  \brief All nodes in the graph curve skeleton
			 *
			 *  \details This storage will be intrepreted by the model
			 */
			std::map<unsigned int,Stor> m_node;
			
			/**
			 *  \brief Graph of the curve skeleton
			 */
			boost::adjacency_list<boost::listS,boost::listS,boost::undirectedS> m_graph;
			
		public:
			/**
			 *  \brief Constructor
			 *
			 *  \param model initialisation of the model to use
			 */
			GraphCurveSkeleton(const typename Model::Ptr model) : m_model(model) {}

			/**
			 *  \brief Constructor
			 *
			 *  \param model initialisation of the model to use
			 */
			GraphCurveSkeleton(const Model &model) : GraphCurveSkeleton(new typename Model::Ptr(model)) {}

			/**
			 *  \brief Copy constructor
			 *
			 *  \param grsk skeleton to copy
			 */
			GraphCurveSkeleton(const GraphCurveSkeleton<Model> &grsk) :
				GraphCurveSkeleton(grsk.m_model), m_node(grsk.m_node), m_graph(grsk.m_graph) {}

			/**
			 *  \brief Adding node function
			 *
			 *  \param node node to add
			 *
			 *  \return node indice
			 *
			 *  \details The function does not check if the node is already in the skeleton or not
			 */
			unsigned int addNode(const Stor &node)
			{
				unsigned int key = *(m_node.rbegin())+1;
				m_node.insert(std::pair<unsigned int,Stor>(key,node));
				return key;
			}

			/**
			 *  \brief Adding node function
			 *
			 *  \tparam TypeNode node type to add
			 *
			 *  \param node node to add
			 *
			 *  \return node indice
			 *
			 *  \details The function does not check if the node is already in the skeleton or not
			 */
			template<typename TypeNode>
			unsigned int addNode(const TypeNode &node)
			{
				Stor vec = m_model->template toVec<TypeNode>(node);
				return addNode<Eigen::Matrix<double,model::meta<Model>::stordim,1> >(vec);
			}
	};
}


#endif //_GRAPHCURVESKELETON_H_

