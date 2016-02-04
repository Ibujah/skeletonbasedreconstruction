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
		public:
			/**
			 *  \brief Graph curve skeleton shared pointer
			 */
			using Ptr = std::shared_ptr<GraphCurveSkeleton<Model> >;

			/**
			 *  \brief Storage type of the objects in the skeleton
			 */
			using Stor = Eigen::Matrix<double,model::meta<Model>::stordim,1>;
			
		protected:
			/**
			 *  \brief Name of storage property in boost graph
			 */
			struct vertex_vec_t
			{
				/**
				 *  \brief Define a vertex tag for the property
				 */
				typedef boost::vertex_property_tag kind;
			};
			
			/**
			 *  \brief Vertex property
			 *
			 *  \details Creates a property called vertex_index_t of type unsigned int
			 *           and a property called vertex_stor of type Stor
			 */
			using VertexProperty = boost::property< boost::vertex_index_t, unsigned int,
								   boost::property< vertex_vec_t,          Stor> >;
			
			/**
			 *  \brief Graph type
			 *
			 *  \details Creates a undirected graph with vertices using VertexProperty
			 */
			using GraphType = boost::adjacency_list<boost::listS,boost::listS,boost::undirectedS,VertexProperty>;
			
			/**
			 *  \brief Model used to give a meaning to the skeleton
			 *
			 *  \details This class will not be modified inside the skeleton:
			 *			 once it is created, it should not be changed
			 */
			typename Model::Ptr m_model;

			/**
			 *  \brief Graph of the curve skeleton
			 */
			GraphType m_graph;
			
			/**
			 *  \brief Last added node, to compute the key of the next added node
			 */
			unsigned int m_last;

		public:
			/**
			 *  \brief Constructor
			 *
			 *  \param model initialisation of the model to use
			 */
			GraphCurveSkeleton(const typename Model::Ptr model) : m_model(model), m_graph(), m_last(0) {}

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
				m_model(grsk.m_model), m_graph(grsk.m_graph), m_last(grsk.m_last) {}
		
		protected:
			/**
			 *  \brief Get the vertex descriptor associated to a vertex index
			 *
			 *  \param index  vertex index
			 *  \param v_desc out vertex descriptor
			 *
			 *  \return false if the index is not in the skeleton
			 */
			bool getDesc(unsigned int index, typename boost::graph_traits<GraphType>::vertex_descriptor &v_desc) const
			{
				typename boost::graph_traits<GraphType>::vertex_iterator vi, vi_end;
				bool v_found = false;
				boost::property_map<GraphType,boost::vertex_index_t> map_index = boost::get(boost::vertex_index_t(),m_graph);
				for(boost::tie(vi,vi_end) = boost::vertex(m_graph); vi != vi_end && v_found; vi++)
				{
					if(map_index[*vi] == index)
					{
						v_desc = *vi;
						v_found = true;
					}
				}
				
				return v_found;
			}

			/**
			 *  \brief Get the vertex descriptors associated to two vertex indices
			 *
			 *  \param ind1    first vertex index
			 *  \param ind2    second vertex index
			 *  \param v_desc1 out first vertex descriptor
			 *  \param v_desc2 out second vertex descriptor
			 *
			 *  \return false if one of the indices is not in the skeleton
			 */
			bool getDesc(unsigned int ind1, unsigned int ind2,
						 typename boost::graph_traits<GraphType>::vertex_descriptor &v_desc1,
						 typename boost::graph_traits<GraphType>::vertex_descriptor &v_desc2) const
			{
				typename boost::graph_traits<GraphType>::vertex_iterator vi, vi_end;
				bool v1_found = false, v2_found = false;
				boost::property_map<GraphType,boost::vertex_index_t> map_index = boost::get(boost::vertex_index_t(),m_graph);
				for(boost::tie(vi,vi_end) = boost::vertex(m_graph); vi != vi_end && (!v1_found || !v2_found); vi++)
				{
					if(map_index[*vi] == ind1)
					{
						v_desc1 = *vi;
						v1_found = true;
					}
					if(map_index[*vi] == ind2)
					{
						v_desc2 = *vi;
						v2_found = true;
					}
				}
				
				return v1_found && v2_found;
			}

		public: // modifying functions
			/**
			 *  \brief Adding node function
			 *
			 *  \param vec vector to add
			 *
			 *  \return node index
			 *
			 *  \details The function does not check if the node is already in the skeleton or not
			 */
			unsigned int addNode(const Stor &vec)
			{
				unsigned int index = m_last++;
				//Adds the vertex in the graph, with index and storage information
				boost::add_vertex(index,vec,m_graph);
				return index;
			}

			/**
			 *  \brief Adding node function
			 *
			 *  \tparam TypeNode node type to add
			 *
			 *  \param node node to add
			 *
			 *  \return node index
			 *
			 *  \details The function does not check if the node is already in the skeleton or not
			 */
			template<typename TypeNode>
			unsigned int addNode(const TypeNode &node)
			{
				Stor vec = m_model->template toVec<TypeNode>(node);
				return addNode<Eigen::Matrix<double,model::meta<Model>::stordim,1> >(vec);
			}
			
			/**
			 *  \brief Removing node function
			 *
			 *  \param index node index
			 *
			 *  \return false if node is not in the skeleton
			 */
			bool remNode(unsigned int index)
			{
				// first step, get the descriptor corresponding to index
				typename boost::graph_traits<GraphType>::vertex_descriptor v_desc;
				bool v_found = getDesc(index,v_desc);

				// second step, remove the node
				if(v_found)
					boost::remove_vertex(v_desc,m_graph);

				return v_found;
			}

			/**
			 *  \brief Adding edge function
			 *
			 *  \param ind1 first node index
			 *  \param ind2 second node index
			 *
			 *  \return false if one of the two nodes is not in the skeleton
			 */
			bool addEdge(unsigned int ind1, unsigned int ind2)
			{
				// first step, get the descriptors corresponding to indices
				typename boost::graph_traits<GraphType>::vertex_descriptor v_desc1, v_desc2;
				bool v_found = getDesc(ind1,ind2,v_desc1,v_desc2);

				// second step, add the edge
				if(v_found)
					boost::add_edge(v_desc1,v_desc2,m_graph);

				return v_found;
			}
			
			/**
			 *  \brief Removing edge function
			 *
			 *  \param ind1 first node index
			 *  \param ind2 second node index
			 *
			 *  \return false if one of the two nodes is not in the skeleton
			 */
			bool remEdge(unsigned int ind1, unsigned int ind2)
			{
				// first step, get the descriptors corresponding to indices
				typename boost::graph_traits<GraphType>::vertex_descriptor v_desc1, v_desc2;
				bool v_found = getDesc(ind1,ind2,v_desc1,v_desc2);

				// second step, remove the edge
				if(v_found)
					boost::remove_edge(v_desc1,v_desc2,m_graph);

				return v_found;
			}

		public: // non modifying functions
			/**
			 *  \brief Model getter
			 *
			 *  \return Const shared pointer to skeleton model
			 */
			inline const typename Model::Ptr getModel() const
			{
				return m_model;
			}
	};
}


#endif //_GRAPHCURVESKELETON_H_

