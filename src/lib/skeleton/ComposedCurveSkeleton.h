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
			 *  \brief Vertex property
			 */
			struct VertexProperty 
			{
				/**
				 *  \brief Vertex index
				 */
				unsigned int index;
			};
			
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
			using GraphType = boost::adjacency_list<boost::listS,boost::listS,boost::directedS,VertexProperty,EdgeProperty>;

			/**
			 *  \brief Graph of the composed skeleton
			 */
			GraphType m_graph;
			
			/**
			 *  \brief Last added node, to compute the key of the next added node
			 */
			unsigned int m_nodelast;
			
			/**
			 *  \brief Last added edge, to compute the key of the next added edge
			 */
			unsigned int m_edgelast;

		public:
			/**
			 *  \brief Constructor
			 */
			ComposedCurveSkeleton() : m_graph(), m_nodelast(0), m_edgelast(0) {}
		
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
				for(boost::tie(vi,vi_end) = boost::vertices(m_graph); vi != vi_end && !v_found; vi++)
				{
					if(m_graph[*vi].index == index)
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
				for(boost::tie(vi,vi_end) = boost::vertices(m_graph); vi != vi_end && (!v1_found || !v2_found); vi++)
				{
					if(m_graph[*vi].index == ind1)
					{
						v_desc1 = *vi;
						v1_found = true;
					}
					if(m_graph[*vi].index == ind2)
					{
						v_desc2 = *vi;
						v2_found = true;
					}
				}
				
				return v1_found && v2_found;
			}

		public://modifying functions
			/**
			 *  \brief Adding node function
			 *
			 *  \return node index
			 */
			unsigned int addNode()
			{
				unsigned int index = m_nodelast++;
				//Adds the vertex in the graph, with index and storage information
				typename boost::graph_traits<GraphType>::vertex_descriptor v_desc = boost::add_vertex(m_graph);
				m_graph[v_desc].index = index;
				return index;
			}

			/**
			 *  \brief Adding node function, with given index
			 *
			 *  \param index new node index
			 *
			 *  \return node index
			 */
			unsigned int addNode(unsigned int index)
			{
				typename boost::graph_traits<GraphType>::vertex_descriptor v_desc;
				if(!getDesc(index,v_desc))
				{
					//Adds the vertex in the graph, with index and storage information
					typename boost::graph_traits<GraphType>::vertex_descriptor v_desc = boost::add_vertex(m_graph);
					m_graph[v_desc].index = index;
				}
				return index;
			}

			/**
			 *  \brief Adding edge function
			 *
			 *  \param ind1   first node index
			 *  \param ind2   second node index
			 *  \param branch branch to add
			 *
			 *  \return false if one of the two nodes is not in the skeleton
			 */
			bool addEdge(unsigned int ind1, unsigned int ind2, const BranchType &branch)
			{
				bool v_found = false;
				if(ind1 != ind2)
				{
					// first step, get the descriptors corresponding to indices
					typename boost::graph_traits<GraphType>::vertex_descriptor v_desc1, v_desc2;
					v_found = getDesc(ind1,ind2,v_desc1,v_desc2);

					// second step, add the edge
					if(v_found)
					{
						typename boost::graph_traits<GraphType>::edge_descriptor e_desc;
						bool added;
						boost::tie(e_desc,added) = boost::add_edge(v_desc1,v_desc2,m_graph);
						if(added)
						{
							m_graph[e_desc].index = m_edgelast++;
							m_graph[e_desc].branch = typename BranchType::Ptr(new BranchType(branch));
						}
					}
				}
				return v_found;
			}
	};
}


#endif //_COMPOSEDCURVESKELETON_H_
