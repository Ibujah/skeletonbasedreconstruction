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
 *  \file ConnectedComponents.cpp
 *  \brief Defines function to separate skeleton in connected components
 *  \author Bastien Durix
 */

#include "ConnectedComponents.h"

/**
 *  \brief Separate skeleton into connected components
 *
 *  \tparam GraphSkel skeleton type
 *  \param  grskel    skeleton to separate
 *
 *  \return list of connected components
 */
template<typename GraphSkel>
std::list<typename GraphSkel::Ptr> SeparateComponents_helper(const typename GraphSkel::Ptr grskel)
{
	std::vector<unsigned int> nodekey(0); // all node keys
	grskel->getAllNodes(nodekey);

	std::vector<unsigned int> connect_comp(nodekey.size(),0); // connected component associated to each node

	std::list<typename GraphSkel::Ptr> list_comp;
	if(nodekey.size())
	{
		/*
		 * Identify connected components
		 */
		bool alllabelled = false;
		unsigned int labelcomp = 1;

		while(!alllabelled)
		{
			unsigned int firstnotlabelled = 0;
			alllabelled=true;
			
			/*
			 * Get the first node not labelled
			 */
			for(unsigned int i=0;i<connect_comp.size() && alllabelled;i++)
			{
				if(!connect_comp[i])
				{
					firstnotlabelled = i;
					alllabelled=false;
				}
			}
			
			if(!alllabelled)
			{
				std::list<unsigned int> tolabel;
				tolabel.push_back(firstnotlabelled);
				
				/*
				 * Label all the nodes connected to firstnotlabelled
				 */
				while(!tolabel.empty())
				{
					unsigned int key = *(tolabel.begin());
					tolabel.pop_front();
					
					/*
					 * If the node key is not labelled,
					 * add all its neighbors to the tolabel list
					 */
					if(!connect_comp[key])
					{
						for(unsigned int i=0;i<connect_comp.size();i++)
						{
							if(nodekey[i]==key)
								connect_comp[i]=labelcomp;
						}
						grskel->getNeighbors(nodekey[key],tolabel);
					}
				}
				
				/*
				 * Increment the label
				 */
				labelcomp++;
			}
		}

		/*
		 * Add components in skeletons
		 */
		for(unsigned int label = 1; label < labelcomp; label++)
		{
			typename GraphSkel::Ptr grskl_comp(new GraphSkel(grskel->getModel()));
			
			/*
			 * Add each labelled node to the current skeleton
			 */
			for(unsigned int i = 0; i < connect_comp.size(); i++)
			{
				if(connect_comp[i] == label)
				{
					/*
					 * If node node i is labelled,
					 * add the node nodekey[i]
					 */
					grskl_comp->addNode(nodekey[i],grskel->getNode(nodekey[i]));
				}
			}
			
			for(unsigned int i = 0; i < connect_comp.size(); i++)
			{
				if(connect_comp[i] == label)
				{
					/*
					 * If the node i is in the skeleton,
					 * add its neighbors
					 */
					std::vector<unsigned int> neigh(0);
					grskel->getNeighbors(nodekey[i],neigh);

					/*
					 * As the nodes do have the sames indices,
					 * we can add directly the edge
					 */
					for(unsigned int j=0; j < neigh.size(); j++)
					{
						grskl_comp->addEdge(nodekey[i],neigh[j]);
					}
				}
			}
			list_comp.push_back(grskl_comp);
		}
	}

	return list_comp;
}

std::list<skeleton::GraphSkel2d::Ptr> algorithm::graphoperation::SeparateComponents(const skeleton::GraphSkel2d::Ptr grskel)
{
	return SeparateComponents_helper<skeleton::GraphSkel2d>(grskel);
}
