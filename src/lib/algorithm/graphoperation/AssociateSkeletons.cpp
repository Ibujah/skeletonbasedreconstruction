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
 *  \file AssociateSkeletons.cpp
 *  \brief Defines algorithm to associate branches of multiple skeletons
 *  \author Bastien Durix
 */

#include "AssociateSkeletons.h"
#include "ConnectedComponents.h"
#include "SeparateBranches.h"
#include <set>

/**
 *  \brief Removes all degree 2 nodes
 */
skeleton::GraphProjSkel::Ptr SimplifySkeleton(const skeleton::GraphProjSkel::Ptr skel, const std::vector<unsigned int> &assoc_ext)
{
	skeleton::GraphProjSkel::Ptr skelsimp(new skeleton::GraphProjSkel(*skel));
	
	std::list<unsigned int> listnodes;
	
	skelsimp->getNodesByDegree(0,listnodes);
	skelsimp->getNodesByDegree(1,listnodes);
	
	for(std::list<unsigned int>::iterator it = listnodes.begin(); it != listnodes.end(); it++)
	{
		if(std::find(assoc_ext.begin(),assoc_ext.end(),*it) == assoc_ext.end())
		{
			algorithm::graphoperation::DeleteExtremity(skelsimp,*it);
		}
	}

	listnodes.clear();
	skelsimp->getAllNodes(listnodes);
	
	for(std::list<unsigned int>::iterator it = listnodes.begin(); it != listnodes.end(); it++)
	{
		std::vector<unsigned int> neigh(0);
		skelsimp->getNeighbors(*it,neigh);
		if(neigh.size() == 2)
		{
			skelsimp->addEdge(neigh[0],neigh[1]);
			skelsimp->remNode(*it);
		}
	}
	
	return skelsimp;
}

/**
 *  \brief Encodes an edge, naming it by the extremities its separates
 */
void EncodeEdge(const skeleton::GraphProjSkel::Ptr skel, const std::pair<unsigned int, unsigned int> &edge, const std::vector<unsigned int> &assocext, std::set<std::set<unsigned int> > &set_edg)
{
	skeleton::GraphProjSkel::Ptr skelcut(new skeleton::GraphProjSkel(*skel));
	
	skelcut->remEdge(edge.first,edge.second);
	
	std::list<skeleton::GraphProjSkel::Ptr> listcomp = algorithm::graphoperation::SeparateComponents(skelcut);

	set_edg = std::set<std::set<unsigned int> >();
	for(std::list<skeleton::GraphProjSkel::Ptr>::iterator it = listcomp.begin(); it != listcomp.end(); it++)
	{
		std::list<unsigned int> node_ext;
		(*it)->getAllNodes(node_ext);
		
		std::set<unsigned int> set_ext;
		for(std::list<unsigned int>::iterator itn = node_ext.begin(); itn != node_ext.end(); itn++)
		{
			unsigned int ind = std::find(assocext.begin(),assocext.end(),*itn)-assocext.begin();
			if(ind != assocext.size())
				set_ext.insert(ind);
		}
		set_edg.insert(set_ext);
	}
}

/**
 *  \brief Asociate the first node to two sets representing an edge
 */
template<typename SkelType>
unsigned int AssociatedNode(const SkelType skel, const std::vector<unsigned int> &assocext, const std::set<unsigned int> &set_ext1, const std::set<unsigned int> &set_ext2)
{
	std::set<unsigned int> set_nod;
	for(std::set<unsigned int>::const_iterator it = set_ext1.begin(); it != set_ext1.end(); it++)
	{
		set_nod.insert(assocext[*it]);
	}
	
	unsigned int opp_nod = assocext[*(set_ext2.begin())];
	
	std::list<std::vector<unsigned int> > paths = algorithm::graphoperation::GetNodes(skel,*(set_nod.begin()),opp_nod);

	if(paths.size() < 1)
	{
		throw std::logic_error("Not able to find the path.");
	}

	std::vector<unsigned int> &path = *(paths.begin());

	std::map<unsigned int, unsigned int> used;

	for(unsigned int i = 0; i < path.size(); i++)
		used[path[i]] = 1;

	for(std::set<unsigned int>::iterator it = std::next(set_nod.begin()); it != set_nod.end(); it++)
	{
		std::list<std::vector<unsigned int> > paths2 = algorithm::graphoperation::GetNodes(skel,*it,opp_nod);
		std::vector<unsigned int> &path2 = *(paths2.begin());

		for(unsigned int i = 0; i < path2.size(); i++)
		{
			std::map<unsigned int, unsigned int>::iterator itu = used.find(path2[i]);
			if(itu!=used.end())
				itu->second++;
		}
	}
	
	unsigned nbusedmax = used[path[path.size()-1]];

	unsigned int indexnode = 0;

	while(used[path[indexnode]] != nbusedmax && indexnode != path.size()-1)
		indexnode++;

	return path[indexnode];
}

/**
 *  \brief Decodes an edge, from the extremities its separates
 */
template<typename SkelType>
void DecodeEdge(const SkelType skel, const std::vector<unsigned int> &assocext, const std::set<std::set<unsigned int> > &set_edg, std::pair<unsigned int, unsigned int> &edge)
{
	edge.first  = AssociatedNode(skel,assocext,(*set_edg.begin()),(*set_edg.rbegin()));
	edge.second = AssociatedNode(skel,assocext,(*set_edg.rbegin()),(*set_edg.begin()));
}

/**
 *  \brief Contracts an edge, replacing it by the middle node
 */
void ContractEdge(skeleton::GraphProjSkel::Ptr skelsimp, const skeleton::GraphProjSkel::Ptr skelori, const std::pair<unsigned int,unsigned int> &edge)
{
	std::list<std::vector<unsigned int> > listbr = algorithm::graphoperation::GetNodes(skelori, edge.first, edge.second);
	std::vector<unsigned int> &br = *(listbr.begin());
	
	unsigned int node = br[br.size()/2];

	std::list<unsigned int> neigh;
	skelsimp->getNeighbors(edge.first,neigh);
	skelsimp->getNeighbors(edge.second,neigh);

	skelsimp->remNode(edge.first);
	skelsimp->remNode(edge.second);

	skelsimp->addNode(node,skelori->getNode(node));
	
	for(std::list<unsigned int>::iterator it = neigh.begin(); it != neigh.end(); it++)
	{
		if(*it != edge.first && *it != edge.second)
		{
			skelsimp->addEdge(node,*it);
		}
	}
}

skeleton::ReconstructionSkeleton::Ptr algorithm::graphoperation::TopoMatch(const std::vector<skeleton::GraphProjSkel::Ptr> &vec_skel, const std::vector<std::vector<unsigned int> > &assoc_ext)
{
	std::vector<skeleton::GraphProjSkel::Ptr> vec_skelsimp(vec_skel.size());
	
	std::map<std::set<std::set<unsigned int> >,unsigned int> cptedg;
	
	// converts edges into sets, an count number of occurences
	for(unsigned int i = 0; i < vec_skel.size(); i++)
	{
		vec_skelsimp[i] = SimplifySkeleton(vec_skel[i],assoc_ext[i]);
		
		std::list<std::pair<unsigned int,unsigned int> > listedges;
		vec_skelsimp[i]->getAllEdges(listedges);
		for(std::list<std::pair<unsigned int,unsigned int> >::iterator it = listedges.begin(); it != listedges.end(); it++)
		{
			std::set<std::set<unsigned int> > edgset;
			EncodeEdge(vec_skelsimp[i],*it,assoc_ext[i],edgset);
			std::map<std::set<std::set<unsigned int> >,unsigned int>::iterator its = cptedg.find(edgset);
			if(its == cptedg.end())
			{
				cptedg[edgset] = 1;
			}
			else
			{
				its->second++;
			}
		}
	}
	
	// contracts edges which occurences is less than half the number of skeletons
	for(unsigned int i = 0; i < vec_skelsimp.size(); i++)
	{
		std::vector<std::pair<unsigned int,unsigned int> > vecedges(0);
		vec_skelsimp[i]->getAllEdges(vecedges);

		std::vector<std::set<std::set<unsigned int> > > vecedgset(vecedges.size());

		for(unsigned int j = 0; j < vecedges.size() ; j++)
		{
			EncodeEdge(vec_skelsimp[i],vecedges[j],assoc_ext[i],vecedgset[j]);
		}
		
		for(std::map<std::set<std::set<unsigned int> >,unsigned int>::iterator it = cptedg.begin(); it != cptedg.end(); it++)
		{
			//if the edge is not kept, it is contracted
			if(it->second <= vec_skel.size()/2)
			{
				unsigned int ind = std::find(vecedgset.begin(),vecedgset.end(),it->first) - vecedgset.begin();
				if(ind != vecedgset.size())
				{
					ContractEdge(vec_skelsimp[i],vec_skel[i],vecedges[ind]);
					
					vecedges.resize(0);
					vec_skelsimp[i]->getAllEdges(vecedges);
					vecedgset.resize(vecedges.size());

					for(unsigned int j = 0; j < vecedges.size() ; j++)
					{
						EncodeEdge(vec_skelsimp[i],vecedges[j],assoc_ext[i],vecedgset[j]);
					}
				}
			}
		}
	}

	std::list<std::set<std::set<unsigned int> > > keptedg;

	// get most present edges
	for(std::map<std::set<std::set<unsigned int> >, unsigned int>::iterator it = cptedg.begin(); it != cptedg.end(); it++)
	{
		if(it->second > vec_skel.size()/2)
		{
			keptedg.push_back(it->first);
		}
	}
	
	// build the final graph
	skeleton::ReconstructionSkeleton::Ptr recskel(new skeleton::ReconstructionSkeleton());
	unsigned int nbext = assoc_ext[0].size();
	std::vector<unsigned int> assoc_med(nbext);
	recskel->addNode(nbext);
	// edges connected to extremities
	for(std::list<std::set<std::set<unsigned int> > >::iterator it = keptedg.begin(); it != keptedg.end(); it++)
	{
		std::set<std::set<unsigned int> > &set_ext = *it;
		const std::set<unsigned int> &set1 = *(set_ext.begin()),
									  set2 = *(set_ext.rbegin());
		if(set1.size() == 1 || set2.size() == 1)
		{
			unsigned int indnod;
			if(set1.size() == 1)
				indnod = *(set1.begin());
			else
				indnod = *(set2.begin());

			assoc_med[indnod] = indnod;

			recskel->addNode(indnod);
			
			// branch creation
			std::vector<unsigned int> indskel(assoc_ext.size());
			std::vector<unsigned int> firstext(assoc_ext.size());
			std::vector<unsigned int> secondext(assoc_ext.size());
			for(unsigned int j = 0; j < indskel.size(); j++)
			{
				indskel[j] = j;
				
				std::pair<unsigned int,unsigned int> edge;
				DecodeEdge(vec_skelsimp[j],assoc_ext[j],set_ext,edge);

				firstext[j] = edge.first;
				secondext[j] = edge.second;
			}
			skeleton::ReconstructionBranch::Ptr br(new skeleton::ReconstructionBranch(indskel,firstext,secondext));

			if(set1.size() == 1)
				recskel->addEdge(indnod,nbext,br);
			else
				recskel->addEdge(nbext,indnod,br);
		}
	}
	
	// other edges
	for(std::list<std::set<std::set<unsigned int> > >::iterator it = keptedg.begin(); it != keptedg.end(); it++)
	{
		std::set<std::set<unsigned int> > &set_ext = *it;
		const std::set<unsigned int> &set1 = *(set_ext.begin()),
									  set2 = *(set_ext.rbegin());
		if(set1.size() != 1 && set2.size() != 1)
		{
			std::pair<unsigned int,unsigned int> edge_med;
			DecodeEdge(recskel,assoc_med,set_ext,edge_med);
			
			// branch creation
			std::vector<unsigned int> indskel(assoc_ext.size());
			std::vector<unsigned int> firstext(assoc_ext.size());
			std::vector<unsigned int> secondext(assoc_ext.size());
			for(unsigned int j = 0; j < indskel.size(); j++)
			{
				indskel[j] = j;
				
				std::pair<unsigned int,unsigned int> edge;
				DecodeEdge(vec_skelsimp[j],assoc_ext[j],set_ext,edge);
				
				firstext[j] = edge.first;
				secondext[j] = edge.second;
			}
			
			unsigned int prev_nod = edge_med.first;
			unsigned int added_node = recskel->addNode();
			
			std::set<unsigned int> add_set; // get nodes connected to prev_nod, on the path to the extremities
			for(std::set<unsigned int>::const_iterator its = set2.begin(); its != set2.end(); its++)
			{
				std::list<std::vector<unsigned int> > paths = algorithm::graphoperation::GetNodes(recskel,prev_nod,*its);
				std::vector<unsigned int> &path = *(paths.begin());
				
				add_set.insert(path[1]);
			}
			
			// add edges from added_node to add_set
			for(std::set<unsigned int>::const_iterator its = add_set.begin(); its != add_set.end(); its++)
			{
				skeleton::ReconstructionBranch::Ptr br;
				recskel->remEdge(prev_nod,*its,br);
				recskel->addEdge(added_node,*its,br);
			}

			skeleton::ReconstructionBranch::Ptr br(new skeleton::ReconstructionBranch(indskel,firstext,secondext));
			recskel->addEdge(prev_nod,added_node,br);
		}
	}

	return recskel;
}
