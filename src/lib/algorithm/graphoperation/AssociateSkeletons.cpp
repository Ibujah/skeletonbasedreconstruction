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
#include <set>

/**
 *  \brief Removes all degree 2 nodes
 */
skeleton::GraphProjSkel::Ptr SimplifySkeleton(const skeleton::GraphProjSkel::Ptr skel)
{
	skeleton::GraphProjSkel::Ptr skelsimp(new skeleton::GraphProjSkel(*skel));
	
	std::list<unsigned int> listnodes;
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

	for(std::list<skeleton::GraphProjSkel::Ptr>::iterator it = listcomp.begin(); it != listcomp.end(); it++)
	{
		std::list<unsigned int> node_ext;
		(*it)->getNodesByDegree(0,node_ext);
		(*it)->getNodesByDegree(1,node_ext);
		
		std::set<unsigned int> set_ext;
		for(std::list<unsigned int>::iterator itn = node_ext.begin(); itn != node_ext.end(); itn++)
		{
			set_ext.insert(std::find(assocext.begin(),assocext.end(),*itn)-assocext.begin());
		}
		set_edg.insert(set_ext);
	}
}

skeleton::ReconstructionSkeleton::Ptr algorithm::graphoperation::TopoMatch(const std::vector<skeleton::GraphProjSkel::Ptr> &vec_skel, const std::vector<std::vector<unsigned int> > &assoc_ext)
{
	std::vector<skeleton::GraphProjSkel::Ptr> vec_skelsimp(vec_skel.size());
	
	std::map<std::set<std::set<unsigned int> >,unsigned int> cptedg;
	
	// converts edges into sets, an count number of occurences
	for(unsigned int i = 0; i < vec_skel.size(); i++)
	{
		vec_skelsimp[i] = SimplifySkeleton(vec_skel[i]);

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
	
	std::list<std::set<std::set<unsigned int> > > keptedg;

	// get most present edges
	for(std::map<std::set<std::set<unsigned int> >, unsigned int>::iterator it = cptedg.begin(); it != cptedg.end(); it++)
	{
		if(it->second > vec_skel.size()/2)
		{
			keptedg.push_back(it->first);
		}
	}
	
	
}
