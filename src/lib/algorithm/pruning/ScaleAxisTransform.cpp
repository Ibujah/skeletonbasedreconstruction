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
 *  \file ScaleAxisTransform.cpp
 *  \brief Defines scale axis transform, a pruning algorithm
 *  \author Bastien Durix
 */

#include "ScaleAxisTransform.h"

// descreasing order
bool compare(const std::pair<unsigned int,double> &p1, const std::pair<unsigned int,double> &p2)
{
	return p1.second > p2.second;
}

template<typename Model>
bool ScaledIsIn(const typename skeleton::GraphCurveSkeleton<Model>::Ptr grskel, unsigned int ind1, unsigned int ind2, double scale)
{
	return grskel->getModel()->included(
			grskel->getModel()->resize(grskel->getNode(ind1),scale),
			grskel->getModel()->resize(grskel->getNode(ind2),scale));
}

template<typename Model>
inline typename skeleton::GraphCurveSkeleton<Model>::Ptr ScaleAxisTransform_helper(const typename skeleton::GraphCurveSkeleton<Model>::Ptr grskl, const double &scale)
{
	typename skeleton::GraphCurveSkeleton<Model>::Ptr grsimp(new skeleton::GraphCurveSkeleton<Model>(grskl->getModel()));

	std::list<unsigned int> list_ind;
	grskl->getAllNodes(list_ind);

	std::list<std::pair<unsigned int,double> > list_ind_size;
	for(std::list<unsigned int>::iterator it = list_ind.begin(); it != list_ind.end(); it++)
	{
		list_ind_size.push_back(std::pair<unsigned int,double>(*it,grskl->getModel()->getSize(grskl->getNode(*it))));
	}

	//sort nodes in decreasing order
	list_ind_size.sort(compare);

	std::list<std::pair<unsigned int,double> >::iterator it = list_ind_size.begin();
	while(it != list_ind_size.end())
	{
		try
		{
			unsigned int refkey = it->first;

			std::list<unsigned int> l_nod;

			// get the neighbors of the node
			grsimp->getNeighbors(refkey,l_nod);

			bool removed = false;
			for(std::list<unsigned int>::iterator itno = l_nod.begin(); itno != l_nod.end() && !removed; itno++)
			{
				// if into is in the resized sphere, it is deleted
				if(ScaledIsIn<Model>(grsimp,refkey,*itno,scale))
				{
					std::vector<unsigned int> v_nod_nei(0);
					grsimp->getNeighbors(*itno,v_nod_nei);

					// link the actual node to the neighbors of the deleted node
					for(unsigned int i=0;i<v_nod_nei.size();i++)
						grsimp->addEdge(refkey,v_nod_nei[i]);

					grsimp->remNode(*itno);
					removed = true;
				}
			}
			if(!removed)
				it++;
		}
		catch(...)
		{
			it++;
		}
	}

	return grsimp;
}

skeleton::GraphSkel2d::Ptr algorithm::pruning::ScaleAxisTransform(const skeleton::GraphSkel2d::Ptr grskl, const double &scale)
{
	return ScaleAxisTransform_helper<skeleton::model::Classic<2> >(grskl,scale);
}

skeleton::GraphProjSkel::Ptr algorithm::pruning::ScaleAxisTransform(const skeleton::GraphProjSkel::Ptr grskl, const double &scale)
{
	return ScaleAxisTransform_helper<skeleton::model::Projective>(grskl,scale);
}
