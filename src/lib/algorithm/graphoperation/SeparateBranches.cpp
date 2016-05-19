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
 *  \file SeparateBranches.cpp
 *  \brief Defines function to separate skeleton in different branches
 *  \author Bastien Durix
 */

#include "SeparateBranches.h"

template<typename Model>
typename skeleton::ComposedCurveSkeleton<skeleton::GraphBranch<Model> >::Ptr SeparateBranches_helper(const typename skeleton::GraphCurveSkeleton<Model>::Ptr grskel)
{
	// nodes identifiants 
	std::vector<unsigned int> vec_nodeid(0);
	grskel->getAllNodes(vec_nodeid);
	
	// set how many times they are used
	std::vector<bool> used(vec_nodeid.size(),false);
	
	// container of starting points
	std::list<unsigned int> extremities;
	grskel->getNodesByDegree(1,extremities);
	
	// branches containers
	std::list<std::pair<Eigen::Matrix<unsigned int,2,1>,std::list<unsigned int> > > compskel;

	/*
	 *  For each extremity, we traverse the graph to estimate the branch starting from this extremity
	 */
	for(std::list<unsigned int>::iterator ext = extremities.begin(); ext != extremities.end(); ext++)
	{
		std::list<unsigned int> cur_branch; // contains the nodes indices
		unsigned int cur_nod = *ext,
					 prev_nod = *ext;
		bool finished = false;
		/*
		 *  Traversing the branch
		 */
		while(!finished)
		{
			// indice of the current node in the graph skeleton
			unsigned int ind_cur_nod = std::find(vec_nodeid.begin(),vec_nodeid.end(),cur_nod) - vec_nodeid.begin();
			cur_branch.push_back(cur_nod);
			
			// mark the node
			used[ind_cur_nod]=true;
			
			// current node neighbors
			std::vector<unsigned int> neigh(0);
			grskel->getNeighbors(cur_nod,neigh);

			if(neigh.size() == 2) // basis case : continue
			{
				if(neigh[0] == prev_nod)
				{
					prev_nod = cur_nod;
					cur_nod = neigh[1];
				}
				else
				{
					prev_nod = cur_nod;
					cur_nod = neigh[0];
				}
			}
			else if(neigh.size() == 1) // extremity case
			{
				// indice of current neighbor
				unsigned int ind_neigh = std::find(vec_nodeid.begin(),vec_nodeid.end(),neigh[0]) - vec_nodeid.begin();
				if(!used[ind_neigh] || grskel->getNodeDegree(neigh[0]) > 2) // if next node is not used, or degree more than 2, continue
				{
					prev_nod=cur_nod;
					cur_nod=neigh[0];
				}
				else
				{
					finished=true;
				}
			}
			else // junction case
			{
				if(prev_nod == cur_nod) // current node == first node
				{
					finished=true;
					for(unsigned int i = 0; i<neigh.size() && finished; i++) // test if there is a branch starting from this point
					{
						unsigned int ind_neigh = std::find(vec_nodeid.begin(),vec_nodeid.end(),neigh[i]) - vec_nodeid.begin();
						if(!used[ind_neigh])
						{
							prev_nod=cur_nod;
							cur_nod=neigh[i];
							finished=false;
						}
					}

				}
				else // current node == ending point
				{
					for(unsigned int i = 0; i<neigh.size(); i++) // adds the ending node if there is still new branches starting from it
					{
						unsigned int ind_neigh = std::find(vec_nodeid.begin(),vec_nodeid.end(),neigh[i]) - vec_nodeid.begin();
						if(!used[ind_neigh])
						{
							extremities.push_back(cur_nod);
						}
					}
					finished=true;
				}
			}
		}

		if(prev_nod != cur_nod) // adds the branch to the composed skeleton
		{
			compskel.push_back(std::pair<Eigen::Matrix<unsigned int,2,1>,std::list<unsigned int> >(Eigen::Matrix<unsigned int,2,1>(*ext,cur_nod),cur_branch));
		}
	}

	std::map<unsigned int, std::vector<std::pair<unsigned int, unsigned int> > > map_links;
	
	// get the extremities/junctions of the skeleton
	extremities.clear();
	for(std::list<std::pair<Eigen::Matrix<unsigned int,2,1>,std::list<unsigned int> > >::iterator branch = compskel.begin(); branch != compskel.end(); branch++)
	{
		extremities.push_back(branch->first(0));
		extremities.push_back(branch->first(1));
	}
	extremities.sort();
	extremities.unique();
	
	// creates the composed skeleton
	typename skeleton::ComposedCurveSkeleton<skeleton::GraphBranch<Model> >::Ptr 
		skelres(new typename skeleton::ComposedCurveSkeleton<skeleton::GraphBranch<Model> >());
	
	// adds the extremities in the composed skeleton
	std::vector<unsigned int> vec_ext(extremities.begin(),extremities.end());
	for(unsigned int i=0;i<extremities.size();i++)
	{
		skelres->addNode(i);
	}

	// construct the branches
	for(std::list<std::pair<Eigen::Matrix<unsigned int,2,1>,std::list<unsigned int> > >::iterator branch = compskel.begin(); branch != compskel.end(); branch++)
	{
		// nodes in the skeleton
		std::vector<typename Model::Stor> vec_br(0);
		grskel->getNodes(branch->second,vec_br);
		
		//create the branch
		skeleton::GraphBranch<Model> disbranch(grskel->getModel(),vec_br);
		
		// compute the extremities
		unsigned int ext1 = std::find(vec_ext.begin(),vec_ext.end(),branch->first(0))-vec_ext.begin();
		unsigned int ext2 = std::find(vec_ext.begin(),vec_ext.end(),branch->first(1))-vec_ext.begin();
		
		// adds the branch, associated to the extremities
		skelres->addEdge(ext1,ext2,disbranch);
	}

	return skelres;
}

typename skeleton::CompGraphSkel2d::Ptr algorithm::graphoperation::SeparateBranches(const typename skeleton::GraphSkel2d::Ptr grskel)
{
	return SeparateBranches_helper<skeleton::model::Classic<2> >(grskel);
}

typename skeleton::CompGraphSkel3d::Ptr algorithm::graphoperation::SeparateBranches(const typename skeleton::GraphSkel3d::Ptr grskel)
{
	return SeparateBranches_helper<skeleton::model::Classic<3> >(grskel);
}

typename skeleton::CompGraphProjSkel::Ptr algorithm::graphoperation::SeparateBranches(const typename skeleton::GraphProjSkel::Ptr grskel)
{
	return SeparateBranches_helper<skeleton::model::Projective>(grskel);
}

template<typename SkelType>
void BrowsePath(std::list<std::list<unsigned int> >& l_path,
				const SkelType grskl, unsigned int first, unsigned int last,
				const std::list<unsigned int> &path = std::list<unsigned int>())
{
	std::list<unsigned int> npath(path.begin(),path.end());
	npath.push_back(first);
	
	/*terminal case : node2 reached*/
	if(first == last)
	{
		l_path.push_back(npath);
	}
	else
	{
		std::list<unsigned int> neigh;
		grskl->getNeighbors(first,neigh);
		
		/*each reached node is removed from neighbors*/
		if(npath.size())
		{
			for(std::list<unsigned int>::iterator it = neigh.begin(); it != neigh.end(); it++)
			{
				if(std::find(npath.begin(),npath.end(),*it)!=npath.end())
				{
					it = neigh.erase(it);
					it--;
				}
			}
		}

		/* 
		 * if there is still some neighbors
		 * implicit terminal case : if there is not any neighbor in neigh
		 */
		if(neigh.size()==1)
		{
			BrowsePath(l_path,grskl,*(neigh.begin()),last,npath);
		}
		else if(neigh.size()>=2)
		{
			for(std::list<unsigned int>::iterator it = neigh.begin(); it != neigh.end(); it++)
			{
				std::list<unsigned int> cur_path = npath;
				std::list<std::list<unsigned int> > sub_list;
				BrowsePath(sub_list,grskl,*it,last,cur_path);
				l_path.insert(l_path.end(),sub_list.begin(),sub_list.end());
			}
		}
	}
}

template<typename Model>
std::list<typename skeleton::GraphBranch<Model>::Ptr> GetBranch_helper(const typename skeleton::GraphCurveSkeleton<Model>::Ptr grskel, unsigned int first, unsigned int last)
{
	std::list<std::list<unsigned int> > paths;
	BrowsePath(paths,grskel,first,last);
	
	std::list<typename skeleton::GraphBranch<Model>::Ptr> l_br;

	for(std::list<std::list<unsigned int> >::iterator it = paths.begin(); it != paths.end(); it++)
	{
		std::vector<typename skeleton::GraphBranch<Model>::Stor> vec;
		vec.reserve(it->size());
		for(std::list<unsigned int>::iterator itv = it->begin(); itv != it->end(); itv++)
		{
			vec.push_back(grskel->getNode(*itv));
		}

		typename skeleton::GraphBranch<Model>::Ptr br(new skeleton::GraphBranch<Model>(grskel->getModel(),vec));

		l_br.push_back(br);
	}
	
	return l_br;
}

template<typename SkelType>
std::list<std::vector<unsigned int> > GetNodes_helper(const SkelType grskel, unsigned int first, unsigned int last)
{
	std::list<std::list<unsigned int> > paths;
	BrowsePath(paths,grskel,first,last);
	
	std::list<std::vector<unsigned int> > l_br;

	for(std::list<std::list<unsigned int> >::iterator it = paths.begin(); it != paths.end(); it++)
	{
		std::vector<unsigned int> vec(it->begin(),it->end());
		l_br.push_back(vec);
	}
	
	return l_br;
}

std::list<std::vector<unsigned int> > algorithm::graphoperation::GetNodes(const typename skeleton::GraphSkel2d::Ptr grskel, unsigned int first, unsigned int last)
{
	return GetNodes_helper(grskel,first,last);
}

std::list<std::vector<unsigned int> > algorithm::graphoperation::GetNodes(const typename skeleton::GraphSkel3d::Ptr grskel, unsigned int first, unsigned int last)
{
	return GetNodes_helper(grskel,first,last);
}

std::list<std::vector<unsigned int> > algorithm::graphoperation::GetNodes(const typename skeleton::GraphProjSkel::Ptr grskel, unsigned int first, unsigned int last)
{
	return GetNodes_helper(grskel,first,last);
}

std::list<std::vector<unsigned int> > algorithm::graphoperation::GetNodes(const typename skeleton::ReconstructionSkeleton::Ptr grskel, unsigned int first, unsigned int last)
{
	return GetNodes_helper(grskel,first,last);
}

std::list<typename skeleton::BranchGraphSkel2d::Ptr> algorithm::graphoperation::GetBranch(const typename skeleton::GraphSkel2d::Ptr grskel, unsigned int first, unsigned int last)
{
	return GetBranch_helper<skeleton::model::Classic<2> >(grskel,first,last);
}

std::list<typename skeleton::BranchGraphSkel3d::Ptr> algorithm::graphoperation::GetBranch(const typename skeleton::GraphSkel3d::Ptr grskel, unsigned int first, unsigned int last)
{
	return GetBranch_helper<skeleton::model::Classic<3> >(grskel,first,last);
}

std::list<typename skeleton::BranchGraphProjSkel::Ptr> algorithm::graphoperation::GetBranch(const typename skeleton::GraphProjSkel::Ptr grskel, unsigned int first, unsigned int last)
{
	return GetBranch_helper<skeleton::model::Projective>(grskel,first,last);
}

std::vector<typename skeleton::CompGraphProjSkel::Ptr> algorithm::graphoperation::GetComposed(
				const typename skeleton::ReconstructionSkeleton::Ptr recskel,
				const std::vector<skeleton::GraphProjSkel::Ptr> &vec_prskel)
{
	std::vector<typename skeleton::CompGraphProjSkel::Ptr> vec_compskel(vec_prskel.size());
		
	std::list<unsigned int> nodes;
	recskel->getAllNodes(nodes);

	std::list<unsigned int> edges;
	recskel->getAllEdges(edges);

	for(unsigned int i = 0; i < vec_prskel.size(); i++)
	{
		typename skeleton::CompGraphProjSkel::Ptr compskel(new skeleton::CompGraphProjSkel());
		
		for(std::list<unsigned int>::iterator it = nodes.begin(); it != nodes.end(); it++)
		{
			compskel->addNode(*it);
		}
		
		for(std::list<unsigned int>::iterator it = edges.begin(); it != edges.end(); it++)
		{
			std::pair<unsigned int,unsigned int> ext = recskel->getExtremities(*it);
			
			const skeleton::ReconstructionBranch::Ptr recbr = recskel->getBranch(ext.first,ext.second);
			
			unsigned int firstext = recbr->getFirstExt()[i];
			unsigned int lastext = recbr->getLastExt()[i];
			
			std::list<typename skeleton::BranchGraphProjSkel::Ptr> listbr = GetBranch(vec_prskel[i],firstext,lastext);
			
			if(listbr.size() == 0)
			{
				throw std::logic_error("algorithm::graphoperation::GetComposed : Branch does not exist in skeleton");
			}
			else if(listbr.size() > 1)
			{
				throw std::logic_error("algorithm::graphoperation::GetComposed : Skeleton contains a cycle");
			}
			
			typename skeleton::BranchGraphProjSkel::Ptr prbranch = *(listbr.begin());
			
			compskel->addEdge(ext.first,ext.second,prbranch);
		}
		
		vec_compskel[i] = compskel;
	}
	
	return vec_compskel;
}
