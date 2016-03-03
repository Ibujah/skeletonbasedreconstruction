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
 *  \file SkelTriangulation.cpp
 *  \brief Triangulates skeleton from projective skeletons
 *  \author Bastien Durix
 */

#include "SkelTriangulation.h"
#include <algorithm/fitbspline/Graph2Bspline.h>
#include <mathtools/geometry/euclidian/Line.h>
#include <mathtools/geometry/euclidian/LineTriangulation.h>

skeleton::BranchContSkel3d::Ptr algorithm::matchskeletons::BranchTriangulation(
		skeleton::ReconstructionBranch::Ptr recbranch,
		const std::vector<skeleton::BranchContProjSkel::Ptr> projbr,
		const OptionsTriang &options)
{
	std::vector<Eigen::Vector4d> pt_approx(recbranch->getMatch().size());

	for(unsigned int i=0; i<pt_approx.size(); i++)
	{
		Eigen::Matrix<double,Eigen::Dynamic,1> match = recbranch->getMatch()[i];;
		std::vector<mathtools::geometry::euclidian::Line<4> > line(match.rows());
		for(unsigned int j = 0; j<match.rows(); j++)
		{
			line[j] = projbr[j]->getNode<mathtools::geometry::euclidian::Line<4> >(match[i]);
		}
		pt_approx[i] = mathtools::geometry::euclidian::Triangulate<4>(line).getCoords();
	}
	
	skeleton::BranchGraphSkel3d::Ptr grbr(new skeleton::BranchGraphSkel3d(skeleton::model::Classic<3>(),pt_approx));

	return algorithm::fitbspline::Graph2Bspline(grbr,options.degree);
}

skeleton::CompContSkel3d::Ptr algorithm::matchskeletons::ComposedTriangulation(
		skeleton::ReconstructionSkeleton::Ptr recskel,
		const std::vector<skeleton::CompContProjSkel::Ptr> projskel,
		const algorithm::matchskeletons::OptionsTriang &options)
{
	skeleton::CompContSkel3d::Ptr compskel(new skeleton::CompContSkel3d());
	
	std::list<unsigned int> l_node;
	recskel->getAllNodes(l_node);

	for(std::list<unsigned int>::iterator it = l_node.begin(); it != l_node.end(); it++)
	{
		compskel->addNode(*it);
	}

	std::list<unsigned int> l_edge;
	recskel->getAllEdges(l_edge);

	for(std::list<unsigned int>::iterator it = l_edge.begin(); it != l_edge.end(); it++)
	{
		skeleton::ReconstructionBranch::Ptr recbr = recskel->getBranch(*it);
		std::vector<skeleton::BranchContProjSkel::Ptr> branches(recbr->getFirstExt().size());
		for(unsigned int i=0;i<branches.size();i++)
		{
			branches[i] = projskel[i]->getBranch(recbr->getFirstExt()[i],recbr->getLastExt()[i]);
		}
		
		skeleton::BranchContSkel3d::Ptr br = BranchTriangulation(recbr,branches,options);
		
		std::pair<unsigned int, unsigned int> ext = recskel->getExtremities(*it);

		compskel->addEdge(ext.first,ext.second,br);
	}
	return compskel;
}
