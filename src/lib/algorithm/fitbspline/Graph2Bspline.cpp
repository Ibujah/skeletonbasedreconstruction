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
 *  \file Graph2Bspline.cpp
 *  \brief Converts graph skeleton to bspline continuous skeleton
 *  \author Bastien Durix
 */

#include "Graph2Bspline.h"

#include "FitBspline.h"
#include "ComputeNodeVector.h"


template<typename Model>
typename skeleton::ContinuousBranch<Model>::Ptr Graph2Bspline_helper(const typename skeleton::GraphBranch<Model>::Ptr grskel, unsigned int degree, double propctrl)
{
	typename mathtools::application::Application<typename Model::Stor,double>::Ptr bspline;
	std::vector<typename Model::Stor> nodes(0);

	grskel->getAllNodes(nodes);

	if(nodes.size() == 1)
	{
		Eigen::Matrix<double,skeleton::model::meta<Model>::stordim,Eigen::Dynamic> ctrl_vec(skeleton::model::meta<Model>::stordim,2);
		Eigen::Matrix<double,1,Eigen::Dynamic> node_vec(1,2);

		ctrl_vec.block(0,0,skeleton::model::meta<Model>::stordim,1) = nodes[0];
		ctrl_vec.block(0,1,skeleton::model::meta<Model>::stordim,1) = nodes[0];
		node_vec(0,0) = 0;
		node_vec(0,1) = 1;

		bspline = typename mathtools::application::Application<typename Model::Stor,double>::Ptr(
				new mathtools::application::Bspline<skeleton::model::meta<Model>::stordim>(ctrl_vec,node_vec,1));
	}
	else
	{
		Eigen::Matrix<double,1,Eigen::Dynamic> vec_approx_nod = algorithm::fitbspline::ComputeApproxNodeVec<skeleton::model::meta<Model>::stordim>(nodes);

		unsigned int nb_ctrl = propctrl*nodes.size();

		unsigned int curv_degree = algorithm::fitbspline::MaxDegree(nodes.size());
		curv_degree = curv_degree<degree?curv_degree:degree;

		unsigned int max_nbctrl = algorithm::fitbspline::MaxCtrlPts(nodes.size(),curv_degree);
		nb_ctrl = max_nbctrl<nb_ctrl?max_nbctrl:nb_ctrl;

		unsigned int min_nbctrl = algorithm::fitbspline::MinCtrlPts(curv_degree);
		nb_ctrl = min_nbctrl>nb_ctrl?min_nbctrl:nb_ctrl;

		Eigen::Matrix<double,1,Eigen::Dynamic> vec_nod = algorithm::fitbspline::ComputeNodeVec(vec_approx_nod,curv_degree,nb_ctrl);

		bspline = typename mathtools::application::Application<typename Model::Stor,double>::Ptr(
				new mathtools::application::Bspline<skeleton::model::meta<Model>::stordim>(
					algorithm::fitbspline::FitBspline<skeleton::model::meta<Model>::stordim>(nodes,vec_approx_nod,vec_nod,curv_degree)));
	}

	return typename skeleton::ContinuousBranch<Model>::Ptr(new skeleton::ContinuousBranch<Model>(grskel->getModel(),bspline));
}

template<typename Model>
typename skeleton::ComposedCurveSkeleton<skeleton::ContinuousBranch<Model> >::Ptr Graph2Bspline_helper(
		const typename skeleton::ComposedCurveSkeleton<skeleton::GraphBranch<Model> >::Ptr grskel, unsigned int degree, double propctrl)
{
	typename skeleton::ComposedCurveSkeleton<skeleton::ContinuousBranch<Model> >::Ptr 
		contskl(new typename skeleton::ComposedCurveSkeleton<skeleton::ContinuousBranch<Model> >());
	
	std::vector<unsigned int> node(0);
	grskel->getAllNodes(node);

	std::vector<unsigned int> edge(0);
	grskel->getAllEdges(edge);

	for(unsigned int i=0;i<node.size();i++)
		contskl->addNode(node[i]);

	for(unsigned int i=0;i<edge.size();i++)
	{
		std::pair<unsigned int,unsigned int> ext = grskel->getExtremities(edge[i]);

		typename skeleton::ContinuousBranch<Model>::Ptr br = Graph2Bspline_helper<Model>(grskel->getBranch(ext.first,ext.second),degree,propctrl);
		
		contskl->addEdge(ext.first,ext.second,br);
	}

	return contskl;
}

skeleton::BranchContSkel2d::Ptr algorithm::fitbspline::Graph2Bspline(const skeleton::BranchGraphSkel2d::Ptr grskel, unsigned int degree, double propctrl)
{
	return Graph2Bspline_helper<skeleton::model::Classic<2> >(grskel,degree,propctrl);
}

skeleton::CompContSkel2d::Ptr algorithm::fitbspline::Graph2Bspline(const skeleton::CompGraphSkel2d::Ptr grskel, unsigned int degree, double propctrl)
{
	return Graph2Bspline_helper<skeleton::model::Classic<2> >(grskel,degree,propctrl);
}


skeleton::BranchContSkel3d::Ptr algorithm::fitbspline::Graph2Bspline(const skeleton::BranchGraphSkel3d::Ptr grskel, unsigned int degree, double propctrl)
{
	return Graph2Bspline_helper<skeleton::model::Classic<3> >(grskel,degree,propctrl);
}

skeleton::CompContSkel3d::Ptr algorithm::fitbspline::Graph2Bspline(const skeleton::CompGraphSkel3d::Ptr grskel, unsigned int degree, double propctrl)
{
	return Graph2Bspline_helper<skeleton::model::Classic<3> >(grskel,degree,propctrl);
}


skeleton::BranchContProjSkel::Ptr algorithm::fitbspline::Graph2Bspline(const skeleton::BranchGraphProjSkel::Ptr grskel, unsigned int degree, double propctrl)
{
	return Graph2Bspline_helper<skeleton::model::Projective>(grskel,degree,propctrl);
}

skeleton::CompContProjSkel::Ptr algorithm::fitbspline::Graph2Bspline(const skeleton::CompGraphProjSkel::Ptr grskel, unsigned int degree, double propctrl)
{
	return Graph2Bspline_helper<skeleton::model::Projective>(grskel,degree,propctrl);
}
