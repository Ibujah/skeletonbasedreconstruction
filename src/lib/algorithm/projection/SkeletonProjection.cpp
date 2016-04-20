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
 *  \file SkeletonProjection.cpp
 *  \brief Computes the 2d projection of a 3D skeleton
 *  \author Bastien Durix
 */

#include "SkeletonProjection.h"
#include <skeleton/model/Orthographic.h>
#include <skeleton/model/Perspective.h>
#include <mathtools/application/Application.h>

template<typename Model>
skeleton::BranchContProjSkel::Ptr SkeletonProjection_helper(const skeleton::BranchContSkel3d::Ptr contbr, const camera::Camera::Ptr camera)
{
	skeleton::model::Projective::Ptr model(new Model(camera->getIntrinsics()->getFrame(),camera->getExtrinsics()->getFrame()));
	mathtools::application::Application<Eigen::Vector3d,double>::Ptr nodefunproj;
	
	// use a NURBS
	
	return skeleton::BranchContProjSkel::Ptr(new skeleton::BranchContProjSkel(model,nodefunproj));
}

skeleton::BranchContProjSkel::Ptr algorithm::projection::SkeletonProjection(const skeleton::BranchContSkel3d::Ptr contbr, const camera::Camera::Ptr camera)
{
	skeleton::BranchContProjSkel::Ptr projbr;
	
	switch(camera->getIntrinsics()->getType())
	{
		case camera::Intrinsics::Type::ortho:
			projbr = SkeletonProjection_helper<skeleton::model::Orthographic>(contbr,camera);
			break;
		case camera::Intrinsics::Type::pinhole:
			projbr = SkeletonProjection_helper<skeleton::model::Projective>(contbr,camera);
			break;
	}
	
	return projbr;
}

skeleton::CompContProjSkel::Ptr algorithm::projection::SkeletonProjection(const skeleton::CompContSkel3d::Ptr contskl, const camera::Camera::Ptr camera)
{
	skeleton::CompContProjSkel::Ptr projskl(new skeleton::CompContProjSkel());
	
	std::vector<unsigned int> node(0);
	contskl->getAllNodes(node);
	
	std::vector<unsigned int> edge(0);
	contskl->getAllEdges(edge);
	
	for(unsigned int i=0;i<node.size();i++)
		projskl->addNode(node[i]);
	
	for(unsigned int i=0;i<edge.size();i++)
	{
		std::pair<unsigned int,unsigned int> ext = contskl->getExtremities(edge[i]);
		
		skeleton::BranchContProjSkel::Ptr br = SkeletonProjection(contskl->getBranch(ext.first,ext.second),camera);
		
		projskl->addEdge(ext.first,ext.second,br);
	}
	
	return projskl;
}
