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
 *  \file ContinuousSkinning.cpp
 *  \brief Computes a discrete 3D boundary associatd to a skeleton
 *  \author Bastien Durix
 */

#include "ContinuousSkinning.h"
#include <vector>
#include <mathtools/vectorial/Basis.h>
#include <mathtools/geometry/euclidian/HyperPlane.h>
#include <mathtools/geometry/euclidian/HyperSphere.h>
#include <mathtools/geometry/euclidian/HyperCircle.h>
#include <mathtools/geometry/euclidian/Intersection.h>

using namespace mathtools::geometry::euclidian;
using namespace mathtools::vectorial;
using namespace mathtools::affine;

inline HyperSphere<3> ComputeSphere(const Eigen::Vector4d &val, const Frame<3>::Ptr frame)
{
	return HyperSphere<3>(Point<3>(val.block<3,1>(0,0),frame),val(3));
}

inline HyperPlane<3> ComputeCaractPlane(const Eigen::Vector4d &val, const Eigen::Vector4d &der, const Basis<3>::Ptr basis)
{
	Eigen::Vector3d normal = basis->getMatrix()*der.block<3,1>(0,0);
	Eigen::Matrix4d coeff = Eigen::Matrix4d::Identity();
	coeff(3,3)=-1;

	double factor = der.transpose()*coeff*val;

	return HyperPlane<3>(factor,basis->getMatrix()*normal);
}

inline Eigen::Matrix3d ComputeFrenetBasis(const Eigen::Vector4d &der, const Eigen::Vector4d &der2, const Basis<3>::Ptr basis)
{
	Eigen::Matrix3d frenet_basis;
	frenet_basis.block<3,1>(0,0) = der.block<3,1>(0,0).normalized();
	frenet_basis.block<3,1>(0,1) = der2.block<3,1>(0,0)*(1 - der2.block<3,1>(0,0).dot(frenet_basis.block<3,1>(0,0)));
	frenet_basis.block<3,1>(0,2) = frenet_basis.block<3,1>(0,0).cross(frenet_basis.block<3,1>(0,1));
	return basis->getMatrix()*frenet_basis;
}

inline void AdjustFrenetBasis(const Eigen::Matrix3d &frenet_basis_prev, Eigen::Matrix3d &frenet_basis)
{
	double scal = frenet_basis_prev.block<3,1>(0,1).dot(frenet_basis.block<3,1>(0,1));
	
	if(scal < 0)
		frenet_basis.block<3,2>(0,1) *= -1.0;
}

inline void LinkCircles(const std::vector<unsigned int> &prev_pts, const std::vector<unsigned int> &cur_pts, boundary::DiscreteBoundary<3>::Ptr grbnd)
{
	unsigned int nb_div_circ = prev_pts.size();
	for(unsigned int j = 0; j<nb_div_circ; j++)
	{
		grbnd->addEdge(prev_pts[j],prev_pts[(j+1)%nb_div_circ]);
		grbnd->addEdge(cur_pts[j],cur_pts[(j+1)%nb_div_circ]);

		grbnd->addEdge(prev_pts[j],cur_pts[j]);
		grbnd->addEdge(prev_pts[(j+1)%nb_div_circ],cur_pts[(j+1)%nb_div_circ]);

		grbnd->addEdge(prev_pts[(j+1)%nb_div_circ],cur_pts[j]);

		grbnd->addFace(prev_pts[j],prev_pts[(j+1)%nb_div_circ],cur_pts[j]);
		grbnd->addFace(cur_pts[j],prev_pts[(j+1)%nb_div_circ],cur_pts[(j+1)%nb_div_circ]);
	}
}

void ComputeCircles(const skeleton::BranchContSkel3d::Ptr contbr,
					const algorithm::skinning::OptionsContSkinning &options,
					std::list<HyperSphere<3> > &list_sph,
					std::list<HyperCircle<3> > &list_cir,
					std::list<Basis<3>::Ptr> &list_basis)
{
	Eigen::Matrix3d frenet_basis_prev;

	for(unsigned int i = 0; i < options.nbcer; i++)
	{
		double t = (double)i/(double)(options.nbcer-1);

		Eigen::Vector4d val = contbr->getCompFun()->operator()(t);
		Eigen::Vector4d der = Eigen::Map<Eigen::Vector4d>((double*)contbr->getCompFun()->der(t).data());
		Eigen::Vector4d der2 = Eigen::Map<Eigen::Vector4d>((double*)contbr->getCompFun()->der2(t).data());

		HyperSphere<3> sphere = ComputeSphere(val,contbr->getModel()->getFrame());
		HyperPlane<3> plane = ComputeCaractPlane(val,der,contbr->getModel()->getFrame()->getBasis());
		Eigen::Matrix3d frenet_basis = ComputeFrenetBasis(der,der2,contbr->getModel()->getFrame()->getBasis());
		HyperCircle<3> circle;
		
		if(i != 0)
		{
			AdjustFrenetBasis(frenet_basis_prev,frenet_basis);
		}
		frenet_basis_prev = frenet_basis;

		if(mathtools::geometry::euclidian::Intersection(plane,sphere,circle))
		{
			list_sph.push_back(sphere);
			list_cir.push_back(circle);
			list_basis.push_back(Basis<3>::CreateBasis(frenet_basis));
		}
	}
}

void ComputeExt(const algorithm::skinning::OptionsContSkinning &options,
				const bool first,
				std::list<HyperSphere<3> > &list_sph,
				std::list<HyperCircle<3> > &list_cir,
				std::list<Basis<3>::Ptr> &list_basis,
				Point<3> &ptext,
				Eigen::Vector3d &norext)
{
	double dir = first?-1.0:1.0;
	HyperSphere<3> sph = first?*(list_sph.begin()):*(list_sph.rbegin());
	HyperCircle<3> cir = first?*(list_cir.begin()):*(list_cir.rbegin());
	Basis<3>::Ptr basis = first?*(list_basis.begin()):*(list_basis.rbegin());
	
	norext = dir * basis->getMatrix().block<3,1>(0,0);
	ptext = sph.getCenter() + sph.getRadius() * norext;
	
	unsigned int nbcer = options.nbcer/4;

	for(unsigned int i = 0; i < nbcer; i++)
	{
		Point<3> ctr = ptext + (double)(i+1)*(cir.getCenter() - ptext)/(double)(nbcer+1);
		double radius = sqrt(pow(sph.getRadius(),2) + (sph.getCenter()-ctr).squaredNorm());
		if(first)
		{
			list_sph.push_front(sph);
			list_cir.push_front(HyperCircle<3>(ctr,radius,cir.getNormal()));
			list_basis.push_front(basis);
		}
		else
		{
			list_sph.push_back(sph);
			list_cir.push_back(HyperCircle<3>(ctr,radius,cir.getNormal()));
			list_basis.push_back(basis);
		}
	}
}

void DiscretizeCircle(const HyperSphere<3> &sph,
					  const HyperCircle<3> &cir,
					  const Basis<3>::Ptr basis,
					  const algorithm::skinning::OptionsContSkinning &options,
					  std::vector<Point<3> > &vec_pts,
					  std::vector<Eigen::Vector3d> &vec_nor)
{
	vec_pts.resize(options.nbpt);
	vec_nor.resize(options.nbpt);
	for(unsigned int i = 0; i < options.nbpt; i++)
	{
		double theta = 2*M_PI*(double)i/(double)options.nbpt;
		double costheta = cos(theta),
		   	   sintheta = sin(theta);
		Point<3> pt = cir.getCenter() + cir.getRadius()*(costheta*basis->getMatrix().block<3,1>(0,1)+sintheta*basis->getMatrix().block<3,1>(0,2));
		Eigen::Vector3d nor = (pt - sph.getCenter()).normalized();
		vec_pts[i] = pt;
		vec_nor[i] = nor;
	}
}

void ContinuousSkinning_helper(boundary::DiscreteBoundary<3>::Ptr disbnd,
							   const skeleton::BranchContSkel3d::Ptr contbr,
							   const algorithm::skinning::OptionsContSkinning &options)
{
	std::list<HyperSphere<3> > list_sph;
	std::list<HyperCircle<3> > list_cir;
	std::list<Basis<3>::Ptr> list_basis;
	Point<3> ptext1, ptext2;
	Eigen::Vector3d norext1, norext2;
	
	ComputeCircles(contbr,options,list_sph,list_cir,list_basis);
	ComputeExt(options,true,list_sph,list_cir,list_basis,ptext1,norext1);
	ComputeExt(options,false,list_sph,list_cir,list_basis,ptext2,norext2);
	
	std::list<HyperSphere<3> >::iterator itsph = list_sph.begin();
	std::list<HyperCircle<3> >::iterator itcir = list_cir.begin();
	std::list<Basis<3>::Ptr>::iterator itbasis = list_basis.begin();
	
	unsigned int indext1 = disbnd->addPoint(ptext1,norext1);
	unsigned int indext2 = disbnd->addPoint(ptext2,norext2);
	std::vector<unsigned int> vec_indprev(options.nbpt);

	for(; itsph != list_sph.end(); itsph++, itcir++, itbasis++)
	{
		std::vector<Point<3> > vec_pts(options.nbpt);
		std::vector<Eigen::Vector3d> vec_nor(options.nbpt);
		std::vector<unsigned int> vec_ind(options.nbpt);
		DiscretizeCircle(*itsph,*itcir,*itbasis,options,vec_pts,vec_nor);
		
		for(unsigned int i = 0; i < options.nbpt; i++)
		{
			vec_ind[i] = disbnd->addPoint(vec_pts[i],vec_nor[i]);
		}

		if(itsph == list_sph.begin())
		{
			for(unsigned int i = 0; i < options.nbpt; i++)
			{
				disbnd->addEdge(indext1,vec_ind[i]);
				disbnd->addFace(vec_ind[i],indext1,vec_ind[(i+1)%options.nbpt]);
			}
		}
		else
		{
			LinkCircles(vec_indprev,vec_ind,disbnd);
		}
		vec_indprev = vec_ind;
	}
	for(unsigned int i = 0; i < options.nbpt; i++)
	{
		disbnd->addEdge(indext2,vec_indprev[i]);
		disbnd->addFace(indext2,vec_indprev[i],vec_indprev[(i+1)%options.nbpt]);
	}
}

boundary::DiscreteBoundary<3>::Ptr algorithm::skinning::ContinuousSkinning(const skeleton::BranchContSkel3d::Ptr contbr, const OptionsContSkinning &options)
{
	boundary::DiscreteBoundary<3>::Ptr disbnd(new boundary::DiscreteBoundary<3>());
	
	ContinuousSkinning_helper(disbnd,contbr,options);
	
	return disbnd;
}

boundary::DiscreteBoundary<3>::Ptr algorithm::skinning::ContinuousSkinning(const skeleton::CompContSkel3d::Ptr contskl, const OptionsContSkinning &options)
{
	boundary::DiscreteBoundary<3>::Ptr disbnd(new boundary::DiscreteBoundary<3>());
	
	std::list<unsigned int> list_edge;
	contskl->getAllEdges(list_edge);
	
	for(std::list<unsigned int>::iterator it = list_edge.begin(); it != list_edge.end(); it++)
	{
		ContinuousSkinning_helper(disbnd,contskl->getBranch(*it),options);
	}
	
	return disbnd;
}
