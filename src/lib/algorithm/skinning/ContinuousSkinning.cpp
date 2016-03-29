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
#include <Eigen/SVD>
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

	double factor = der.transpose()*(1.0/normal.norm())*coeff*val;

	return HyperPlane<3>(factor,normal.normalized());
}

inline Eigen::Matrix3d ComputeFrenetBasis(const Eigen::Vector4d &der, const Eigen::Vector4d &der2, const Basis<3>::Ptr basis)
{
	Eigen::Matrix3d frenet_basis;
	frenet_basis.block<3,1>(0,0) = der.block<3,1>(0,0).normalized();
	frenet_basis.block<3,1>(0,1) = (der2.block<3,1>(0,0) - der2.block<3,1>(0,0).dot(frenet_basis.block<3,1>(0,0))*frenet_basis.block<3,1>(0,0)).normalized();
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

void CompleteFrenetBasisBegining(const std::list<HyperCircle<3> > &list_cir, std::list<Basis<3>::Ptr> &list_basis)
{
	std::list<HyperCircle<3> >::const_reverse_iterator itcir = list_cir.rend();
	std::list<Basis<3>::Ptr >::reverse_iterator itbas = list_basis.rend();
	
	do
	{
		itcir--;
		itbas--;
	}while((*itbas) == NULL);
	
	Eigen::Vector3d nor = (*itbas)->getMatrix().block<3,1>(0,1);
	
	for(; itbas != list_basis.rend(); itbas++, itcir++)
	{
		Eigen::Vector3d tgt = itcir->getNormal().normalized();
		Eigen::Vector3d binor = tgt.cross(nor).normalized();
		nor = binor.cross(tgt).normalized();
		
		Basis<3>::Ptr basis = Basis<3>::CreateBasis(tgt,nor,binor);
		*itbas = basis;
	}
}

void CompleteFrenetBasisEnding(const std::list<HyperCircle<3> > &list_cir, std::list<Basis<3>::Ptr> &list_basis)
{
	std::list<HyperCircle<3> >::const_iterator itcir = list_cir.end();
	std::list<Basis<3>::Ptr >::iterator itbas = list_basis.end();
	
	do
	{
		itcir--;
		itbas--;
	}while((*itbas) == NULL);
	
	Eigen::Vector3d nor = (*itbas)->getMatrix().block<3,1>(0,1);
	
	for(; itbas != list_basis.end(); itbas++, itcir++)
	{
		Eigen::Vector3d tgt = itcir->getNormal().normalized();
		Eigen::Vector3d binor = tgt.cross(nor).normalized();
		nor = binor.cross(tgt).normalized();
		
		Basis<3>::Ptr basis = Basis<3>::CreateBasis(tgt,nor,binor);
		*itbas = basis;
	}
}

void CompleteFrenetBasis(const std::list<HyperCircle<3> > &list_cir, std::list<Basis<3>::Ptr> &list_basis)
{
	std::list<HyperCircle<3> >::const_iterator itcirbeg = list_cir.begin();
	std::list<Basis<3>::Ptr >::iterator itbasbeg = list_basis.begin();
	
	bool finished = true;
	
	while(itbasbeg != list_basis.end() && finished)
	{
		if(std::next(itbasbeg) == list_basis.end())
		{
			itbasbeg++;
			itcirbeg++;
		}
		else if(!(*std::next(itbasbeg)))
		{
			finished = false;
		}
		else
		{
			itbasbeg++;
			itcirbeg++;
		}
	}
	
	while(!finished)
	{
		std::list<HyperCircle<3> >::const_iterator itcirend = itcirbeg;
		std::list<Basis<3>::Ptr >::iterator itbasend = itbasbeg;
		
		do
		{
			itcirend++;
			itbasend++;
		}while(!(*itbasend));
		
		Eigen::Vector3d norend = (*itbasend)->getMatrix().block<3,1>(0,1);
		
		Eigen::Vector3d coordsend = (*itbasbeg)->getMatrixInverse() * norend;
		Basis<3>::Ptr basbeg = (*itbasbeg);
		
		coordsend *= (1.0/coordsend.block<2,1>(1,0).norm());

		double angleend = atan2(coordsend.z(),coordsend.y());
		if(angleend > M_PI) angleend -= 2*M_PI;
		
		Eigen::Vector3d ptbeg = itcirbeg->getCenter().getCoords();
		Eigen::Vector3d ptend = itcirend->getCenter().getCoords();
		
		itcirbeg++;
		itbasbeg++;
		
		for(;itbasbeg != itbasend; itbasbeg++, itcirbeg++)
		{
			Eigen::Vector3d ptcur = itcirbeg->getCenter().getCoords();
			double prop = (ptcur - ptbeg).norm() / ((ptcur - ptbeg).norm() + (ptend - ptcur).norm());
			
			double anglecur = prop * angleend;

			Eigen::Vector3d norcur = basbeg->getMatrix() * Eigen::Vector3d(0.0,cos(anglecur),sin(anglecur));
			
			Eigen::Vector3d tgt = itcirbeg->getNormal().normalized();
			Eigen::Vector3d binor = tgt.cross(norcur).normalized();
			norcur = binor.cross(tgt).normalized();
			
			Basis<3>::Ptr basis = Basis<3>::CreateBasis(tgt,norcur,binor);
			*itbasbeg = basis;
		}
		
		finished = true;
		
		while(itbasbeg != list_basis.end() && finished)
		{
			if(std::next(itbasbeg) == list_basis.end())
			{
				itbasbeg++;
				itcirbeg++;
			}
			else if(!(*std::next(itbasbeg)))
			{
				finished = false;
			}
			else
			{
				itbasbeg++;
				itcirbeg++;
			}
		}
	}
}

void FillFrenetBasis(const std::list<HyperCircle<3> > &list_cir, std::list<Basis<3>::Ptr> &list_basis)
{
	Eigen::Matrix<double,3,Eigen::Dynamic> matpt(3,list_cir.size());
	Eigen::Vector3d mean = Eigen::Vector3d::Zero();
	
	unsigned int numpt = 0;
	for(std::list<HyperCircle<3> >::const_iterator it = list_cir.begin(); it != list_cir.end(); it++)
	{
		matpt.block<3,1>(0,numpt) = it->getCenter().getCoords();
		mean += it->getCenter().getCoords();
		numpt++;
	}
	
	mean *= (1.0/(double)numpt);
	matpt -= mean * Eigen::Matrix<double,1,Eigen::Dynamic>::Ones(1,list_cir.size());
	
	Eigen::JacobiSVD<Eigen::Matrix<double,3,Eigen::Dynamic> > svd(matpt,Eigen::ComputeThinU);
	svd.computeU();
	Eigen::Matrix3d U = svd.matrixU();
	
	Eigen::Vector3d nor = U.block<3,1>(0,2);
	
	std::list<HyperCircle<3> >::const_iterator itcir = list_cir.begin();
	std::list<Basis<3>::Ptr >::iterator itbas = list_basis.begin();
	
	for(; itbas != list_basis.end(); itbas++, itcir++)
	{
		Eigen::Vector3d tgt = itcir->getNormal().normalized();
		Eigen::Vector3d binor = tgt.cross(nor).normalized();
		nor = binor.cross(tgt).normalized();
		
		Basis<3>::Ptr basis = Basis<3>::CreateBasis(tgt,nor,binor);
		*itbas = basis;
	}
}

void CheckFrenetBasis(const std::list<HyperCircle<3> > &list_cir, std::list<Basis<3>::Ptr> &list_basis)
{
	// cases
	// 1 - list_basis is empty
	// 2 - list_basis misses some elements (not extremities)
	// 3 - list_basis misses 1 or 2 extremities
	// 4 - list_basis is full
	
	bool full = true;
	bool empty = true;
	
	for(std::list<Basis<3>::Ptr>::iterator it = list_basis.begin(); it != list_basis.end(); it++)
	{
		if((*it).operator bool())
		{
			empty = false;
		}
		else
		{
			full = false;
		}
	}
	
	bool miss_beg = !(*(list_basis.begin()));
	bool miss_end = !(*(list_basis.rbegin()));

	if(empty)
	{
		FillFrenetBasis(list_cir,list_basis);
	}
	else if(!full)
	{
		if(miss_beg)
			CompleteFrenetBasisBegining(list_cir,list_basis);
		if(miss_end)
			CompleteFrenetBasisEnding(list_cir,list_basis);
		CompleteFrenetBasis(list_cir,list_basis);
	}
}

void ComputeCirclesFrenet(const skeleton::BranchContSkel3d::Ptr contbr,
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
		
		if(list_sph.size())
		{
			AdjustFrenetBasis(frenet_basis_prev,frenet_basis);
		}
		frenet_basis_prev = frenet_basis;

		if(mathtools::geometry::euclidian::Intersection(plane,sphere,circle))
		{
			list_sph.push_back(sphere);
			list_cir.push_back(circle);
			Basis<3>::Ptr basis;
			if(der2.block<3,1>(0,0).norm() != 0.0)
			{
				try
				{
					basis = Basis<3>::CreateBasis(frenet_basis);
				}
				catch(...)
				{}
			}
			list_basis.push_back(basis);
		}
	}
	CheckFrenetBasis(list_cir,list_basis);
}

void ComputeCirclesProjBasis(const skeleton::BranchContSkel3d::Ptr contbr,
							 const algorithm::skinning::OptionsContSkinning &options,
							 std::list<HyperSphere<3> > &list_sph,
							 std::list<HyperCircle<3> > &list_cir,
							 std::list<Basis<3>::Ptr> &list_basis)
{
	for(unsigned int i = 0; i < options.nbcer; i++)
	{
		double t = (double)i/(double)(options.nbcer-1);

		Eigen::Vector4d val = contbr->getCompFun()->operator()(t);
		Eigen::Vector4d der = Eigen::Map<Eigen::Vector4d>((double*)contbr->getCompFun()->der(t).data());
		Eigen::Vector4d der2 = Eigen::Map<Eigen::Vector4d>((double*)contbr->getCompFun()->der2(t).data());

		HyperSphere<3> sphere = ComputeSphere(val,contbr->getModel()->getFrame());
		HyperPlane<3> plane = ComputeCaractPlane(val,der,contbr->getModel()->getFrame()->getBasis());
		HyperCircle<3> circle;
		
		if(mathtools::geometry::euclidian::Intersection(plane,sphere,circle))
		{
			list_sph.push_back(sphere);
			list_cir.push_back(circle);
			list_basis.push_back(Basis<3>::Ptr());
		}
	}
	FillFrenetBasis(list_cir,list_basis);
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
	
	unsigned int nbcer = options.nbcer/options.fracnbcer;

	for(unsigned int i = 0; i < nbcer; i++)
	{
		Point<3> ctr = cir.getCenter() + (double)(i+1)*(ptext - cir.getCenter())/(double)(nbcer+1);
		double radius = sqrt(pow(sph.getRadius(),2) - (sph.getCenter()-ctr).squaredNorm());
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
	
	switch(options.computebasis)
	{
		case algorithm::skinning::OptionsContSkinning::enum_computebasis::projbasis :
			ComputeCirclesProjBasis(contbr,options,list_sph,list_cir,list_basis);
			break;
		case algorithm::skinning::OptionsContSkinning::enum_computebasis::frenet :
			ComputeCirclesFrenet(contbr,options,list_sph,list_cir,list_basis);
			break;
		
	}
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
		vec_indprev.assign(vec_ind.begin(),vec_ind.end());
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
