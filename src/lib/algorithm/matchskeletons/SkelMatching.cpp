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
 *  \file SkelMatching.cpp
 *  \brief Matches multiple skeletal branches
 *  \author Bastien Durix
 */

#include "SkelMatching.h"
#include <nlopt.hpp>
#include <mathtools/application/Compositor.h>

using namespace mathtools::application;

void Dist_grad(const Eigen::Matrix<double,Eigen::Dynamic,1> &t,
			   const std::vector<skeleton::BranchContProjSkel::Ptr> &skel,
			   double &value,
			   Eigen::Matrix<double,Eigen::Dynamic,1> &gradient)
{
	Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
	Eigen::Vector4d b = Eigen::Vector4d::Zero();
	value = 0.0;
	gradient = Eigen::Matrix<double,Eigen::Dynamic,1>::Zero(t.rows(),1);

	std::vector<Eigen::Vector4d> ori(skel.size());
	std::vector<Eigen::Vector4d> vec(skel.size());
	std::vector<Eigen::Vector4d> orijac(skel.size());
	std::vector<Eigen::Vector4d> vecjac(skel.size());
	std::vector<Eigen::Matrix4d> A_part_jac(skel.size());
	std::vector<Eigen::Vector4d> b_part_jac(skel.size());

	for(unsigned int i=0;i<skel.size();i++)
	{
		Compositor<Application<Eigen::Matrix<double,8,1>,Eigen::Vector3d>,Application<Eigen::Vector3d,double> >
			fun(skel[i]->getModel()->getR8Fun(),skel[i]->getCompFun());
		Eigen::Matrix<double,8,1> veclin = fun(t(i));
		Eigen::Matrix<double,8,1> jaclin = fun.jac(t(i));

		ori[i]  = veclin.block<4,1>(0,0);
		vec[i]  = veclin.block<4,1>(4,0).normalized();

		orijac[i] = jaclin.block<4,1>(0,0);
		vecjac[i] = jaclin.block<4,1>(4,0)*(1.0/veclin.block<4,1>(4,0).norm());

		Eigen::Matrix4d A_part = Eigen::Matrix4d::Identity() - vec[i]*vec[i].transpose();
		Eigen::Vector4d b_part = A_part*ori[i];

		A_part_jac[i] = - vecjac[i]*vec[i].transpose() - vec[i]*vecjac[i].transpose();
		b_part_jac[i] = A_part_jac[i] * ori[i] + A_part * orijac[i];

		A+=A_part;
		b+=b_part;
	}

	Eigen::Matrix4d A_inv = A.inverse();

	Eigen::Vector4d P = A_inv*b;
	for(unsigned int i=0;i<skel.size();i++)
	{
		Eigen::Vector4d P_jac = A_inv * A_part_jac[i] * P + A_inv * b_part_jac[i];

		double Dist = (P - ori[i]).squaredNorm() - pow((P - ori[i]).dot(vec[i]),2);

		gradient(i) =  2*(P_jac - orijac[i]).dot(P - ori[i])
					  -2*(P_jac - orijac[i]).dot(vec[i]) * (P - ori[i]).dot(vec[i])
					  -2*(P - ori[i]).dot(vecjac[i]) * (P - ori[i]).dot(vec[i]);

		value += Dist;
	}
}

//out of bounds
bool Oob(const Eigen::Matrix<double,Eigen::Dynamic,1> &q)
{
	for(unsigned int i=0;i<q.rows();i++)
	{
		if(q(i,0)>1.0) return true;
		if(q(i,0)<0.0) return true;
	}
	return false;
}

double SolveODE(
		std::list<double> &list_val,
		std::list<Eigen::Matrix<double,Eigen::Dynamic,1> > &list_pos,
		std::list<Eigen::Matrix<double,Eigen::Dynamic,1> > &list_vit,
		std::list<Eigen::Matrix<double,Eigen::Dynamic,1> > &list_grad,
		double &t_tot,
		double &v_tot,
		const std::vector<skeleton::BranchContProjSkel::Ptr> &skel,
		const Eigen::Matrix<double,Eigen::Dynamic,1> &q_init,
		const Eigen::Matrix<double,Eigen::Dynamic,1> &v_init,
		const double &lambda,
		const double &deltat)
{
	double value = 0.0;
	double valuef = 0.0;
	Eigen::Matrix<double,Eigen::Dynamic,1> gradient(q_init.rows(),1);
	Eigen::Matrix<double,Eigen::Dynamic,1> gradientf(q_init.rows(),1);
	v_tot = 0.0;
	t_tot = 0.0;

	Dist_grad(q_init,skel,valuef,gradientf);

	list_pos.push_back(q_init);
	list_vit.push_back(v_init);
	list_grad.push_back(gradientf);
	list_val.push_back(valuef);


	//first step, to avoid boundary problem :
	Eigen::Matrix<double,Eigen::Dynamic,1> q = q_init + v_init * deltat;
	Eigen::Matrix<double,Eigen::Dynamic,1> v = v_init;

	gradient = gradientf;
	value    = valuef;
	Dist_grad(q,skel,valuef,gradientf);

	while(!Oob(q) && t_tot<10.0)
	{
		list_pos.push_back(q);
		list_vit.push_back(v);
		list_grad.push_back(gradientf);
		list_val.push_back(valuef);

		value = valuef;
		gradient = gradientf;
		Dist_grad(q,skel,valuef,gradientf);

		double vnor = v.norm();

		double frac = deltat/vnor;
		v_tot+=(vnor*lambda + (value+valuef)/2.0)*frac;
		t_tot += frac;

		Eigen::Matrix<double,Eigen::Dynamic,1> acc = gradientf*(1.0/lambda);

		q = q + v*frac;
		v = v + acc*frac;
	}


	Eigen::Matrix<double,Eigen::Dynamic,1> qf = Eigen::Matrix<double,Eigen::Dynamic,1>::Ones(q.rows(),1);
	value = valuef;
	gradient = gradientf;
	Dist_grad(qf,skel,valuef,gradientf);

	double df = (q-qf).norm();

	v_tot+=df*lambda + ((valuef+value)/2.0)*df;

	list_pos.push_back(qf);
	list_vit.push_back(v);
	list_grad.push_back(gradientf);
	list_val.push_back(valuef);

	return df;
}

struct DataODE
{
	std::vector<skeleton::BranchContProjSkel::Ptr> skel;
	std::list<Eigen::Matrix<double,Eigen::Dynamic,1> > list_pos;
	std::list<Eigen::Matrix<double,Eigen::Dynamic,1> > list_vit;
	std::list<Eigen::Matrix<double,Eigen::Dynamic,1> > list_grad;
	std::list<double> list_val;
	double t_tot;
	double v_tot;
	double deltat;
	double lambda;
	std::vector<double> vit_init;
};

double minFunODE(const std::vector<double> &x, std::vector<double> &, void *dataFun)
{
	DataODE *data = (DataODE*) dataFun;

	data->list_pos.erase(data->list_pos.begin(),data->list_pos.end());
	data->list_vit.erase(data->list_vit.begin(),data->list_vit.end());
	data->list_grad.erase(data->list_grad.begin(),data->list_grad.end());
	data->list_val.erase(data->list_val.begin(),data->list_val.end());
	data->t_tot = 0.0;
	data->v_tot = 0.0;

	Eigen::Matrix<double,Eigen::Dynamic,1> v_init(x.size()+1,1);
	v_init *= 0;
	v_init(0,0) = 1.0;
	for(unsigned int i=0;i<x.size();i++)
	{
		v_init *= cos(x[i]);
		v_init(i+1,0) = sin(x[i]);
	}

	double val = SolveODE(
			data->list_val,
			data->list_pos,
			data->list_vit,
			data->list_grad,
			data->t_tot,
			data->v_tot,
			data->skel,
			Eigen::Matrix<double,Eigen::Dynamic,1>::Zero(v_init.size(),1),
			v_init,
			data->lambda,
			data->deltat);

	return val;
}

double FindSpeed(DataODE &data)
{
	Eigen::Matrix<double,Eigen::Dynamic,1> q = Eigen::Matrix<double,Eigen::Dynamic,1>::Zero(data.skel.size(),1); //starting point
	
	std::list<Eigen::Matrix<double,Eigen::Dynamic,1> > list_pos;
	
	nlopt::opt opt(nlopt::LN_COBYLA, data.skel.size()-1);
	
	std::vector<double> lb(data.skel.size()-1);
	std::vector<double> ub(data.skel.size()-1);
	std::vector<double> vit_init(data.skel.size()-1);
	for(unsigned int i = 0;i<lb.size();i++)
	{
		lb[i] = 0.0;
		ub[i] = M_PI/2.0;
		vit_init[i] = M_PI/4.0;
	}
	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);

	opt.set_min_objective(minFunODE, &data);
	
	opt.set_xtol_rel(1e-4);
	
	double minf;
	try
	{
		opt.optimize(vit_init, minf);
	}
	catch(...)
	{
	}

	data.vit_init = vit_init;
	
	std::vector<double> tmp;

	double val = minFunODE(vit_init,tmp, &data);
	
	return val;
}

void SkelMatchingOde(
		skeleton::ReconstructionBranch::Ptr recbranch,
		const std::vector<skeleton::BranchContProjSkel::Ptr> projbr,
		const algorithm::matchskeletons::OptionsMatch &options)
{
	DataODE data;
	data.skel = projbr;
	data.deltat = options.deltat;
	data.lambda = options.lambdamin;

	double minf;
	do{
		minf = FindSpeed(data);
		data.lambda+=options.lambdastep;
	}while(minf>options.deltat && data.lambda<options.lambdamax);
	
	std::vector<Eigen::Matrix<double,Eigen::Dynamic,1> > vectcoords;
	if(data.lambda<options.lambdamax)
	{
		vectcoords.resize(data.list_pos.size());
		unsigned int ind = 0;
		for(std::list<Eigen::Matrix<double,Eigen::Dynamic,1> >::iterator it = data.list_pos.begin(); it != data.list_pos.end() ;it++)
		{
			vectcoords[ind] = *it;
			ind++;
		}
	}
	else
	{
		vectcoords.resize(101);
		for(unsigned int i = 0; i<101; i++)
		{
			vectcoords[i] = Eigen::Matrix<double,Eigen::Dynamic,1>::Ones(projbr.size(),1)*((double)i/100.0);
		}
	}

	recbranch->setMatch(vectcoords);
}

void algorithm::matchskeletons::BranchMatching(
		skeleton::ReconstructionBranch::Ptr recbranch,
		const std::vector<skeleton::BranchContProjSkel::Ptr> projbr,
		const OptionsMatch &options)
{
	switch(options.methodmatch)
	{
		case OptionsMatch::enum_methodmatch::ode:
			SkelMatchingOde(recbranch,projbr,options);
			break;
	}
}

void algorithm::matchskeletons::ComposedMatching(
		skeleton::ReconstructionSkeleton::Ptr recskel,
		const std::vector<skeleton::CompContProjSkel::Ptr> projskel,
		const OptionsMatch &options)
{
	std::list<unsigned int> l_edge;
	recskel->getAllEdges(l_edge);

	for(std::list<unsigned int>::iterator it = l_edge.begin(); it != l_edge.end(); it++)
	{
		skeleton::ReconstructionBranch::Ptr recbr = recskel->getBranch(*it);
		std::pair<unsigned int,unsigned int> ext = recskel->getExtremities(*it);
		std::vector<skeleton::BranchContProjSkel::Ptr> projbr(recbr->getIndSkel().size());
		
		for(unsigned int i = 0; i < recbr->getIndSkel().size(); i++)
		{
			projbr[i] = projskel[recbr->getIndSkel()[i]]->getBranch(ext.first,ext.second);
		}

		BranchMatching(recbr,projbr,options);
	}
}
