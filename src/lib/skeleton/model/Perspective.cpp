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
 *  \file Perspective.cpp
 *  \brief Defines perspective projective skeleton
 *  \author Bastien Durix
 */

#include "Perspective.h"
#include <mathtools/application/Compositor.h>
#include <mathtools/application/LinearApp.h>
#include <mathtools/application/Coord2Homog.h>

using namespace mathtools::application;

constexpr unsigned int skeleton::model::meta<skeleton::model::Perspective>::stordim;

skeleton::model::Perspective::Perspective(const mathtools::affine::Frame<2>::Ptr frame2, const mathtools::affine::Frame<3>::Ptr frame3) : skeleton::model::Projective(frame2,frame3)
{
	Eigen::Matrix3d basis3 = frame3->getBasis()->getMatrix();
	Eigen::Vector3d ori3   = frame3->getOrigin();
	Eigen::Matrix<double,8,4> retromat;
	retromat << 0.0			 , 0.0 			, 0.0 , ori3(0)	 ,
				0.0 		 , 0.0  		, 0.0 , ori3(1)	 ,
				0.0 		 , 0.0  		, 0.0 , ori3(2)	 ,
				0.0 		 , 0.0  		, 0.0 , 0.0		 ,
				basis3(0,0)  , basis3(0,1)	, 0.0 , basis3(0,2) ,
				basis3(1,0)  , basis3(1,1)	, 0.0 , basis3(1,2) ,
				basis3(2,0)  , basis3(2,1)	, 0.0 , basis3(2,2) ,
				0.0			 , 0.0			, 1.0 , 0.0			 ;
	
	Compositor<LinearApp<8,4>,Coord2Homog<3> >::Ptr comp(new Compositor<LinearApp<8,4>,Coord2Homog<3> >(LinearApp<8,4>(retromat),Coord2Homog<3>()));
	
	m_r8fun = std::static_pointer_cast<Application<Eigen::Matrix<double,8,1>,Eigen::Vector3d> >(comp);
}

skeleton::model::Perspective::Perspective(const Perspective &model) : skeleton::model::Projective(model.m_frame2,model.m_frame3)
{}

skeleton::model::Projective::Type skeleton::model::Perspective::getType() const
{
	return skeleton::model::Projective::Type::perspective;
}

double skeleton::model::Perspective::getSize(const Eigen::Matrix<double,meta<Projective>::stordim,1> &vec) const
{
	return vec(2)/sqrt(vec(0)*vec(0) + vec(1)*vec(1) + 1.0*1.0);
}

Eigen::Matrix<double,skeleton::model::meta<skeleton::model::Perspective>::stordim,1> skeleton::model::Perspective::resize(const Eigen::Matrix<double,meta<Perspective>::stordim,1> &vec, double size) const
{
	Eigen::Matrix<double,meta<Projective>::stordim,1> resized = vec;
	resized(meta<Projective>::stordim-1,0) *= size;
	return resized;
}

bool skeleton::model::Perspective::included(const Eigen::Matrix<double,meta<Perspective>::stordim,1> &vec1, const Eigen::Matrix<double,meta<Perspective>::stordim,1> &vec2) const
{
	double nor1 = sqrt(vec1(0)*vec1(0) + vec1(1)*vec1(1) + 1.0*1.0); 
	double nor2 = sqrt(vec2(0)*vec2(0) + vec2(1)*vec2(1) + 1.0*1.0); 
	
	// construction of point projected on sphere
	Eigen::Vector3d ctr1(vec1(0)/nor1,vec1(1)/nor1,1.0/nor1);
	Eigen::Vector3d ctr2(vec2(0)/nor2,vec2(1)/nor2,1.0/nor2);

	double rad1 = vec1(2)/nor1;
	double rad2 = vec2(2)/nor2;

	return ((ctr1-ctr2).norm()+rad2 <= rad1);
}

mathtools::affine::Point<2> skeleton::model::Perspective::toObj(
		const Eigen::Matrix<double,skeleton::model::meta<skeleton::model::Projective>::stordim,1> &vec,
		const mathtools::affine::Point<2> &) const
{
	return mathtools::affine::Point<2>(vec(0),vec(1),m_frame2);
}

mathtools::geometry::euclidian::Line<4> skeleton::model::Perspective::toObj(
		const Eigen::Matrix<double,skeleton::model::meta<skeleton::model::Projective>::stordim,1> &vec,
		const mathtools::geometry::euclidian::Line<4> &) const
{
	Eigen::Vector4d origin;
	origin.block<3,1>(0,0) = m_frame3->getOrigin();
	origin(3) = 0.0;

	Eigen::Vector4d vecdir;
	vecdir.block<3,1>(0,0) = m_frame3->getBasis()->getMatrix()*Eigen::Vector3d(vec(0),vec(1),1.0);
	vecdir(3) = vec(2);
	vecdir.normalize();

	return mathtools::geometry::euclidian::Line<4>(origin,vecdir);
}
