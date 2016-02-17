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

skeleton::model::Perspective::Perspective(const mathtools::affine::Frame<3>::Ptr frame) : skeleton::model::Projective(frame)
{}

skeleton::model::Perspective::Perspective(const Perspective &model) : skeleton::model::Projective(model.m_frame)
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
