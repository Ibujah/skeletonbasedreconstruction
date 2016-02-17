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
 *  \file Orthographic.cpp
 *  \brief Defines orthographic projective skeleton
 *  \author Bastien Durix
 */

#include "Orthographic.h"

skeleton::model::Orthographic::Orthographic(const mathtools::affine::Frame<3>::Ptr frame) : skeleton::model::Projective(frame)
{}

skeleton::model::Orthographic::Orthographic(const Orthographic &model) : skeleton::model::Projective(model.m_frame)
{}

skeleton::model::Projective::Type skeleton::model::Orthographic::getType() const
{
	return skeleton::model::Projective::Type::orthographic;
}

double skeleton::model::Orthographic::getSize(const Eigen::Matrix<double,meta<Projective>::stordim,1> &vec) const
{
	return vec(2);
}
