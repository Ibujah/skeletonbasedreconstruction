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
 *  \file ComputeNodeVector.cpp
 *  \brief Computes node from vector for Bsplines
 *  \author Bastien Durix
 */

#include "ComputeNodeVector.h"


unsigned int algorithm::fitbspline::MaxDegree(const unsigned int nb_approx)
{
	return nb_approx-1;
}

unsigned int algorithm::fitbspline::MaxCtrlPts(const unsigned int &nb_approx, const unsigned int &degree)
{
	return nb_approx-degree+1;
}

unsigned int algorithm::fitbspline::MinCtrlPts(const unsigned int &degree)
{
	return degree+1;
}

Eigen::Matrix<double,1,Eigen::Dynamic> algorithm::fitbspline::ComputeNodeVec(const Eigen::Matrix<double,1,Eigen::Dynamic> &vec_approxnod, const unsigned int &degree, const unsigned int &nb_ctr_pt)
{
	unsigned int nb_nod = nb_ctr_pt + degree - 1;
	Eigen::Matrix<double,1,Eigen::Dynamic> vec_nod(1,nb_nod);
	
	for(unsigned int i=0; i<degree; i++)
	{
		vec_nod(0,i) = vec_approxnod(0,0);
		vec_nod(0,vec_nod.cols()-1-i) = vec_approxnod(0,vec_approxnod.cols()-1);
	}
	
	for(unsigned int i = 0; i<nb_nod-2*degree; i++)
	{
		double prop = ((double)i+1.0)/(double)(nb_nod-2*degree+2);
		unsigned int ind = prop*(vec_approxnod.size()-1);
		vec_nod(0,degree+i) = vec_approxnod[ind];
	}

	return vec_nod;
}

