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
 *  \file ComputeNodeVector.h
 *  \brief Computes node from vector for Bsplines
 *  \author Bastien Durix
 */

#ifndef _COMPUTENODEVECTOR_H_
#define _COMPUTENODEVECTOR_H_

#include <Eigen/Dense>
#include <vector>
#include <iostream>

/**
 *  \brief Defines a lot of algorithms using skeletons!
 */
namespace algorithm
{
	/*
	 *  \brief Defines tools to compute Bsplines
	 */
	namespace bsplinefitting
	{
		/**
		 *  \brief Computes approximative node vector for a set of successive points
		 *  
		 *  \tparam Dim				space dimension
		 *  
		 *  \param vec_approx_pts   successive points to approxime
		 *  
		 *  \return node vector associated to vec_approx_pts
		 */
		template<unsigned int Dim>
		Eigen::Matrix<double,1,Eigen::Dynamic> ComputeApproxNodeVec(const std::vector<Eigen::Matrix<double,Dim,1> > &vec_approx_pts)
		{
			Eigen::Matrix<double,1,Eigen::Dynamic> vec_approxnod(1,vec_approx_pts.size());

			vec_approxnod(0,0) = 0.0;
			
			for(unsigned int i=1;i<vec_approx_pts.size();i++)
			{
				vec_approxnod(0,i) = vec_approxnod(0,i-1) + (vec_approx_pts[i]-vec_approx_pts[i-1]).norm();
			}
			double max = vec_approxnod(0,vec_approx_pts.size()-1);
			vec_approxnod*=(1.0/max);

			return vec_approxnod;
		}
		
		/**
		 *  \brief Computes maximum degree
		 *
		 *  \param nb_approx  number of points to approxime
		 *
		 *  \return Maximum degree
		 */
		unsigned int MaxDegree(const unsigned int nb_approx);

		/**
		 *  \brief Computes maximum number of control points
		 *
		 *  \param nb_approx  number of points to approxime
		 *  \param degree     bspline degree
		 *
		 *  \return Maximum number of control points
		 */
		unsigned int MaxCtrlPts(const unsigned int &nb_approx, const unsigned int &degree);

		/**
		 *  \brief Computes maximum number of control points
		 *
		 *  \param degree     bspline degree
		 *
		 *  \return Minimum number of control points
		 */
		unsigned int MinCtrlPts(const unsigned int &degree);

		/**
		 *  \brief Computes node vector for a set of nodes
		 *  
		 *  \param vec_approx_nod  successive node to be approximed
		 *  \param degree          bspline degree
		 *  \param nb_ctr_pt       number of control points
		 *  
		 *  \return node vector associated to control points
		 */
		Eigen::Matrix<double,1,Eigen::Dynamic> ComputeNodeVec(const Eigen::Matrix<double,1,Eigen::Dynamic> &vec_approxnod, const unsigned int &degree, const unsigned int &nb_ctr_pt);
	}
}



#endif //_COMPUTENODEVECTOR_H_
