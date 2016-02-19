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
 *  \file FitBspline.h
 *  \brief Provides Bspline fitting algorithm
 *  \author Bastien Durix
 */

#ifndef _FITBSPLINE_H_
#define _FITBSPLINE_H_

#include <Eigen/Dense>
#include <vector>

#include <mathtools/application/Bspline.h>
#include <mathtools/application/BsplineUtils.h>

/**
 *  \brief Lots of algorithms
 */
namespace algorithm
{
	/*
	 *  \brief Defines tools to compute Bsplines
	 */
	namespace fitbspline
	{
		/**
		 *  \brief Fits Bspline from a set of vectors
		 *  
		 *  \tparam Dim : Space dimension
		 * 
		 *  \param approx_vec : Vectors to approxime
		 *  \param approx_nod : Param associated to each vector
		 *  \param nod_vec    : Node vector to use
		 *  \param degree     : Bspline degree
		 * 
		 *  \return Fitted Bspline
		 */
		template<unsigned int Dim>
		mathtools::application::Bspline<Dim> FitBspline(const std::vector<Eigen::Matrix<double,Dim,1> > &approx_vec, 
				const Eigen::Matrix<double,1,Eigen::Dynamic> &approx_nod,
				const Eigen::Matrix<double,1,Eigen::Dynamic> &nod_vec,
				unsigned int degree)
		{
			unsigned int nb_approx = approx_vec.size();
			unsigned int nb_nodes = nod_vec.cols()-degree+1;
			Eigen::Matrix<double,Eigen::Dynamic,Dim> approx_mat(nb_approx,Dim);
			Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> approx_param(nb_approx,nb_nodes);

			for(unsigned int i = 0; i < nb_approx; i++)
			{
				approx_mat.template block<1,Dim>(i,0) = approx_vec[i].transpose();
				for(unsigned int j = 0; j < nb_nodes; j++)
				{
					approx_param(i,j) = mathtools::application::BsplineBasis(approx_nod(0,i),degree,j,nod_vec);
				}
			}
			Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> approx_param_block = approx_param.block(0,1,nb_approx,nb_nodes-2);
			Eigen::Matrix<double,Eigen::Dynamic,Dim> res_mat = approx_mat - approx_param.block(0,0,nb_approx,1)*approx_mat.template block<1,Dim>(0,0)
				- approx_param.block(0,nb_nodes-1,nb_approx,1)*approx_mat.template block<1,Dim>(nb_approx-1,0);

			Eigen::Matrix<double,Eigen::Dynamic,Dim> ctrl_mat = approx_param_block.householderQr().solve(res_mat);

			Eigen::Matrix<double,Dim,Eigen::Dynamic> ctrl_vec(Dim,nb_nodes);
			ctrl_vec.template block<Dim,1>(0,0) = approx_mat.template block<1,Dim>(0,0).transpose();
			for(unsigned int i=0;i<nb_nodes-2;i++)
			{
				ctrl_vec.template block<Dim,1>(0,i+1) = ctrl_mat.template block<1,Dim>(i,0).transpose();
			}
			ctrl_vec.template block<Dim,1>(0,nb_nodes-1) = approx_mat.template block<1,Dim>(nb_approx-1,0).transpose();

			return mathtools::application::Bspline<Dim>(ctrl_vec,nod_vec,degree);
		}
	}
}

#endif //_FITBSPLINE_H_
