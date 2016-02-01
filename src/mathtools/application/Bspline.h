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
 *  \file Bspline.h
 *  \brief Defines bspline function
 *  \author Bastien Durix
 */

#ifndef _BSPLINE_H_
#define _BSPLINE_H_

#include "Application.h"
#include <Eigen/Dense>

/**
 *  \brief Mathematical tools
 */
namespace mathtools
{
	/**
	 *  \brief Application tools
	 */
	namespace application
	{
		/**
		 *  \brief Defines Bspline basis function
		 *
		 *  \tparam degree  Basis degree
		 *  \tparam indice  Basis indice
		 */
		template<unsigned int degree, unsigned int indice>
		class BsplineBasis : public Application<double,double>
		{
			public:
				/**
				 *  \brief Evaluate the basis
				 *
				 *	\param t    Parameter for which evaluate the basis
				 *	\param node Node vector of the basis
				 *
				 *  \return evaluation of the basis for parameter t
				 */
				static double eval(double t, const Eigen::Matrix<double,1,Eigen::Dynamic> &node)
				{
					double res = 0.0;
					if(indice > 0)
					{
						if(node_vec(0,indice-1) <= t && ( t < node_vec(indice+degree-1) || indice+degree == node_vec.cols() ))
						{
							double num1 = t                           - node_vec(0,indice-1);
							double den1 = node_vec(0,indice+degree-1) - node_vec(0,indice-1);

							if(den1 != 0)
								res += (num1/den1) * BsplineBasis<degree-1,indice-1>::eval(t,node_vec.block(0,1,1,node_vec.cols()-2));
						}
					}

					if(indice + degree < node_vec.cols())
					{
						if(node_vec(0,indice) <= t && (t < node_vec(0,indice+degree) || indice+degree+1 == node_vec.cols()))
						{
							double num2 = node_vec(0,indice+degree) - t;
							double den2 = node_vec(0,indice+degree) - node_vec(0,indice);

							if(den2 != 0)
								res += (num2/den2) * BsplineBasis<degree-1,indice>::eval(t,node_vec.block(0,1,1,node_vec.cols()-2));	
						}
					}

					return res;
				}
		};

		/**
		 *  \brief Defines Degree 0 bspline basis function
		 *
		 *  \tparam indice  Basis indice
		 */
		template<unsigned int indice>
		class BsplineBasis<0,indice> : public Application<double,double>
		{
			public:
				/**
				 *  \brief Evaluate the basis
				 *
				 *	\param t    Parameter for which evaluate the basis
				 *	\param node Node vector of the basis
				 *
				 *  \return evaluation of the basis for parameter t
				 *
				 *  \throws std::logic_error if basis indice is out of node vector
				 */
				static double eval(double t, const Eigen::Matrix<double,1,Eigen::Dynamic> &node)
				{
					if(indice >= node.cols()-1)
						throw std::logic_error("Basis indice is out of node vector");
					double val = 0.0;
					if(node(0,indice) <= t && t < node(0,indice+1))
						val = 1.0;
					return val;
				}
		};

		/**
		 *  \brief Defines Bspline curve
		 *
		 *  \tparam Dim Curve dimension
		 */
		template<unsigned int Dim>
		class Bspline : public Application<Eigen::Matrix<double,Dim,1>,double>
		{
			public:
				/**
				 *  \brief Out type of result funtion
				 */
				using outType = Eigen::Matrix<double,Dim,1>;
				/**
				 *  \brief In type of result funtion
				 */
				using inType = double;
			
		};
	}
}


#endif //_BSPLINE_H_
