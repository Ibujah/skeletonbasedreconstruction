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

#include <Eigen/Dense>
#include <memory>
#include "Application.h"
#include "BsplineUtils.h"

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

				/**
				 *  \brief Shared pointer definition
				 */
				using Ptr = std::shared_ptr<Bspline>;

			protected:
				/**
				 *  \brief Control points
				 */
				Eigen::Matrix<double,Dim,Eigen::Dynamic> m_ctrlpt;

				/**
				 *  \brief Node vector
				 */
				Eigen::Matrix<double,1,Eigen::Dynamic> m_nodevec;

				/**
				 *  \brief Bspline degree
				 */
				unsigned int m_degree;

			public:
				/**
				 *  \brief Constructor
				 *
				 *  \param ctrlpt   Control points
				 *  \param nodevec  Node vector
				 *  \param degree   Bspline degree
				 *
				 *  \throws std::logic_error if number of control points, nodes and degree does not match
				 */
				Bspline(const Eigen::Matrix<double,Dim,Eigen::Dynamic> &ctrlpt,
						const Eigen::Matrix<double,1,Eigen::Dynamic>   &nodevec,
						const unsigned int &degree) : 
					Application<Eigen::Matrix<double,Dim,1>,double>(), m_ctrlpt(ctrlpt), 
					m_nodevec(nodevec), m_degree(degree)
				{
					if( ctrlpt.cols() + degree != nodevec.cols()+1  )
						throw std::logic_error("Bspline : not verified #CtrlPt + degree = #NodeVec + 1");
				}

				/**
				 *  \brief Copy constructor
				 *
				 *  \param bspline Bspline to copy
				 */
				Bspline(const Bspline<Dim> &bspline) : 
					Application<Eigen::Matrix<double,Dim,1>,double>(), m_ctrlpt(bspline.m_ctrlpt), 
					m_nodevec(bspline.m_nodevec), m_degree(bspline.m_degree) {};
				
				/**
				 *  \brief Bspline call
				 *
				 *  \param t Bspline parameter
				 * 
				 *  \return Bspline evaluation at t
				 */
				Eigen::Matrix<double,Dim,1> operator()(const double &t) const
				{
					Eigen::Matrix<double,Dim,1> res = Eigen::Matrix<double,Dim,1>::Zero();

					for(unsigned int ind = 0; ind<m_ctrlpt.cols(); ind++)
					{
						double basis_ind = BsplineBasis(t,m_degree,ind,m_nodevec);
						res+=m_ctrlpt.block(0,ind,Dim,1)*basis_ind;
					}

					return res;
				}

				/**
				 *  \brief Function first derivative
				 *
				 *  \param t Input of the application
				 *
				 *  \returns First derivative associated to input t
				 */
				virtual typename derivativematrix<1,dimension<outType>::value,dimension<inType>::value>::type
					der(const double &t) const
				{
					Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> ctrl_vec_der1 = m_ctrlpt,
						ctrl_vec_der2 = m_ctrlpt,
						ctrl_vec_der(Dim,m_ctrlpt.cols()-1);
					double tbeg, tend;

					unsigned int interval = m_degree-1;

					while(m_nodevec(0,interval+1) <= t && interval != m_nodevec.cols()-m_degree-1) {interval++;}

					tbeg = m_nodevec(0,interval);
					tend = m_nodevec(0,interval+1);

					Blossom(tbeg,m_nodevec,ctrl_vec_der1);
					Blossom(tend,m_nodevec,ctrl_vec_der2);

					ctrl_vec_der = (ctrl_vec_der2 - ctrl_vec_der1).block(0,0,Dim,ctrl_vec_der.cols())*(1/(tend-tbeg));
					
					typename derivativematrix<1,dimension<outType>::value,dimension<inType>::value>::type arr;
					
					Eigen::Map<Eigen::Matrix<double,Dim,1> > res((double*)arr.data());
					
					double treel = t<tbeg?tbeg:(t>tend?tend:t);
					
					res.setZero();

					for(unsigned int ind = 0; ind<ctrl_vec_der.cols(); ind++)
					{
						double basis_ind = BsplineBasis(treel,m_degree-1,ind,m_nodevec.block(0,1,1,m_nodevec.cols()-2));
						res+=ctrl_vec_der.block(0,ind,Dim,1)*basis_ind;
					}

					return arr;
				}

				/**
				 *  \brief Function second derivative
				 *
				 *  \param t Input of the application
				 *
				 *  \returns Second derivative associated to input t
				 */
				virtual typename derivativematrix<2,dimension<outType>::value,dimension<inType>::value>::type
					der2(const double &t) const
				{
					Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> ctrl_vec_der1 = m_ctrlpt,
						ctrl_vec_der2 = m_ctrlpt,
						ctrl_vec_der11(Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>::Zero(Dim,m_ctrlpt.cols()-1)),
						ctrl_vec_der12(Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>::Zero(Dim,m_ctrlpt.cols()-1)),
						ctrl_vec_der22(Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>::Zero(Dim,m_ctrlpt.cols()-2));
					double tbeg, tmid, tend;

					unsigned int interval = m_degree-2;

					while(m_nodevec(0,interval+1) <= t && interval != m_nodevec.cols()-m_degree-1) {interval++;}

					tbeg = m_nodevec(0,interval);
					tmid = m_nodevec(0,interval+1);
					tend = m_nodevec(0,interval+2);

					Blossom(tbeg,m_nodevec,ctrl_vec_der1);
					Blossom(tend,m_nodevec,ctrl_vec_der2);
					
					if(tmid != tbeg)
						ctrl_vec_der11 = (ctrl_vec_der2 - ctrl_vec_der1).block(0,0,Dim,ctrl_vec_der11.cols())*(1/(tmid-tbeg));

					Blossom(tbeg,m_nodevec,ctrl_vec_der1);
					Blossom(tend,m_nodevec,ctrl_vec_der2);
					
					if(tend != tmid)
						ctrl_vec_der12 = (ctrl_vec_der2 - ctrl_vec_der1).block(0,0,Dim,ctrl_vec_der12.cols())*(1/(tend-tmid));

					if(tend != tbeg)
						ctrl_vec_der22 = (ctrl_vec_der12 - ctrl_vec_der11).block(0,0,Dim,ctrl_vec_der22.cols())*(1/(tend-tbeg));

					typename derivativematrix<2,dimension<outType>::value,dimension<inType>::value>::type arr;
					
					Eigen::Map<Eigen::Matrix<double,dimension<outType>::value,dimension<inType>::value> > res((double*)arr.data());

					double treel = t<tbeg?tbeg:(t>tend?tend:t);
					
					res.setZero();

					for(unsigned int ind = 0; ind<ctrl_vec_der22.cols(); ind++)
					{
						double basis_ind = BsplineBasis(treel,m_degree-2,ind,m_nodevec.block(0,2,1,m_nodevec.cols()-3));
						res+=ctrl_vec_der22.block(0,ind,Dim,1)*basis_ind;
					}

					return arr;
				}

				/**
				 *  \brief Inferior boundary accessor
				 *
				 *  \return Inferior boundary
				 */
				double getInfBound() const
				{
					return m_nodevec(0,m_degree-1);
				}
				
				/**
				 *  \brief Superior boundary accessor
				 *
				 *  \return Superior boundary
				 */
				double getSupBound() const
				{
					return m_nodevec(0,m_nodevec.cols()-m_degree);
				}
		};
	}
}


#endif //_BSPLINE_H_
