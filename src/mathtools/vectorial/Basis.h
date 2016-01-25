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

#ifndef _BASIS_H_
#define _BASIS_H_

/**
 *  \file  Basis.h
 *  \brief Defines a basis of a vectorial space
 *  \author Bastien Durix
 */

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

/**
 *  \brief Mathematical tools
 */
namespace mathtools
{
	/**
	 *  \brief Vectorial tools
	 */
	namespace vectorial
	{
		/**
		 *  \class Basis
		 *  \brief Basis of a vectorial space
		 *  \tparam Dim: dimension of the vectorial space
		 */
		template<unsigned int Dim>
		class Basis
		{
			public:
				/**
				 *  \brief Basis shared pointer
				 */
				typedef boost::shared_ptr<Frame<Dim> > Ptr;

			private:
				/**
				 *  \brief Basis coordinates in canonic basis
				 */
				Eigen::Matrix<double,Dim,Dim> m_basis;

				/**
				 *  \brief Canonic basis coordinates in basis
				 */
				Eigen::Matrix<double,Dim,Dim> m_basis_inv;
			public:
				/**
				 *  \brief Constructor
				 *
				 *  \param basis : full rank matrix
				 *
				 *  \throws logic_error if basis parameter not invertible
				 */
				Basis(const Eigen::Matrix<double,Dim,Dim> &basis = Eigen::Matrix<double,Dim,Dim>::Identity()) : m_basis(basis)
				{
					double det = m_basis.determinant();
					if(det*det <= Eigen::NumTraits<double>::dummy_precision())
					{
						throw new std::logic_error("mathtools::vectorial::Basis() : basis parameter not invertible");
					}
					m_basis_inv = m_basis.inverse();
				}
		}
	}
}

#endif //_BASIS_H_
