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
 *  \file  Frame.h
 *  \brief Defines a frame of an affine space
 *  \author Bastien Durix
 */

#ifndef _FRAME_H_
#define _FRAME_H_

#include <memory>
#include <Eigen/Dense>
#include <mathtools/vectorial/Basis.h>

/**
 *  \brief Mathematical tools
 */
namespace mathtools
{
	/**
	 *  \brief Affine tools
	 */
	namespace affine
	{
		/**
		 *  \class Frame
		 *  \brief Frame of an affine space
		 *  \tparam Dim dimension of the affine space
		 */
		template<unsigned int Dim>
		class Frame
		{
			public:
				/**
				 *  \brief Frame shared pointer
				 */
				using Ptr = std::shared_ptr<Frame<Dim> >;
			
			protected:
				/**
				 *  \brief Vectorial basis
				 */
				vectorial::Basis<Dim> m_basis;

				/**
				 *  \brief Frame origin
				 */
				Eigen::Matrix<double,Dim,1> m_origin;

			public:
				/**
				 *  \brief Constructor
				 *
				 *  \param basis  vectorial basis of the frame
				 *  \param origin frame origin
				 */
				Frame(const vectorial::Basis<Dim> &basis = vectorial::Basis<Dim>(), const Eigen::Matrix<double,Dim,1> &origin = Eigen::Matrix<double,Dim,1>::Zero()) :
					m_basis(basis), m_origin(origin) {}

				/**
				 *  \brief Copy constructor
				 *
				 *  \param frame frame to copy
				 */
				Frame(const Frame<Dim> &frame) : m_basis(frame.m_basis), m_origin(frame.m_origin) {}

				/**
				 *  \brief Vectorial basis getter
				 *
				 *  \return Current basis
				 */
				inline const vectorial::Basis<Dim>& getBasis() const
				{
					return m_basis;
				}

				/**
				 *  \brief Frame origin coordinates getter
				 *
				 *  \return Current origin coordinates in canonic frame
				 */
				inline const Eigen::Matrix<double,Dim,1>& getOrigin() const
				{
					return m_origin;
				}
		};
	}
}

#endif //_FRAME_H_
