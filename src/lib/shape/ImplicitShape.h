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
 *  \file ImplicitShape.h
 *  \brief Defines implicit shape
 *  \author Bastien Durix
 */

#ifndef _IMPLICITSHAPE_H_
#define _IMPLICITSHAPE_H_

#include <mathtools/affine/Frame.h>
#include <mathtools/affine/Point.h>

/**
 *  \brief Defines shape tools
 */
namespace shape
{
	/**
	 *  \brief Implicit shape
	 *
	 *  \tparam Dim espace dimension
	 */
	template<unsigned int Dim>
	class ImplicitShape
	{
		protected:
			/**
			 *  \brief Shape frame
			 */
			typename mathtools::affine::Frame<Dim>::Ptr m_frame;

		public:
			/**
			 *  \brief Constructor
			 *
			 *  \tparam frame Shape frame
			 */
			ImplicitShape(typename mathtools::affine::Frame<Dim>::Ptr frame) : m_frame(frame) {}

			/**
			 *  \brief Constructor
			 *
			 *  \tparam frame Shape frame
			 */
			ImplicitShape(mathtools::affine::Frame<Dim> frame = mathtools::affine::Frame<Dim>()) :
				ImplicitShape(typename mathtools::affine::Frame<Dim>(new mathtools::affine::Frame<Dim>(frame))) {}

			/**
			 *  \brief Test if a point is in the shape
			 *
			 *  \param point point to test
			 *
			 *  \return true is the point is in the shape
			 */
			virtual bool isIn(const mathtools::affine::Point<Dim> &point) const = 0;
	};
}

#endif //_IMPLICITSHAPE_H_
