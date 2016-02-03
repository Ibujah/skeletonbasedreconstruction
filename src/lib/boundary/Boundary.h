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
 *  \file  Boundary.h
 *  \brief Defines boundary of a shape
 *  \author Bastien Durix
 */

#include <memory>

#include <mathtools/affine/Frame.h>

/**
 *  \brief Boundary representations
 */
namespace boundary
{
	/**
	 *  \brief Describe generic boundary
	 *
	 *  \tparam Dim dimension of the space
	 */
	template<unsigned int Dim>
	class Boundary
	{
		protected:
			/**
			 *  \brief Boundary shared pointer
			 */
			using Ptr = std::shared_ptr<Boundary>;

			/**
			 *  \brief Boundary frame
			 */
			typename mathtools::affine::Frame<Dim>::Ptr m_frame;

		public:
			/**
			 *  \brief Constructor
			 *
			 *  \param frame boundary frame
			 */
			Boundary(const typename mathtools::affine::Frame<Dim>::Ptr frame) : m_frame(frame) {}

			/**
			 *  \brief Constructor
			 *
			 *  \param frame boundary frame
			 */
			Boundary(const mathtools::affine::Frame<Dim> &frame = mathtools::affine::Frame<Dim>()) :
				Boundary(typename mathtools::affine::Frame<Dim>::Ptr(new mathtools::affine::Frame<Dim>(frame))) {}
	};
}
