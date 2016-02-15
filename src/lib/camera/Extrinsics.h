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
 *  \file  Extrinsics.h
 *  \brief Defines extrinsic camera parameters
 *  \author Bastien Durix
 */

#ifndef _EXTRINSICS_H_
#define _EXTRINSICS_H_

#include <memory>
#include <mathtools/affine/Frame.h>

/**
 *  \brief Camera tools
 */
namespace camera
{
	/**
	 *  \brief Defines extrinsic camera parameters
	 */
	class Extrinsics
	{
		public:
			/**
			 *  \brief Shared pointer definition
			 */
			using Ptr = std::shared_ptr<Extrinsics>;

		protected:
			/**
			 *  \brief Camera frame
			 */
			mathtools::affine::Frame<3>::Ptr m_frame;

		public:
			/**
			 *  \brief Constructor
			 *
			 *  \param frame camera frame
			 */
			Extrinsics(const mathtools::affine::Frame<3>::Ptr frame);

			/**
			 *  \brief Constructor
			 *
			 *  \param frame camera frame
			 */
			Extrinsics(const mathtools::affine::Frame<3> &frame);
	};
}

#endif //_EXTRINSICS_H_
