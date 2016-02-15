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
 *  \file  OrthoCam.h
 *  \brief Defines orthographic camera parameters
 *  \author Bastien Durix
 */

#ifndef _ORTHOCAM_H_
#define _ORTHOCAM_H_

#include "Intrinsics.h"
#include <mathtools/affine/Frame.h>

/**
 *  \brief Camera tools
 */
namespace camera
{
	/**
	 *  \brief Defines orthographic camera parameters
	 */
	class OrthoCam : public Intrinsics
	{
		protected:
			/**
			 *  \brief Camera frame
			 */
			mathtools::affine::Frame<2>::Ptr m_frame;

		public:
			/**
			 *  \brief Constructor
			 *
			 *  \param width   image width
			 *  \param height  image height
			 *  \param frame   image frame
			 */
			OrthoCam(unsigned int width, unsigned int height, const mathtools::affine::Frame<2>::Ptr frame = mathtools::affine::Frame<2>::CanonicFrame());

			/**
			 *  \brief Frame getter
			 *
			 *  \return image frame
			 */
			const mathtools::affine::Frame<2>::Ptr getFrame() const;
	};
}

#endif //_ORTHOCAM_H_
