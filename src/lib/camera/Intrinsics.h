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
 *  \file  Intrinsics.h
 *  \brief Defines intrinsic camera parameters
 *  \author Bastien Durix
 */

#ifndef _INTRINSICS_H_
#define _INTRINSICS_H_

#include <memory>
#include <mathtools/affine/Frame.h>

/**
 *  \brief Camera tools
 */
namespace camera
{
	/**
	 *  \brief Defines intrinsic camera parameters
	 */
	class Intrinsics
	{
		public:
			/**
			 *  \brief Shared pointer definition
			 */
			using Ptr = std::shared_ptr<Intrinsics>;
			
			/**
			 *  \brief Camera type
			 */
			enum class Type
			{
				pinhole,
				ortho
			};

		protected:
			/**
			 *  \brief Image width
			 */
			unsigned int m_width;
			
			/**
			 *  \brief Image height
			 */
			unsigned int m_height;
			
			/**
			 *  \brief Image frame
			 */
			mathtools::affine::Frame<2>::Ptr m_frame;

		public:
			/**
			 *  \brief Constructor
			 *
			 *  \param width   image width
			 *  \param height  image height
			 */
			Intrinsics(unsigned int width, unsigned int height, const mathtools::affine::Frame<2>::Ptr frame = mathtools::affine::Frame<2>::CanonicFrame());
			
			/**
			 *  \brief Virtual destructor
			 */
			virtual ~Intrinsics();

			/**
			 *  \brief Image width getter
			 */
			unsigned int getWidth() const;

			/**
			 *  \brief Image height getter
			 */
			unsigned int getHeight() const;

			/**
			 *  \brief Image frame getter
			 */
			const mathtools::affine::Frame<2>::Ptr getFrame() const;

			/**
			 *  \brief Camera type getter
			 */
			virtual Type getType() const = 0;
	};
}

#endif //_INTRINSICS_H_
