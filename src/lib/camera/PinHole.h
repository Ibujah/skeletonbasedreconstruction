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
 *  \file  PinHole.h
 *  \brief Defines pinhole camera parameters
 *  \author Bastien Durix
 */

#ifndef _PINHOLE_H_
#define _PINHOLE_H_

#include "Intrinsics.h"

/**
 *  \brief Camera tools
 */
namespace camera
{
	/**
	 *  \brief Defines pinhole camera parameters
	 */
	class PinHole : public Intrinsics
	{
		protected:
			/**
			 *  \brief x-coordinate of principal point
			 */
			double m_u0;
			
			/**
			 *  \brief y-coordinate of principal point
			 */
			double m_v0;
			
			/**
			 *  \brief focal ratio along x-axis
			 */
			double m_ku;
			
			/**
			 *  \brief focal ratio along y-axis
			 */
			double m_kv;
		
		public:
			/**
			 *  \brief Constructor
			 * 
			 *  \param width   image width
			 *  \param height  image height
			 *  \param u0      x-coordinate of principal point
			 *  \param v0      y-coordinate of principal point
			 *  \param ku      focal ratio along x-axis
			 *  \param kv      focal ratio along y-axis
			 */
			PinHole(unsigned int width, unsigned int height, double u0, double v0, double ku, double kv) :
				Intrinsics(width,height), m_u0(u0), m_v0(v0), m_ku(ku), m_kv(kv) {}
	};
}

#endif //_PINHOLE_H_