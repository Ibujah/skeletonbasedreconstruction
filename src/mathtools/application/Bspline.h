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
