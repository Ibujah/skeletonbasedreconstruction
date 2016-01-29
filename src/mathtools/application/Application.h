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
 *  \file Application.h
 *  \brief Defines generic application
 *  \author Bastien Durix
 */

#ifndef _APPLICATION_H_
#define _APPLICATION_H_

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
		 *  \brief Main application class
		 *
		 *  \tparam OutType out type of the application
		 *  \tparam InType  in type of the application
		 */
		template<typename OutType, typename InType>
		class Application
		{
			public:
				/**
				 *  \brief Out type of result funtion
				 */
				using outType = OutType;
				/**
				 *  \brief In type of result funtion
				 */
				using inType = InType;

		};

		/**
		 *  \brief Type dimension getter
		 *
		 *  \tparam Type: out or in type of an application
		 */
		template<typename Type> struct dimension;

		/**
		 *  \brief Eigen vector dimension getter
		 *
		 *  \tparam Dim Eigen vector dimension
		 */
		template<unsigned int Dim> struct dimension<Eigen::Matrix<double,Dim,1> >
		{
			static constexpr unsigned int value = Dim;
		};
	}
}


#endif //_APPLICATION_H_


