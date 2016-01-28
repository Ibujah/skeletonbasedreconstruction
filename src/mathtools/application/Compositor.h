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
 *  \file Compositor.h
 *  \brief Defines application compositor
 *  \author Bastien Durix
 */

#ifndef _COMPOSITOR_H_
#define _COMPOSITOR_H_

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
		 * \brief Recursive template composition of n function
		 */
		template<typename Fct, typename... Args>
		class Compositor
		{
			public:
				/**
				 *  \brief Out type of result funtion
				 */
				typedef typename Fct::outType outType;
				/**
				 *  \brief In type of result funtion
				 */
				typedef typename Fct::inType inType;
			protected:
				/**
				 *  \brief Current function
				 */
				Fct m_fct;
			public:
				/**
				 *  \brief Constructor
				 *
				 *  \param fct : reference to function to compose
				 */
				Compositor(const Fct &fct = Fct()) : m_fct(fct) {};

				/**
				 *  \brief Function call
				 *
				 *  \param t : input variable
				 *  \return Function evaluation at t
				 */
				outType operator()(const inType &t) const
				{
					return m_fct(t);
				};

				/**
				 *  \brief Jacobian call
				 *
				 *  \param t : input variable
				 *  \return Jacobian evaluation at t
				 */
				Eigen::Matrix<double,> jac(const inType &t) const
				{
					return m_fct.jac(t); 
				}
		};

		/**
		 * \brief Recursive template composition of n function
		 */
		template<typename Fct, typename Fct_, typename... Args>
		class Compositor<Fct,Fct_,Args...> : Compositor<Fct_,Args...>
		{
			public:
				/**
				 *  \brief Out type of result funtion
				 */
				typedef typename Fct::outType outType;
				/**
				 *  \brief In type of result funtion
				 */
				typedef typename Compositor<Fct_,Args...>::inType inType;
			protected:
				/**
				 *  \brief Current function
				 */
				Fct m_fct;
			public:
				/**
				 *  \brief Constructor
				 *
				 *  \param fct  : reference to first function to compose
				 *  \param fct_ : reference to second function to compose
				 *  \param fcts : next function arguments
				 */
				Compositor(const Fct &fct = Fct(), const Fct_ &fct_ = Fct_(), const Args&... fcts) : Compositor<Fct_,Args...>(fct_,fcts...), m_fct(fct) {};
				
				/**
				 *  \brief Function call
				 *
				 *  \param t : input variable
				 *  \return Function evaluation at t
				 */
				outType operator()(const inType &t) const
				{
					return m_fct(  Compositor<Fct_,Args...>::operator()(t) );
				};

				/**
				 *  \brief Jacobian call
				 *
				 *  \param t : input variable
				 *  \return Jacobian evaluation at t
				 */
				Eigen::Matrix<double,dimension<outType>::value,dimension<inType>::value> jac(const inType &t) const
				{
					return m_fct.jac(Compositor<Fct_,Args...>::operator()(t))*Compositor<Fct_,Args...>::jac(t);
				}

				/**
				 *  \brief Next function getter
				 *
				 *  \return next function
				 */
				inline const Compositor<Fct_,Args...>* next() const
				{
					return (Compositor<Fct_,Args...>*) (this);
				}
		};
	}
}

#endif //_COMPOSITOR_H_
