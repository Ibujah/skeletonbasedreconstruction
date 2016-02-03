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
 *  \file  DiscreteBoundary.h
 *  \brief Defines discrete boundary of a shape
 *  \author Bastien Durix
 */

#include <memory>
#include <list>
#include "Boundary.h"
#include <mathtools/affine/Frame.h>
#include <mathtools/affine/Point.h>

/**
 *  \brief Boundary representations
 */
namespace boundary
{
	/**
	 *  \brief Describe discrete boundary
	 *
	 *  \tparam Dim dimension of the space
	 */
	template<unsigned int Dim>
	class DiscreteBoundary : public Boundary<Dim>
	{};

	/**
	 *  \brief Describe discrete boundary in 2d space
	 */
	template<>
	class DiscreteBoundary<2> : public Boundary<2>
	{
		protected:
			/**
			 *  \brief vector of vertices composing the boundary
			 */
			std::vector<Eigen::Vector2d> m_vecvert;

			/**
			 *  \brief list of lists of all vertices indices, in direct order
			 */
			std::list<std::list<unsigned int> > m_listind;

		public:
			/**
			 *  \brief Constructor
			 *
			 *  \param frame boundary frame
			 */
			DiscreteBoundary<2>(const mathtools::affine::Frame<2>::Ptr frame);

			/**
			 *  \brief Constructor
			 *
			 *  \param frame boundary frame
			 */
			DiscreteBoundary<2>(const mathtools::affine::Frame<2> &frame = mathtools::affine::Frame<2>());
			
			/**
			 *  \brief Adds new string of vertices
			 *  
			 *  \param vertstr    container of vertices to add
			 *
			 *  \tparam Container vertices container
			 *  \tparam Vert      vertice type, either Eigen::Vector2d or mathtools::affine::Point<2>
			 */
			template<template<typename> class Container, typename Vert>
			void addVerticesString(const Container<Vert> &vertstr)
			{
				unsigned int ind = m_vecvert.size();
				m_vecvert.resize(m_vecvert.size() + vertstr.size());
				m_listind.push_back(std::list<unsigned int>());
				for(typename Container<Eigen::Vector2d>::const_iterator it = vertstr.begin(); it != vertstr.end(); it++)
				{
					m_vecvert[ind] = mathtools::affine::Point<2>(*it).getCoords(m_frame);
					m_listind.rbegin()->push_back(ind);
					ind++;
				}
			}
	};
}

