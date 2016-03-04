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
 *  \file  DiscreteBoundary3.h
 *  \brief Defines discrete boundary of a 3d shape
 *  \author Bastien Durix
 */

#ifndef _DISCRETEBOUNDARY3_H_
#define _DISCRETEBOUNDARY3_H_

#include <memory>
#include <list>
#include <map>
#include <mathtools/affine/Frame.h>
#include <mathtools/affine/Point.h>
#include "DiscreteBoundary.h"

/**
 *  \brief Boundary representations
 */
namespace boundary
{
	/**
	 *  \brief Describe discrete boundary in 3d space
	 */
	template<>
	class DiscreteBoundary<3>
	{
		public:
			/**
			 *  \brief Boundary shared pointer
			 */
			using Ptr = std::shared_ptr<DiscreteBoundary<3> >;

		protected:
			/**
			 *  \brief Boundary frame
			 */
			typename mathtools::affine::Frame<3>::Ptr m_frame;

			/**
			 *  \brief Point list
			 */
			std::map<unsigned int, Eigen::Vector3d> m_node;

			/**
			 *  \brief Normal list
			 */
			std::map<unsigned int, Eigen::Vector3d> m_normal;
			
			/**
			 *  \brief Edge list
			 */
			std::list<std::pair<unsigned int, unsigned int> > m_edge;

			/**
			 *  \brief Face list
			 */
			std::list<std::tuple<unsigned int, unsigned int, unsigned int> > m_face;

		public:
			/**
			 *  \brief Constructor
			 *
			 *  \param frame boundary frame
			 */
			DiscreteBoundary<3>(const mathtools::affine::Frame<3>::Ptr frame = mathtools::affine::Frame<3>::CanonicFrame());
			
		public://modifying functions
			/**
			 *  \brief Adding point/normal function
			 *
			 *  \param point  point to add
			 *  \param normal normal to add
			 *
			 *  \return point key
			 */
			unsigned int addPoint(const mathtools::affine::Point<3> &point, const Eigen::Vector3d &normal = Eigen::Vector3d::Zero());

			/**
			 *  \brief Adding edge function
			 *
			 *  \param ind1 first node index
			 *  \param ind2 second node index
			 */
			void addEdge(unsigned int ind1, unsigned int ind2);

			/**
			 *  \brief Adding face function
			 *
			 *  \param ind1 first node index
			 *  \param ind2 second node index
			 *  \param ind3 third node index
			 */
			void addFace(unsigned int ind1, unsigned int ind2, unsigned int ind3);

		public://non-modifying functions
			/**
			 *  \brief Frame getter
			 *
			 *  \return frame of the boundary
			 */
			const mathtools::affine::Frame<3>::Ptr getFrame() const;

			/**
			 *  \brief Points getter
			 *
			 *  \return Map containing the points
			 */
			const std::map<unsigned int, Eigen::Vector3d> getPoints() const;

			/**
			 *  \brief Normals getter
			 *
			 *  \return Map containing the normals
			 */
			const std::map<unsigned int, Eigen::Vector3d> getNormals() const;

			/**
			 *  \brief Edges getter
			 *
			 *  \return List containing the edges
			 */
			const std::list<std::pair<unsigned int, unsigned int> > getEdges() const;

			/**
			 *  \brief Faces getter
			 *
			 *  \return List containing the faces
			 */
			const std::list<std::tuple<unsigned int, unsigned int, unsigned int> > getFaces() const;
	};
}

#endif //_DISCRETEBOUNDARY3_H_
