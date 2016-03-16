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
 *  \file  DiscreteBoundary3.cpp
 *  \brief Defines 3d discrete boundary of a shape
 *  \author Bastien Durix
 */

#include "DiscreteBoundary3.h"

boundary::DiscreteBoundary<3>::DiscreteBoundary(const mathtools::affine::Frame<3>::Ptr frame) : m_frame(frame) {}

unsigned int boundary::DiscreteBoundary<3>::addPoint(const mathtools::affine::Point<3> &point, const Eigen::Vector3d &normal)
{
	unsigned int key = 0;
    if(m_node.size()) key = m_node.rbegin()->first+1;
	m_node.insert(std::pair<unsigned int,Eigen::Vector3d>(key,point.getCoords(m_frame)));
	m_normal.insert(std::pair<unsigned int,Eigen::Vector3d>(key,normal));
	return key;
}

void boundary::DiscreteBoundary<3>::addEdge(unsigned int ind1, unsigned int ind2)
{
	if(!isPointIn(ind1) || !isPointIn(ind2))
		throw std::logic_error("boundary::DiscreteBoundary<3>::addEdge : Index is not in the boundary");
	m_edge.push_back(std::pair<unsigned int, unsigned int>(ind1,ind2));
}
 
void boundary::DiscreteBoundary<3>::addFace(unsigned int ind1, unsigned int ind2, unsigned int ind3)
{
	if(!isPointIn(ind1) || !isPointIn(ind2) || !isPointIn(ind3))
		throw std::logic_error("boundary::DiscreteBoundary<3>::addFace : Index is not in the boundary");
	m_face.push_back(std::make_tuple(ind1,ind2,ind3));
}

const mathtools::affine::Frame<3>::Ptr boundary::DiscreteBoundary<3>::getFrame() const
{
	return m_frame;
}

const std::map<unsigned int, Eigen::Vector3d>& boundary::DiscreteBoundary<3>::getPoints() const
{
	return m_node;
}

const std::map<unsigned int, Eigen::Vector3d>& boundary::DiscreteBoundary<3>::getNormals() const
{
	return m_normal;
}

const std::list<std::pair<unsigned int, unsigned int> >& boundary::DiscreteBoundary<3>::getEdges() const
{
	return m_edge;
}

const std::list<std::tuple<unsigned int, unsigned int, unsigned int> >& boundary::DiscreteBoundary<3>::getFaces() const
{
	return m_face;
}

bool boundary::DiscreteBoundary<3>::isPointIn(unsigned int index) const
{
	return (m_node.find(index) != m_node.end());
}
