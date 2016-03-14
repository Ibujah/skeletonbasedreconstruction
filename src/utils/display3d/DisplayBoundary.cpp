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
 *  \file DisplayBoundary.cpp
 *  \brief Provides method to display 3d boundary
 *  \author Bastien Durix
 */

#include "DisplayBoundary.h"

unsigned int display3d::DisplayBoundary_Wired(display3d::DisplayClass &disclass, const boundary::DiscreteBoundary<3>::Ptr disbnd, float red, float green, float blue)
{
	unsigned int indexlist = disclass.startList();
	
	glLineWidth(1.0);
	glColor3f(red,green,blue);
	glBegin(GL_LINES);
	for(std::list<std::pair<unsigned int,unsigned int> >::const_iterator it = disbnd->getEdges().begin(); it != disbnd->getEdges().end(); it++)
	{
		unsigned int ind1 = it->first;
		unsigned int ind2 = it->second;
		
		Eigen::Vector3d pt1 = disbnd->getPoints().find(ind1)->second;
		Eigen::Vector3d pt2 = disbnd->getPoints().find(ind2)->second;
		Eigen::Vector3d nor1 = disbnd->getNormals().find(ind1)->second;
		Eigen::Vector3d nor2 = disbnd->getNormals().find(ind2)->second;

		glNormal3f(pt1.x(),pt1.y(),pt1.z());
		glVertex3f(nor1.x(),nor1.y(),nor1.z());

		glNormal3f(pt2.x(),pt2.y(),pt2.z());
		glVertex3f(nor2.x(),nor2.y(),nor2.z());
	}
	glEnd();
	
	disclass.endList();
	return indexlist;
}


unsigned int display3d::DisplayBoundary_Filled(display3d::DisplayClass &disclass, const boundary::DiscreteBoundary<3>::Ptr disbnd, float red, float green, float blue)
{
	unsigned int indexlist = disclass.startList();
	
	glLineWidth(1.0);
	glColor3f(red,green,blue);
	glBegin(GL_TRIANGLES);
	for(std::list<std::tuple<unsigned int, unsigned int, unsigned int> >::const_iterator it = disbnd->getFaces().begin(); it != disbnd->getFaces().end(); it++)
	{
		unsigned int ind1 = std::get<0>(*it);
		unsigned int ind2 = std::get<1>(*it);
		unsigned int ind3 = std::get<2>(*it);
		
		Eigen::Vector3d pt1 = disbnd->getPoints().find(ind1)->second;
		Eigen::Vector3d pt2 = disbnd->getPoints().find(ind2)->second;
		Eigen::Vector3d pt3 = disbnd->getPoints().find(ind3)->second;
		Eigen::Vector3d nor1 = disbnd->getNormals().find(ind1)->second;
		Eigen::Vector3d nor2 = disbnd->getNormals().find(ind2)->second;
		Eigen::Vector3d nor3 = disbnd->getNormals().find(ind3)->second;

		glNormal3f(pt1.x(),pt1.y(),pt1.z());
		glVertex3f(nor1.x(),nor1.y(),nor1.z());

		glNormal3f(pt2.x(),pt2.y(),pt2.z());
		glVertex3f(nor2.x(),nor2.y(),nor2.z());

		glNormal3f(pt3.x(),pt3.y(),pt3.z());
		glVertex3f(nor3.x(),nor3.y(),nor3.z());
	}
	glEnd();
	
	disclass.endList();
	return indexlist;
}
