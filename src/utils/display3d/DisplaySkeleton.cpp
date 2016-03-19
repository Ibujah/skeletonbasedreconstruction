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
 *  \file DisplaySkeleton.cpp
 *  \brief Provide method to display a skeleton
 *  \author Bastien Durix
 */

#include "DisplaySkeleton.h"
#include <mathtools/affine/Point.h>

using namespace mathtools::affine;

void DisplayBranch_helper(const skeleton::BranchContSkel3d::Ptr contbr, unsigned int nb_pts, float red, float green, float blue)
{
	glColor3f(red,green,blue);
	Point<3> pt_prev = contbr->getNode<Point<3> >(0.0);
	glDisable(GL_DEPTH_TEST);
    glLineWidth(4.0);
	glBegin(GL_LINES);
	for(unsigned int i=0;i<nb_pts;i++)
	{
		double t = (double)i/(double)(nb_pts-1);
		Point<3> pt_cur = contbr->getNode<Point<3> >(t);
		
		Eigen::Vector3d pt1 = pt_prev.getCoords();
		Eigen::Vector3d pt2 = pt_cur.getCoords();
		
		glVertex3f(pt1.x(),pt1.y(),pt1.z());
		glVertex3f(pt2.x(),pt2.y(),pt2.z());

		pt_prev = pt_cur;
	}
	glEnd();
	glEnable(GL_DEPTH_TEST);
    glLineWidth(1.0);
}

unsigned int display3d::DisplayBranch(DisplayClass &disclass, const skeleton::BranchContSkel3d::Ptr contbr, float red, float green, float blue)
{
	unsigned int indexlist = disclass.startList();
	DisplayBranch_helper(contbr,100,red,green,blue);
	disclass.endList();
	return indexlist;
}


unsigned int display3d::DisplaySkeleton(DisplayClass &disclass, const skeleton::CompContSkel3d::Ptr contskl, float red, float green, float blue)
{
	unsigned int indexlist = disclass.startList();
	std::vector<unsigned int> edge(0);
	contskl->getAllEdges(edge);

	for(unsigned int i=0;i<edge.size();i++)
	{
		DisplayBranch_helper(contskl->getBranch(edge[i]),100,red,green,blue);
	}

	disclass.endList();
	return indexlist;
}
