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
 *  \file DisplayCamera.h
 *  \brief Provides mathod to display a camera
 *  \author Bastien Durix
 */

#include "DisplayCamera.h"
#include <mathtools/affine/Point.h>

unsigned int display3d::DisplayCamera(DisplayClass &disclass, const camera::Camera::Ptr cam, float red, float green, float blue)
{
	unsigned int indexlist = disclass.startList();
	
	if(cam->getIntrinsics()->getType() == camera::Intrinsics::Type::pinhole)
	{
		Eigen::Vector3d oricam = cam->getExtrinsics()->getFrame()->getOrigin();

		unsigned int width = cam->getIntrinsics()->getWidth();
		unsigned int height = cam->getIntrinsics()->getHeight();

		std::vector<Eigen::Vector2d> corner2d(4);
		corner2d[0] = Eigen::Vector2d(0,0);
		corner2d[1] = Eigen::Vector2d(0,height);
		corner2d[2] = Eigen::Vector2d(width,height);
		corner2d[3] = Eigen::Vector2d(width,0);

		std::vector<Eigen::Vector3d> corner3d(4);

		for(unsigned int i = 0;i<4;i++)
		{
			Eigen::Vector2d vec = mathtools::affine::Point<2>(corner2d[i],cam->getIntrinsics()->getFrame()).getCoords();

			corner3d[i] = mathtools::affine::Point<3>(0.5*vec(0),0.5*vec(1),0.5,cam->getExtrinsics()->getFrame()).getCoords();
		}
		
		glColor3f(red,green,blue);
		glBegin(GL_LINES);

		for(unsigned int i = 0;i<4;i++)
		{
			glVertex3f(oricam.x(),oricam.y(),oricam.z());
			glVertex3f(corner3d[i].x(),corner3d[i].y(),corner3d[i].z());


			glVertex3f(corner3d[i].x(),corner3d[i].y(),corner3d[i].z());
			glVertex3f(corner3d[(i+1)%4].x(),corner3d[(i+1)%4].y(),corner3d[(i+1)%4].z());
		}
		glEnd();
	}
	disclass.endList();
	return indexlist;
}
