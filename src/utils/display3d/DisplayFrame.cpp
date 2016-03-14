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
 *  \file DisplayFrame.cpp
 *  \brief Provide method to display a frame
 *  \author Bastien Durix
 */

#include "DisplayFrame.h"

unsigned int display3d::DisplayFrame(display3d::DisplayClass &disclass, const mathtools::affine::Frame<3>::Ptr frame)
{
	unsigned int indexlist = disclass.startList();
	
	glMatrixMode( GL_MODELVIEW );
	
	glPushMatrix( );
	
	glLoadIdentity( );
	
	Eigen::Matrix4d trmat = Eigen::Matrix4d::Identity();
 	trmat.block<3,3>(0,0) = frame->getBasis()->getMatrix();
 	trmat.block<3,1>(0,3) = frame->getOrigin();
	
	GLfloat modelMatrix[16];
	for(unsigned int i = 0; i<4; i++)
	{
		for(unsigned int j=0; j<4; j++)
		{
			modelMatrix[j*4+i] = trmat(i,j);
		}
	}
	glMultMatrixf(modelMatrix);
	
	glBegin( GL_LINES );
	
	glColor3f( 1.0, 0.0, 0.0 );
	/*  X axis */
	glVertex3f( 0.0, 0.0, 0.0 );
	glVertex3f( 1.0, 0.0, 0.0 );
	/*  Letter X */
	glVertex3f( .8f, 0.05f, 0.0 ); // "backslash""
	glVertex3f( 1.0, 0.25f, 0.0 );
	glVertex3f( 0.8f, .25f, 0.0 ); // "slash"
	glVertex3f( 1.0, 0.05f, 0.0 );
	
	glColor3f( 0.0, 1.0, 0.0 );
	/*  Y axis */
	glVertex3f( 0.0, 0.0, 0.0 );
	glVertex3f( 0.0, 1.0, 0.0 );
	/*  Letter Y */
	glVertex3f( 0.25f, 1.0f, 0.0 );
	glVertex3f( 0.15f, 0.9f, 0.0 );
	
	glVertex3f( 0.05,  1.0f, 0.0 );
	glVertex3f( 0.15f, 0.9f, 0.0 );
	
	glVertex3f( 0.15f, 0.9f, 0.0 );
	glVertex3f( 0.15f, 0.8f, 0.0 );
	
	glColor3f( 0.0, 0.0, 1.0 );
	// z-axis
	glVertex3f( 0.0, 0.0, 0.0 );
	glVertex3f( 0.0, 0.0, 1.0 );
	/*  Letter Z */
	glVertex3f( 0.0, 0.05f, 0.8 ); // bottom horizontal leg
	glVertex3f( 0.0, 0.05f, 1.0 );
	glVertex3f( 0.0, 0.05f, 1.0 ); // slash
	glVertex3f( 0.0, 0.25f, 0.8 );
	glVertex3f( 0.0, 0.25f, 0.8 ); // upper horizontal leg
	glVertex3f( 0.0, 0.25f, 1.0 );
	
	glPopMatrix();
	
	glEnd( );
	
	glColor3f( 1.0, 1.0, 1.0 );
	glPopMatrix( );
	
	disclass.endList();
	return indexlist;
}
