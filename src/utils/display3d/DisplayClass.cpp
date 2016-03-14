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
 *  \file DisplayClass.cpp
 *  \brief Provides display methods with SFML
 *  \author Bastien Durix
 */

#include "DisplayClass.h"
#include <camera/OrthoCam.h>
#include <camera/PinHole.h>

display3d::DisplayClass::DisplayClass(const std::string &title, unsigned int width, unsigned int height) :
	m_window(sf::VideoMode(width,height),title), m_nblists(0), m_znear(0.01), m_zfar(1000.)
{}

unsigned int display3d::DisplayClass::startList()
{
	m_window.setActive(true);
	glNewList(m_nblists,GL_COMPILE);
	return m_nblists;
}

void display3d::DisplayClass::endList()
{
	m_window.setActive(true);
	glEndList();
	m_nblists++;
}
			
void display3d::DisplayClass::setView(const camera::Camera::Ptr camera)
{
	setIntrinsics(camera->getIntrinsics());
	setExtrinsics(camera->getExtrinsics());
}
			
void display3d::DisplayClass::setIntrinsics(const camera::Intrinsics::Ptr intrinsics)
{
	m_window.setActive(true);
	unsigned int width = intrinsics->getWidth();
	unsigned int height = intrinsics->getHeight();
	m_window.setSize(sf::Vector2u(width,height));
	Eigen::Vector2d origin = intrinsics->getFrame()->getOrigin();
	Eigen::Matrix2d basis = intrinsics->getFrame()->getBasis()->getMatrix();
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	switch(intrinsics->getType())
	{
		case camera::Intrinsics::Type::pinhole:
			glFrustum(
			  		origin[0]*m_znear,
			  		(width*basis(0,0)+origin[0])*m_znear,
			  		(height*basis(1,1)+origin[1])*m_znear,
			  		origin[1]*m_znear,
			  		m_znear,m_zfar);
			glScalef(1,1,-1);
			glViewport( 0, 0, width, height );
			break;
		case camera::Intrinsics::Type::ortho:
			glOrtho(-origin[0]/basis(0,0),(width-origin[0])/basis(0,0),(height-origin[1])/basis(1,1),-origin[1]/basis(1,1),m_znear,m_zfar);
			glViewport( 0, 0, width, height );
			break;
	}
}
			
void display3d::DisplayClass::setExtrinsics(const camera::Extrinsics::Ptr extrinsics)
{
	m_window.setActive(true);
	Eigen::Matrix4d trmat = Eigen::Matrix4d::Identity();
 	trmat.block<3,3>(0,0) = extrinsics->getFrame()->getBasis()->getMatrix();
 	trmat.block<3,1>(0,3) = extrinsics->getFrame()->getOrigin();

	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity( );

	GLfloat modelMatrix[16];
	for(unsigned int i = 0; i<4; i++)
	{
		for(unsigned int j=0; j<4; j++)
		{
			modelMatrix[j*4+i] = trmat(i,j);
		}
	}
	glMultMatrixf(modelMatrix);
}

sf::Window& display3d::DisplayClass::getWindow()
{
	return m_window;
}
			
unsigned int display3d::DisplayClass::getNblists()
{
	return m_nblists;
}
			
void display3d::DisplayClass::display() const
{
	m_window.setActive(true);
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glClearColor(1.0,1.0,1.0,0);

	for(unsigned int i = 0; i < m_nblists; i++)
		glCallList(i);
}
			
template<typename Container>
void display3d::DisplayClass::display(const Container &cont) const
{
	m_window.setActive(true);
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glClearColor(1.0,1.0,1.0,0);

	for(typename Container::const_iterator it = cont.begin(); it != cont.end(); it++)
		glCallList(*it);
}


namespace display3d
{
	template<>
	void display3d::DisplayClass::display<std::vector<unsigned int> >(const std::vector<unsigned int> &cont) const;

	template<>
	void display3d::DisplayClass::display<std::list<unsigned int> >(const std::list<unsigned int> &cont) const;
}
