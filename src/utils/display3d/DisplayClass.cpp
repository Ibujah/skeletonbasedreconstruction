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

#define ZNEAR 0.01
#define ZFAR 1000.

#include <opencv2/imgproc/imgproc.hpp>

#include "DisplayClass.h"
#include <camera/OrthoCam.h>
#include <camera/PinHole.h>

display3d::DisplayClass::DisplayClass(const std::string &title, unsigned int width, unsigned int height) :
	m_window(sf::VideoMode(width,height),title,sf::Style::Titlebar), m_usebg(false), m_nblists(0), m_clock(), m_ctrlenabled(false), m_campos(0,0,0), m_camrot(0,0,0)
{
	setView();
	m_window.setVerticalSyncEnabled(true);
	m_window.setActive(true);

	m_bgimg.create(1024,1024);
	glGenTextures(1,&m_bgtexture);
}

unsigned int display3d::DisplayClass::startList()
{
	m_window.setActive(true);
	glNewList(m_nblists+1,GL_COMPILE);
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

void display3d::DisplayClass::setView()
{
	setIntrinsics();
	setExtrinsics();
}

void display3d::DisplayClass::setIntrinsics(const camera::Intrinsics::Ptr intrinsics)
{
	m_window.setActive(true);
	unsigned int width = intrinsics->getWidth();
	unsigned int height = intrinsics->getHeight();
	m_window.setSize(sf::Vector2u(width,height));
	Eigen::Vector2d origin = intrinsics->getFrame()->getFrameInverse()->getOrigin();
	Eigen::Matrix2d basis = intrinsics->getFrame()->getFrameInverse()->getBasis()->getMatrix();
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	switch(intrinsics->getType())
	{
		case camera::Intrinsics::Type::pinhole:
			glFrustum(
					origin.x()*ZNEAR,
					(width*basis(0,0)+origin.x())*ZNEAR,
					(height*basis(1,1)+origin.y())*ZNEAR,
					origin.y()*ZNEAR,
					ZNEAR,
					ZFAR);
			break;
		case camera::Intrinsics::Type::ortho:
			glOrtho(origin[0],origin[0]+width*basis(0,0),origin[1]+height*basis(1,1),origin[1],ZNEAR,ZFAR);
			break;
	}
	glScalef(1,1,-1);
	glViewport( 0, 0, width, height );
}

void display3d::DisplayClass::setIntrinsics()
{
	m_window.setActive(true);
	unsigned int width = m_window.getSize().x;
	unsigned int height = m_window.getSize().y;
	Eigen::Vector2d origin(-((double)width/(double)width)/2.0,-((double)height/(double)width)/2.0);
	Eigen::Matrix2d basis = Eigen::Matrix2d::Identity()/(double)width;
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	glFrustum(
			origin.x()*ZNEAR,
			(width*basis(0,0)+origin.x())*ZNEAR,
			(height*basis(1,1)+origin.y())*ZNEAR,
			origin.y()*ZNEAR,
			ZNEAR,
			ZFAR);
	glScalef(1,1,-1);
	glViewport( 0, 0, width, height );
}

void display3d::DisplayClass::setExtrinsics(const camera::Extrinsics::Ptr extrinsics)
{
	m_window.setActive(true);
	Eigen::Matrix4d trmat = Eigen::Matrix4d::Identity();
 	trmat.block<3,3>(0,0) = extrinsics->getFrame()->getFrameInverse()->getBasis()->getMatrix();
 	trmat.block<3,1>(0,3) = extrinsics->getFrame()->getFrameInverse()->getOrigin();

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

	m_campos = sf::Vector3f(0,0,0);
	m_camrot = sf::Vector3f(0,0,0);
}

void display3d::DisplayClass::setExtrinsics(const Eigen::Matrix<double,3,4> &matext)
{
	m_window.setActive(true);
	Eigen::Matrix4d trmat = Eigen::Matrix4d::Identity();
 	trmat.block<3,4>(0,0) = matext;

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

	m_campos = sf::Vector3f(0,0,0);
	m_camrot = sf::Vector3f(0,0,0);
}

void display3d::DisplayClass::setExtrinsics()
{
	m_window.setActive(true);
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity( );

	m_campos = sf::Vector3f(0,0,0);
	m_camrot = sf::Vector3f(0,0,0);
}

sf::Window& display3d::DisplayClass::getWindow()
{
	return m_window;
}

void display3d::DisplayClass::setBackground(const sf::Image &bgimg)
{
	int width = bgimg.getSize().x;
	int height = bgimg.getSize().y;
	if(width>1024) width = 1024;
	if(height>1024) height = 1024;
	m_bgimg.copy(bgimg,0,0,sf::IntRect(0,0,width,height));
	m_usebg = true;
}

void display3d::DisplayClass::setBackground(const cv::Mat &bgimg)
{
	int height = bgimg.rows;
	int width = bgimg.cols;
	cv::Mat bgimgrgba;
	cv::cvtColor(bgimg,bgimgrgba,cv::COLOR_BGR2RGBA);
	sf::Image bgimginter;
    bgimginter.create(width, height, bgimgrgba.ptr());
	if(width>1024) width = 1024;
	if(height>1024) height = 1024;
	m_bgimg.copy(bgimginter,0,0,sf::IntRect(0,0,width,height));
	m_usebg = true;
}


unsigned int display3d::DisplayClass::getNblists() const
{
	return m_nblists;
}
			
void display3d::DisplayClass::display()
{
	m_window.setActive(true);
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glClearColor(1.0,1.0,1.0,0);
	
	if(m_usebg)
	{ 
		glLineWidth(1.0);
        glBindTexture( GL_TEXTURE_2D, m_bgtexture );
		glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
		glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S , GL_REPEAT );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

        glEnable(GL_TEXTURE_2D);
        glTexImage2D(GL_TEXTURE_2D, 0, 3, 1024, 1024, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_bgimg.getPixelsPtr());
         
        //Switch to Ortho for drawing background
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
		glLoadIdentity();
        glOrtho(0.0, m_window.getSize().x, m_window.getSize().y, 0.0, -10000, 10000);

        //Finally, draw the texture using a simple quad with texture coords in corners.
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
		glLoadIdentity();
        glTranslated(0,0,-9999);//why these parameters?!

        glBegin(GL_QUADS);
		glColor3f(1.0,1.0,1.0);
        glTexCoord2i(0, 0); glVertex2i(0, 0);
        glTexCoord2i(1, 0); glVertex2i(1024, 0);
        glTexCoord2i(1, 1); glVertex2i(1024, 1024);
        glTexCoord2i(0, 1); glVertex2i(0, 1024);
        glEnd();


        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();
		
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();

        glDisable(GL_TEXTURE_2D);
		m_usebg=false;
	}
	
	glMatrixMode(GL_PROJECTION);

	glPushMatrix( );
	glRotatef( -m_camrot.x, 1, 0, 0 );
	glRotatef( -m_camrot.y, 0, 1, 0 );
	glRotatef( -m_camrot.z, 0, 0, 1 );
	glTranslatef( m_campos.x, m_campos.y, m_campos.z );

	for(unsigned int i = 0; i < m_nblists; i++)
		glCallList(i+1);
	
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	m_window.display();
}
			
template<typename Container>
void display3d::DisplayClass::display(const Container &cont)
{
	m_window.setActive(true);
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glClearColor(1.0,1.0,1.0,0);
	
	if(m_usebg)
	{ 
		glLineWidth(1.0);
        glBindTexture( GL_TEXTURE_2D, m_bgtexture );
		glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
		glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S , GL_REPEAT );
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

        glEnable(GL_TEXTURE_2D);
        glTexImage2D(GL_TEXTURE_2D, 0, 3, 1024, 1024, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_bgimg.getPixelsPtr());
         
        //Switch to Ortho for drawing background
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
		glLoadIdentity();
        glOrtho(0.0, m_window.getSize().x, m_window.getSize().y, 0.0, -10000, 10000);

        //Finally, draw the texture using a simple quad with texture coords in corners.
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
		glLoadIdentity();
        glTranslated(0,0,-9999);//why these parameters?!

        glBegin(GL_QUADS);
		glColor3f(1.0,1.0,1.0);
        glTexCoord2i(0, 0); glVertex2i(0, 0);
        glTexCoord2i(1, 0); glVertex2i(1024, 0);
        glTexCoord2i(1, 1); glVertex2i(1024, 1024);
        glTexCoord2i(0, 1); glVertex2i(0, 1024);
        glEnd();


        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();
		
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();

        glDisable(GL_TEXTURE_2D);
		m_usebg=false;
	}

	glMatrixMode(GL_PROJECTION);

	glPushMatrix( );
	glRotatef( -m_camrot.x, 1, 0, 0 );
	glRotatef( -m_camrot.y, 0, 1, 0 );
	glRotatef( -m_camrot.z, 0, 0, 1 );
	glTranslatef( m_campos.x, m_campos.y, m_campos.z );

	for(typename Container::const_iterator it = cont.begin(); it != cont.end(); it++)
		glCallList(*it+1);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	m_window.display();
}

void display3d::DisplayClass::getRender(sf::Image &img)
{
	std::vector<unsigned char> image(m_window.getSize().x*m_window.getSize().y*4);
	glReadPixels(0,0,m_window.getSize().x,m_window.getSize().y,GL_RGBA,GL_UNSIGNED_BYTE,&image[0]);
	img.create(m_window.getSize().x,m_window.getSize().y,&image[0]);
	img.flipVertically();
}

namespace display3d
{
	template
	void display3d::DisplayClass::display<std::vector<unsigned int> >(const std::vector<unsigned int> &cont);

	template
	void display3d::DisplayClass::display<std::list<unsigned int> >(const std::list<unsigned int> &cont);
}

void display3d::DisplayClass::enableCtrl(const sf::Keyboard::Key &key)
{
	m_window.setMouseCursorVisible(false);
	m_clock.restart();
	sf::Mouse::setPosition(sf::Vector2i(m_window.getSize().x/2,m_window.getSize().y/2),m_window);
	m_ctrlenabled = true;
	m_quitkey = key;
}

void display3d::DisplayClass::disableCtrl()
{
	m_window.setMouseCursorVisible(true);
	m_ctrlenabled = false;
}

bool display3d::DisplayClass::isCtrlEnabled() const
{
	return m_ctrlenabled;
}

void display3d::DisplayClass::manageKeyboard(float camvit)
{
	if(m_ctrlenabled)
	{
		float sec = m_clock.getElapsedTime().asSeconds();
		m_clock.restart();
		
		// Position
		float dx = 0, dy = 0, dz = 0;
		if(sf::Keyboard::isKeyPressed(sf::Keyboard::Z))
		{
			dx = -sin(m_camrot.y/180.0*3.14159)*cos(-m_camrot.x/180.0*3.14159);
			dy = -sin(-m_camrot.x/180.0*3.14159);
			dz = -cos(m_camrot.y/180.0*3.14159)*cos(-m_camrot.x/180.0*3.14159);
		}
		if(sf::Keyboard::isKeyPressed(sf::Keyboard::S))
		{
			dx =   sin( m_camrot.y/180.0*3.14159)*cos(-m_camrot.x/180.0*3.14159);
			dy =   sin(-m_camrot.x/180.0*3.14159);
			dz =   cos( m_camrot.y/180.0*3.14159)*cos(-m_camrot.x/180.0*3.14159);
		}
		if(sf::Keyboard::isKeyPressed(sf::Keyboard::D))
		{
			dx =  -cos(-m_camrot.y/180.0*3.14159);
			dy =  -0.0;
			dz =  -sin(-m_camrot.y/180.0*3.14159);
		}
		if(sf::Keyboard::isKeyPressed(sf::Keyboard::Q))
		{
			dx =  cos(-m_camrot.y/180.0*3.14159);
			dy =  0.0;
			dz =  sin(-m_camrot.y/180.0*3.14159);
		}

		float dist = camvit*sec;
		
		float nor = sqrt(dx*dx+dy*dy+dz*dz);
		if(nor != 0)
		{
			dx *= dist/nor;
			dy *= dist/nor;
			dz *= dist/nor;
		}
		
		m_campos.x +=dx;
		m_campos.y +=dy;
		m_campos.z +=dz;


		// Rotation
		sf::Vector2i coords = sf::Mouse::getPosition(m_window);
		coords.x -= m_window.getSize().x/2;
		coords.y -= m_window.getSize().y/2;
		sf::Mouse::setPosition(sf::Vector2i(m_window.getSize().x/2,m_window.getSize().y/2),m_window);
		
		m_camrot.x -= (float)coords.y/2.0;
		if(m_camrot.x<-90.0) m_camrot.x=-90.0;
		if(m_camrot.x>90.0) m_camrot.x=90.0;
		m_camrot.y += (float)coords.x/2.0;
		if(m_camrot.y<0.0) m_camrot.y+=360.0;
		if(m_camrot.y>360.0) m_camrot.y-=360.0;

		// Quit Event
		sf::Event event;
		while(m_window.pollEvent(event))
		{
			if(event.type == sf::Event::KeyPressed)
			{
				if(event.key.code == m_quitkey)
					disableCtrl();
			}
		}
	}
}
