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
 *  \file Display3D.h
 *  \brief Provides display methods with SFML
 *  \author Bastien Durix
 */

#ifndef _DISPLAY3D_H_
#define _DISPLAY3D_H_

#include <string>
#include <list>
#include <memory>
#include <SFML/Window.hpp>
#include <SFML/OpenGL.hpp>

#include <camera/Camera.h>
#include <camera/Intrinsics.h>
#include <camera/Extrinsics.h>

/**
 *  \brief 3D displaying functions
 */
namespace display3d
{
	/**
	 *  \brief Displaying class
	 */
	class DisplayClass
	{
		public:
			/**
			 *  \brief DisplayClass shared pointer
			 */
			typedef std::shared_ptr<DisplayClass> Ptr;
			
		private:
			/**
 			 *  \brief SFML window
 			 */
			sf::Window m_window;

			/**
 			 *  \brief Number of lists created
 			 */
			unsigned int m_nblists;
			
			/**
 			 *  \brief z-near value
 			 */
			double m_znear;
			
			/**
 			 *  \brief z-far value
 			 */
			double m_zfar;

		public:
			/**
 			 *  \brief Constructor
 			 *
 			 *  \param title   window title
 			 *  \param width   window width
 			 *  \param height  window height
 			 */
			DisplayClass(const std::string &title, unsigned int width = 800, unsigned int height = 600);
			
			/**
 			 *  \brief Creates a new list to store 3D objects
 			 *
 			 *  \return list indice
 			 */
			unsigned int StartList();
			
			/**
 			 *  \brief Ends a list
 			 */
			void EndList();
			
			/**
 			 *  \brief Set camera parameters to view
 			 *
 			 *  \param camera  camera to set
 			 */
			void setView(const camera::Camera::Ptr camera);
			
			/**
 			 *  \brief Set intrinsics parameters to current view
 			 *
 			 *  \param intrinsics  intrinsics parameters to set
 			 */
			void setIntrinsics(const camera::Intrinsics::Ptr intrinsics);
			
			/**
 			 *  \brief Set extrinsics parameters to current view
 			 *
 			 *  \param extrinsics  extrinsics parameters to set
 			 */
			void setExtrinsics(const camera::Extrinsics::Ptr extrinsics);

			/**
 			 *  \brief SFML window getter
 			 *
 			 *  \return SFML window
 			 */
			sf::Window& getWindow();
			
			/**
 			 *  \brief Gets the number of lists
 			 *
 			 *  \return number of lists
 			 */
			unsigned int getNblists();
			
			/**
 			 *  \brief Displays all registered lists
 			 */
			void Display() const;
			
			/**
 			 *  \brief Displays lists in container
 			 *
 			 *  \tparam Container container type
 			 *
 			 *  \param cont  container in which the lists are
 			 */
			template<typename Container>
			void Display(const Container &cont) const;
	};
}

#endif // _DISPLAY3D_H_
