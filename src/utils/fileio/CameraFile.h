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
 *  \file CameraFile.h
 *  \brief Defines camera file reader and writer
 *  \author Bastien Durix
 */

#include <camera/Camera.h>

/**
 *  \brief Gives file input and output functions
 */
namespace fileio
{
	/**
	 *  \brief Reads a camera file
	 *
	 *  \param filename camera filename
	 *
	 *  \return read file
	 */
	camera::Camera::Ptr ReadCamera(const std::string &filename);

	/**
	 *  \brief Writes a camera file
	 *
	 *  \param cam      camera to write
	 *  \param filename camera filename
	 */
	 void WriteCamera(const camera::Camera::Ptr cam, const std::string &filename);
}
