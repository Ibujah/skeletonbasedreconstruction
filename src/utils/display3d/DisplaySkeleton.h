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
 *  \file DisplaySkeleton.h
 *  \brief Provide method to display a skeleton
 *  \author Bastien Durix
 */

#ifndef _DISPLAYSKELETON_H_
#define _DISPLAYSKELETON_H_

#include "DisplayClass.h"
#include <skeleton/Skeletons.h>

/**
 *  \brief 3D displaying functions
 */
namespace display3d
{
	/**
	 *  \brief Displays a continuous skeletal branch
	 *
	 *  \param disclass  display class containing the opengl context
	 *  \param contbr    continuous branch to display
	 *
	 *  \return index of the list containing the branch
	 */
	unsigned int DisplayBranch(DisplayClass &disclass, const skeleton::BranchContSkel3d::Ptr contbr);

	/**
	 *  \brief Displays a continuous skeleton
	 *
	 *  \param disclass  display class containing the opengl context
	 *  \param contskel  continuous skeleton to display
	 *
	 *  \return index of the list containing the skeleton
	 */
	unsigned int DisplaySkeleton(DisplayClass &disclass, const skeleton::CompContSkel3d::Ptr contskel);
}


#endif // _DISPLAYSKELETON_H_
