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
 *  \brief Provides tests for skeleton library
 *  \author Bastien Durix
 */

#include <iostream>
#include <math.h>

#include <cstdlib>

#include <skeleton/GraphCurveSkeleton.h>
#include <skeleton/model/Classic.h>

using namespace mathtools::affine;

int main()
{
	unsigned int return_value = 0;
	
	// frame creation
	Frame<2>::Ptr frame(new Frame<2>{Eigen::Vector2d(0.0,1.0),Eigen::Vector2d(1.0,0.0),Eigen::Vector2d(0.0,0.0)});

	// model creation
	skeleton::model::Classic<2>::Ptr modclass(new skeleton::model::Classic<2>{frame});
	
	// creating a random node
	Eigen::Vector3d vec3((double)(rand()%201)/10.0-1.0,
						 (double)(rand()%201)/10.0-1.0,
						 (double)(rand()%201)/10.0);

	// skeleton creation
	skeleton::GraphCurveSkeleton<skeleton::model::Classic<2> > skel(modclass);
	
	std::cout << "Adding Node test... ";

	// adding a node
	unsigned int ind = skel.addNode(vec3);
	
	// getting the node (vector form)
	Eigen::Vector3d vec3_2 = skel.getNode(ind);
	
	if(vec3_2.isApprox(vec3,std::numeric_limits<double>::epsilon()))
	{
		std::cout << "Success!" << std::endl;
	}
	else
	{
		return_value = -1;
		std::cout << "Fail!" << std::endl;
	}
	
	std::cout << "Getter test... ";

	// getting the node (center form)
	Point<2> pt = skel.getNode<Point<2> >(ind);
	
	if(pt.getCoords().isApprox(frame->getBasis().getMatrix()*vec3.block<2,1>(0,0)+frame->getOrigin()))
	{
		std::cout << "Success!" << std::endl;
	}
	else
	{
		return_value = -1;
		std::cout << "Fail!" << std::endl;
	}

	return return_value;
}
