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
#include <mathtools/geometry/euclidian/HyperSphere.h>

using namespace mathtools::affine;
using namespace mathtools::geometry::euclidian;

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE TestMathtools
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE( SkeletonTest )
{
	// frame creation
	Frame<2>::Ptr frame = Frame<2>::CreateFrame(Eigen::Vector2d(1.0,0.0),Eigen::Vector2d(2.0,1.0),Eigen::Vector2d(1.0,0.0));

	// model creation
	skeleton::model::Classic<2>::Ptr modclass(new skeleton::model::Classic<2>{frame});
	
	// creating a random node
	Eigen::Vector3d vec3((double)(rand()%201)/10.0-1.0,
						 (double)(rand()%201)/10.0-1.0,
						 (double)(rand()%201)/10.0);

	// skeleton creation
	skeleton::GraphCurveSkeleton<skeleton::model::Classic<2> > skel(modclass);
	
	// adding a node
	unsigned int ind = skel.addNode(vec3);
	
	// getting the node (vector form)
	Eigen::Vector3d vec3_2 = skel.getNode(ind);
	
	BOOST_CHECK( vec3_2.isApprox(vec3,std::numeric_limits<double>::epsilon()) );
	
	// getting the node (center form)
	Point<2> pt = skel.getNode<Point<2> >(ind);
	
	BOOST_CHECK( pt.getCoords().isApprox(frame->getBasis()->getMatrix()*vec3.block<2,1>(0,0)+frame->getOrigin()) );

	// hypersphere creation
	HyperSphere<2> sph(Point<2>(1.0,0.0),5.0,frame);

	// adding a node
	unsigned int ind2 = skel.addNode<HyperSphere<2> >(sph);
	
	// get the vector associated to the node
	Eigen::Vector3d vecsph = skel.getNode(ind2);
	
	// convert the node to a sphere
	HyperSphere<2> sphcpy = skel.getModel()->toObj<HyperSphere<2> >(vecsph);
	
	BOOST_CHECK( sphcpy.getCenter().getCoords().isApprox(sph.getCenter().getCoords(),std::numeric_limits<double>::epsilon()) );
	BOOST_CHECK( sphcpy.getRadius() == sph.getRadius() );
	BOOST_CHECK( sphcpy.getFrame()->getBasis()->getMatrix().isApprox(sph.getFrame()->getBasis()->getMatrix(),std::numeric_limits<double>::epsilon()) );
	BOOST_CHECK( ind != ind2);
	
	// get all indices
	std::list<unsigned int> listnod;

	skel.getAllNodes(listnod);

	listnod.unique();

	BOOST_CHECK( listnod.size() == 2 );
	BOOST_CHECK( std::find(listnod.begin(),listnod.end(),ind) != listnod.end() );
	BOOST_CHECK( std::find(listnod.begin(),listnod.end(),ind2) != listnod.end() );
	
	// adding an edge
	skel.addEdge(ind, ind2);

	std::list<unsigned int> neigh;
	skel.getNeighbors(ind,neigh);
	
	BOOST_CHECK( neigh.size() == 1 );
	BOOST_CHECK( *(neigh.begin()) == ind2 );

	// removing an edge
	skel.remEdge(ind, ind2);
	
	neigh.erase(neigh.begin(),neigh.end());
	skel.getNeighbors(ind,neigh);
	
	BOOST_CHECK( neigh.size() == 0 );

	skel.remNode(ind);

	try
	{
		skel.getNode(ind);
		BOOST_ERROR( "Deleted a node already removed" );
	}
	catch(...)
	{}
}
