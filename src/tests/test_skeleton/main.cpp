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

#include <skeleton/GraphCurveSkeleton.h>
#include <skeleton/model/Classic.h>
#include <mathtools/geometry/euclidian/HyperSphere.h>

using namespace mathtools::affine;
using namespace mathtools::geometry::euclidian;

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE TestMathtools
#include <boost/test/unit_test.hpp>

skeleton::GraphCurveSkeleton<skeleton::model::Classic<2> >::Ptr skel = skeleton::GraphCurveSkeleton<skeleton::model::Classic<2> >::Ptr();
Frame<2>::Ptr frame = Frame<2>::Ptr();
skeleton::model::Classic<2>::Ptr modclass = skeleton::model::Classic<2>::Ptr();
unsigned int ind1 = 0;
unsigned int ind2 = 0;

BOOST_AUTO_TEST_CASE( SkeletonCreation )
{
	// frame creation
	frame = Frame<2>::CreateFrame(Eigen::Vector2d(1.0,0.0),Eigen::Vector2d(2.0,1.0),Eigen::Vector2d(1.0,0.0));

	// model creation
	modclass = skeleton::model::Classic<2>::Ptr(new skeleton::model::Classic<2>{frame});
	
	// skeleton creation
	skel = skeleton::GraphCurveSkeleton<skeleton::model::Classic<2> >::Ptr(new skeleton::GraphCurveSkeleton<skeleton::model::Classic<2> >(modclass));
}

BOOST_AUTO_TEST_CASE( AddingNode )
{
	// creating a random node
	Eigen::Vector3d vec3((double)(rand()%201)/10.0-1.0,
						 (double)(rand()%201)/10.0-1.0,
						 (double)(rand()%201)/10.0);

	// adding a node
	ind1 = skel->addNode(vec3);
	
	// getting the node (vector form)
	Eigen::Vector3d vec = skel->getNode(ind1);
	
	BOOST_REQUIRE( vec.isApprox(vec3,std::numeric_limits<double>::epsilon()) );
	
	// getting the node (center form)
	Point<2> pt = skel->getNode<Point<2> >(ind1);
	
	BOOST_REQUIRE( pt.getCoords().isApprox(frame->getBasis()->getMatrix()*vec3.block<2,1>(0,0)+frame->getOrigin()) );
}

BOOST_AUTO_TEST_CASE( AddingSphere )
{
	// hypersphere creation
	HyperSphere<2> sph(Point<2>(1.0,0.0),5.0,frame);

	// adding a node
	ind2 = skel->addNode<HyperSphere<2> >(sph);
	
	// get the vector associated to the node
	Eigen::Vector3d vecsph = skel->getNode(ind2);
	
	// convert the node to a sphere
	HyperSphere<2> sphcpy = skel->getModel()->toObj<HyperSphere<2> >(vecsph);
	
	BOOST_REQUIRE( sphcpy.getCenter().getCoords().isApprox(sph.getCenter().getCoords(),std::numeric_limits<double>::epsilon()) );
	BOOST_REQUIRE( sphcpy.getRadius() == sph.getRadius() );
	BOOST_REQUIRE( sphcpy.getFrame()->getBasis()->getMatrix().isApprox(sph.getFrame()->getBasis()->getMatrix(),std::numeric_limits<double>::epsilon()) );
	BOOST_REQUIRE( ind1 != ind2);
}

BOOST_AUTO_TEST_CASE( getAllNodes )
{
	// get all indices
	std::list<unsigned int> listnod;

	skel->getAllNodes(listnod);

	listnod.unique();

	BOOST_REQUIRE( listnod.size() == 2 );
	BOOST_REQUIRE( std::find(listnod.begin(),listnod.end(),ind1) != listnod.end() );
	BOOST_REQUIRE( std::find(listnod.begin(),listnod.end(),ind2) != listnod.end() );
}

BOOST_AUTO_TEST_CASE( addEdge )
{
	// adding an edge
	skel->addEdge(ind1, ind2);

	std::list<unsigned int> neigh;
	skel->getNeighbors(ind1,neigh);
	
	BOOST_REQUIRE( neigh.size() == 1 );
	BOOST_REQUIRE( *(neigh.begin()) == ind2 );
}

BOOST_AUTO_TEST_CASE( remEdge )
{
	// removing an edge
	skel->remEdge(ind1, ind2);
	
	std::list<unsigned int> neigh;
	skel->getNeighbors(ind1,neigh);
	
	BOOST_REQUIRE( neigh.size() == 0 );
}

BOOST_AUTO_TEST_CASE( remNode )
{
	skel->remNode(ind1);

	try
	{
		skel->getNode(ind1);
		BOOST_FAIL( "Deleted a node already removed" );
	}
	catch(...)
	{}
}
