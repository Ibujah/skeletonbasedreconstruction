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
 *  \brief Provides tests for marching squares algorithm
 *  \author Bastien Durix
 */

#include <shape/DiscreteShape.h>
#include <boundary/DiscreteBoundary.h>

#include <algorithm/extractboundary/MarchingSquares.h>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE TestMarchingSquares
#include <boost/test/unit_test.hpp>

void verifybound(const boundary::DiscreteBoundary<2>::Ptr disbnd, const std::vector<Eigen::Vector2d> &wantedbnd, const std::map<unsigned int,unsigned int> &wantedneigh)
{
	std::vector<Eigen::Vector2d> vertbnd(0);
	disbnd->getVerticesVector(vertbnd);
	
	// step 1: associate each vertex from disbnd to a vertex of wantedbnd
	std::map<unsigned int,unsigned int> map_assoc;
	for(unsigned int i=0; i < vertbnd.size(); i++)
	{
		for(unsigned int j=0; j < wantedbnd.size(); j++)
		{
			if(vertbnd[i].isApprox(wantedbnd[j],2.0*std::numeric_limits<double>::epsilon()))
				map_assoc[j] = i;
		}
	}
	
	BOOST_REQUIRE( map_assoc.size() == wantedbnd.size() );

	for(std::map<unsigned int,unsigned int>::const_iterator it = wantedneigh.begin(); it != wantedneigh.end(); it++)
	{
		unsigned int ind1 = map_assoc[it->first];
		unsigned int ind2 = map_assoc[it->second];

		BOOST_REQUIRE( disbnd->getNext(ind1) == ind2 );
	}
}

BOOST_AUTO_TEST_CASE( MarchingSquares )
{
	// image preparation
	unsigned char img[11*11] = 
	{
	0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,1,1,1,1,1,0,0,0,
	0,1,0,1,0,1,0,1,0,0,0,
	0,1,1,1,1,1,1,1,1,1,0,
	0,0,0,1,1,0,1,1,0,1,0,
	0,0,0,1,1,0,1,1,0,0,0,
	0,0,0,1,1,1,1,1,1,0,0,
	0,0,0,1,0,0,0,0,1,0,0,
	0,0,0,1,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0
	};
	
	//wanted boundary preparation
	std::vector<Eigen::Vector2d> wantedbnd(42+4+4+6);
	std::map<unsigned int,unsigned int> wantedneigh;
	unsigned int num=0;
	wantedbnd[num++] = Eigen::Vector2d( 3.0, 1.5);
	wantedbnd[num++] = Eigen::Vector2d( 3.0, 2.5);
	wantedbnd[num++] = Eigen::Vector2d( 2.5, 3.0);
	wantedbnd[num++] = Eigen::Vector2d( 2.0, 2.5);
	wantedbnd[num++] = Eigen::Vector2d( 1.5, 2.0);
	wantedbnd[num++] = Eigen::Vector2d( 1.0, 2.5);
	wantedbnd[num++] = Eigen::Vector2d( 1.0, 3.5);
	wantedbnd[num++] = Eigen::Vector2d( 1.5, 4.0);
	wantedbnd[num++] = Eigen::Vector2d( 2.5, 4.0);
	wantedbnd[num++] = Eigen::Vector2d( 3.0, 4.5);
	wantedbnd[num++] = Eigen::Vector2d( 3.0, 5.5);
	wantedbnd[num++] = Eigen::Vector2d( 3.0, 6.5);
	wantedbnd[num++] = Eigen::Vector2d( 3.0, 7.5);
	wantedbnd[num++] = Eigen::Vector2d( 3.0, 8.5);
	wantedbnd[num++] = Eigen::Vector2d( 3.5, 9.0);
	wantedbnd[num++] = Eigen::Vector2d( 4.0, 8.5);
	wantedbnd[num++] = Eigen::Vector2d( 4.0, 7.5);
	wantedbnd[num++] = Eigen::Vector2d( 4.5, 7.0);
	wantedbnd[num++] = Eigen::Vector2d( 5.5, 7.0);
	wantedbnd[num++] = Eigen::Vector2d( 6.5, 7.0);
	wantedbnd[num++] = Eigen::Vector2d( 7.5, 7.0);
	wantedbnd[num++] = Eigen::Vector2d( 8.0, 7.5);
	wantedbnd[num++] = Eigen::Vector2d( 8.5, 8.0);
	wantedbnd[num++] = Eigen::Vector2d( 9.0, 7.5);
	wantedbnd[num++] = Eigen::Vector2d( 9.0, 6.5);
	wantedbnd[num++] = Eigen::Vector2d( 8.5, 6.0);
	wantedbnd[num++] = Eigen::Vector2d( 8.0, 5.5);
	wantedbnd[num++] = Eigen::Vector2d( 8.0, 4.5);
	wantedbnd[num++] = Eigen::Vector2d( 8.5, 4.0);
	wantedbnd[num++] = Eigen::Vector2d( 9.0, 4.5);
	wantedbnd[num++] = Eigen::Vector2d( 9.5, 5.0);
	wantedbnd[num++] = Eigen::Vector2d(10.0, 4.5);
	wantedbnd[num++] = Eigen::Vector2d(10.0, 3.5);
	wantedbnd[num++] = Eigen::Vector2d( 9.5, 3.0);
	wantedbnd[num++] = Eigen::Vector2d( 8.5, 3.0);
	wantedbnd[num++] = Eigen::Vector2d( 8.0, 2.5);
	wantedbnd[num++] = Eigen::Vector2d( 8.0, 1.5);
	wantedbnd[num++] = Eigen::Vector2d( 7.5, 1.0);
	wantedbnd[num++] = Eigen::Vector2d( 6.5, 1.0);
	wantedbnd[num++] = Eigen::Vector2d( 5.5, 1.0);
	wantedbnd[num++] = Eigen::Vector2d( 4.5, 1.0);
	wantedbnd[num++] = Eigen::Vector2d( 3.5, 1.0);
	for(unsigned int i=0; i < 41; i++)
		wantedneigh[i+1] = i;
	wantedneigh[0] = 41;

	wantedbnd[num++] = Eigen::Vector2d( 4.5, 2.0);
	wantedbnd[num++] = Eigen::Vector2d( 5.0, 2.5);
	wantedbnd[num++] = Eigen::Vector2d( 4.5, 3.0);
	wantedbnd[num++] = Eigen::Vector2d( 4.0, 2.5);
	for(unsigned int i=42; i < 45; i++)
		wantedneigh[i+1] = i;
	wantedneigh[42] = 45;

	wantedbnd[num++] = Eigen::Vector2d( 6.5, 2.0);
	wantedbnd[num++] = Eigen::Vector2d( 7.0, 2.5);
	wantedbnd[num++] = Eigen::Vector2d( 6.5, 3.0);
	wantedbnd[num++] = Eigen::Vector2d( 6.0, 2.5);
	for(unsigned int i=46; i < 49; i++)
		wantedneigh[i+1] = i;
	wantedneigh[46] = 49;

	wantedbnd[num++] = Eigen::Vector2d( 5.5, 4.0);
	wantedbnd[num++] = Eigen::Vector2d( 6.0, 4.5);
	wantedbnd[num++] = Eigen::Vector2d( 6.0, 5.5);
	wantedbnd[num++] = Eigen::Vector2d( 5.5, 6.0);
	wantedbnd[num++] = Eigen::Vector2d( 5.0, 5.5);
	wantedbnd[num++] = Eigen::Vector2d( 5.0, 4.5);
	for(unsigned int i=50; i < 55; i++)
		wantedneigh[i+1] = i;
	wantedneigh[50] = 55;
	
	shape::DiscreteShape<2>::Ptr disshp(new shape::DiscreteShape<2>(11,11));
	
	std::vector<unsigned char> &matbin = disshp->getContainer();
	
	for(unsigned int i=0; i < matbin.size(); i++)
		matbin[i] = img[i];
	
	boundary::DiscreteBoundary<2>::Ptr disbnd = algorithm::extractboundary::MarchingSquare(disshp);
	
	verifybound(disbnd,wantedbnd,wantedneigh);
}
