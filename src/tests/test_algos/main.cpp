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
 *  \brief Provides tests for some algorithms
 *  \author Bastien Durix
 */

#include <shape/DiscreteShape.h>
#include <boundary/DiscreteBoundary.h>
#include <skeleton/GraphCurveSkeleton.h>

#include <algorithm/extractboundary/MarchingSquares.h>
#include <algorithm/graphoperation/ConnectedComponents.h>
#include <algorithm/skeletonization/VoronoiSkeleton2D.h>

#include <iostream>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE TestAlgos
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

BOOST_AUTO_TEST_CASE( ComponentSeparation )
{
	skeleton::GraphSkel2d::Ptr grskel(new skeleton::GraphSkel2d(skeleton::model::Classic<2>{}));
	
	unsigned int ind0 = grskel->addNode(Eigen::Vector3d(1.0,1.0,0.5));
	unsigned int ind1 = grskel->addNode(Eigen::Vector3d(2.0,1.0,0.5));
	unsigned int ind2 = grskel->addNode(Eigen::Vector3d(3.0,1.0,0.5));
	unsigned int ind3 = grskel->addNode(Eigen::Vector3d(4.0,1.0,0.5));
	unsigned int ind4 = grskel->addNode(Eigen::Vector3d(5.0,1.0,0.5));
	unsigned int ind5 = grskel->addNode(Eigen::Vector3d(6.0,1.0,0.5));
	
	std::vector<std::list<unsigned int> > list_nod(3);
	// 0 (alone)
	list_nod[0].push_back(ind0);
	list_nod[0].sort();

	// 1-2
	grskel->addEdge(ind1,ind2);
	list_nod[1].push_back(ind1);
	list_nod[1].push_back(ind2);
	list_nod[1].sort();

	// 3-4-5
	grskel->addEdge(ind4,ind3);
	grskel->addEdge(ind4,ind5);
	list_nod[2].push_back(ind3);
	list_nod[2].push_back(ind4);
	list_nod[2].push_back(ind5);
	list_nod[2].sort();

	std::list<skeleton::GraphSkel2d::Ptr> list_gr = algorithm::graphoperation::SeparateComponents(grskel);
	
	BOOST_REQUIRE(list_gr.size() == 3);

	for(std::list<skeleton::GraphSkel2d::Ptr>::iterator it = list_gr.begin(); it != list_gr.end(); it++)
	{
		skeleton::GraphSkel2d::Ptr cur_skel = *it;
		std::list<unsigned int> ind;
		cur_skel->getAllNodes(ind);
		unsigned int cur_list = 0;
		if(ind.size() == 1) // 0
		{
			cur_list = 0;
		}
		else if(ind.size() == 2) // 1-2
		{
			cur_list = 1;
		}
		else if(ind.size() == 3) // 3-4-5
		{
			cur_list = 2;
		}
		else
		{
			BOOST_FAIL("Wrong number of nodes in the sub skeleton");
		}
		
		ind.sort();
		for(std::list<unsigned int>::iterator itl = list_nod[cur_list].begin(); itl != list_nod[cur_list].end(); itl++)
		{
			BOOST_REQUIRE(std::find(ind.begin(),ind.end(),*itl) != ind.end());
		}
	}
}

void verifyskel(const skeleton::GraphSkel2d::Ptr grskel, const std::vector<Eigen::Vector3d> &wantedskl, const std::list<std::pair<unsigned int,unsigned int> > &wantededg)
{
	std::vector<unsigned int> index(0);
	grskel->getAllNodes(index);
	
	std::map<unsigned int, unsigned int> map_assoc;
	for(unsigned int i = 0; i < wantedskl.size(); i++)
	{
		for(unsigned int j = 0; j < wantedskl.size(); j++)
		{
			if(grskel->getNode(index[i]).isApprox(wantedskl[j],std::numeric_limits<double>::epsilon()))
				map_assoc[j] = i;
		}
	}

	BOOST_REQUIRE(map_assoc.size() == wantedskl.size());

	for(std::list<std::pair<unsigned int,unsigned int> >::const_iterator it = wantededg.begin(); it != wantededg.end(); it++)
	{
		BOOST_CHECK(grskel->areNeighbors(it->first,it->second));
		BOOST_CHECK(grskel->areNeighbors(it->second,it->first));
	}

}

BOOST_AUTO_TEST_CASE( Skeletonization )
{
	// image preparation
	unsigned char img[4*4] = 
	{
	0,0,0,0,
	0,1,1,0,
	0,0,1,0,
	0,0,0,0
	};

	//wanted boundary preparation
	std::vector<Eigen::Vector2d> wantedbnd(8);
	std::map<unsigned int,unsigned int> wantedneigh;
	unsigned int num=0;
	wantedbnd[num++] = Eigen::Vector2d(1.5,1.0);
	wantedbnd[num++] = Eigen::Vector2d(2.5,1.0);
	wantedbnd[num++] = Eigen::Vector2d(3.0,1.5);
	wantedbnd[num++] = Eigen::Vector2d(3.0,2.5);
	wantedbnd[num++] = Eigen::Vector2d(2.5,3.0);
	wantedbnd[num++] = Eigen::Vector2d(2.0,2.5);
	wantedbnd[num++] = Eigen::Vector2d(1.5,2.0);
	wantedbnd[num++] = Eigen::Vector2d(1.0,1.5);
	for(unsigned int i=0; i < 8; i++)
		wantedneigh[i] = i+1;
	wantedneigh[7] = 0;

	shape::DiscreteShape<2>::Ptr disshp(new shape::DiscreteShape<2>(4,4));
	
	std::vector<unsigned char> &matbin = disshp->getContainer();
	
	for(unsigned int i=0; i < matbin.size(); i++)
		matbin[i] = img[i];
	
	boundary::DiscreteBoundary<2>::Ptr disbnd = algorithm::extractboundary::MarchingSquare(disshp);
	
	verifybound(disbnd,wantedbnd,wantedneigh);

	skeleton::GraphSkel2d::Ptr grskel = algorithm::skeletonization::VoronoiSkeleton2d(disbnd);
	
	std::vector<Eigen::Vector3d> wantedskl(5);
	std::list<std::pair<unsigned int,unsigned int> > wantededg;
	num=0;
	wantedskl[num++] = Eigen::Vector3d(1.5 , 1.5 , 0.5);
	wantedskl[num++] = Eigen::Vector3d(2.0 , 1.5 , sqrt(2.0)/2.0);
	wantedskl[num++] = Eigen::Vector3d(2.25, 1.75, sqrt(10.0)/2.0);
	wantedskl[num++] = Eigen::Vector3d(2.5 , 2.0 , sqrt(2.0)/2.0);
	wantedskl[num++] = Eigen::Vector3d(2.5 , 2.5 , 0.5);
	wantededg.push_back(std::pair<unsigned int, unsigned int>(0,1));
	wantededg.push_back(std::pair<unsigned int, unsigned int>(1,2));
	wantededg.push_back(std::pair<unsigned int, unsigned int>(2,3));
	wantededg.push_back(std::pair<unsigned int, unsigned int>(3,4));
}
