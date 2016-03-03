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
 *  \file SkelMatching2.cpp
 *  \brief Matches two skeletal branches
 *  \author Bastien Durix
 */

#include "SkelMatching2.h"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <mathtools/geometry/euclidian/Line.h>
#include <mathtools/geometry/euclidian/LineTriangulation.h>

void SkelMatchingGraph(
		skeleton::ReconstructionBranch::Ptr recbranch,
		const skeleton::BranchContProjSkel::Ptr projbr1,
		const skeleton::BranchContProjSkel::Ptr projbr2,
		const algorithm::matchskeletons::OptionsMatch &options)
{
	unsigned int size_map = options.nb_triang;
	Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> map_energy(size_map,size_map);
	
	double max_ener = 0;
	for(unsigned int i = 0;i<size_map;i++)
	{
		for(unsigned int j = 0; j<size_map; j++)
		{
			double t1 = (double)i/(double)(size_map-1);
			double t2 = (double)j/(double)(size_map-1);
			
			mathtools::geometry::euclidian::Line<4> line1 = projbr1->getNode<mathtools::geometry::euclidian::Line<4> >(t1);
			mathtools::geometry::euclidian::Line<4> line2 = projbr2->getNode<mathtools::geometry::euclidian::Line<4> >(t2);
			
			map_energy(i,j) = mathtools::geometry::euclidian::Triangulate(line1,line2).second;
			if(map_energy(i,j)>max_ener)
				max_ener = map_energy(i,j);
		}
	}
	
	//graph construction
	typedef boost::adjacency_list < boost::listS, boost::vecS, boost::directedS,
			boost::no_property, boost::property < boost::edge_weight_t, double > > graph_t;
	typedef boost::graph_traits < graph_t >::vertex_descriptor vertex_descriptor;
	typedef boost::graph_traits < graph_t >::edge_descriptor edge_descriptor;
	typedef std::pair<unsigned int, unsigned int> Edge;

	const unsigned int num_nodes = size_map*size_map;
	unsigned int num_arcs = (size_map-1)*size_map + size_map*(size_map-1) + (size_map-1)*(size_map-1);
	std::vector<Edge> edge_array(num_arcs);
	std::vector<double> weights(num_arcs);
	
	//vertical edges
	for(unsigned int i = 0; i<size_map-1;i++)
	{
		for(unsigned int j = 0; j<size_map;j++)
		{
			edge_array[i+j*(size_map-1)] = std::pair<unsigned int, unsigned int>(i+j*size_map,(i+1)+j*size_map);
			double diff = (map_energy(i+1,j) + map_energy(i,j))/2.0;
			weights[i+j*(size_map-1)] = diff + options.lambda;
		}
	}
	
	//horizontal edges
	for(unsigned int i = 0; i<size_map;i++)
	{
		for(unsigned int j = 0; j<size_map-1;j++)
		{
			edge_array[(size_map-1)*size_map + i+j*(size_map)] = std::pair<unsigned int, unsigned int>(i+j*size_map,i+(j+1)*size_map);
			double diff = (map_energy(i,j+1) + map_energy(i,j))/2.0;
			weights[(size_map-1)*size_map + i+j*(size_map)] = diff + options.lambda;
		}
	}
	
	//diagonal edges
	for(unsigned int i = 0; i<size_map-1;i++)
	{
		for(unsigned int j = 0; j<size_map-1;j++)
		{
			edge_array[(size_map-1)*size_map + size_map*(size_map-1) + i+j*(size_map-1)] = std::pair<unsigned int, unsigned int>(i+j*size_map,(i+1)+(j+1)*size_map);
			double diff = (map_energy(i+1,j+1) + map_energy(i,j))/2.0;
			weights[(size_map-1)*size_map + size_map*(size_map-1) + i+j*(size_map-1)] = sqrt(2)*(diff + options.lambda);
		}
	}

	graph_t g(&edge_array[0], &edge_array[0] + num_arcs, &weights[0], num_nodes);
	
	std::vector<vertex_descriptor> p(num_vertices(g));
	std::vector<double> d(num_vertices(g));
	vertex_descriptor s = vertex(0, g);
	
	//dijkstra computation
	dijkstra_shortest_paths(g, s, boost::predecessor_map(&p[0]).distance_map(&d[0]));
	
	std::list<Eigen::Vector2d> l_t;
	
	unsigned int ind = size_map*size_map-1;
	
	l_t.push_back(Eigen::Vector2d(1.0,1.0));
	
	do
	{
		ind = p[ind];
		unsigned int indx=ind%size_map,
				     indy=ind/size_map;
		double tx = (double)indx/(double)(size_map-1);
		double ty = (double)indy/(double)(size_map-1);
		Eigen::Matrix<double,Eigen::Dynamic,1> coords(2,1);
		coords(0) = tx;
		coords(1) = ty;
		l_t.push_front(coords);
	}while(ind !=0);
	
	std::vector<Eigen::Matrix<double,Eigen::Dynamic,1> > vectcoords(l_t.begin(),l_t.end());

	recbranch->setMatch(vectcoords);
}

void algorithm::matchskeletons::BranchMatching(
		skeleton::ReconstructionBranch::Ptr recbranch,
		const skeleton::BranchContProjSkel::Ptr projbr1,
		const skeleton::BranchContProjSkel::Ptr projbr2,
		const algorithm::matchskeletons::OptionsMatch &options)
{
	switch(options.methodmatch)
	{
		case OptionsMatch::enum_methodmatch::graph:
			SkelMatchingGraph(recbranch,projbr1,projbr2,options);
			break;
	}
}

void algorithm::matchskeletons::ComposedMatching(
		skeleton::ReconstructionSkeleton::Ptr recskel,
		const skeleton::CompContProjSkel::Ptr projskel1,
		const skeleton::CompContProjSkel::Ptr projskel2,
		const algorithm::matchskeletons::OptionsMatch &options)
{
	std::list<unsigned int> l_edge;
	recskel->getAllEdges(l_edge);

	for(std::list<unsigned int>::iterator it = l_edge.begin(); it != l_edge.end(); it++)
	{
		skeleton::ReconstructionBranch::Ptr recbr = recskel->getBranch(*it);
		typename skeleton::BranchContProjSkel::Ptr projbr1 = projskel1->getBranch(recbr->getFirstExt()[0],recbr->getLastExt()[0]);
		typename skeleton::BranchContProjSkel::Ptr projbr2 = projskel2->getBranch(recbr->getFirstExt()[1],recbr->getLastExt()[1]);

		BranchMatching(recbr,projbr1,projbr2,options);
	}
}
