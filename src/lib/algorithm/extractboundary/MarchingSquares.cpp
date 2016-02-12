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
 *  \file MarchingSquares.cpp
 *  \brief Extracts a boundary with marching square algorithm
 *  \author Bastien Durix
 */

#include "MarchingSquares.h"
#include <map>
#include <Eigen/Dense>

boundary::DiscreteBoundary<2>::Ptr algorithm::extractboundary::MarchingSquare(const shape::DiscreteShape<2>::Ptr dissh, unsigned int step)
{
	boundary::DiscreteBoundary<2>::Ptr bnd(new boundary::DiscreteBoundary<2>(dissh->getFrame()));
	
	std::list<std::pair<unsigned int,unsigned int> > list_edg; //list of edges

	//cf: https://en.wikipedia.org/wiki/Marching_squares
	//first step: vertices adjacency computation
	#pragma omp parallel for
	for(unsigned int c = 0; c < dissh->getWidth() - step; c+=step)
	{
		#pragma omp parallel for
		for(unsigned int l = 0; l < dissh->getHeight() - step; l+=step)
		{
			unsigned int ind1 = c        + dissh->getWidth() * l;
			unsigned int ind2 = (c+step) + dissh->getWidth() * l;
			unsigned int ind3 = (c+step) + dissh->getWidth() * (l+step);
			unsigned int ind4 = c        + dissh->getWidth() * (l+step);
			
			unsigned char cellvalue = 0;
			if(c != 0                          && l != 0)                           cellvalue += dissh->getContainer()[ind1]?1:0;
			if(c+step < dissh->getWidth()-step && l != 0)                           cellvalue += dissh->getContainer()[ind2]?2:0;
			if(c+step < dissh->getWidth()-step && l+step < dissh->getHeight()-step) cellvalue += dissh->getContainer()[ind3]?4:0;
			if(c != 0                          && l+step < dissh->getHeight()-step) cellvalue += dissh->getContainer()[ind4]?8:0;
			
			unsigned int down  = (c*2+1*step) + (dissh->getWidth()*2+2) * (l*2       );
			unsigned int right = (c*2+2*step) + (dissh->getWidth()*2+2) * (l*2+1*step);
			unsigned int up    = (c*2+1*step) + (dissh->getWidth()*2+2) * (l*2+2*step);
			unsigned int left  = (c*2       ) + (dissh->getWidth()*2+2) * (l*2+1*step);
			
			std::list<std::pair<unsigned int,unsigned int> > local_edg;

			switch(cellvalue)
			{
				case 1:
					local_edg.push_back(std::pair<unsigned int,unsigned int>(down,left));
					break;
				case 2:
					local_edg.push_back(std::pair<unsigned int,unsigned int>(right,down));
					break;
				case 3:
					local_edg.push_back(std::pair<unsigned int,unsigned int>(right,left));
					break;
				case 4:
					local_edg.push_back(std::pair<unsigned int,unsigned int>(up,right));
					break;
				case 5:
					local_edg.push_back(std::pair<unsigned int,unsigned int>(left,up));
					local_edg.push_back(std::pair<unsigned int,unsigned int>(right,down));
					break;
				case 6:
					local_edg.push_back(std::pair<unsigned int,unsigned int>(up,down));
					break;
				case 7:
					local_edg.push_back(std::pair<unsigned int,unsigned int>(up,left));
					break;
				case 8:
					local_edg.push_back(std::pair<unsigned int,unsigned int>(left,up));
					break;
				case 9:
					local_edg.push_back(std::pair<unsigned int,unsigned int>(down,up));
					break;
				case 10:
					local_edg.push_back(std::pair<unsigned int,unsigned int>(right,up));
					local_edg.push_back(std::pair<unsigned int,unsigned int>(left,down));
					break;
				case 11:
					local_edg.push_back(std::pair<unsigned int,unsigned int>(right,up));
					break;
				case 12:
					local_edg.push_back(std::pair<unsigned int,unsigned int>(left,right));
					break;
				case 13:
					local_edg.push_back(std::pair<unsigned int,unsigned int>(down,right));
					break;
				case 14:
					local_edg.push_back(std::pair<unsigned int,unsigned int>(left,down));
					break;

				case 0:
				case 15:
				default:
					break;
			}
			#pragma omp critical
			list_edg.insert(list_edg.end(),local_edg.begin(),local_edg.end());
		}
	}

	std::map<unsigned int, unsigned int> map_neigh(list_edg.begin(),list_edg.end()); //map of nodes, linked in direct order
	
	//second step: link all the vertices on the boundary
	while(map_neigh.size() != 0)
	{
		std::list<Eigen::Vector2d> list_vert;
		
		std::map<unsigned int,unsigned int>::iterator it = map_neigh.begin();

		do
		{
			unsigned int l = ((it->first)/(dissh->getWidth()*2+2));
			unsigned int c = ((it->first)%(dissh->getWidth()*2+2));
			Eigen::Vector2d vec(0.5 + (double)(c)/2.0, 0.5 + (double)(l)/2.0);
			list_vert.push_back(vec);
			unsigned int val = it->second;
			map_neigh.erase(it);
			it = map_neigh.find(val);
		}while(it != map_neigh.end());
		
		bnd->addVerticesVector(list_vert);
	}

	return bnd;
}
