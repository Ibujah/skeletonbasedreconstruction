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

boundary::DiscreteBoundary<2>::Ptr algorithm::extractboundary::MarchingSquare(const shape::DiscreteShape<2>::Ptr dissh)
{
	boundary::DiscreteBoundary<2>::Ptr bnd(new boundary::DiscreteBoundary<2>(dissh->getFrame()));
	
	std::map<unsigned int, Eigen::Vector2d> map_vert; //map of vertices
	std::map<unsigned int, unsigned int> map_neigh; //map of nodes, linked in direct order

	//first step: vertices adjacency computation
	for(unsigned int c = 0; c < dissh->getWidth() - 1; c++)
	{
		for(unsigned int l = 0; l < dissh->getHeight() - 1; l++)
		{
			unsigned int ind1 = c     + dissh->getWidth() * l;
			unsigned int ind2 = (c+1) + dissh->getWidth() * l;
			unsigned int ind3 = (c+1) + dissh->getWidth() * (l+1);
			unsigned int ind4 = c     + dissh->getWidth() * (l+1);
			
			unsigned char cellvalue = 0;
			cellvalue += dissh->getContainer()[ind1]?1:0;
			cellvalue += dissh->getContainer()[ind2]?2:0;
			cellvalue += dissh->getContainer()[ind3]?4:0;
			cellvalue += dissh->getContainer()[ind4]?8:0;
			
			unsigned int up    = (c*2+1) + (dissh->getWidth()*2+1) * (l*2);
			unsigned int right = (c*2+2) + (dissh->getWidth()*2+1) * (l*2+1);
			unsigned int down  = (c*2+1) + (dissh->getWidth()*2+1) * (l*2+2);
			unsigned int left  = (c*2)   + (dissh->getWidth()*2+1) * (l*2+1);
			
			map_vert[up]    = Eigen::Vector2d(0.5 + (double)c + 0.5, 0.5 + (double)l);
			map_vert[right] = Eigen::Vector2d(0.5 + (double)c + 1.0, 0.5 + (double)l + 0.5);
			map_vert[down]  = Eigen::Vector2d(0.5 + (double)c + 0.5, 0.5 + (double)l + 1.0);
			map_vert[left]  = Eigen::Vector2d(0.5 + (double)c,       0.5 + (double)l + 0.5);
			
			//cf: https://en.wikipedia.org/wiki/Marching_squares

			switch(cellvalue)
			{
				case 1:
					map_neigh[down] = left;
					break;
				case 2:
					map_neigh[right] = down;
					break;
				case 3:
					map_neigh[right] = left;
					break;
				case 4:
					map_neigh[up] = right;
					break;
				case 5:
					map_neigh[left] = up;
					map_neigh[right] = down;
					break;
				case 6:
					map_neigh[down] = up;
					break;
				case 7:
					map_neigh[left] = up;
					break;
				case 8:
					map_neigh[up] = left;
					break;
				case 9:
					map_neigh[up] = down;
					break;
				case 10:
					map_neigh[down] = left;
					map_neigh[up] = right;
					break;
				case 11:
					map_neigh[up] = right;
					break;
				case 12:
					map_neigh[left] = right;
					break;
				case 13:
					map_neigh[down] = right;
					break;
				case 14:
					map_neigh[left] = down;
					break;

				case 0:
				case 15:
				default:
					break;
			}
		}
	}
	
	//second step: link all the vertices on the boundary
	do
	{
		std::list<Eigen::Vector2d> list_vert;
		
		std::map<unsigned int,unsigned int>::iterator it = map_neigh.begin();

		while(it != map_neigh.end())
		{
			list_vert.push_back(map_vert[it->first]);
			unsigned int val = it->second;
			map_neigh.erase(it);
			it = map_neigh.find(val);
		}
		
		bnd->addVerticesVector(list_vert);

	}while(map_neigh.size() != 0);
	return bnd;
}
