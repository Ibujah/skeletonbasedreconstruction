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
 *  \file VoronoiSkeleton2D.cpp
 *  \brief Defines functions to compute 2d skeleton with voronoi algorithm
 *  \author Bastien Durix
 */

#include "VoronoiSkeleton2D.h"
#include <voro++/voro++.hh>
#include <Eigen/Dense>
#include <mathtools/affine/Point.h>

#include <algorithm/graphoperation/ConnectedComponents.h>

skeleton::GraphSkel2d::Ptr algorithm::skeletonization::VoronoiSkeleton2d(const boundary::DiscreteBoundary<2>::Ptr disbnd)
{
	skeleton::model::Classic<2>::Ptr model(new skeleton::model::Classic<2>(disbnd->getFrame()));
	skeleton::GraphSkel2d::Ptr grskel(new skeleton::GraphSkel2d(model));
	
	std::vector<mathtools::affine::Point<2> > bndpts(0);
	disbnd->getVerticesPoint(bndpts);
	std::vector<Eigen::Vector2d> bndvec(bndpts.size());
	
	/**
	 *  Get the points coordinates in skeleton frame
	 */
	for(unsigned int i=0; i< bndpts.size(); i++)
	{
		bndvec[i] = bndpts[i].getCoords(model->getFrame());
	}
	
	/*
	 *  Build bounding box
	 */
	double xinf = bndvec[0](0);
	double xsup = bndvec[0](0);
	double yinf = bndvec[0](1);
	double ysup = bndvec[0](1);
	
	for(unsigned int i=0;i<bndpts.size();i++)
	{
		if(xinf>bndvec[i](0))
			xinf=bndvec[i](0);
		if(yinf>bndvec[i](1))
			yinf=bndvec[i](1);
		if(xsup<bndvec[i](0))
			xsup=bndvec[i](0);
		if(ysup<bndvec[i](1))
			ysup=bndvec[i](1);
	}
	
	double xsize = xsup-xinf;
	double ysize = ysup-yinf;
	
	xinf -= xsize*10;
	xsup += xsize*10;
	yinf -= ysize*10;
	ysup += ysize*10;
	
	/*
	 *  Build Voronoi container
	 */
	voro::container vorocont(xinf,xsup,
							 yinf,ysup,
							 -1.0, 1.0,
							 6,6,6,
							 false,false,false,
							 8);
	
	/*
	 *  Put points in voronoi container
	 */
	for(unsigned int i=0;i<bndpts.size();i++)
	{
		vorocont.put(i,bndvec[i](0),bndvec[i](1),0.0);
	}
	
	/*
	 *  Loop on cells (with neighbor information)
	 */
	voro::c_loop_all voroloopall(vorocont);
	
	/*
	 *  Nodes that are inside the skeleton
	 */
	std::vector<unsigned int> v_intsph;
	
	if(voroloopall.start())
	{
		do
		{
			voro::voronoicell_neighbor voroneigh;
			if(vorocont.compute_cell(voroneigh,voroloopall))/*For each cell*/
			{
				/*Put cell corners in skeleton*/
				double x,y,z;
				voroloopall.pos(x,y,z);
				
				std::vector<double> vert;
				voroneigh.vertices(x,y,z,vert);
				
				Eigen::Vector2d center(x,y);
				
				std::vector<unsigned int> indices(vert.size()/3);
				
				for(unsigned int i=0;i<vert.size()/3;i++)
				{
					Eigen::Vector3d corner(vert[i*3],vert[i*3+1],0.0);
					if(vert[i*3+2]==1.0)
					{
						corner(2) = (corner.block<2,1>(0,0)-center).norm();
						indices[i] = grskel->addNode(corner);
					}
				}
				
				/*
				 * Then link each corner to its neighbors
				 * Except if there is a link between the two cells
				 */
				for(unsigned int i=0;i<vert.size()/3;i++)
				{
					if(vert[i*3+2]==1.0)
					{
						for(int j=0;j<voroneigh.nu[i];j++) //nu : corner order (number of edge from this corner)
						{
							int ind_j = voroneigh.ed[i][j];
							if(vert[ind_j*3+2]==1.0)
							{
								bool found=false;
								unsigned int ind_neigh=0;
								/*Test if there is a link between the two cells*/
								for(int k=0;k<voroneigh.nu[i] && !found;k++)
								{
									if(voroneigh.ne[i][k] >=0 )
									{
										int face = voroneigh.ne[i][k];
										for(int l=0;l<voroneigh.nu[ind_j] && !found;l++)
										{
											if(face == voroneigh.ne[ind_j][l])
											{
												found=true;
												ind_neigh=face;
											}
										}
									}
								}

								if(!found)
								{
									grskel->addEdge(indices[i],indices[ind_j]);
								}
								else if(disbnd->getNext(voroloopall.pid())!=ind_neigh && disbnd->getNext(ind_neigh)!=(unsigned int)voroloopall.pid())
								{
									grskel->addEdge(indices[i],indices[ind_j]);
								}
								else if(disbnd->getNext(voroloopall.pid())==ind_neigh)
								{
									Eigen::Matrix2d mat;
									mat.block<2,1>(0,0) = bndpts[ind_neigh].getCoords() - bndpts[voroloopall.pid()].getCoords();
									mat.block<2,1>(0,1) = grskel->getNode(indices[ind_j]).block<2,1>(0,0) - grskel->getNode(indices[i]).block<2,1>(0,0);

									if(mat.determinant()<0)
										v_intsph.push_back(indices[ind_j]);
									else
										v_intsph.push_back(indices[i]);
								}
								else
								{
									Eigen::Matrix2d mat;
									mat.block<2,1>(0,0) = bndpts[ind_neigh].getCoords() - bndpts[voroloopall.pid()].getCoords();
									mat.block<2,1>(0,1) = grskel->getNode(indices[ind_j]).block<2,1>(0,0) - grskel->getNode(indices[i]).block<2,1>(0,0);

									if(mat.determinant()>0)
										v_intsph.push_back(indices[ind_j]);
									else
										v_intsph.push_back(indices[i]);
								}
							}
						}
					}
				}

			}
		}while(voroloopall.inc());
	}
	
	/*
	 *  Separation into connected components
	 */
	std::list<skeleton::GraphSkel2d::Ptr> list_comp = algorithm::graphoperation::SeparateComponents(grskel);

	skeleton::GraphSkel2d::Ptr skel_int(new skeleton::GraphSkel2d(model));
	for(std::list<skeleton::GraphSkel2d::Ptr>::iterator it = list_comp.begin(); it!=list_comp.end(); it++)
	{
		std::list<unsigned int> list_ver;
		(*it)->getAllNodes(list_ver);

		bool intern = false;
		for(unsigned int i=0;i<v_intsph.size() && !intern;i++)
		{
			if( std::find(list_ver.begin(),list_ver.end(),v_intsph[i]) != list_ver.end() )
			{
				intern=true;
			}
		}

		if(intern)
		{
			skel_int->insertExactSkel(*it);
		}
	}

	return skel_int;
}
