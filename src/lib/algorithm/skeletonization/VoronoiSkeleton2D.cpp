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
#include <skeleton/model/Orthographic.h>
#include <skeleton/model/Perspective.h>

#include <algorithm/graphoperation/ConnectedComponents.h>

template<typename Model>
void VoronoiOrtho(typename skeleton::GraphCurveSkeleton<Model>::Ptr grskel, const boundary::DiscreteBoundary<2>::Ptr disbnd, const mathtools::affine::Frame<2>::Ptr frame)
{
	std::vector<mathtools::affine::Point<2> > bndpts(0);
	disbnd->getVerticesPoint(bndpts);
	std::vector<Eigen::Vector2d> bndvec(bndpts.size());
	
	/**
	 *  Get the points coordinates in skeleton frame
	 */
	double xinf = 0;
	double xsup = 0;
	double yinf = 0;
	double ysup = 0;
	for(unsigned int i=0; i< bndpts.size(); i++)
	{
		bndvec[i] = bndpts[i].getCoords(frame);
		if(xinf>bndvec[i](0) || i==0)
			xinf=bndvec[i](0);
		if(yinf>bndvec[i](1) || i==0)
			yinf=bndvec[i](1);
		if(xsup<bndvec[i](0) || i==0)
			xsup=bndvec[i](0);
		if(ysup<bndvec[i](1) || i==0)
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
	std::list<unsigned int> v_intsph;
	
	/*
	 *  Added nodes
	 */
	std::map<unsigned int,Eigen::Vector3d> v_added;

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
				
				std::vector<unsigned int> indices(voroneigh.p);
				
				for(unsigned int i=0;i<(unsigned int)voroneigh.p;i++)
				{
					if(vert[i*3+2]==1.0)
					{
						Eigen::Vector3d corner(vert[i*3],vert[i*3+1],0.0);
						corner(2) = (corner.block<2,1>(0,0)-center).norm();
						bool isin=false;
						for(std::map<unsigned int,Eigen::Vector3d>::iterator it = v_added.begin(); it != v_added.end() & !isin; it++)
						{
							if(it->second.isApprox(corner,std::numeric_limits<float>::epsilon()))
							{
								isin=true;
								indices[i] = it->first;
							}
						}
						if(!isin)
						{
							indices[i] = grskel->addNode(corner);
							v_added[indices[i]] = corner;
						}
					}
				}
				
				/*
				 * Then link each corner to its neighbors
				 * Except if there is a link between the two cells
				 */
				for(unsigned int i=0;i<(unsigned int)voroneigh.p;i++)
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
								else if(disbnd->getNext(ind_neigh)==(unsigned int)voroloopall.pid() && disbnd->getFrame()->getBasis()->isDirect())
								{
									Eigen::Matrix2d mat;
									mat.block<2,1>(0,0) = bndpts[ind_neigh].getCoords() - bndpts[voroloopall.pid()].getCoords();
									mat.block<2,1>(0,1) = grskel->getNode(indices[ind_j]).template block<2,1>(0,0) - grskel->getNode(indices[i]).template block<2,1>(0,0);

									if(mat.determinant()<0)
										v_intsph.push_back(indices[ind_j]);
									else
										v_intsph.push_back(indices[i]);
								}
								else if(disbnd->getNext((unsigned int)voroloopall.pid())==ind_neigh && !disbnd->getFrame()->getBasis()->isDirect())
								{
									Eigen::Matrix2d mat;
									mat.block<2,1>(0,0) = bndpts[ind_neigh].getCoords() - bndpts[voroloopall.pid()].getCoords();
									mat.block<2,1>(0,1) = grskel->getNode(indices[ind_j]).template block<2,1>(0,0) - grskel->getNode(indices[i]).template block<2,1>(0,0);

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
	
	v_intsph.sort();
	v_intsph.unique();

	/*
	 *  Separation into connected components
	 */
	std::list<typename skeleton::GraphCurveSkeleton<Model>::Ptr> list_comp = algorithm::graphoperation::SeparateComponents(grskel);

	typename skeleton::GraphCurveSkeleton<Model>::Ptr skel_int(new skeleton::GraphCurveSkeleton<Model>(grskel->getModel()));
	for(typename std::list<typename skeleton::GraphCurveSkeleton<Model>::Ptr>::iterator it = list_comp.begin(); it!=list_comp.end(); it++)
	{
		std::list<unsigned int> list_ver;
		(*it)->getAllNodes(list_ver);

		bool intern = false;
		for(std::list<unsigned int>::iterator itl = v_intsph.begin(); itl != v_intsph.end() && !intern; itl++)
		{
			if( std::find(list_ver.begin(),list_ver.end(),*itl) != list_ver.end() )
			{
				intern=true;
			}
		}

		if(intern)
		{
			skel_int->insertExactSkel(*it);
		}
	}
}

void VoronoiPersp(skeleton::GraphProjSkel::Ptr grskel, const boundary::DiscreteBoundary<2>::Ptr disbnd)
{
	std::vector<mathtools::affine::Point<2> > bndpts(0);
	disbnd->getVerticesPoint(bndpts);
	std::vector<Eigen::Vector2d> bndvec(bndpts.size());

	/*
	 *  Build Voronoi container
	 */

	voro::container vorocont(-10.0,10.0,
							 -10.0,10.0,
							 -10.0,10.0,
							 6,6,6,
							 false,false,false,
							 8);

	/*
	 *  Put points in voronoi container
	 */
	for(unsigned int i=0;i<bndpts.size();i++)
	{
		bndvec[i] = bndpts[i].getCoords().normalized();
		vorocont.put(i,bndvec[i](0),bndvec[i](1),bndvec[i](2));
	}

	/*
	 *  Loop on cells (with neighbor information)
	 */
	voro::voronoicell_neighbor voroneigh;

	vorocont.compute_ghost_cell(voroneigh,0.0,0.0,0.0);

	/*
	 *  Nodes that are inside the skeleton
	 */
	std::list<unsigned int> v_intsph;

	/*Put cell corners in skeleton*/
	std::vector<double> vert;
	voroneigh.vertices(0.0,0.0,0.0,vert);

	std::vector<unsigned int> indices((unsigned int)voroneigh.p);
	std::vector<char> isIn((unsigned int)voroneigh.p);

	for(unsigned int i=0;i<(unsigned int)voroneigh.p;i++)
	{
		Eigen::Vector3d corner(vert[i*3],vert[i*3+1],vert[i*3+2]);
		double sqnorm=corner.squaredNorm();
		if(sqnorm<1.0)
		{
			isIn[i]=1;
			Eigen::Vector3d vec_dir(corner.x()/corner.z(),corner.y()/corner.z(),sqrt(sqnorm-1./4.)/corner.z());
			indices[i] = grskel->addNode(vec_dir);
		}
	}

	/*
	 * Then link each corner to its neighbors
	 * Except if there is a link between the two cells
	 */
	for(unsigned int i=0;i<vert.size()/3;i++)
	{
		if(isIn[i])
		{
			for(int j=0;j<voroneigh.nu[i];j++) //nu : corner order (number of edge from this corner)
			{
				int ind_j = voroneigh.ed[i][j];
				if(isIn[ind_j])
				{
					/*Test if there is a link between the two cells*/
					std::vector<unsigned int> ind_neigh(0);
					for(int k=0;k<voroneigh.nu[i];k++)
					{
						if(voroneigh.ne[i][k]>=0)
						{
							int face = voroneigh.ne[i][k];
							for(int l=0;l<voroneigh.nu[ind_j];l++)
							{
								if(face == voroneigh.ne[ind_j][l])
									ind_neigh.push_back(face);
							}
						}
					}

					if(ind_neigh.size()!=2)
					{
						grskel->addEdge(indices[i],indices[ind_j]);
					}
					else if(disbnd->getNext(ind_neigh[0]) != ind_neigh[1] && disbnd->getNext(ind_neigh[1]) != ind_neigh[0])
					{
						grskel->addEdge(indices[i],indices[ind_j]);
					}
					else if(disbnd->getNext(ind_neigh[0]) == ind_neigh[1] && disbnd->getFrame()->getBasis()->isDirect())
					{
						Eigen::Matrix2d mat;
						mat.block<2,1>(0,0) = bndpts[ind_neigh[1]].getCoords() - bndpts[ind_neigh[0]].getCoords();
						mat.block<2,1>(0,1) = grskel->getNode(indices[ind_j]).block<2,1>(0,0) - bndpts[ind_neigh[0]].getCoords();

						if(mat.determinant()>0)
							v_intsph.push_back(indices[i]);
					}
					else if(disbnd->getNext(ind_neigh[1]) == ind_neigh[0] && !disbnd->getFrame()->getBasis()->isDirect())
					{
						Eigen::Matrix2d mat;
						mat.block<2,1>(0,0) = bndpts[ind_neigh[1]].getCoords() - bndpts[ind_neigh[0]].getCoords();
						mat.block<2,1>(0,1) = grskel->getNode(indices[ind_j]).block<2,1>(0,0) - bndpts[ind_neigh[0]].getCoords();

						if(mat.determinant()<0)
							v_intsph.push_back(indices[i]);
					}
				}
			}
		}
	}

	v_intsph.sort();
	v_intsph.unique();

	/*
	 *  Separation into connected components
	 */
	std::list<skeleton::GraphProjSkel::Ptr> list_comp = algorithm::graphoperation::SeparateComponents(grskel);

	skeleton::GraphProjSkel::Ptr skel_int(new skeleton::GraphProjSkel(grskel->getModel()));
	for(std::list<skeleton::GraphProjSkel::Ptr>::iterator it = list_comp.begin(); it!=list_comp.end(); it++)
	{
		std::list<unsigned int> list_ver;
		(*it)->getAllNodes(list_ver);

		bool intern = false;
		for(std::list<unsigned int>::iterator itl = v_intsph.begin(); itl != v_intsph.end() && !intern; itl++)
		{
			if( std::find(list_ver.begin(),list_ver.end(),*itl) != list_ver.end() )
			{
				intern=true;
			}
		}

		if(intern)
		{
			skel_int->insertExactSkel(*it);
		}
	}
}

skeleton::GraphSkel2d::Ptr algorithm::skeletonization::VoronoiSkeleton2d(const boundary::DiscreteBoundary<2>::Ptr disbnd)
{
	skeleton::model::Classic<2>::Ptr model(new skeleton::model::Classic<2>(disbnd->getFrame()));
	skeleton::GraphSkel2d::Ptr grskel(new skeleton::GraphSkel2d(model));
	
	VoronoiOrtho<skeleton::model::Classic<2> >(grskel,disbnd,model->getFrame());

	return grskel;
}

skeleton::GraphProjSkel::Ptr algorithm::skeletonization::ProjectiveVoronoi(const boundary::DiscreteBoundary<2>::Ptr disbnd, const camera::Camera::Ptr camera)
{
	skeleton::model::Projective::Ptr model;
	skeleton::GraphProjSkel::Ptr grskel(new skeleton::GraphProjSkel(model));
	switch(camera->getIntrinsics()->getType())
	{
		case camera::Intrinsics::Type::ortho:
			model  = skeleton::model::Projective::Ptr(new skeleton::model::Orthographic(camera->getExtrinsics()->getFrame()));
			grskel = skeleton::GraphProjSkel::Ptr(new skeleton::GraphProjSkel(model));
			VoronoiOrtho<skeleton::model::Projective>(grskel,disbnd,camera->getIntrinsics()->getFrame());
			break;
		case camera::Intrinsics::Type::pinhole:
			model = skeleton::model::Projective::Ptr(new skeleton::model::Orthographic(camera->getExtrinsics()->getFrame()));
			grskel = skeleton::GraphProjSkel::Ptr(new skeleton::GraphProjSkel(model));
			break;
	}
	
	return grskel;
}
