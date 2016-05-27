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
 *  \brief Skeleton reconstruction evaluation
 *  \author Bastien Durix
 */

#include <boost/program_options.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include <skeleton/Skeletons.h>
#include <camera/Camera.h>
#include <shape/DiscreteShape.h>
#include <boundary/DiscreteBoundary2.h>
#include <boundary/DiscreteBoundary3.h>

#include <algorithm/extractboundary/MarchingSquares.h>
#include <algorithm/skeletonization/VoronoiSkeleton2D.h>
#include <algorithm/pruning/ScaleAxisTransform.h>
#include <algorithm/graphoperation/SeparateBranches.h>
#include <algorithm/graphoperation/AssociateSkeletons.h>
#include <algorithm/fitbspline/Graph2Bspline.h>
#include <algorithm/matchskeletons/SkelMatching2.h>
#include <algorithm/matchskeletons/SkelMatching.h>
#include <algorithm/matchskeletons/SkelTriangulation.h>
#include <algorithm/skinning/ContinuousSkinning.h>
#include <algorithm/evaluation/ReprojError.h>

#include <userinput/ClickSkelNode.h>
#include <fileio/CameraFile.h>
#include <fileio/BoundaryFile.h>
#include <fileio/RecSkelFile.h>
#include <fileio/ExtClickFile.h>
#include <displayopencv/DisplayShapeOCV.h>
#include <displayopencv/DisplayBoundaryOCV.h>
#include <displayopencv/DisplaySkeletonOCV.h>

void ReconstructionEvals(algorithm::matchskeletons::OptionsMatch &optionsmatch,
						 const std::vector<typename skeleton::GraphProjSkel::Ptr> &vec_prskel,
						 const skeleton::ReconstructionSkeleton::Ptr recskel,
						 const std::vector<camera::Camera::Ptr> &veccam,
						 const std::vector<shape::DiscreteShape<2>::Ptr> &vecshape,
						 const double lambda)
{
	std::vector<typename skeleton::CompGraphProjSkel::Ptr> vec_comppr = algorithm::graphoperation::GetComposed(recskel,vec_prskel);
	
	std::vector<typename skeleton::CompContProjSkel::Ptr> vec_compcontpr(vec_comppr.size());
	
	for(unsigned int i = 0; i < vec_comppr.size(); i++)
	{
		vec_compcontpr[i] = algorithm::fitbspline::Graph2Bspline(vec_comppr[i]);
	}

	algorithm::matchskeletons::OptionsMatch2 optionsmatch2;
	optionsmatch2.lambda = lambda;
	optionsmatch2.methodmatch = algorithm::matchskeletons::OptionsMatch2::graph;

	double minerr_graph = 1.0;

	std::cout << std::endl << "~~ Matching graph ~~" << std::endl;
	for(unsigned int sel0 = 0; sel0 < vec_prskel.size()-1; sel0++)
	{
		for(unsigned int sel1 = sel0+1; sel1 < vec_prskel.size(); sel1++)
		{
			skeleton::ReconstructionSkeleton::Ptr recskel2(new skeleton::ReconstructionSkeleton());
			std::list<unsigned int> lnodes;
			recskel->getAllNodes(lnodes);
			for(std::list<unsigned int>::iterator it = lnodes.begin(); it != lnodes.end(); it++)
			{
				recskel2->addNode(*it);
			}
			std::list<unsigned int> ledges;
			recskel->getAllEdges(ledges);
			for(std::list<unsigned int>::iterator it = ledges.begin(); it != ledges.end(); it++)
			{
				std::pair<unsigned int,unsigned int> ext = recskel->getExtremities(*it);
				const skeleton::ReconstructionBranch::Ptr br = recskel->getBranch(ext.first,ext.second);
				std::vector<unsigned int> indskel(2);
				std::vector<unsigned int> firstext(2);
				std::vector<unsigned int> lastext(2);

				indskel[0] = sel0;
				indskel[1] = sel1;
				firstext[0] = br->getFirstExt()[sel0];
				firstext[1] = br->getFirstExt()[sel1];
				lastext[0] = br->getLastExt()[sel0];
				lastext[1] = br->getLastExt()[sel1];

				const skeleton::ReconstructionBranch::Ptr br2(new skeleton::ReconstructionBranch(indskel,firstext,lastext));
				recskel2->addEdge(ext.first,ext.second,br2);
			}

			std::cout << "Images " << sel0 << " and " << sel1 << " -> ";
			algorithm::matchskeletons::ComposedMatching(recskel2,vec_compcontpr[sel0],vec_compcontpr[sel1],optionsmatch2);
			skeleton::CompContSkel3d::Ptr skelreconstructed = algorithm::matchskeletons::ComposedTriangulation(recskel2,vec_compcontpr);

			double err_graph = algorithm::evaluation::HausDist(skelreconstructed,vecshape,veccam);
			std::cout << "Reprojection error : " << err_graph*100 << "%" << std::endl;

			if(minerr_graph > err_graph)
				minerr_graph = err_graph;
		}
	}

	std::cout << "Minimal reprojection error : " << minerr_graph*100 << "%" << std::endl;

	optionsmatch.methodmatch = algorithm::matchskeletons::OptionsMatch::ode;
	std::cout << std::endl << "~~ Matching ode ~~" << std::endl;
	algorithm::matchskeletons::ComposedMatching(recskel,vec_compcontpr,optionsmatch);
	
	skeleton::CompContSkel3d::Ptr skelreconstructedode = algorithm::matchskeletons::ComposedTriangulation(recskel,vec_compcontpr);

	double err_ode = algorithm::evaluation::HausDist(skelreconstructedode,vecshape,veccam);
	std::cout << "Reprojection error : " << err_ode*100 << "%" << std::endl;
}

int main(int argc, char** argv)
{
	std::string imgfile;
	std::string orifile;
	std::string camfile;
	std::string recskelfile;
	std::string extskelfile;
	double sat;
	double lambda;
	unsigned int nbimg;
	
	boost::program_options::options_description desc("OPTIONS");
	
	desc.add_options()
		("help", "Help message")
		("nbimg", boost::program_options::value<unsigned int>(&nbimg)->default_value(2), "Number of images")
		("imgfile", boost::program_options::value<std::string>(&imgfile)->default_value("mask"), "Binary image file (*.png)")
		("orifile", boost::program_options::value<std::string>(&orifile)->default_value("img"), "Real image file (*.jpg)")
		("camfile", boost::program_options::value<std::string>(&camfile)->default_value("cam"), "Camera file (*.xml)")
		("recskelfile", boost::program_options::value<std::string>(&recskelfile)->default_value("recskel.txt"), "Reconstruction Skeleton file")
		("extskelfile", boost::program_options::value<std::string>(&extskelfile)->default_value("extskel.txt"), "Extremities Skeleton file")
		("sat", boost::program_options::value<double>(&sat)->default_value(1.2), "Scale Axis Transform parameter")
		("lambda", boost::program_options::value<double>(&lambda)->default_value(0.2), "Lambda parameter")
		;
	
	boost::program_options::variables_map vm;
	boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
	boost::program_options::notify(vm);
	
	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 0;
	}

	std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
	std::cout << "~         Initialisation          ~" << std::endl;
	
	
	std::vector<std::vector<unsigned int> > assoc_ext;
	assoc_ext = fileio::ReadExtSkel(extskelfile);
	if(assoc_ext.size() != 0)
		nbimg = assoc_ext.size();
	
	std::vector<camera::Camera::Ptr> veccam(nbimg);
	std::vector<shape::DiscreteShape<2>::Ptr> vecshape(nbimg);
	std::vector<cv::Mat> vecimage(nbimg);
	std::vector<typename skeleton::GraphProjSkel::Ptr> vecprskel(nbimg);

	for(unsigned int i = 0; i < nbimg; i++)
	{
		std::cout << "Image " << i+1 << "... ";
		std::ostringstream camfilename;
		camfilename << camfile << i+1 << ".xml";
		veccam[i] = fileio::ReadCamera(camfilename.str());

		std::ostringstream imgfilename;
		imgfilename << imgfile << i+1 << ".png";
		cv::Mat shpimg = cv::imread(imgfilename.str(),CV_LOAD_IMAGE_GRAYSCALE);

		vecshape[i] = shape::DiscreteShape<2>::Ptr(new shape::DiscreteShape<2>(shpimg.cols,shpimg.rows));
		cv::Mat cpymat(shpimg.rows,shpimg.cols,CV_8U,&vecshape[i]->getContainer()[0]);
		shpimg.copyTo(cpymat);

		boundary::DiscreteBoundary<2>::Ptr bnd = algorithm::extractboundary::MarchingSquare(vecshape[i],4);

		vecprskel[i] = algorithm::pruning::ScaleAxisTransform(algorithm::skeletonization::ProjectiveVoronoi(bnd,veccam[i]),sat);

		vecimage[i] = cv::Mat(shpimg.rows,shpimg.cols,CV_8UC3,cv::Scalar(0,0,0));
		displayopencv::DisplayDiscreteShape(vecshape[i],vecimage[i],mathtools::affine::Frame<2>::CanonicFrame(),cv::Scalar(255,255,255));
		displayopencv::DisplayGraphSkeleton(vecprskel[i],vecimage[i],mathtools::affine::Frame<2>::CanonicFrame(),cv::Scalar(255,0,0));
		std::cout << "Done" << std::endl;
	}

	algorithm::matchskeletons::OptionsMatch optionsmatch;
	std::cout.precision(2);
	std::cout.setf(std::ios::fixed);
	std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
	

	/********************************************************************************************
	 *                           Reconstruction with clicked skeleton                           *
	 ********************************************************************************************/
	
	std::cout << std::endl << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
	std::cout << "~ Clicked skeleton reconstruction ~" << std::endl;
	skeleton::ReconstructionSkeleton::Ptr recskelclick = fileio::ReadRecSkel(recskelfile);

	if(!recskelclick)
	{
		for(unsigned int i = 0; i < nbimg; i++)
		{
			std::ostringstream winname;
			winname << "Shape " << i+1;
			cv::namedWindow(winname.str(), CV_WINDOW_AUTOSIZE);
			cv::imshow(winname.str(),vecimage[i]);
			cv::waitKey(100);
		}
		cv::waitKey(100);

		recskelclick = skeleton::ReconstructionSkeleton::Ptr(new skeleton::ReconstructionSkeleton());
		std::cout << "How many branches have to be reconstructed?" << std::endl;
		unsigned int nbbranches;
		std::cin >> nbbranches;

		for(unsigned int i = 0; i < nbbranches; i++)
		{
			std::cout << "Please click extremities of branch " << i+1 << std::endl;
			std::vector<unsigned int> firstext(nbimg);
			std::vector<unsigned int> lastext(nbimg);
			std::vector<unsigned int> vecindpr(nbimg);
			for(unsigned int i = 0; i < nbimg; i++)
			{
				cv::Mat imgcpy = vecimage[i].clone();
				std::vector<unsigned int> indnod = userinput::ClickSkelNodes(imgcpy,vecprskel[i],2,mathtools::affine::Frame<2>::CanonicFrame());
				firstext[i] = indnod[0];
				lastext[i] = indnod[1];
				vecindpr[i] = i;
			}

			skeleton::ReconstructionBranch::Ptr recbr(new skeleton::ReconstructionBranch(vecindpr,firstext,lastext));

			recskelclick->addNode(i*2);
			recskelclick->addNode(i*2+1);
			recskelclick->addEdge(i*2,i*2+1,recbr);
		}
		fileio::WriteRecSkel(recskelclick,recskelfile);
	}
	
	ReconstructionEvals(optionsmatch,vecprskel,recskelclick,veccam,vecshape,lambda);
	std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;

	/********************************************************************************************
	 *                           Reconstruction with topologic matching                         *
	 ********************************************************************************************/
	
	std::cout << std::endl << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
	std::cout << "~       Topologic matching        ~" << std::endl;

	if(assoc_ext.size() == 0)
	{
		for(unsigned int i = 0; i < nbimg; i++)
		{
			std::ostringstream winname;
			winname << "Shape " << i+1;
			cv::namedWindow(winname.str(), CV_WINDOW_AUTOSIZE);
			cv::imshow(winname.str(),vecimage[i]);
			cv::waitKey(100);
		}
		cv::waitKey(100);

		std::cout << "How many extremities are visible in all images?" << std::endl;
		unsigned int nbext;
		std::cin >> nbext;
		
		std::cout << "Please click extremities of graphs " << std::endl;
		assoc_ext.resize(nbimg);
		for(unsigned int i = 0; i < nbimg; i++)
		{
			assoc_ext[i] = userinput::ClickSkelNodes(vecimage[i],vecprskel[i],nbext,mathtools::affine::Frame<2>::CanonicFrame(),true);
		}
		fileio::WriteExtSkel(assoc_ext,extskelfile);
	}
	
	skeleton::ReconstructionSkeleton::Ptr recskeltopo = algorithm::graphoperation::TopoMatch(vecprskel,assoc_ext);
	
	ReconstructionEvals(optionsmatch,vecprskel,recskeltopo,veccam,vecshape,lambda);
	std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;

	return 0;
}
