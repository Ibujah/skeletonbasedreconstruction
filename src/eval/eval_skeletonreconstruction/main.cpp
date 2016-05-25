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
						 const std::vector<shape::DiscreteShape<2>::Ptr> &vecshape)
{
	std::vector<typename skeleton::CompGraphProjSkel::Ptr> vec_comppr = algorithm::graphoperation::GetComposed(recskel,vec_prskel);
	
	std::vector<typename skeleton::CompContProjSkel::Ptr> vec_compcontpr(2);
	
	for(unsigned int i = 0; i < vec_comppr.size(); i++)
	{
		vec_compcontpr[i] = algorithm::fitbspline::Graph2Bspline(vec_comppr[i]);
	}
	
	optionsmatch.methodmatch = algorithm::matchskeletons::OptionsMatch::graph;
	std::cout << std::endl << "~~ Matching graph ~~" << std::endl;
	algorithm::matchskeletons::ComposedMatching(recskel,vec_compcontpr[0],vec_compcontpr[1],optionsmatch);
	skeleton::CompContSkel3d::Ptr skelreconstructed = algorithm::matchskeletons::ComposedTriangulation(recskel,vec_compcontpr);

	double err_graph = algorithm::evaluation::HausDist(skelreconstructed,vecshape,veccam);
	std::cout << "Reprojection error : " << err_graph*100 << "%" << std::endl;
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
	
	boost::program_options::options_description desc("OPTIONS");
	
	desc.add_options()
		("help", "Help message")
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
	
	std::cout << "Image 1" << std::endl;
	std::cout << "Opening camera file" << std::endl;
	std::ostringstream camfilename1;
	camfilename1 << camfile << 1 << ".xml";
	camera::Camera::Ptr cam1 = fileio::ReadCamera(camfilename1.str());
	
	std::cout << "Opening image file" << std::endl;
	std::ostringstream imgfilename1;
	imgfilename1 << imgfile << 1 << ".png";
	cv::Mat shpimg1 = cv::imread(imgfilename1.str(),CV_LOAD_IMAGE_GRAYSCALE);
	
	shape::DiscreteShape<2>::Ptr shape1(new shape::DiscreteShape<2>(shpimg1.cols,shpimg1.rows));
	cv::Mat cpymat1(shpimg1.rows,shpimg1.cols,CV_8U,&shape1->getContainer()[0]);
	shpimg1.copyTo(cpymat1);
	
	std::cout << "Extract boundary" << std::endl;
	boundary::DiscreteBoundary<2>::Ptr bnd1 = algorithm::extractboundary::MarchingSquare(shape1,4);
	
	std::cout << "Extract projective skeleton" << std::endl;
	skeleton::GraphProjSkel::Ptr prskel1 = algorithm::pruning::ScaleAxisTransform(algorithm::skeletonization::ProjectiveVoronoi(bnd1,cam1),sat);
	
	
	std::cout << "Image 2" << std::endl;
	std::cout << "Opening camera file" << std::endl;
	std::ostringstream camfilename2;
	camfilename2 << camfile << 2 << ".xml";
	camera::Camera::Ptr cam2 = fileio::ReadCamera(camfilename2.str());
	
	std::cout << "Opening image file" << std::endl;
	std::ostringstream imgfilename2;
	imgfilename2 << imgfile << 2 << ".png";
	cv::Mat shpimg2 = cv::imread(imgfilename2.str(),CV_LOAD_IMAGE_GRAYSCALE);
	
	shape::DiscreteShape<2>::Ptr shape2(new shape::DiscreteShape<2>(shpimg2.cols,shpimg2.rows));
	cv::Mat cpymat2(shpimg2.rows,shpimg2.cols,CV_8U,&shape2->getContainer()[0]);
	shpimg2.copyTo(cpymat2);
	
	std::cout << "Extract boundary" << std::endl;
	boundary::DiscreteBoundary<2>::Ptr bnd2 = algorithm::extractboundary::MarchingSquare(shape2,4);
	
	std::cout << "Extract projective skeleton" << std::endl;
	skeleton::GraphProjSkel::Ptr prskel2 = algorithm::pruning::ScaleAxisTransform(algorithm::skeletonization::ProjectiveVoronoi(bnd2,cam2),sat);
	
	std::vector<typename skeleton::GraphProjSkel::Ptr> vec_prskel(2);
	vec_prskel[0] = prskel1;
	vec_prskel[1] = prskel2;

	cv::Mat image1(shpimg1.rows,shpimg1.cols,CV_8UC3,cv::Scalar(0,0,0));
	displayopencv::DisplayDiscreteShape(shape1,image1,mathtools::affine::Frame<2>::CanonicFrame(),cv::Scalar(255,255,255));
	displayopencv::DisplayGraphSkeleton(prskel1,image1,mathtools::affine::Frame<2>::CanonicFrame(),cv::Scalar(255,0,0));
	cv::Mat image2(shpimg2.rows,shpimg2.cols,CV_8UC3,cv::Scalar(0,0,0));
	displayopencv::DisplayDiscreteShape(shape2,image2,mathtools::affine::Frame<2>::CanonicFrame(),cv::Scalar(255,255,255));
	displayopencv::DisplayGraphSkeleton(prskel2,image2,mathtools::affine::Frame<2>::CanonicFrame(),cv::Scalar(255,0,0));


	algorithm::matchskeletons::OptionsMatch optionsmatch;
	optionsmatch.lambda = lambda;
	std::vector<shape::DiscreteShape<2>::Ptr> vecshape(2);
	vecshape[0] = shape1;
	vecshape[1] = shape2;
	std::vector<camera::Camera::Ptr> veccam(2);
	veccam[0] = cam1;
	veccam[1] = cam2;
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
		cv::namedWindow("Shape 1", CV_WINDOW_AUTOSIZE);
		cv::imshow("Shape 1",image1);
		cv::namedWindow("Shape 2", CV_WINDOW_AUTOSIZE);
		cv::imshow("Shape 2",image2);

		cv::waitKey(100);

		recskelclick = skeleton::ReconstructionSkeleton::Ptr(new skeleton::ReconstructionSkeleton());
		std::cout << "How many branches have to be reconstructed?" << std::endl;
		unsigned int nbbranches;
		std::cin >> nbbranches;

		for(unsigned int i = 0; i < nbbranches; i++)
		{
			std::cout << "Please click extremities of branch " << (i+1) << std::endl;
			std::vector<unsigned int> indnod1 = userinput::ClickSkelNodes(image1,prskel1,2,mathtools::affine::Frame<2>::CanonicFrame());
			std::vector<unsigned int> indnod2 = userinput::ClickSkelNodes(image2,prskel2,2,mathtools::affine::Frame<2>::CanonicFrame());

			std::vector<unsigned int> vecindpr(2);
			vecindpr[0] = 0;
			vecindpr[1] = 1;
			std::vector<unsigned int> firstext(2);
			firstext[0] = indnod1[0];
			firstext[1] = indnod2[0];
			std::vector<unsigned int> lastext(2);
			lastext[0] = indnod1[1];
			lastext[1] = indnod2[1];
			skeleton::ReconstructionBranch::Ptr recbr(new skeleton::ReconstructionBranch(vecindpr,firstext,lastext));

			recskelclick->addNode(i*2);
			recskelclick->addNode(i*2+1);
			recskelclick->addEdge(i*2,i*2+1,recbr);
		}
		fileio::WriteRecSkel(recskelclick,recskelfile);
	}
	
	ReconstructionEvals(optionsmatch,vec_prskel,recskelclick,veccam,vecshape);
	std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;

	/********************************************************************************************
	 *                           Reconstruction with topologic matching                         *
	 ********************************************************************************************/
	
	std::cout << std::endl << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
	std::cout << "~       Topologic matching        ~" << std::endl;

	std::vector<std::vector<unsigned int> > assoc_ext;
	
	assoc_ext = fileio::ReadExtSkel(extskelfile);
	if(assoc_ext.size() == 0)
	{
		cv::namedWindow("Shape 1", CV_WINDOW_AUTOSIZE);
		cv::imshow("Shape 1",image1);
		cv::namedWindow("Shape 2", CV_WINDOW_AUTOSIZE);
		cv::imshow("Shape 2",image2);

		cv::waitKey(100);

		std::cout << "How many extremities are visible in all images?" << std::endl;
		unsigned int nbext;
		std::cin >> nbext;
		
		std::cout << "Please click extremities of graphs " << std::endl;
		std::vector<unsigned int> indnod1 = userinput::ClickSkelNodes(image1,prskel1,nbext,mathtools::affine::Frame<2>::CanonicFrame(),true);
		std::vector<unsigned int> indnod2 = userinput::ClickSkelNodes(image2,prskel2,nbext,mathtools::affine::Frame<2>::CanonicFrame(),true);
		
		assoc_ext.resize(2);
		assoc_ext[0] = indnod1;
		assoc_ext[1] = indnod2;
		fileio::WriteExtSkel(assoc_ext,extskelfile);
	}
	
	skeleton::ReconstructionSkeleton::Ptr recskeltopo = algorithm::graphoperation::TopoMatch(vec_prskel,assoc_ext);
	
	ReconstructionEvals(optionsmatch,vec_prskel,recskeltopo,veccam,vecshape);
	std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
	
	return 0;
}
