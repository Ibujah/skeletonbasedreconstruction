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
 *  \brief Skeleton reconstruction software
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
#include <algorithm/matchskeletons/SkelMatching.h>
#include <algorithm/matchskeletons/SkelTriangulation.h>
#include <algorithm/skinning/ContinuousSkinning.h>
#include <algorithm/evaluation/ReprojError.h>

#include <userinput/ClickSkelNode.h>
#include <fileio/CameraFile.h>
#include <fileio/BoundaryFile.h>
#include <fileio/ExtClickFile.h>
#include <displayopencv/DisplayShapeOCV.h>
#include <displayopencv/DisplayBoundaryOCV.h>
#include <displayopencv/DisplaySkeletonOCV.h>
#include <display3d/DisplayClass.h>
#include <display3d/DisplayFrame.h>
#include <display3d/DisplayCamera.h>
#include <display3d/DisplaySkeleton.h>
#include <display3d/DisplayBoundary.h>

int main(int argc, char** argv)
{
	std::string imgfile;
	std::string orifile;
	std::string camfile;
	std::string outbound;
	std::string extskelfile;
	double sat;
	unsigned int nbimg;
	
	boost::program_options::options_description desc("OPTIONS");
	
	desc.add_options()
		("help", "Help message")
		("nbimg", boost::program_options::value<unsigned int>(&nbimg)->default_value(2), "Number of images")
		("imgfile", boost::program_options::value<std::string>(&imgfile)->default_value("mask"), "Binary image file (*.png)")
		("orifile", boost::program_options::value<std::string>(&orifile)->default_value("img"), "Real image file (*.jpg)")
		("camfile", boost::program_options::value<std::string>(&camfile)->default_value("cam"), "Camera file (*.xml)")
		("outbound", boost::program_options::value<std::string>(&outbound)->default_value("skelrec.obj"), "Boundary output file")
		("extskelfile", boost::program_options::value<std::string>(&extskelfile)->default_value("extskel.txt"), "Extremities Skeleton file")
		("sat", boost::program_options::value<double>(&sat)->default_value(1.2), "Scale Axis Transform parameter")
		;
	
	boost::program_options::variables_map vm;
	boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
	boost::program_options::notify(vm);
	
	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 0;
	}

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
		std::cout << "Image " << i+1 << std::endl;
		std::cout << "Opening camera file" << std::endl;
		std::ostringstream camfilename;
		camfilename << camfile << i+1 << ".xml";
		veccam[i] = fileio::ReadCamera(camfilename.str());

		std::cout << "Opening image file" << std::endl;
		std::ostringstream imgfilename;
		imgfilename << imgfile << i+1 << ".png";
		cv::Mat shpimg = cv::imread(imgfilename.str(),CV_LOAD_IMAGE_GRAYSCALE);

		vecshape[i] = shape::DiscreteShape<2>::Ptr(new shape::DiscreteShape<2>(shpimg.cols,shpimg.rows));
		cv::Mat cpymat(shpimg.rows,shpimg.cols,CV_8U,&vecshape[i]->getContainer()[0]);
		shpimg.copyTo(cpymat);

		std::cout << "Extract boundary" << std::endl;
		boundary::DiscreteBoundary<2>::Ptr bnd = algorithm::extractboundary::MarchingSquare(vecshape[i],4);

		std::cout << "Extract projective skeleton" << std::endl;
		vecprskel[i] = algorithm::pruning::ScaleAxisTransform(algorithm::skeletonization::ProjectiveVoronoi(bnd,veccam[i]),sat);

		std::cout << "Display" << std::endl;
		vecimage[i] = cv::Mat(shpimg.rows,shpimg.cols,CV_8UC3,cv::Scalar(0,0,0));
		displayopencv::DisplayDiscreteShape(vecshape[i],vecimage[i],mathtools::affine::Frame<2>::CanonicFrame(),cv::Scalar(255,255,255));
		displayopencv::DisplayGraphSkeleton(vecprskel[i],vecimage[i],mathtools::affine::Frame<2>::CanonicFrame(),cv::Scalar(255,0,0));

		std::ostringstream winname;
		winname << "Shape " << i+1;
		cv::namedWindow(winname.str(), CV_WINDOW_AUTOSIZE);
		cv::imshow(winname.str(),vecimage[i]);
		cv::waitKey(100);
	}

	
	std::cout << "Creating reconstruction skeleton" << std::endl;
	skeleton::ReconstructionSkeleton::Ptr recskel(new skeleton::ReconstructionSkeleton());
	
	if(assoc_ext.size() == 0)
	{
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
	
	for(unsigned int i = 0; i < nbimg; i++)
	{
		std::ostringstream winname;
		winname << "Shape " << i+1;
		cv::namedWindow(winname.str(), CV_WINDOW_AUTOSIZE);
		cv::imshow(winname.str(),vecimage[i]);
	}
	
	cv::waitKey(10);
	
	std::cout << "Topologic matching" << std::endl;
	recskel = algorithm::graphoperation::TopoMatch(vecprskel,assoc_ext);
	std::cout << "Done" << std::endl;
	
	std::vector<typename skeleton::CompGraphProjSkel::Ptr> vec_comppr = algorithm::graphoperation::GetComposed(recskel,vecprskel);
	
	std::vector<typename skeleton::CompContProjSkel::Ptr> vec_compcontpr(nbimg);
	
	std::cout << "Fitting Bspline" << std::endl;
	for(unsigned int i = 0; i < vec_comppr.size(); i++)
	{
		vec_compcontpr[i] = algorithm::fitbspline::Graph2Bspline(vec_comppr[i]);
	}
	
	std::cout << "Matching" << std::endl;
	algorithm::matchskeletons::OptionsMatch optionsmatch;
	optionsmatch.methodmatch = algorithm::matchskeletons::OptionsMatch::enum_methodmatch::ode;
	algorithm::matchskeletons::ComposedMatching(recskel,vec_compcontpr,optionsmatch);
	
	std::cout << "Triangulation" << std::endl;
	skeleton::CompContSkel3d::Ptr skelreconstructed = algorithm::matchskeletons::ComposedTriangulation(recskel,vec_compcontpr);

	std::cout << "Reprojection error evaluation" << std::endl;
	double err = algorithm::evaluation::HausDist(skelreconstructed,vecshape,veccam);
	std::cout.precision(2);
	std::cout.setf(std::ios::fixed);
	std::cout << "Reprojection error : " << err*100 << "%" << std::endl;
	
	std::cout << "Skinning" << std::endl;
	boundary::DiscreteBoundary<3>::Ptr bnd = algorithm::skinning::ContinuousSkinning(skelreconstructed);
	
	std::cout << "Writing boundary" << std::endl;
	fileio::WriteBoundaryOBJ(bnd,outbound);
	
	std::cout << "3D display" << std::endl;
	display3d::DisplayClass disclass("Reconstructed skeleton");
	
	display3d::DisplayFrame(disclass,mathtools::affine::Frame<3>::CanonicFrame());
	display3d::DisplayBoundary_Wired(disclass,bnd);
	display3d::DisplaySkeleton(disclass,skelreconstructed,1.0,0.0,0.0);
	for(unsigned int i = 0; i < nbimg; i++)
		display3d::DisplayCamera(disclass,veccam[i]);
	
	disclass.enableCtrl();
	
	while(disclass.isCtrlEnabled())
	{
		disclass.manageKeyboard();
		disclass.display();
	}
	
	
	return 0;
}
