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
#include <algorithm/matchskeletons/SkelMatching2.h>
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
	
	boost::program_options::options_description desc("OPTIONS");
	
	desc.add_options()
		("help", "Help message")
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
	
	std::cout << "Display" << std::endl;
	cv::Mat image1(shpimg1.rows,shpimg1.cols,CV_8UC3,cv::Scalar(0,0,0));
	displayopencv::DisplayDiscreteShape(shape1,image1,mathtools::affine::Frame<2>::CanonicFrame(),cv::Scalar(255,255,255));
	displayopencv::DisplayGraphSkeleton(prskel1,image1,mathtools::affine::Frame<2>::CanonicFrame(),cv::Scalar(255,0,0));

	cv::namedWindow("Shape 1", CV_WINDOW_AUTOSIZE);
    cv::imshow("Shape 1",image1);
	
	
	
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
	
	std::cout << "Display" << std::endl;
	cv::Mat image2(shpimg2.rows,shpimg2.cols,CV_8UC3,cv::Scalar(0,0,0));
	displayopencv::DisplayDiscreteShape(shape2,image2,mathtools::affine::Frame<2>::CanonicFrame(),cv::Scalar(255,255,255));
	displayopencv::DisplayGraphSkeleton(prskel2,image2,mathtools::affine::Frame<2>::CanonicFrame(),cv::Scalar(255,0,0));
	
	cv::namedWindow("Shape 2", CV_WINDOW_AUTOSIZE);
    cv::imshow("Shape 2",image2);
	cv::waitKey(10);
	cv::waitKey(10);
	
	std::cout << "Creating reconstruction skeleton" << std::endl;
	skeleton::ReconstructionSkeleton::Ptr recskel(new skeleton::ReconstructionSkeleton());
	
	std::vector<std::vector<unsigned int> > assoc_ext;
	
	assoc_ext = fileio::ReadExtSkel(extskelfile);
	if(assoc_ext.size() == 0)
	{
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
	
	cv::namedWindow("Shape 1", CV_WINDOW_AUTOSIZE);
    cv::imshow("Shape 1",image1);
	cv::namedWindow("Shape 2", CV_WINDOW_AUTOSIZE);
    cv::imshow("Shape 2",image2);
	
	cv::waitKey(10);
	
	
	std::vector<typename skeleton::GraphProjSkel::Ptr> vec_prskel(2);
	vec_prskel[0] = prskel1;
	vec_prskel[1] = prskel2;
	
	std::cout << "Topologic matching" << std::endl;
	recskel = algorithm::graphoperation::TopoMatch(vec_prskel,assoc_ext);
	std::cout << "Done" << std::endl;
	
	std::vector<typename skeleton::CompGraphProjSkel::Ptr> vec_comppr = algorithm::graphoperation::GetComposed(recskel,vec_prskel);
	
	std::vector<typename skeleton::CompContProjSkel::Ptr> vec_compcontpr(2);
	
	std::cout << "Fitting Bspline" << std::endl;
	for(unsigned int i = 0; i < vec_comppr.size(); i++)
	{
		vec_compcontpr[i] = algorithm::fitbspline::Graph2Bspline(vec_comppr[i]);
	}
	
	std::cout << "Matching" << std::endl;
	algorithm::matchskeletons::OptionsMatch2 optionsmatch;
	optionsmatch.methodmatch = algorithm::matchskeletons::OptionsMatch2::enum_methodmatch::ode;
	algorithm::matchskeletons::ComposedMatching(recskel,vec_compcontpr[0],vec_compcontpr[1],optionsmatch);
	
	std::cout << "Triangulation" << std::endl;
	skeleton::CompContSkel3d::Ptr skelreconstructed = algorithm::matchskeletons::ComposedTriangulation(recskel,vec_compcontpr);

	std::cout << "Reprojection error evaluation" << std::endl;
	std::vector<shape::DiscreteShape<2>::Ptr> vecshape(2);
	vecshape[0] = shape1;
	vecshape[1] = shape2;
	std::vector<camera::Camera::Ptr> veccam(2);
	veccam[0] = cam1;
	veccam[1] = cam2;
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
	display3d::DisplayCamera(disclass,cam1);
	display3d::DisplayCamera(disclass,cam2,0.0,1.0,0.0);
	
	disclass.enableCtrl();
	
	while(disclass.isCtrlEnabled())
	{
		disclass.manageKeyboard();
		disclass.display();
	}
	
	
	return 0;
}
