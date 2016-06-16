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
 *  \brief Images acquisition software
 *  \author Bastien Durix
 */

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <iomanip>
#include <ctime>

#include <camera/Camera.h>
#include <tracking/Tracker.h>
#include <display3d/DisplayClass.h>
#include <display3d/DisplayFrame.h>
#include <display3d/DisplayCamera.h>

#include <fileio/CameraFile.h>

int main(int argc, char** argv)
{
	std::string camfile;
	std::string fold;
	unsigned int camID;
	unsigned int nbmarkers;
	double markersize;

	boost::program_options::options_description desc("OPTIONS");
	time_t tim = time(0);
	struct tm *now = std::localtime(&tim);
	
	std::ostringstream ossdate;
	ossdate << "./acq" << std::setfill('0') << std::setw(2) << now->tm_year+1900 << "_" 
					   << std::setfill('0') << std::setw(2) << now->tm_mon+1 << "_" 
					   << std::setfill('0') << std::setw(2) << now->tm_mday << "_" 
					   << std::setfill('0') << std::setw(2) << now->tm_hour << "_" 
					   << std::setfill('0') << std::setw(2) << now->tm_min << "/";

	desc.add_options()
		("help", "Help message")
		("camfile", boost::program_options::value<std::string>(&camfile)->default_value("cam.xml"), "Camera file")
		("camID", boost::program_options::value<unsigned int>(&camID)->default_value(1), "Camera number")
		("fold", boost::program_options::value<std::string>(&fold)->default_value(ossdate.str()), "Folder to save acquisitions in")
		("nbmarkers", boost::program_options::value<unsigned int>(&nbmarkers)->default_value(6), "Number of markers to detect")
		("markersize", boost::program_options::value<double>(&markersize)->default_value(0.4), "Marker size")
		;

	boost::program_options::variables_map vm;
	boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
	boost::program_options::notify(vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 0;
	}

	boost::filesystem::path dir(fold);
	if(!boost::filesystem::create_directory(dir))
		return -1;
	

	camera::Camera::Ptr cam = fileio::ReadCamera(camfile);
	tracking::Tracker tracker(nbmarkers,markersize);
	tracker.init(cam);

	cv::VideoCapture capture;
	capture.open( camID );
	capture.set(CV_CAP_PROP_FRAME_WIDTH, cam->getIntrinsics()->getWidth());
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, cam->getIntrinsics()->getHeight());
	capture.set(CV_CAP_PROP_FPS, 20);

	const std::string WINDOW_NAME = "Images acquisition";
	bool fini = false;

	while(!fini)
	{
        cv::Mat view;
		cv::Mat aff;
		// get the new frame from capture and copy it to view
        capture >> aff;
		aff.copyTo(view);

       	// if no more images to process exit the loop
        if(view.empty())
		{
			std::cerr << "No view taken" <<  std::endl;
        	break;
		}

        // detect the markers
		tracker.detect(aff);
        
		cv::imshow(WINDOW_NAME , aff);

		char key;
		if( (key=cv::waitKey( 10 )) >= 0)
		{
			if(key == 'a' || key == 'A')
			{
				if(tracker.addCurrDetection())
					std::cout << "View taken !" << std::endl;
				else
					std::cout << "Some markers are not seen in the image" << std::endl;
			}
			if(key=='q' || key=='Q')
			{
				std::cout << "Computing markers relative position..." << std::endl;
				fini = tracker.computeMulti();
				if(!fini)
					std::cout << "Not enough images" << std::endl;
			}
		}
	}
	
	cv::destroyWindow(WINDOW_NAME);
	cv::waitKey(1);
	cv::waitKey(1);
	
	std::cout << "Success !! " << std::endl;

	display3d::DisplayClass disclass(WINDOW_NAME,cam->getIntrinsics()->getWidth(),cam->getIntrinsics()->getHeight());
	display3d::DisplayFrame(disclass,mathtools::affine::Frame<3>::CanonicFrame());
	disclass.setIntrinsics(cam->getIntrinsics());
	
	fini = false;
	unsigned int nimg = 1;
	while(!fini)
	{
        cv::Mat view;
		cv::Mat aff;
		// get the new frame from capture and copy it to view
        capture >> aff;
		aff.copyTo(view);

       	// if no more images to process exit the loop
        if(view.empty())
		{
			std::cerr << "No acquired view" <<  std::endl;
        	break;
		}
		
        // detect the markers
		tracker.detect(aff);
		Eigen::Matrix<double,3,4> matTr;
		bool correct = tracker.getCurrTr(matTr);
		if(correct) disclass.setExtrinsics(matTr);
        
		disclass.setBackground(aff);
		if(correct)
			disclass.display();
		else
			disclass.display(std::list<unsigned int>());
		
		sf::Event event;
		while(disclass.getWindow().pollEvent(event))
		{
			if(event.type == sf::Event::KeyPressed)
			{
				if(event.key.code == sf::Keyboard::A)
				{
					mathtools::affine::Frame<3>::Ptr frame_cur;
					frame_cur = mathtools::affine::Frame<3>::CreateFrame(matTr.col(3),matTr.col(0),matTr.col(1),matTr.col(2));
					camera::Extrinsics::Ptr extr(new camera::Extrinsics(frame_cur->getFrameInverse()));
					camera::Camera::Ptr camimg(new camera::Camera(cam->getIntrinsics(),extr));
					
					std::ostringstream imgfname;
					imgfname << fold << "img" << nimg << ".jpg";
					std::ostringstream camfname;
					camfname << fold << "cam" << nimg << ".xml";
					cv::imwrite(imgfname.str(),view);
					fileio::WriteCamera(camimg,camfname.str());
					
					std::cout << "Saved image at " << imgfname.str() << std::endl;
					std::cout << "Saved camera at " << camfname.str() << std::endl;
					nimg++;
					
					display3d::DisplayCamera(disclass,camimg);
				}

				if(event.key.code == sf::Keyboard::Q)
				{
					fini = true;
				}
			}
		}
	}
	

	capture.release();


	return 0;
}
