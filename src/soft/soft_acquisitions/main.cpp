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
#include <iostream>

#include <camera/Camera.h>
#include <tracking/Tracker.h>

#include <fileio/CameraFile.h>

int main(int argc, char** argv)
{
	std::string camfile;
	
	boost::program_options::options_description desc("OPTIONS");
	
	desc.add_options()
		("help", "Help message")
		("camfile", boost::program_options::value<std::string>(&camfile)->default_value("cam.xml"), "Camera file")
		;
	
	boost::program_options::variables_map vm;
	boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
	boost::program_options::notify(vm);
	
	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 0;
	}
	
	camera::Camera::Ptr cam = fileio::ReadCamera(camfile);
	tracking::Tracker tracker;
	tracker.Init(cam);

	return 0;
}
