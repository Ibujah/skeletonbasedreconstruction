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
 *  \brief 3D environment test
 *  \author Bastien Durix
 */

#include <mathtools/application/Bspline.h>
#include <skeleton/Skeletons.h>
#include <boundary/DiscreteBoundary3.h>

#include <algorithm/skinning/ContinuousSkinning.h>

#include <display3d/DisplayClass.h>
#include <display3d/DisplayFrame.h>
#include <display3d/DisplaySkeleton.h>
#include <display3d/DisplayBoundary.h>

int main()
{
	unsigned int degree = 3;
	unsigned int nbctrlpt = 4;
	Eigen::Matrix<double,1,Eigen::Dynamic> nodevec(1,nbctrlpt + degree - 1);
	nodevec << 0.0, 0.0, 0.0, 1.0, 1.0, 1.0;
	Eigen::Matrix<double,4,Eigen::Dynamic> ctrlpt(4,nbctrlpt);
	ctrlpt << 0.0, 1.0, 1.0, 0.0,
			  0.0, 4.0, 1.0, 1.0,
			  0.0, 2.0, 2.0, 1.0,
			  0.1, 0.2, 0.2, 0.1;
	mathtools::application::Application<Eigen::Vector4d,double>::Ptr bspline(new mathtools::application::Bspline<4>(ctrlpt,nodevec,degree));
	skeleton::BranchContSkel3d::Ptr contbr(
			new skeleton::BranchContSkel3d(
					skeleton::model::Classic<3>::Ptr(new skeleton::model::Classic<3>()),
					bspline));

	//boundary::DiscreteBoundary<3>::Ptr bnd = algorithm::skinning::ContinuousSkinning(contbr);
	
	display3d::DisplayClass disclass("Test SFML");
	
	display3d::DisplayFrame(disclass,mathtools::affine::Frame<3>::CanonicFrame());
	display3d::DisplayBranch(disclass,contbr);
	//display3d::DisplayBoundary_Wired(disclass,bnd);
	
	
	disclass.enableCtrl();
	
	while(disclass.isCtrlEnabled())
	{
		disclass.manageKeyboard();
		disclass.display();
	}
	
	return 0;
}
