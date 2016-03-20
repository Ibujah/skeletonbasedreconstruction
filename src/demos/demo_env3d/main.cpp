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
	
	// Some skeleton
	unsigned int nbctrlpt1 = 4;
	Eigen::Matrix<double,1,Eigen::Dynamic> nodevec1(1,nbctrlpt1 + degree - 1);
	nodevec1 << 0.0, 0.0, 0.0, 1.0, 1.0, 1.0;
	Eigen::Matrix<double,4,Eigen::Dynamic> ctrlpt1(4,nbctrlpt1);
	ctrlpt1 << 0.0, 1.0, 3.0, 0.0,
			   0.0, 3.0, 1.0, 0.0,
			   0.0, 2.0, 2.0, 1.5,
			   0.1, 0.2, 0.2, 0.1;
	mathtools::application::Application<Eigen::Vector4d,double>::Ptr bspline1(new mathtools::application::Bspline<4>(ctrlpt1,nodevec1,degree));
	
	// Straight skeleton
	unsigned int nbctrlpt2 = 4;
	Eigen::Matrix<double,1,Eigen::Dynamic> nodevec2(1,nbctrlpt2 + degree - 1);
	nodevec2 << 0.0, 0.0, 0.0, 1.0, 1.0, 1.0;
	Eigen::Matrix<double,4,Eigen::Dynamic> ctrlpt2(4,nbctrlpt2);
	ctrlpt2 << 0.0, 0.0, 0.0, 0.0,
			   2.0, 2.0, 2.0, 2.0,
			   0.0, 0.5, 1.0, 1.5,
			   0.1, 0.2, 0.2, 0.1;
	mathtools::application::Application<Eigen::Vector4d,double>::Ptr bspline2(new mathtools::application::Bspline<4>(ctrlpt2,nodevec2,degree));
	
	// Partial straight skeleton
	unsigned int nbctrlpt3 = 6;
	Eigen::Matrix<double,1,Eigen::Dynamic> nodevec3(1,nbctrlpt3 + degree - 1);
	nodevec3 << 0.0, 0.0, 0.0, 0.3, 0.6, 1.0, 1.0, 1.0;
	Eigen::Matrix<double,4,Eigen::Dynamic> ctrlpt3(4,nbctrlpt3);
	ctrlpt3 << 3.0, 3.0, 3.0, 3.0, 3.0, 3.0,
			   0.0, 1.5, 1.5, 1.5, 1.5, 3.0,
			   0.0, 1.0, 1.5, 2.0, 2.5, 3.0,
			   0.1, 0.2, 0.2, 0.1, 0.1, 0.2;
	mathtools::application::Application<Eigen::Vector4d,double>::Ptr bspline3(new mathtools::application::Bspline<4>(ctrlpt3,nodevec3,degree));
	
	skeleton::BranchContSkel3d::Ptr contbr1(
			new skeleton::BranchContSkel3d(
					skeleton::model::Classic<3>::Ptr(new skeleton::model::Classic<3>()),
					bspline1));
	boundary::DiscreteBoundary<3>::Ptr bnd1 = algorithm::skinning::ContinuousSkinning(contbr1->reverted()->reverted());
	
	skeleton::BranchContSkel3d::Ptr contbr2(
			new skeleton::BranchContSkel3d(
					skeleton::model::Classic<3>::Ptr(new skeleton::model::Classic<3>()),
					bspline2));
	boundary::DiscreteBoundary<3>::Ptr bnd2 = algorithm::skinning::ContinuousSkinning(contbr2->reverted()->reverted());
	
	skeleton::BranchContSkel3d::Ptr contbr3(
			new skeleton::BranchContSkel3d(
					skeleton::model::Classic<3>::Ptr(new skeleton::model::Classic<3>()),
					bspline3));
	boundary::DiscreteBoundary<3>::Ptr bnd3 = algorithm::skinning::ContinuousSkinning(contbr3->reverted()->reverted());
	
	display3d::DisplayClass disclass("Test SFML");
	
	display3d::DisplayFrame(disclass,mathtools::affine::Frame<3>::CanonicFrame());
	display3d::DisplayBranch(disclass,contbr1);
	display3d::DisplayBoundary_Wired(disclass,bnd1);
	display3d::DisplayBranch(disclass,contbr2);
	display3d::DisplayBoundary_Wired(disclass,bnd2);
	display3d::DisplayBranch(disclass,contbr3);
	display3d::DisplayBoundary_Wired(disclass,bnd3);
	
	
	disclass.enableCtrl();
	
	while(disclass.isCtrlEnabled())
	{
		disclass.manageKeyboard();
		disclass.display();
	}
	
	return 0;
}
