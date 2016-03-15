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
 *  \brief Provides tests for mathtools library
 *  \author Bastien Durix
 */

#include <mathtools/application/Bspline.h>
#include <mathtools/application/Compositor.h>
#include <mathtools/application/LinearApp.h>

#include <mathtools/vectorial/Basis.h>

#include <mathtools/affine/Frame.h>
#include <mathtools/affine/Point.h>

using namespace mathtools;
using namespace mathtools::application;
using namespace mathtools::vectorial;
using namespace mathtools::affine;

#ifdef _WIN32
#define BOOST_TEST_STATIC_LINK
#else
#define BOOST_TEST_DYN_LINK
#endif
#define BOOST_TEST_MODULE TestMathtools
#include <boost/test/unit_test.hpp>

#include <iostream>

BOOST_AUTO_TEST_CASE( BsplineTest )
{
	unsigned int fraction = 100;
	
	std::vector<unsigned int> bin_coeff(6,0);
	bin_coeff[0] = 1;
	for(unsigned int degree = 1; degree < 5; degree++)
	{
		// Filling node vector
		Eigen::Matrix<double,1,Eigen::Dynamic> node(1,degree*2);
		for(unsigned int i = 0; i < degree; i++)
		{
			node(0,i)        = 0.0;
			node(0,i+degree) = 1.0;
		}
		
		for(unsigned int i = degree; i > 0; i--)
		{
			bin_coeff[i] += bin_coeff[i-1];
		}
		
		for(unsigned int indice = 0; indice <= degree ; indice++)
		{
			for(unsigned int i = 0; i < fraction; i++)
			{
				double t = (double)i*(1.0/(double)fraction);
				
				double res = BsplineBasis(t,degree,indice,node);
				
				double ref = pow((1-t),degree-indice)*pow(t,indice)*(double)bin_coeff[indice];
				
				BOOST_CHECK(fabs(res - ref) < std::numeric_limits<double>::epsilon());
			}
		}
	}
}

BOOST_AUTO_TEST_CASE( CompositorTest )
{
	Eigen::Matrix3d mat_lin;
	mat_lin <<  0.0, 2.0, 0.0,
			   -1.0, 0.0, 0.0,
			    0.0, 0.0, 3.0;
	unsigned int degree = 3;
	unsigned int nbctrlpt = 4;
	Eigen::Matrix<double,1,Eigen::Dynamic> nodevec(1,nbctrlpt + degree - 1);
	nodevec << 0.0, 0.0, 0.0, 1.0, 1.0, 1.0;
	Eigen::Matrix<double,3,Eigen::Dynamic> ctrlpt(3,nbctrlpt);
	ctrlpt << 0.0, 1.0, 1.0, 0.0,
			  0.0, 0.0, 1.0, 1.0,
			  0.0, 0.0, 0.0, 0.0;
	Bspline<3> bsp(ctrlpt,nodevec,degree);
	Compositor<LinearApp<3,3>,Bspline<3> > comp(LinearApp<3,3>(mat_lin),bsp);
	
	for(unsigned int i = 0; i < 101; i++)
	{
		double t = (double)i*0.01;

		Eigen::Vector3d vecres = comp(t);
		Eigen::Vector3d dvecres = Eigen::Map<Eigen::Vector3d>((double*)comp.der(t).data());
		Eigen::Vector3d ddvecres = Eigen::Map<Eigen::Vector3d>((double*)comp.der2(t).data());

		Eigen::Vector3d vecref = mat_lin * bsp(t);
		Eigen::Vector3d dvecref = mat_lin * Eigen::Map<Eigen::Vector3d>((double*)bsp.der(t).data());
		Eigen::Vector3d ddvecref = mat_lin * Eigen::Map<Eigen::Vector3d>((double*)bsp.der2(t).data());
		
		BOOST_CHECK( (vecres-vecref).norm() < std::numeric_limits<double>::epsilon() );
		BOOST_CHECK( (dvecres-dvecref).norm() < std::numeric_limits<double>::epsilon() );
		BOOST_CHECK( (ddvecres-ddvecref).norm() < std::numeric_limits<double>::epsilon() );
	}
}

BOOST_AUTO_TEST_CASE( BasisTest )
{
	// matrix inversion test
	Eigen::Matrix3d mat;
	mat <<  1,  2, -7,
		-2, 45,  6,
		0, -5,  3;
	Basis<3>::Ptr basis = Basis<3>::CreateBasis(mat);

	Eigen::Matrix3d matinv = mat.inverse();

	BOOST_CHECK( matinv.isApprox(basis->getMatrixInverse(),std::numeric_limits<double>::epsilon()) );

	// non invertible matrix test
	try
	{
		Eigen::Matrix3d mat;
		mat <<  1, 0, 0,
			    0, 1, 1,
				1, 0, 0;
		Basis<3>::Ptr basis = Basis<3>::CreateBasis(mat);
		BOOST_ERROR( "Accepted non invertible matrix" );
	}
	catch(...)
	{
	}
}

BOOST_AUTO_TEST_CASE( AffineTest )
{
	Point<2> A(0,0), B(10,0), C(4,5);
	Eigen::Vector2d AB = B-A;
	
	BOOST_CHECK( AB.isApprox(Eigen::Vector2d(10,0),std::numeric_limits<double>::epsilon()) );
	
	Frame<2>::Ptr frame1 = Frame<2>::CreateFrame(B.getCoords());
	
	BOOST_CHECK( B.getCoords(frame1).norm() < std::numeric_limits<double>::epsilon() );

	Eigen::Vector2d AB_1 = B.getCoords(frame1) - A.getCoords(frame1);

	BOOST_CHECK( AB.isApprox(AB_1,std::numeric_limits<double>::epsilon()) );
	
	Eigen::Matrix2d mat;
	mat << 1,  2,
		   0, -1;

	Frame<2>::Ptr frame2 = Frame<2>::CreateFrame(C.getCoords(),Basis<2>::CreateBasis(mat));
	
	Eigen::Vector2d coordsB_2 = B.getCoords(frame2);

	Point<2> D(coordsB_2,frame2);

	BOOST_CHECK( D.getCoords().isApprox(B.getCoords(),std::numeric_limits<double>::epsilon()) );
}
